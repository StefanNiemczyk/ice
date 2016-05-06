/*
 * RosGMessagePublisher.cpp
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#include "RosGContainerPublisher.h"

#include <locale>

#include <ice/ontology/OntologyInterface.h>

#include "IceServalBridge.h"

namespace ice
{

RosGContainerPublisher::RosGContainerPublisher(std::shared_ptr<OntologyInterface> ontology, std::string templateXMLFile)
    : ontology(ontology), templateXMLFile(templateXMLFile)
{
  _log = el::Loggers::getLogger("RosGContainerPublisher");

  setlocale(LC_ALL, "C");// TODO
}

RosGContainerPublisher::~RosGContainerPublisher()
{
}

bool RosGContainerPublisher::init()
{
  this->readXMLFile(this->templateXMLFile);
  return true;
}

bool RosGContainerPublisher::cleanUp()
{

  return true;
}

bool RosGContainerPublisher::findTemplate(std::string const &message, std::string const &representation, MessageTemplate &msgTemplate)
{
  bool found = false;

  for (auto &tmp : this->messageTemplates)
  {
    if (tmp.message == message)
    {
      msgTemplate = tmp;
      found = true;

      if (tmp.representation == representation)
      {
        return true;
      }
    }
  }

  return found;
}

bool RosGContainerPublisher::publish(std::shared_ptr<RequiredInfo> const &reqInfo, std::shared_ptr<GContainer> &container)
{
  MessageTemplate msgTemplate;

  if (false == this->findTemplate(reqInfo->message, reqInfo->infoSpec.getRepresentation(), msgTemplate))
  {
    _log->error("No message template found for representation '%v'", reqInfo->message);
    return false;
  }
  std::string message = msgTemplate.messageTemplate;

  bool result = this->transformToMessage(message, container);

  if (false == result)
    return false;

  return this->publish(reqInfo->topic, reqInfo->message, message);
}

bool RosGContainerPublisher::transformToMessage(std::string &message, std::shared_ptr<GContainer> &container)
{
  int index = message.find("{");
  int index2 = 0;
  std::string pathStr, valueStr;

  while (index != std::string::npos)
  {
    index2 = message.find("}");
    pathStr = message.substr(index + 1, index2 - index - 1);

    auto path = container->representation->accessPath(pathStr);

    if (path == nullptr)
    {
      _log->error("Unknown path '%v' in container of representation '%v'", pathStr, container->representation->name);
      return false;
    }

    auto pair = container->getPair(path);

    switch (pair.first)
    {
      case UNSET:
        _log->error("Unknown path '%v' in container of representation '%v'", pathStr, container->representation->name);
        return false;
        break;
      case BOOL:
        valueStr = *((bool*) pair.second) ? "true" : "false";
        break;
      case BYTE:
        valueStr = std::to_string(*((int8_t*) pair.second));
        break;
      case UNSIGNED_BYTE:
        valueStr = std::to_string(*((uint8_t*) pair.second));
        break;
      case SHORT:
        valueStr = std::to_string(*((short*) pair.second));
        break;
      case INT:
        valueStr = std::to_string(*((int*) pair.second));
        break;
      case LONG:
        valueStr = std::to_string(*((long*) pair.second));
        break;
      case UNSIGNED_SHORT:
        valueStr = std::to_string(*((unsigned short*) pair.second));
        break;
      case UNSIGNED_INT:
        valueStr = std::to_string(*((unsigned int*) pair.second));
        break;
      case UNSIGNED_LONG:
        valueStr = std::to_string(*((unsigned long*) pair.second));
        break;
      case FLOAT:
        valueStr = std::to_string(*((float*) pair.second));
        break;
      case DOUBLE:
        valueStr = std::to_string(*((double*) pair.second));
        break;
      case STRING:
        valueStr = *((std::string*) pair.second);
        break;
    }

    message = message.replace(index, index2 - index + 1, valueStr);
    index = message.find("{");
  }

  return true;
}

bool RosGContainerPublisher::publish(std::string const &topic, std::string const &messageName, std::string const &message) {
    std::stringstream output;
    std::string command = "rostopic pub --once " + topic + " " + messageName + " \"" + message + "\"";
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) return -1;
    char buffer[128];

    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
          output << buffer;
    }

    // todo

    output.seekg(0, std::ios::end);
    int size = output.tellg();
    output.seekg(0, std::ios::beg);

    return size;
}

bool RosGContainerPublisher::readXMLFile(const std::string& fileName)
{
  TiXmlDocument doc(fileName);
  if (!doc.LoadFile())
  {
    _log->error("Could not load XML file '%v'", fileName);
    return false;
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* element;
  TiXmlHandle hRoot(0);

  // find root element
  {
    element = hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!element || strcmp("templates", element->Value()) != 0)
    {
      _log->error("Root element should be 'templates' but is '%v' in file '%v'", element->Value(), fileName);
      return false;
    }

    // save this for later
    hRoot = TiXmlHandle(element);
  }

  for (TiXmlElement* cElement = hRoot.FirstChildElement().Element(); cElement; cElement = cElement->NextSiblingElement())
  {
    const char *child = cElement->Value();

    if (!child)
    {
      _log->error("Invalid child '%v' of operations", child);
      return nullptr;
    }
    else if (strcmp("template", child) == 0)
    {
      bool result = this->readTemplate(cElement);
    }
    else
    {
      _log->warn("Unknown child '%v', will be ignored", child);
    }
  }

  return true;
}

bool RosGContainerPublisher::readTemplate(TiXmlElement* element)
{
  const char *tagName = element->Value();
  const char *message = element->Attribute("message");
  const char *representation = element->Attribute("representation");

  if (!tagName || strcmp("template", tagName) != 0)
  {
    _log->error("Invalid tag '%v' for template tag", tagName);

    return false;
  }

  if (representation == nullptr)
    representation = "";

  std::string msgTemplate = element->GetText();

  if (message == "" || msgTemplate == "")
  {
    _log->warn("Incomplete template description: message '%v', representation '%v', messageTemplate '%v'",
               message, representation, msgTemplate);
    return false;
  }

  int index = msgTemplate.find("{");
  int index2 = 0;
  std::string longIri, shortIri;

  while (index != std::string::npos)
  {
    index2 = msgTemplate.find("}");
    longIri = msgTemplate.substr(index + 1, index2 - index - 1);

    shortIri = this->ontology->toShortIri(longIri);

    msgTemplate = msgTemplate.replace(index + 1, index2 - index - 1, shortIri);
    index = msgTemplate.find("{", index + 1);
  }

  _log->debug("Extracted template description: message '%v', representation '%v', messageTemplate '%v'",
             message, representation, msgTemplate);

  MessageTemplate tmp;
  tmp.message = message;
  tmp.representation = representation;
  tmp.messageTemplate = msgTemplate;

  this->messageTemplates.push_back(tmp);

  return true;
}

} /* namespace ice */
