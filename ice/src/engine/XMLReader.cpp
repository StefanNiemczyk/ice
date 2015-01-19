/*
 * XMLReader.cpp
 *
 *  Created on: May 27, 2014
 *      Author: sni
 */

#include "ice/XMLReader.h"
#include "easylogging++.h"

namespace ice
{

XMLReader::XMLReader()
{
  this->_log = el::Loggers::getLogger("XMLReader");
}

XMLReader::~XMLReader()
{
  this->clear();
}

std::vector<XMLInformation*>* XMLReader::getInformations()
{
  return &this->informations;
}

std::vector<XMLStream*>* XMLReader::getStreams()
{
  return &this->streams;
}

std::vector<XMLStreamTeamplate*>* XMLReader::getStreamTemplates()
{
  return &this->streamTemplates;
}

std::vector<XMLNode*>* XMLReader::getNodes()
{
  return &this->nodes;
}

bool XMLReader::readFile(const std::string& fileName)
{
  TiXmlDocument doc(fileName);
  if (!doc.LoadFile())
  {
    return false;
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* element;
  TiXmlHandle hRoot(0);

  // find root element
  {
    element = hDoc.FirstChildElement().Element();
    // should always have a valid root but handle gracefully if it does
    if (!element || strcmp("ice", element->Value()) != 0)
    {
      std::cerr << "XMLReader (" << fileName << ") Root element should be 'ice' but is " << element->Value()
          << std::endl;
      return false;
    }

    // save this for later
    hRoot = TiXmlHandle(element);
  }

  // block: read information
  {
    element = hRoot.FirstChild("Informations").Element();

    if (!element || element->NoChildren())
    {
      //  std::cerr << "XMLReader (" << fileName << ") No information specification" << std::endl;
    }
    else
    {
      for (element = element->FirstChildElement("Information"); element; element = element->NextSiblingElement())
      {
        XMLInformation* information = this->readInformation(element);

        if (information)
          this->informations.push_back(information);
        else
          return false;
      }
    }
  }

  // block: read stream
  {
    element = hRoot.FirstChild("NamedStreams").Element();

    if (!element || element->NoChildren())
    {
      // std::cerr << "XMLReader (" << fileName << ") No stream specification" << std::endl;
    }
    else
    {
      for (element = element->FirstChildElement("Stream"); element; element = element->NextSiblingElement())
      {
        XMLStream* stream = this->readStream(element, true);

        if (stream)
          this->streams.push_back(stream);
        else
          return false;
      }
    }
  }

  // block: read stream template
  {
    element = hRoot.FirstChild("StreamRequests").Element();

    if (!element || element->NoChildren())
    {
      // std::cerr << "XMLReader (" << fileName << ") No stream template specification" << std::endl;
    }
    else
    {
      for (element = element->FirstChildElement("StreamTemplate"); element; element = element->NextSiblingElement())
      {
        XMLStreamTeamplate* streamTemplate = this->readStreamTemplate(element, true);

        if (streamTemplate)
          this->streamTemplates.push_back(streamTemplate);
        else
          return false;
      }
    }
  }

  // block: read nodes
  {
    element = hRoot.FirstChild("Nodes").Element();

    if (!element || element->NoChildren())
    {
      // std::cerr << "XMLReader (" << fileName << ") No nodes specification" << std::endl;
    }
    else
    {

      for (element = element->FirstChildElement("Node"); element; element = element->NextSiblingElement())
      {
        XMLNode* node = this->readNode(element);

        if (node)
          this->nodes.push_back(node);
        else
          return false;
      }
    }
  }

  return true;
}

bool XMLReader::readFiles(std::initializer_list<std::string> fileNameList)
{
  bool result = true;

  for (auto elem : fileNameList)
  {
    result &= this->readFile(elem);
  }

  return result;
}

XMLInformation* XMLReader::readInformation(TiXmlElement* element)
{
  const char *name = element->Value();
  const char *uuid = element->Attribute("uuid");

  if (!name || strcmp("Information", name) != 0 || !uuid)
  {
    std::cerr << "Invalid Information " << name << " uuid " << uuid << std::endl;

    return NULL;
  }

  XMLInformation* information = new XMLInformation();

  information->uuid = uuid;

  for (TiXmlElement* infoElement = element->FirstChildElement(); infoElement;
      infoElement = infoElement->NextSiblingElement())
  {
    const char *child = infoElement->Value();
    if (!child)
    {
      std::cerr << "Invalid child of information " << name << " : " << child << std::endl;
      delete information;
      return NULL;
    }
    else if (strcmp("Topic", child) == 0)
    {
      information->topic = infoElement->GetText();
    }
    else if (strcmp("Type", child) == 0)
    {
      information->type = infoElement->GetText();
    }
    else if (strcmp("_desc", child) == 0)
    {
      information->_desc = infoElement->GetText();
    }
    else if (strcmp("Information", child) == 0)
    {
      XMLInformation* info = this->readInformation(infoElement);

      if (info)
        information->nested.push_back(info);
      else
      {
        std::cerr << "Invalid nested information " << name << " : " << child << std::endl;
        delete information;
        return NULL;
      }
    }
    else
    {
      std::cerr << "Unknown child of information " << name << " : " << child << std::endl;
      delete information;
      return NULL;
    }
  }

  return information;
}

XMLStream* XMLReader::readStream(TiXmlElement* element, bool checkName)
{
  const char *name = element->Value();
  const char *streamName = element->Attribute("name");
  const char *trigger = element->Attribute("trigger");

  if (!name || strcmp("Stream", name) != 0 || (!streamName && checkName))
  {
    std::cerr << "Invalid Stream " << name << " name " << streamName << std::endl;

    return NULL;
  }

  XMLStream* stream = new XMLStream();

  if (streamName)
    stream->name = streamName;
  else
    stream->name = "#ANONYMOUS";

  if (trigger)
  {
    std::string strigger(trigger);
    stream->trigger = ("true" == strigger);
  }

  for (TiXmlElement* streamElement = element->FirstChildElement(); streamElement;
      streamElement = streamElement->NextSiblingElement())
  {
    const char *child = streamElement->Value();
    if (!child)
    {
      std::cerr << "Invalid child of stream " << name << " : " << child << std::endl;
      delete stream;
      return NULL;
    }
    else if (strcmp("Information", child) == 0)
    {
      stream->informationUuid = streamElement->GetText();
    }
    else if (strcmp("Provider", child) == 0)
    {
      stream->provider = streamElement->GetText();
    }
    else if (strcmp("Sharing", child) == 0)
    {
      this->readStreamSharing(streamElement, stream);
      //stream->shared = streamElement->GetText();
    }
    else if (strcmp("Size", child) == 0)
    {
      stream->size = std::stoi(streamElement->GetText());
    }
    else if (strcmp("_desc", child) == 0)
    {
      stream->_desc = streamElement->GetText();
    }
    else
    {
      std::cerr << "Unknown child of stream " << name << " : " << child << std::endl;
      delete stream;
      return NULL;
    }
  }

  return stream;
}

bool XMLReader::readStreamSharing(TiXmlElement* element, XMLStream* stream)
{
  const char *name = element->Value();
  const char *state = element->Attribute("state");

  if (!name || strcmp("Sharing", name) != 0 || !state)
  {
    _log->error("Invalid sharing element %s, with state %s", (name ? name : "null"),
                (state ? state : "null"));
    return false;
  }

  if (state)
  {
    stream->sharingState = state;
  }

  for (TiXmlElement* streamElement = element->FirstChildElement(); streamElement;
      streamElement = streamElement->NextSiblingElement())
  {
    const char *child = streamElement->Value();
    if (!child)
    {
      _log->error("Invalid child of sharing element %s: %s", (name ? name : "null"), "null");
      return false;
    }
    else if (strcmp("MaxSharingCount", child) == 0)
    {
      stream->sharingMaxCount = std::stoi(streamElement->GetText());
    }
    else
    {
      _log->error("Invalid child of sharing element %s: %s", (name ? name : "null"),
                  (child ? child : "null"));
      return false;
    }
  }

  return stream;
}

XMLStreamTeamplate* XMLReader::readStreamTemplate(TiXmlElement* element, bool checkName)
{
  const char *name = element->Value();
  const char *streamName = element->Attribute("name");
  const char *trigger = element->Attribute("trigger");

  if (!name || strcmp("StreamTemplate", name) != 0 || (checkName && !streamName))
  {
    std::cerr << "Invalid StreamTemplate " << name << " name " << streamName << std::endl;

    return NULL;
  }

  XMLStreamTeamplate* streamTemplate = new XMLStreamTeamplate();

  if (trigger)
  {
    std::string strigger(trigger);
    streamTemplate->trigger = ("true" == strigger);
  }

  if (streamName)
    streamTemplate->name = streamName;
  else
    streamTemplate->name = "#ANONYMOUS";

  for (TiXmlElement* streamTemplateElement = element->FirstChildElement(); streamTemplateElement;
      streamTemplateElement = streamTemplateElement->NextSiblingElement())
  {
    const char *child = streamTemplateElement->Value();
    if (!child)
    {
      std::cerr << "Invalid child of stream template " << name << " : " << child << std::endl;
      delete streamTemplate;
      return NULL;
    }
    else if (strcmp("Information", child) == 0)
    {
      streamTemplate->informationUuid = streamTemplateElement->GetText();
    }
    else if (strcmp("Provider", child) == 0)
    {
      streamTemplate->provider = streamTemplateElement->GetText();
    }
    else if (strcmp("MaxStreamCount", child) == 0)
    {
      streamTemplate->maxStreamCount = std::stoi(streamTemplateElement->GetText());
    }
    else if (strcmp("MaxStreamCount", child) == 0)
    {
      streamTemplate->maxStreamCount = std::stoi(streamTemplateElement->GetText());
    }
    else if (strcmp("Size", child) == 0)
    {
      streamTemplate->size = std::stoi(streamTemplateElement->GetText());
    }
    else if (strcmp("_desc", child) == 0)
    {
      streamTemplate->_desc = streamTemplateElement->GetText();
    }
    else
    {
      std::cerr << "Unknown child of stream template " << name << " : " << child << std::endl;
      delete streamTemplate;
      return NULL;
    }
  }

  return streamTemplate;
}

XMLNode* XMLReader::readNode(TiXmlElement* element)
{
  const char *name = element->Value();
  const char *type = element->Attribute("type");
  const char *nodeName = element->Attribute("name");

  if (!name || strcmp("Node", name) != 0 || !type || !nodeName)
  {
    std::cerr << "Invalid Node '" << (name ? name : "NULL") << "' type '" << (type ? type : "NULL") << "' name '"
        << (name ? name : "NULL") << std::endl;

    return NULL;
  }

  XMLNode* node = new XMLNode();
  std::string sType = type;

  if (sType == "source")
  {
    node->type = NodeType::SOURCE;
  }
  else if (sType == "processing")
  {
    node->type = NodeType::PROCESSING;
  }
  else
  {
    std::cerr << "Invalid type of node '" << name << "' : '" << type << "'" << std::endl;
    delete node;
    return NULL;
  }

  node->name = nodeName;

  for (TiXmlElement* nodeElement = element->FirstChildElement(); nodeElement;
      nodeElement = nodeElement->NextSiblingElement())
  {
    const char *child = nodeElement->Value();
    if (!child)
    {
      std::cerr << "Invalid child of node " << name << " : " << child << std::endl;
      delete node;
      return NULL;
    }
    else if (strcmp("Source", child) == 0)
    {
      node->source = nodeElement->GetText();
    }
    else if (strcmp("ClassName", child) == 0)
    {
      node->className = nodeElement->GetText();
    }
    else if (strcmp("CyclicTrigger", child) == 0)
    {
      node->trigger = std::stol(nodeElement->GetText());
    }
    else if (strcmp("_desc", child) == 0)
    {
      node->_desc = nodeElement->GetText();
    }
    else if (strcmp("Inputs", child) == 0)
    {
      for (TiXmlElement* input = nodeElement->FirstChildElement("Stream"); input; input = input->NextSiblingElement())
      {
        if (strcmp("Stream", input->Value()) == 0)
        {
          XMLStream* stream = this->readStream(input, false);

          if (stream)
          {
            node->inputs.push_back(stream);
          }
          else
          {
            delete node;
            return NULL;
          }
        }
        else if (strcmp("StreamTemplate", input->Value()) == 0)
        {
          XMLStreamTeamplate* streamTemplate = this->readStreamTemplate(input, false);

          if (streamTemplate)
          {
            node->inputTemplates.push_back(streamTemplate);
          }
          else
          {
            delete node;
            return NULL;
          }
        }
        else
        {
          std::cerr << "Invalid input for node: '" << input->Value() << "'" << std::endl;
          delete node;
          return NULL;
        }
      }
    }
    else if (strcmp("Outputs", child) == 0)
    {
      for (TiXmlElement* output = nodeElement->FirstChildElement("Stream"); output; output =
          output->NextSiblingElement())
      {
        XMLStream* stream = this->readStream(output, false);

        if (stream)
        {
          node->outputs.push_back(stream);
        }
        else
        {
          delete node;
          return NULL;
        }
      }
    }
    else if (strcmp("Configurations", child) == 0)
    {
      for (TiXmlElement* configElement = nodeElement->FirstChildElement(); configElement;
          configElement = configElement->NextSiblingElement())
      {
        const char *elementName = configElement->Value();
        const char *configName = configElement->Attribute("name");
        const char *configValue = configElement->GetText();

        if (!elementName || strcmp("Config", elementName) != 0 || !configName || !configValue)
        {
          std::cerr << "Invalid config for node '" << (elementName ? elementName : "NULL") << "' with '"
              << (configName ? configName : "NULL") << "' value '" << (configValue ? configValue : "NULL") << "'"
              << std::endl;
          delete node;
          return NULL;
        }

        node->configs.insert(std::pair<std::string, std::string>(std::string(configName), std::string(configValue)));
      }
    }
    else
    {
      std::cerr << "Unknown child of node '" << (name ? name : "NULL") << "' : '" << (child ? child : "NULL") << "'"
          << std::endl;
      delete node;
      return NULL;
    }
  }

  return node;
}

void XMLReader::clear()
{
  // deleting information
  while (this->informations.size() > 0)
  {
    XMLInformation* information = this->informations.back();
    this->informations.pop_back();

    this->clearInformation(information);

    delete information;
  }

  // deleting streams
  while (this->streams.size() > 0)
  {
    XMLStream* stream = this->streams.back();
    this->streams.pop_back();

    delete stream;
  }

  // deleting stream templates
  while (this->streamTemplates.size() > 0)
  {
    XMLStreamTeamplate* streamTemplate = this->streamTemplates.back();
    this->streamTemplates.pop_back();

    delete streamTemplate;
  }

  // deleting nodes
  while (this->nodes.size() > 0)
  {
    XMLNode* node = this->nodes.back();
    this->nodes.pop_back();

    while (node->inputs.size() > 0)
    {
      XMLStream* stream = node->inputs.back();
      node->inputs.pop_back();

      delete stream;
    }

    while (node->inputTemplates.size() > 0)
    {
      XMLStreamTeamplate* streamTemplate = node->inputTemplates.back();
      node->inputTemplates.pop_back();

      delete streamTemplate;
    }

    while (node->outputs.size() > 0)
    {
      XMLStream* stream = node->outputs.back();
      node->outputs.pop_back();

      delete stream;
    }

    // node->configs.clear();

    delete node;
  }
}

void XMLReader::clearInformation(XMLInformation* information)
{
  while (information->nested.size() > 0)
  {
    XMLInformation* nestedInfo = information->nested.back();
    information->nested.pop_back();

    this->clearInformation(nestedInfo);

    delete nestedInfo;
  }
}

} /* namespace ice */
