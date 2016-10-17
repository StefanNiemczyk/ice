/*
 * XMLInformationReader.cpp
 *
 *  Created on: Apr 22, 2016
 *      Author: sni
 */

#include "XMLInformationReader.h"

namespace ice
{

XMLInformationReader::XMLInformationReader()
{
  _log = el::Loggers::getLogger("XMLInformationReader");
}

XMLInformationReader::~XMLInformationReader()
{
  // TODO Auto-generated destructor stub
}

bool XMLInformationReader::readFile(const std::string& fileName)
{
  this->offered.clear();
  this->required.clear();

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
    if (!element || strcmp("ice_serval_bridge", element->Value()) != 0)
    {
      _log->error("Root element should be 'ice_serval_bridge' but is '%v' in file '%v'", element->Value(), fileName);
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
      return false;
    }
    else if (strcmp("offered", child) == 0)
    {
      for (element = cElement->FirstChildElement("information"); element; element = element->NextSiblingElement())
      {
        bool result = this->readOffered(element);
      }
    }
    else if (strcmp("required", child) == 0)
    {
      for (element = cElement->FirstChildElement("information"); element; element = element->NextSiblingElement())
      {
        bool result = this->readRequired(element);
      }
    }
    else
    {
      _log->warn("Unknown child '%v', will be ignored", child);
    }
  }

  return true;
}

std::vector<std::shared_ptr<OfferedInfo>> XMLInformationReader::getOffered()
{
  return this->offered;
}

std::vector<std::shared_ptr<RequiredInfo>> XMLInformationReader::getRequired()
{
  return this->required;
}

bool XMLInformationReader::readOffered(TiXmlElement* element)
{
  const char *tagName = element->Value();
//  const char *name = element->Attribute("name");

  if (!tagName || strcmp("information", tagName) != 0)
  {
    _log->error("Invalid tag '%v' for information tag", tagName);

    return false;
  }

  std::string entity, entityType, scope, representation, relatedEntity;

  for (TiXmlElement* cElement = element->FirstChildElement(); cElement; cElement = cElement->NextSiblingElement())
  {

    const char *child = cElement->Value();
    if (!child)
    {
      _log->error("Invalid child '%v' of operations", child);
      return false;
    }
    else if (strcmp("entity", child) == 0)
    {
      entity = cElement->GetText();
    }
    else if (strcmp("entityType", child) == 0)
    {
      entityType = cElement->GetText();
    }
    else if (strcmp("scope", child) == 0)
    {
      scope = cElement->GetText();
    }
    else if (strcmp("representation", child) == 0)
    {
      representation = cElement->GetText();
    }
    else if (strcmp("relatedEntity", child) == 0)
    {
      relatedEntity = cElement->GetText();
    }
  }

  if (entity == "" || entityType == "" || scope == "" || representation == "")
  {
    _log->warn("Incomplete information description: entity '%v', entityType '%v', scope '%v', representation '%v', relatedEntity '%v'",
               entity, entityType, scope, representation, relatedEntity);
    return false;
  }

  _log->debug("Extracted information description: entity '%v', entityType '%v', scope '%v', representation '%v', relatedEntity '%v'",
             entity, entityType, scope, representation, relatedEntity);

  std::shared_ptr<OfferedInfo> info = std::make_shared<OfferedInfo>(entity, entityType, scope, representation, relatedEntity);
  this->offered.push_back(info);

  return true;
}

bool XMLInformationReader::readRequired(TiXmlElement* element)
{
  const char *tagName = element->Value();
//  const char *name = element->Attribute("name");

  if (!tagName || strcmp("information", tagName) != 0)
  {
    _log->error("Invalid tag '%v' for information tag", tagName);

    return false;
  }

  std::string entity, entityType, scope, representation, relatedEntity, topic, message;

  for (TiXmlElement* cElement = element->FirstChildElement(); cElement; cElement = cElement->NextSiblingElement())
  {

    const char *child = cElement->Value();
    if (!child)
    {
      _log->error("Invalid child '%v' of operations", child);
      return false;
    }
    else if (strcmp("entity", child) == 0)
    {
      entity = cElement->GetText();
    }
    else if (strcmp("entityType", child) == 0)
    {
      entityType = cElement->GetText();
    }
    else if (strcmp("scope", child) == 0)
    {
      scope = cElement->GetText();
    }
    else if (strcmp("representation", child) == 0)
    {
      representation = cElement->GetText();
    }
    else if (strcmp("relatedEntity", child) == 0)
    {
      relatedEntity = cElement->GetText();
    }
    else if (strcmp("topic", child) == 0)
    {
      topic = cElement->GetText();
    }
    else if (strcmp("message", child) == 0)
    {
      message = cElement->GetText();
    }
  }

  if (entity == "" || entityType == "" || scope == "" || representation == "" || topic == "" || message == "")
  {
    _log->warn("Incomplete information description: entity '%v', entityType '%v', scope '%v', representation '%v', relatedEntity '%v', topic '%v', message '%v'",
               entity, entityType, scope, representation, relatedEntity, topic, message);
    return false;
  }

  _log->debug("Extracted information description: entity '%v', entityType '%v', scope '%v', representation '%v', relatedEntity '%v', topic '%v', message '%v'",
             entity, entityType, scope, representation, relatedEntity, topic, message);

  std::shared_ptr<RequiredInfo> info = std::make_shared<RequiredInfo>(entity, entityType, scope, representation, relatedEntity);
  info->topic = topic;
  info->message = message;
  this->required.push_back(info);

  return true;
}

} /* namespace ice */
