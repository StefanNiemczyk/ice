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
      return nullptr;
    }
    else if (strcmp("offered", child) == 0)
    {
      for (element = cElement->FirstChildElement("information"); element; element = element->NextSiblingElement())
      {
        bool result = this->readInformation(element, this->offered);
      }
    }
    else if (strcmp("required", child) == 0)
    {
      for (element = cElement->FirstChildElement("information"); element; element = element->NextSiblingElement())
      {
        bool result = this->readInformation(element, this->required);
      }
    }
    else
    {
      _log->warn("Unknown child '%v', will be ignored", child);
    }
  }

  return true;
}

std::vector<std::shared_ptr<InformationSpecification>> XMLInformationReader::getOffered()
{
  return this->offered;
}

std::vector<std::shared_ptr<InformationSpecification>> XMLInformationReader::getRequired()
{
  return this->required;
}

bool XMLInformationReader::readInformation(TiXmlElement* element, std::vector<std::shared_ptr<InformationSpecification>> &infos)
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
      return nullptr;
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
    _log->warn("Incomplete information description: entity '%s', entityType '%s', scope '%s', representation '%s', relatedEntity '%s'",
               entity, entityType, scope, representation, relatedEntity);
    return false;
  }

  _log->debug("Extracted information description: entity '%s', entityType '%s', scope '%s', representation '%s', relatedEntity '%s'",
             entity, entityType, scope, representation, relatedEntity);

  std::shared_ptr<InformationSpecification> spec = std::make_shared<InformationSpecification>(
      entity, entityType, scope, representation, relatedEntity);
  infos.push_back(spec);

  return true;
}

} /* namespace ice */
