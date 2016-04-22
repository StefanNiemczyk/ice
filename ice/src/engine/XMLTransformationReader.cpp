/*
 * XMLTransformationReader.cpp
 *
 *  Created on: Nov 26, 2015
 *      Author: sni
 */

#include <ice/representation/XMLTransformationReader.h>

namespace ice
{

XMLTransformationReader::XMLTransformationReader()
{
  // TODO Auto-generated constructor stub
  _log = el::Loggers::getLogger("XMLTransformationReader");
}

XMLTransformationReader::~XMLTransformationReader()
{
  this->clear();
}

bool XMLTransformationReader::readFile(const std::string& fileName)
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
    if (!element || strcmp("transformations", element->Value()) != 0)
    {
      _log->error("Root element should be 'transformations' but is '%v' in file '%v'", element->Value(), fileName);
      return false;
    }

    // save this for later
    hRoot = TiXmlHandle(element);
  }

  // read Transformations
  for (element = hRoot.FirstChildElement("transformation").Element(); element; element = element->NextSiblingElement())
  {
    TransDesc* transformation = this->readTransformation(element);

    if (transformation)
      this->transformationDescs.push_back(transformation);
    else
      return false;
  }

  return true;
}

TransDesc* XMLTransformationReader::readTransformation(TiXmlElement* element)
{
  const char *tagName = element->Value();
  const char *name = element->Attribute("name");
  const char *scope = element->Attribute("scope");

  if (!name || strcmp("transformation", tagName) != 0 || !name)
  {
    _log->error("Invalid tag '%v' for transformation '%v'", tagName, name);

    return nullptr;
  }

  TransDesc* transformation = new TransDesc();

  transformation->name = name;
  transformation->scope = scope;

  for (TiXmlElement* cElement = element->FirstChildElement(); cElement; cElement = cElement->NextSiblingElement())
  {
    const char *child = cElement->Value();
    if (!child)
    {
      _log->error("Invalid child '%v' in transformation '%v'", child, name);
      delete transformation;
      return nullptr;
    }
    else if (strcmp("input", child) == 0)
    {
      TransInput ti;
      ti.id = std::stoi(cElement->Attribute("id"));
      ti.representation = std::string(cElement->Attribute("representation"));
      transformation->inputs.push_back(ti);
    }
    else if (strcmp("output", child) == 0)
    {
      transformation->output = std::string(cElement->Attribute("representation"));
    }
    else if (strcmp("operations", child) == 0)
    {
      bool result = readOperations(cElement, transformation->ops);

      if (false == result)
      {
        delete transformation;
        return nullptr;
      }
    }
    else
    {
      _log->warn("Unknown child '%v' in transformation '%v', will be ignored", child, name);
    }
  }

  return transformation;
}

bool XMLTransformationReader::readOperations(TiXmlElement* element, std::vector<DimensionDesc>& dimensions)
{
  const char *tagName = element->Value();
//  const char *name = element->Attribute("name");

  if (!tagName || strcmp("operations", tagName) != 0)
  {
    _log->error("Invalid tag '%v' for operations tag", tagName);

    return false;
  }

  for (TiXmlElement* cElement = element->FirstChildElement(); cElement; cElement = cElement->NextSiblingElement())
  {

    const char *child = cElement->Value();
    if (!child)
    {
      _log->error("Invalid child '%v' of operations", child);
      return nullptr;
    }
    else if (strcmp("dimension", child) == 0)
    {
      auto e = cElement->FirstChildElement();
      const char *eName = e->Value();

      DimensionDesc desc;
      desc.name = std::string(cElement->Attribute("name"));

      if (strcmp("use", eName) == 0)
      {
        desc.type = XMLDimensionOperations::XML_USE;
        desc.sourceId = std::stoi(e->Attribute("id"));
        desc.path = std::string(e->Attribute("path"));
      }
      else if (strcmp("default", eName) == 0)
      {
        desc.type = XMLDimensionOperations::XML_DEFAULT;
        desc.value = std::string(e->Attribute("value"));
      }
      else if (strcmp("formula", eName) == 0)
      {
        desc.type = XMLDimensionOperations::XML_FORMULA;
        desc.formula = std::string(e->Attribute("formula"));

	const auto fc = e->FirstChild();
	if (fc != nullptr) { /* If there are multiple variables defined */
          for (auto velem = fc; velem != nullptr; velem = velem->NextSiblingElement()) {
            auto e = velem->ToElement();
            desc.varmap[std::string(e->Attribute("name"))] =
              std::make_pair(std::string(e->Attribute("path")),
                             std::stoi(e->Attribute("id"))); 
          }
	} else {
          desc.varmap[std::string(e->Attribute("varname"))] =
            std::make_pair(std::string(e->Attribute("path")),
                           std::stoi(e->Attribute("id"))); 
	}
      }
      else if (strcmp("operations", eName) == 0)
      {
        desc.type = XMLDimensionOperations::XML_COMPLEX;
        bool result = this->readOperations(e, desc.dims);

        if (false == result)
          return false;
      }
      else
      {
        _log->warn("Unknown child '%v' in dimension '%v', will be ignored", child, desc.name);
      }

      dimensions.push_back(desc);
    }
  }

  return true;
}

void XMLTransformationReader::clear()
{
  for (int i = 0; i < this->transformationDescs.size(); ++i)
  {
    delete this->transformationDescs.at(i);
  }
}

std::vector<TransDesc*>& XMLTransformationReader::getTransformations()
{
  return this->transformationDescs;
}

} /* namespace ice */
