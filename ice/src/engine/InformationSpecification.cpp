/*
 * InformationSpecification.cpp
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#include "ice/information/InformationSpecification.h"

namespace ice
{

InformationSpecification::InformationSpecification(boost::uuids::uuid uuid, const std::string name) :
    uuid(uuid), name(name)
{
  //
}

InformationSpecification::~InformationSpecification()
{
  // TODO Auto-generated destructor stub
}

const boost::uuids::uuid& InformationSpecification::getUUID() const
{
  return this->uuid;
}

const std::string& InformationSpecification::getName() const
{
  return this->name;
}

const std::string InformationSpecification::getDescription() const
{
  return this->description;
}

void InformationSpecification::setDescription(std::string description)
{
  this->description = description;
}

const std::string InformationSpecification::getTypeString() const
{
  return this->typeString;
}

void InformationSpecification::setTypeString(std::string type)
{
  this->typeString = type;
}

} /* namespace ice */
