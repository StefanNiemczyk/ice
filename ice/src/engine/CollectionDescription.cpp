/*
 * StreamDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include <ice/information/CollectionDescription.h>
#include <sstream>

namespace ice
{

CollectionDescription::CollectionDescription(const std::shared_ptr<InformationSpecification> informationSpeciviation,
                                     const std::string name, const std::string provider, const std::string sourceSystem,
                                     std::map<std::string, int> metadatas) :
    specification(informationSpeciviation), provider(provider), sourceSystem(sourceSystem)
{
  this->name = name;
  this->metadatas = metadatas;
  this->id = IDGenerator::getInstance()->getIdentifier();
}

CollectionDescription::~CollectionDescription()
{
  //
}

const std::shared_ptr<InformationSpecification> CollectionDescription::getInformationSpecification() const
{
  return this->specification;
}

const std::string CollectionDescription::getProvider() const
{
  return this->provider;
}

const std::string CollectionDescription::getSourceSystem() const
{
  return this->sourceSystem;
}

const std::string CollectionDescription::getName() const
{
  return this->name;
}

const bool CollectionDescription::isNamedStream() const
{
  return this->name != "";
}

void CollectionDescription::setMetadataValue(std::string metadata, int value)
{
  this->metadatas[metadata] = value;
}

int CollectionDescription::getMetadataValue(std::string metadata, bool* found)
{
  if (this->metadatas.find(metadata) == this->metadatas.end())
  {
    *found = false;
    return 0;
  }

  *found = true;

  return this->metadatas[metadata];
}

std::map<std::string, int> CollectionDescription::getMetadatas()
{
  return this->metadatas;
}

const bool CollectionDescription::equals(CollectionDescription const* rhs) const
{
  return this->specification == rhs->getInformationSpecification();
}

identifier CollectionDescription::getId()
{
  return this->id;
}


std::string CollectionDescription::toString()
{
  std::stringstream ss;

  ss << "streamDescription(" << this->specification->toString() << ","
     << this->provider << "," << this->sourceSystem << ")";

  return ss.str();
}

} /* namespace ice */
