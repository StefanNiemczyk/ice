/*
 * StreamDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/information/StreamDescription.h"

#include <sstream>

namespace ice
{

StreamDescription::StreamDescription(const std::shared_ptr<InformationSpecification> informationSpeciviation,
                                     const std::string name, const std::string provider, const std::string sourceSystem,
                                     std::map<std::string, int> metadatas, bool shared) :
    informationSpecification(informationSpeciviation), provider(provider), sourceSystem(sourceSystem)
{
  this->name = name;
  this->shared = shared;
  this->metadatas = metadatas;
  this->id = IDGenerator::getInstance()->getIdentifier();
}

StreamDescription::~StreamDescription()
{
  //
}

bool StreamDescription::isShared() const
{
  return shared;
}

void StreamDescription::setShared(bool shared)
{
  this->shared = shared;
}

const std::shared_ptr<InformationSpecification> StreamDescription::getInformationSpecification() const
{
  return this->informationSpecification;
}

const std::string StreamDescription::getProvider() const
{
  return this->provider;
}

const std::string StreamDescription::getSourceSystem() const
{
  return this->sourceSystem;
}

const std::string StreamDescription::getName() const
{
  return this->name;
}

const bool StreamDescription::isNamedStream() const
{
  return this->name != "";
}

void StreamDescription::setMetadataValue(std::string metadata, int value)
{
  this->metadatas[metadata] = value;
}

int StreamDescription::getMetadataValue(std::string metadata, bool* found)
{
  if (this->metadatas.find(metadata) == this->metadatas.end())
  {
    *found = false;
    return 0;
  }

  *found = true;

  return this->metadatas[metadata];
}

std::map<std::string, int> StreamDescription::getMetadatas()
{
  return this->metadatas;
}

const bool StreamDescription::equals(StreamDescription const* rhs) const
{
  return this->informationSpecification == rhs->getInformationSpecification() && this->shared == rhs->isShared();
}

identifier StreamDescription::getId()
{
  return this->id;
}


std::string StreamDescription::toString()
{
  std::stringstream ss;

  ss << "streamDescription(" << this->informationSpecification->toString() << ",";
  ss << this->provider << "," << this->sourceSystem << "," << this->shared << ")";

  return ss.str();
}

//const bool StreamDescription::equals(StreamTemplateDescription const* rhs) const
//{
//  return this->informationSpecification == rhs->getId();
//}

} /* namespace ice */
