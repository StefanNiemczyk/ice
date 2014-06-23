/*
 * StreamDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/StreamDescription.h"
#include "ice/coordination/StreamTemplateDescription.h"

namespace ice
{
StreamDescription::StreamDescription()
{
  this->shared = false;
}

StreamDescription::StreamDescription(const boost::uuids::uuid uuid, bool shared)
{
  this->uuid = uuid;
  this->shared = shared;
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

const boost::uuids::uuid& StreamDescription::getUuid() const
{
  return uuid;
}

void StreamDescription::setUuid(const boost::uuids::uuid& uuid)
{
  this->uuid = uuid;
}

const bool StreamDescription::equals(StreamDescription const* rhs) const
{
  return this->uuid == rhs->getUuid() && this->shared == rhs->isShared();
}

const bool StreamDescription::equals(StreamTemplateDescription const* rhs) const
{
  return this->uuid == rhs->getUuid();
}

} /* namespace ice */
