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

StreamDescription::StreamDescription(const identifier id, bool shared)
{
  this->id = id;
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

const identifier& StreamDescription::getId() const
{
  return id;
}

void StreamDescription::setId(const identifier& id)
{
  this->id = id;
}

const bool StreamDescription::equals(StreamDescription const* rhs) const
{
  return this->id == rhs->getId() && this->shared == rhs->isShared();
}

const bool StreamDescription::equals(StreamTemplateDescription const* rhs) const
{
  return this->id == rhs->getId();
}

} /* namespace ice */
