/*
 * StreamTemplateDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/StreamTemplateDescription.h"
#include "ice/coordination/StreamDescription.h"

namespace ice
{
StreamTemplateDescription::StreamTemplateDescription()
{
  //
}

StreamTemplateDescription::StreamTemplateDescription(const boost::uuids::uuid uuid)
{
  this->uuid = uuid;
}

StreamTemplateDescription::~StreamTemplateDescription()
{
  //
}


const boost::uuids::uuid& StreamTemplateDescription::getUuid() const
{
  return uuid;
}

void StreamTemplateDescription::setUuid(const boost::uuids::uuid& uuid)
{
  this->uuid = uuid;
}

const bool StreamTemplateDescription::equals(StreamDescription const* rhs) const
{
  return this->uuid == rhs->getUuid();
}

const bool StreamTemplateDescription::equals(StreamTemplateDescription const* rhs) const
{
  return this->uuid == rhs->getUuid();
}

} /* namespace ice */
