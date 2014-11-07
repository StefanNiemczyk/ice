/*
 * StreamTemplateDescription.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: sni
 */

#include "ice/coordination/StreamTemplateDescription.h"

#include "ice/information/StreamDescription.h"

namespace ice
{
StreamTemplateDescription::StreamTemplateDescription()
{
  //
}

StreamTemplateDescription::StreamTemplateDescription(const identifier id)
{
  this->id = id;
}

StreamTemplateDescription::~StreamTemplateDescription()
{
  //
}


const identifier& StreamTemplateDescription::getId() const
{
  return id;
}

void StreamTemplateDescription::setId(const identifier& id)
{
  this->id = id;
}

const bool StreamTemplateDescription::equals(StreamDescription const* rhs) const
{
  return this->id == rhs->getId();
}

const bool StreamTemplateDescription::equals(StreamTemplateDescription const* rhs) const
{
  return this->id == rhs->getId();
}

} /* namespace ice */
