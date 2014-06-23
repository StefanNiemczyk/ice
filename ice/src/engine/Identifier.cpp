/*
 * Identifier.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/Identifier.h"
#include <iostream>

namespace ice {

std::string IDGenerator::toString(identifier id)
{
  return boost::lexical_cast<std::string>(id);
}

const uint8_t* IDGenerator::toByte(identifier id)
{
  return id.data;
}

const int IDGenerator::compare(identifier identifier1, identifier identifier2)
{
  if (identifier1 == identifier2)
    return 0;

  if (identifier1 > identifier2)
    return 1;

  return -1;
}

IDGenerator::IDGenerator()
{
  //
}

IDGenerator::~IDGenerator()
{
  //
}

identifier IDGenerator::getIdentifier()
{
  return boost::uuids::random_generator()();
}

identifier IDGenerator::getIdentifier(std::string value)
{
  return boost::lexical_cast<boost::uuids::uuid>(value);
}

identifier IDGenerator::getIdentifier(std::vector<uint8_t> value)
{
  boost::uuids::uuid uuid;

  for (int i=0; i < 16; ++i)
    uuid.data[i] = value.at(i);

  return uuid;
}

} /* namespace ice */
