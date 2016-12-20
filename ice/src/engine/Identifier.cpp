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
  return std::to_string(id);
//  return boost::lexical_cast<std::string>(id);
}

const std::unique_ptr<char[]> IDGenerator::toByte(identifier id)
{
  std::unique_ptr<char[]> bytes(new char[sizeof id]);
  std::copy(static_cast<const char*>(static_cast<const void*>(&id)),
            static_cast<const char*>(static_cast<const void*>(&id)) + sizeof id,
            bytes.get());

  return std::move(bytes);
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
  static int counter = 1;
  return ++counter;
 // return boost::uuids::random_generator()();
}

identifier IDGenerator::getIdentifier(std::string value)
{
  return std::stoul(value);
//  return boost::lexical_cast<boost::uuids::uuid>(value);
}

identifier IDGenerator::getIdentifier(char* value)
{
  int id;

  std::copy(value, value + sizeof id, &id);

  return id;

//  boost::uuids::uuid uuid;
//
//  for (int i=0; i < 16; ++i)
//    uuid.data[i] = value.at(i);
//
//  return uuid;
}

} /* namespace ice */
