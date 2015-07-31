/*
 * RepresentationFactory.cpp
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#include <stddef.h>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "ice/representation/split.h"
#include "ice/representation/RepresentationFactory.h"

namespace ice {

RepresentationFactory::RepresentationFactory()
{

}

RepresentationFactory::~RepresentationFactory()
{

}

RepresentationInstance RepresentationFactory::fromCSV(std::string reprStr,
    const char delim)
{
  const size_t length = reprStr.length() + 1;
  char *repCstr = (char*) calloc(length, sizeof(char));
  RepresentationInstance res;

  if (reprStr.empty()) {
    return res;
  }
  if (!isprint(delim)) {
    return res;
  }

  strncpy(repCstr, reprStr.c_str(), length);

  if (repCstr == NULL) {
    free(repCstr);
    return res;
  }

  std::vector<std::string> tokens = split(repCstr, ';');
  const char *typeStr = tokens.at(0).c_str();
  res.name = tokens.at(1);

  errno = 0;
  char *ep;
  const int typeNum = strtol(typeStr, &ep, 10);
  if (typeStr == ep || *ep != '\0') {
    std::cerr << "Error while converting " << typeStr << " to int."
        << std::endl;
    free(repCstr);
    return res;
  }

  if (typeNum < 0 || typeNum > UnknownType) {
    std::cerr << "Error: invalid superclass" << std::endl;
    free(repCstr);
    return res;
  }

  res.type = static_cast<RepresentationType>(typeNum);

  free(repCstr);
  return res;
}

std::shared_ptr<std::vector<RepresentationInstance>> RepresentationFactory::fromCSVStrings(
    std::vector<std::string> lines)
{
  instances = std::make_shared<std::vector<RepresentationInstance>>();

  for (std::string line : lines) {
    RepresentationInstance r = fromCSV(line);
    instances->push_back(r);
  }

  return instances;
}

}
