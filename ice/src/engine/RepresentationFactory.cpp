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

Representation RepresentationFactory::fromCSV(std::string reprStr,
    const char delim)
{

  const size_t length = reprStr.length() + 1;
  char *repCstr = (char*) calloc(length, sizeof(char));
  Representation res;

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

  // TODO: Implement recursive handling of parent types

  free(repCstr);
  return res;
}

std::shared_ptr<std::vector<Representation>> RepresentationFactory::fromCSVStrings(
    std::vector<std::string> lines)
{
  reps = std::make_shared<std::vector<Representation>>();

  for (std::string line : lines) {
    Representation r = fromCSV(line);
    reps->push_back(r);
  }

  return reps;
}

}
