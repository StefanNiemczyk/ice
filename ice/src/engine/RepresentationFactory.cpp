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
  reps = std::make_shared<std::vector<Representation*>>();
}

RepresentationFactory::~RepresentationFactory()
{

}

Representation* RepresentationFactory::fromCSV(std::string reprStr,
    const char delim)
{
  const size_t length = reprStr.length() + 1;
  std::string resName;
  Representation *rep = NULL;
  Representation *lastRep = NULL;

  if (reprStr.empty()) {
    return rep;
  }
  if (!isprint(delim)) {
    return rep;
  }

  std::vector<std::string> tokens = split(reprStr.c_str(), ';');
  std::vector<std::string>::reverse_iterator rit = tokens.rbegin();

  for (int i = 0; rit != tokens.rend(); rit++, i++) {
    lastRep = rep;
    rep = addOrGet(*rit);
    if (rep > 0 && lastRep != NULL) {
      rep->parent = lastRep;
    }
  }

  return rep;
}

Representation* RepresentationFactory::addOrGet(std::string name)
{
  Representation *res = NULL;

  if (repNameMap.count(name) > 0) {
    res = repNameMap.at(name);
  } else {
    res = new Representation;
    res->name = name;
    res->parent = NULL;
    repNameMap.insert(std::pair<std::string, Representation*>(name, res));
  }

  return res;
}

std::shared_ptr<std::vector<Representation*>> RepresentationFactory::fromCSVStrings(
    std::vector<std::string> lines)
{

  for (std::string line : lines) {
    Representation *r = fromCSV(line);
    reps->push_back(r);
  }

  return reps;
}

void RepresentationFactory::printReps()
{
  for (Representation* r : *reps) {
    std::cout << r->name << std::endl;
    fflush(stdout);
    Representation* par = r->parent;
    while (par->name != "null") {
      std::cout << "    " << par->name << " ADDR: " << par << std::endl;
      fflush(stdout);
      par = par->parent;
    }
  }
}

}
