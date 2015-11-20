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
  repVec = std::make_shared<std::vector<Representation*>>();
  repMap = std::make_shared<std::map<std::string, Representation*>>();
  repInstanceMap = std::make_shared<std::map<std::string, BaseRepresentationInstance*>>();
}

RepresentationFactory::~RepresentationFactory()
{

}

Representation* RepresentationFactory::fromCSV(std::string reprStr,
    const char delim)
{
  const size_t length = reprStr.length() + 1;

  if (reprStr.empty()) {
    return NULL;
  }
  if (!isprint(delim)) {
    return NULL;
  }

  std::vector<std::string> tokens = split(reprStr.c_str(), ';');
  if (tokens.size() < 1)
    return NULL;

  Representation *rep = addOrGet(tokens.front());
  tokens.erase(tokens.begin());

  for (int i = 0; i < tokens.size(); ++i) {
    std::string t = tokens.at(i);
    rep->mapping[t] = i;
    rep->subclasses.push_back(addOrGet(t));
  }

  return rep;
}

Representation* RepresentationFactory::addOrGet(std::string name) {
  Representation *res = NULL;

  if (repMap->count(name) > 0) {
    res = repMap->at(name);
  } else {
    res = new Representation;
    res->name = name;
    res->parent = NULL;
    repMap->insert(std::pair<std::string, Representation*>(name, res));
  }

  return res;
}

std::shared_ptr<std::vector<Representation*>> RepresentationFactory::fromCSVStrings(
    std::vector<std::string> lines)
{

  for (std::string line : lines) {
    Representation *r = fromCSV(line);
    repVec->push_back(r);
  }

  return repVec;
}

std::shared_ptr<std::vector<Representation*>> RepresentationFactory::getRepVec() {
  return repVec;
}

std::shared_ptr<std::map<std::string, Representation*>> RepresentationFactory::getRepMap() {
  return repMap;
}

std::shared_ptr<std::map<std::string, BaseRepresentationInstance*>> RepresentationFactory::getInstanceMap() {
  return repInstanceMap;
}

RepresentationInstance *RepresentationFactory::makeInstance(std::string repName)
{
  Representation *rep = addOrGet(repName);
  return this->makeInstance(rep);
}

RepresentationInstance *RepresentationFactory::makeInstance(Representation* representation)
{
  RepresentationInstance *ins = new RepresentationInstance(representation);

  for (Representation *sc : representation->subclasses) {
    RepresentationInstance *subIns = makeInstance(sc);
    ins->subs.push_back(subIns);
  }
  
  return ins;
}

void RepresentationFactory::printReps()
{
  for (Representation* r : *repVec) {
    std::cout << r->name << " = {";
    for (Representation *sc : r->subclasses) {
    	std::cout << sc->name << ", ";
    }
    std::cout << "}" << std::endl;
  }
}

}
