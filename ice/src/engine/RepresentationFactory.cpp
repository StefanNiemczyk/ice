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
  repInstanceMap = std::make_shared<std::map<std::string, RepresentationInstance*>>();

  typeMap.insert(std::pair<std::string, BasicRepresentationType>("booleanRep", BasicRepresentationType::BOOL));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("byteRep", BasicRepresentationType::BYTE));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("unsignedByteRep", BasicRepresentationType::UNSIGNED_BYTE));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("shortRep", BasicRepresentationType::SHORT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("integerRep", BasicRepresentationType::INT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("longRep", BasicRepresentationType::LONG));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("unsignedShortRep", BasicRepresentationType::UNSIGNED_SHORT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("unsignedIntegerRep", BasicRepresentationType::UNSIGNED_INT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("unsignedLongRep", BasicRepresentationType::UNSIGNED_LONG));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("floatRep", BasicRepresentationType::FLOAT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("doubleRep", BasicRepresentationType::DOUBLE));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("stringRep", BasicRepresentationType::STRING));
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
  if (tokens.size() < 2)
    return NULL;

  std::string repStr = tokens.front();
  tokens.erase(tokens.begin());
  Representation *rep = addOrGet(repStr);

  rep->type = this->getBasicRep(tokens.at(0));

  if (rep->isBasic())
    return rep;

  for (int i = 0; i < tokens.size(); ++i) {
    std::string t = tokens.at(i);
    rep->mapping[t] = i;
    rep->subclasses.push_back(addOrGet(t));
  }

  return rep;
}

BasicRepresentationType RepresentationFactory::getBasicRep(std::string rep)
{
  auto pos = this->typeMap.find(rep);

  if (pos == this->typeMap.end())
  {
    return BasicRepresentationType::NONE;
  }

  return pos->second;
}


Representation* RepresentationFactory::addOrGet(std::string name) {
  Representation *rep = NULL;

  if (repMap->count(name) > 0) {
    rep = repMap->at(name);
  }
  else {
    rep = new Representation;
    rep->name = name;
    rep->type = BasicRepresentationType::UNSET;
    repMap->insert(std::pair<std::string, Representation*>(name, rep));
  }

  return rep;
}

std::shared_ptr<std::vector<Representation*>> RepresentationFactory::fromCSVStrings(
    std::vector<std::string> lines)
{

  for (std::string line : lines) {
    if (line == "")
      continue;

    Representation *r = fromCSV(line);

    if (r == nullptr)
    {
      std::cout << "Error: No representation extracted from " << line << std::endl;
      continue;
    }

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

std::shared_ptr<std::map<std::string, RepresentationInstance*>> RepresentationFactory::getInstanceMap() {
  return repInstanceMap;
}

Representation* RepresentationFactory::getRepresentation(std::string representation)
{
  return this->repMap->at(representation);
}

RepresentationInstance *RepresentationFactory::makeInstance(std::string repName)
{
  Representation *rep = addOrGet(repName);
  return this->makeInstance(rep);
}

RepresentationInstance *RepresentationFactory::makeInstance(Representation* representation)
{
  if (representation->isBasic())
  {
    BasicRepresentationInstance* ins = nullptr;

    switch(representation->type)
    {
      case BOOL:
        ins = new BoolRepresentationInstance(representation);
        break;
      case BYTE:
        ins = new ByteRepresentationInstance(representation);
        break;
      case UNSIGNED_BYTE:
        ins = new UnsignedByteRepresentationInstance(representation);
        break;
      case SHORT:
        ins = new ShortRepresentationInstance(representation);
        break;
      case INT:
        ins = new IntegerRepresentationInstance(representation);
        break;
      case LONG:
        ins = new LongRepresentationInstance(representation);
        break;
      case UNSIGNED_SHORT:
        ins = new UnsignedShortRepresentationInstance(representation);
        break;
      case UNSIGNED_INT:
        ins = new UnsignedIntegerRepresentationInstance(representation);
        break;
      case UNSIGNED_LONG:
        ins = new UnsignedLongRepresentationInstance(representation);
        break;
      case FLOAT:
        ins = new FloatRepresentationInstance(representation);
        break;
      case DOUBLE:
        ins = new DoubleRepresentationInstance(representation);
        break;
      case STRING:
        ins = new StringRepresentationInstance(representation);
        break;
      default:
        std::cout << "Error: Unknown representation basic type " << representation->name << std::endl;
        break;
    }

    return ins;
  }
  else
  {
    CompositeRepresentationInstance *ins = new CompositeRepresentationInstance(representation);
  
    for (Representation *sc : representation->subclasses) {
      RepresentationInstance *subIns = makeInstance(sc);
      ins->subs.push_back(subIns);
    }

    return ins;
  }
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
