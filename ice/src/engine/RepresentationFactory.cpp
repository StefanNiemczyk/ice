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

namespace ice
{

RepresentationFactory::RepresentationFactory()
{
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("booleanRep", BasicRepresentationType::BOOL));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("byteRep", BasicRepresentationType::BYTE));
  typeMap.insert(
      std::pair<std::string, BasicRepresentationType>("unsignedByteRep", BasicRepresentationType::UNSIGNED_BYTE));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("shortRep", BasicRepresentationType::SHORT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("integerRep", BasicRepresentationType::INT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("longRep", BasicRepresentationType::LONG));
  typeMap.insert(
      std::pair<std::string, BasicRepresentationType>("unsignedShortRep", BasicRepresentationType::UNSIGNED_SHORT));
  typeMap.insert(
      std::pair<std::string, BasicRepresentationType>("unsignedIntegerRep", BasicRepresentationType::UNSIGNED_INT));
  typeMap.insert(
      std::pair<std::string, BasicRepresentationType>("unsignedLongRep", BasicRepresentationType::UNSIGNED_LONG));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("floatRep", BasicRepresentationType::FLOAT));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("doubleRep", BasicRepresentationType::DOUBLE));
  typeMap.insert(std::pair<std::string, BasicRepresentationType>("stringRep", BasicRepresentationType::STRING));
}

RepresentationFactory::~RepresentationFactory()
{

}

std::shared_ptr<Representation> RepresentationFactory::fromCSV(std::string reprStr,
                                                               std::map<std::string, std::shared_ptr<Representation>> *tmpMap,
                                                               const char delim)
{
  if (reprStr.empty())
  {
    return NULL;
  }
  if (!isprint(delim))
  {
    return NULL;
  }

  std::vector<std::string> tokens = split(reprStr.c_str(), ';');
  if (tokens.size() != 3)
    return NULL;

  std::string repStr = tokens.at(0);
  std::string dimStr = tokens.at(1);
  std::string dimRepStr = tokens.at(2);

//  std::cout << repStr << " " << dimStr << " " << dimRepStr << " " << this->getBasicRep(dimRepStr) << std::endl;

  auto rep = addOrGet(repStr, tmpMap);
  rep->type = this->getBasicRep(repStr);

  auto rep2 = addOrGet(dimRepStr, tmpMap);
  rep2->type = this->getBasicRep(dimRepStr);

  rep->dimensionNames.push_back(dimStr);
  rep->dimensions.push_back(rep2);

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

std::shared_ptr<Representation> RepresentationFactory::addOrGet(std::string name, std::map<std::string, std::shared_ptr<Representation>> *tmpMap)
{
  std::shared_ptr<Representation> rep = NULL;

  if (tmpMap->count(name) > 0)
  {
    rep = tmpMap->at(name);
  }
  else
  {
    rep = std::make_shared<Representation>();
    rep->name = name;
    rep->type = BasicRepresentationType::UNSET;
    tmpMap->insert({name, rep});
  }

  return rep;
}

int RepresentationFactory::fromCSVStrings(std::vector<std::string> lines)
{
  std::map<std::string, std::shared_ptr<Representation>> tmpMap;

  for (std::string line : lines)
  {
    if (line == "")
      continue;

    if (line == "|")
    {
      tmpMap.clear();
      continue;
    }

    auto r = fromCSV(line, &tmpMap);

    if (false == r)
    {
      std::cout << "Error: No representation extracted from " << line << std::endl;
      continue;
    }

    this->repMap[r->name] = r;
  }

  return this->repMap.size();
}

std::shared_ptr<Representation> RepresentationFactory::getRepresentation(std::string representation)
{
  auto it = this->repMap.find(representation);

  if (this->repMap.end() == it)
    return nullptr;

  return it->second;
}

std::shared_ptr<RepresentationInstance> RepresentationFactory::makeInstance(std::string repName)
{
  return this->makeInstance(this->getRepresentation(repName));
}

std::shared_ptr<RepresentationInstance> RepresentationFactory::makeInstance(std::shared_ptr<Representation> representation)
{
  if (representation->isBasic())
  {
    std::shared_ptr<BasicRepresentationInstance> ins;
    auto rep = representation;

    switch (rep->type)
    {
      case BOOL:
        ins = std::make_shared<BoolRepresentationInstance>(representation);
        break;
      case BYTE:
        ins = std::make_shared<ByteRepresentationInstance>(representation);
        break;
      case UNSIGNED_BYTE:
        ins = std::make_shared<UnsignedByteRepresentationInstance>(representation);
        break;
      case SHORT:
        ins = std::make_shared<ShortRepresentationInstance>(representation);
        break;
      case INT:
        ins = std::make_shared<IntegerRepresentationInstance>(representation);
        break;
      case LONG:
        ins = std::make_shared<LongRepresentationInstance>(representation);
        break;
      case UNSIGNED_SHORT:
        ins = std::make_shared<UnsignedShortRepresentationInstance>(representation);
        break;
      case UNSIGNED_INT:
        ins = std::make_shared<UnsignedIntegerRepresentationInstance>(representation);
        break;
      case UNSIGNED_LONG:
        ins = std::make_shared<UnsignedLongRepresentationInstance>(representation);
        break;
      case FLOAT:
        ins = std::make_shared<FloatRepresentationInstance>(representation);
        break;
      case DOUBLE:
        ins = std::make_shared<DoubleRepresentationInstance>(representation);
        break;
      case STRING:
        ins = std::make_shared<StringRepresentationInstance>(representation);
        break;
      default:
        std::cout << "Error: Unknown representation basic type " << rep->name << std::endl;
        break;
    }

    return ins;
  }
  else
  {
    std::shared_ptr<CompositeRepresentationInstance> ins = std::make_shared<CompositeRepresentationInstance>(representation);

    for (auto sc : representation->dimensions)
    {
      auto subIns = makeInstance(sc);
      ins->subs.push_back(subIns);
    }

    return ins;
  }
}

void RepresentationFactory::printReps()
{
  for (auto it = this->repMap.begin(); it != this->repMap.end(); it++)
  {
    this->printReps(it->second, 0);
  }
}

void RepresentationFactory::printReps(std::shared_ptr<Representation> representation, int depth)
{
  std::cout << std::string(depth, ' ') << representation->name << std::endl;

  for (int i = 0; i < representation->dimensions.size(); ++i)
  {
    auto rep = representation->dimensions.at(i);
    std::cout << std::string(depth + 1, ' ') << representation->dimensionNames.at(i) << " " << rep->name << " "
        << std::endl;
    if (rep->isBasic() == false)
      this->printReps(rep, depth + 2);
  }

}

}
