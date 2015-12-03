/*
 * RepresentationFactory.cpp
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

//#include <cctype>
//#include <cerrno>
//#include <cstdlib>
//#include <cstring>
#include <iostream>
#include <map>
//#include <stddef.h>

#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/split.h"
#include "ice/representation/Transformation.h"
#include "ice/representation/XMLTransformationReader.h"
#include "ice/ICEngine.h"

#include "easylogging++.h"

namespace ice
{

GContainerFactory::GContainerFactory()
{
  _log = el::Loggers::getLogger("GContainerFactory");

  typeMap.insert( {"booleanRep", BasicRepresentationType::BOOL});
  typeMap.insert( {"byteRep", BasicRepresentationType::BYTE});
  typeMap.insert( {"unsignedByteRep", BasicRepresentationType::UNSIGNED_BYTE});
  typeMap.insert( {"shortRep", BasicRepresentationType::SHORT});
  typeMap.insert( {"integerRep", BasicRepresentationType::INT});
  typeMap.insert( {"longRep", BasicRepresentationType::LONG});
  typeMap.insert( {"unsignedShortRep", BasicRepresentationType::UNSIGNED_SHORT});
  typeMap.insert( {"unsignedIntegerRep", BasicRepresentationType::UNSIGNED_INT});
  typeMap.insert( {"unsignedLongRep", BasicRepresentationType::UNSIGNED_LONG});
  typeMap.insert( {"floatRep", BasicRepresentationType::FLOAT});
  typeMap.insert( {"doubleRep", BasicRepresentationType::DOUBLE});
  typeMap.insert( {"stringRep", BasicRepresentationType::STRING});
}

GContainerFactory::GContainerFactory(std::weak_ptr<ICEngine> engine) :
    engine(engine)
{
  _log = el::Loggers::getLogger("GContainerFactory");
}

GContainerFactory::~GContainerFactory()
{
  //
}

void GContainerFactory::init()
{
  auto e = this->engine.lock();

  if (e)
    this->ontologyInterface = e->getOntologyInterface();

  this->readFromOntology();
}

void GContainerFactory::cleanUp()
{
  this->ontologyInterface.reset();
}

void GContainerFactory::setOntologyInterface(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontologyInterface = ontology;
}

void GContainerFactory::readFromOntology()
{
  std::string iri = "http://www.semanticweb.org/sni/ontologies/2013/7/Ice#";

  auto csv = ontologyInterface->readRepresentations();

  typeMap.insert( {ontologyInterface->toShortIri(iri + "BooleanRep"), BasicRepresentationType::BOOL});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "ByteRep"), BasicRepresentationType::BYTE});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "UnsignedByteRep"), BasicRepresentationType::UNSIGNED_BYTE});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "ShortRep"), BasicRepresentationType::SHORT});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "IntegerRep"), BasicRepresentationType::INT});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "LongRep"), BasicRepresentationType::LONG});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "UnsignedShortRep"), BasicRepresentationType::UNSIGNED_SHORT});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "UnsignedIntegerRep"), BasicRepresentationType::UNSIGNED_INT});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "UnsignedLongRep"), BasicRepresentationType::UNSIGNED_LONG});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "FloatRep"), BasicRepresentationType::FLOAT});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "DoubleRep"), BasicRepresentationType::DOUBLE});
  typeMap.insert( {ontologyInterface->toShortIri(iri + "StringRep"), BasicRepresentationType::STRING});

  this->fromCSVStrings(std::move(csv));
}

std::shared_ptr<Representation> GContainerFactory::fromCSV(
    std::string reprStr, std::map<std::string, std::shared_ptr<Representation>> *tmpMap, const char delim)
{
  if (reprStr.empty())
  {
    return NULL;
  }
  if (!isprint(delim))
  {
    return NULL;
  }

  auto tokens = split(reprStr.c_str(), ';');
  if (tokens->size() != 3)
    return NULL;

  std::string repStr = tokens->at(0);
  std::string dimStr = tokens->at(1);
  std::string dimRepStr = tokens->at(2);

//  std::cout << repStr << " " << dimStr << " " << dimRepStr << " " << this->getBasicRep(dimRepStr) << std::endl;

  auto rep = addOrGet(repStr, tmpMap);
  rep->type = this->getBasicRep(repStr);

  auto rep2 = addOrGet(dimRepStr, tmpMap);
  rep2->type = this->getBasicRep(dimRepStr);

  rep->dimensionNames.push_back(dimStr);
  rep->dimensions.push_back(rep2);

  return rep;
}

BasicRepresentationType GContainerFactory::getBasicRep(std::string rep)
{
  auto pos = this->typeMap.find(rep);

  if (pos == this->typeMap.end())
  {
    return BasicRepresentationType::NONE;
  }

  return pos->second;
}

std::shared_ptr<Representation> GContainerFactory::addOrGet(
    std::string name, std::map<std::string, std::shared_ptr<Representation>> *tmpMap)
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
    tmpMap->insert( {name, rep});
  }

  return rep;
}

int GContainerFactory::fromCSVStrings(std::unique_ptr<std::vector<std::string>> lines)
{
  std::map<std::string, std::shared_ptr<Representation>> tmpMap;

  for (std::string line : *lines)
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
      _log->error("No representation extracted from CSV line '%s'", line);
      continue;
    }

    this->repMap[r->name] = r;
  }

  return this->repMap.size();
}

std::shared_ptr<Representation> GContainerFactory::getRepresentation(std::string representation)
{
  auto it = this->repMap.find(representation);

  if (this->repMap.end() == it)
    return nullptr;

  return it->second;
}

std::shared_ptr<GContainer> GContainerFactory::makeInstance(std::string repName)
{
  return this->makeInstance(this->getRepresentation(repName));
}

std::shared_ptr<GContainer> GContainerFactory::makeInstance(std::shared_ptr<Representation> representation)
{
  auto container = this->makeGContainerInstance(representation);

  if (container == nullptr)
    return std::shared_ptr<GContainer>();

  return std::shared_ptr<GContainer>(container);
}

GContainer* GContainerFactory::makeGContainerInstance(std::shared_ptr<Representation> representation)
{
  if (representation->isBasic())
  {
    BasicGContainer* ins;
    auto rep = representation;

    switch (rep->type)
    {
      case BOOL:
        ins = new BoolGContainer(representation);
        break;
      case BYTE:
        ins = new ByteGContainer(representation);
        break;
      case UNSIGNED_BYTE:
        ins = new UnsignedByteGContainer(representation);
        break;
      case SHORT:
        ins = new ShortGContainer(representation);
        break;
      case INT:
        ins = new IntGContainer(representation);
        break;
      case LONG:
        ins = new LongGContainer(representation);
        break;
      case UNSIGNED_SHORT:
        ins = new UnsignedShortGContainer(representation);
        break;
      case UNSIGNED_INT:
        ins = new UnsignedIntGContainer(representation);
        break;
      case UNSIGNED_LONG:
        ins = new UnsignedLongGContainer(representation);
        break;
      case FLOAT:
        ins = new FloatGContainer(representation);
        break;
      case DOUBLE:
        ins = new DoubleGContainer(representation);
        break;
      case STRING:
        ins = new StringGContainer(representation);
        break;
      default:
        ins = nullptr;
        _log->error("Unknown representation basic type '%v' for representation '%v'", rep->type, rep->name);
        break;
    }

    return ins;
  }
  else
  {
    CompositeGContainer* ins = new CompositeGContainer(representation);

    for (auto sc : representation->dimensions)
    {
      auto subIns = makeGContainerInstance(sc);
      ins->subs.push_back(subIns);
    }

    return ins;
  }
}

std::shared_ptr<Transformation> GContainerFactory::fromXMLDesc(TransDesc* desc)
{
  auto rep = this->getRepresentation(this->ontologyInterface->toShortIri(desc->output));

  if (false == rep)
  {
    _log->error("Unknown target representation '%v' for transformation '%v', transformation can not be created",
                desc->output, desc->name);

    return std::shared_ptr<Transformation>();
  }

  int inputCount = desc->inputs.size();

  if (inputCount <= 0)
  {
    _log->error("Invalid number of inputs '%v' for transformation '%v', transformation can not be created", inputCount,
                desc->name);

    return std::shared_ptr<Transformation>();
  }

  std::string scope = this->ontologyInterface->toShortIri(desc->scope);


  if (scope == "")
  {
    _log->error("Invalid scope '%v' for transformation '%v', transformation can not be created", desc->scope,
                desc->name);

    return std::shared_ptr<Transformation>();
  }

  std::shared_ptr<Transformation> trans = std::make_shared<Transformation>(this->shared_from_this(), desc->name, scope, rep);

  // reading inputs
  std::map<int, std::shared_ptr<Representation>> inputs;

  for (auto input : desc->inputs)
  {
    auto inRep = this->getRepresentation(this->ontologyInterface->toShortIri(input.representation));

    if (false == rep)
    {
      _log->error("Unknown input representation '%v' for transformation '%v', transformation can not be created",
                  input.representation, desc->name);

      return std::shared_ptr<Transformation>();
    }

    if (inputs.find(input.id) != inputs.end())
    {
      _log->error(
          "Duplicated input id for input representation '%v' of transformation '%v', transformation can not be created",
          input.id, input.representation, desc->name);

      return std::shared_ptr<Transformation>();
    }

    inputs[input.id] = inRep;
  }

  vector<int> v;
  for (auto element : inputs)
  {
    v.push_back(element.first);
  }

  std::sort(v.begin(), v.end());

  for (int index : v)
  {
    trans->getInputs().push_back(inputs[index]);
  }

  // reading dimensions
  std::vector<std::string> path;

  bool result = this->extractOperations(trans, rep, desc->ops, path, inputs);

  if (false == result)
  {
    return std::shared_ptr<Transformation>();
  }

  return trans;
}

bool GContainerFactory::extractOperations(std::shared_ptr<Transformation> transformation,
                                          std::shared_ptr<Representation> representation,
                                          std::vector<DimensionDesc> &ops, std::vector<std::string> &path,
                                          std::map<int, std::shared_ptr<Representation>> &reps)
{
  for (auto operation : ops)
  {
    std::shared_ptr<Representation> repDim;
    std::string shortName = this->ontologyInterface->toShortIri(operation.name);

    for (int i=0; i < representation->dimensionNames.size(); ++i)
    {
      if (representation->dimensionNames.at(i) == shortName)
      {
        repDim = representation->dimensions.at(i);

        break;
      }
    }

    if (false == repDim)
    {
      _log->error("Unknown dimension '%v' for transformation '%v', transformation can not be created",
                  operation.name, transformation->getName());

      return false;
    }

    // add to path
    path.push_back(shortName);
    auto pathTarget = transformation->getTargetRepresentation()->accessPath(path);
    bool result = false;

    if (nullptr == pathTarget)
    {
      _log->error("Unknown path to target dimension '%v' for transformation '%v', transformation can not be created",
                  shortName, transformation->getName());

      return false;
    }

    switch (operation.type)
    {
      case (XML_USE):
      {
        auto repSource = reps[operation.sourceId];

        if (false == repSource)
        {
          _log->error("Unknown dimension source '%v' for transformation '%v', transformation can not be created",
                      operation.sourceId, transformation->getName());

          return false;
        }

        auto pathDim = split(operation.path.c_str(), ';');

        for (int i=0; i < pathDim->size(); ++i)
        {
          pathDim->at(i) = this->ontologyInterface->toShortIri(pathDim->at(i));
        }

        auto pathSource = representation->accessPath(pathDim.get());

        if (nullptr == pathSource)
        {
          _log->error(
              "Unknown path '%v' to source dimension for transformation '%v', transformation can not be created",
              operation.path, transformation->getName());

          return false;
        }

        int index = -1;

        for (int i=0; i < transformation->getInputs().size(); ++i)
        {
          if (transformation->getInputs().at(i) == reps[operation.sourceId])
          {
            index = i;
            break;
          }
        }

        if (index < 0)
        {
          _log->error(
              "Unknown input with id '%v' to source dimension for transformation '%v', transformation can not be created",
              operation.sourceId, transformation->getName());

          return false;
        }

        ice::TransformationOperation* o = new ice::TransformationOperation();
        o->sourceIndex = index;
        o->sourceDimension = pathSource;
        o->type = ice::TransformationOperationType::USE;
        o->targetDimension = pathTarget;
        transformation->getOperations().push_back(o);

        break;
      }
      case (XML_DEFAULT):
      {
        auto pathDim = split(operation.path.c_str(), ';');

        for (int i=0; i < pathDim->size(); ++i)
        {
          pathDim->at(i) = this->ontologyInterface->toShortIri(pathDim->at(i));
        }

        void* value = this->convertStringToBasic(repDim->type, operation.value);

        if (nullptr == value)
        {
          _log->error(
              "Value '%v' could not be interpreted as default value for dimension '%v' in transformation '%v', transformation can not be created",
              operation.value, repDim->name, transformation->getName());

          return false;
        }

        ice::TransformationOperation* o = new ice::TransformationOperation();
        o->valueType = repDim->type;
        o->value = value;
        o->type = TransformationOperationType::DEFAULT;
        o->targetDimension = pathTarget;
        transformation->getOperations().push_back(o);
        break;
      }
      case (XML_FORMULA):
      {
        auto repSource = reps[operation.sourceId];

        if (false == repSource)
        {
          _log->error("Unknown dimension source '%v' for transformation '%v', transformation can not be created",
                      operation.sourceId, transformation->getName());

          return false;
        }

        auto pathDim = split(operation.path.c_str(), ';');

        for (int i=0; i < pathDim->size(); ++i)
        {
          pathDim->at(i) = this->ontologyInterface->toShortIri(pathDim->at(i));
        }

        auto pathSource = representation->accessPath(pathDim.get());

        if (nullptr == pathSource)
        {
          _log->error(
              "Unknown path '%v' to source dimension for transformation '%v', transformation can not be created",
              operation.path, transformation->getName());

          return false;
        }

        int index = -1;

        for (int i=0; i < transformation->getInputs().size(); ++i)
        {
          if (transformation->getInputs().at(i) == reps[operation.sourceId])
          {
            index = i;
            break;
          }
        }

        if (index < 0)
        {
          _log->error(
              "Unknown input with id '%v' to source dimension for transformation '%v', transformation can not be created",
              operation.sourceId, transformation->getName());

          return false;
        }
        ice::TransformationOperation* o = new ice::TransformationOperation();
        o->sourceIndex = index;
        o->sourceDimension = pathSource;
        o->valueType = repDim->type;
        o->formula = operation.formula;
        o->varname= operation.varname;
        o->type = TransformationOperationType::FORMULA;
        o->targetDimension = pathTarget;
        transformation->getOperations().push_back(o);
        break;
      }
      case (XML_COMPLEX):
        result = this->extractOperations(transformation, repDim, operation.dims, path, reps);

        if (false == result)
          return false;

        break;
      default:
        _log->error("Unknown operation type '%v' in transformation '%v' from XML", operation.type,
                    transformation->getName());
        return false;
        break;
    }

    // remove from path
    path.pop_back();
  }

  return true;
}

void* GContainerFactory::convertStringToBasic(BasicRepresentationType type, std::string value)
{
  switch (type)
  {
    case BOOL:
      if ("true" == value)
        return new bool(true);
      else
        return new bool(false);
      break;
    case BYTE:
      return new int8_t((int8_t) std::stoi(value));
      break;
    case UNSIGNED_BYTE:
      return new uint8_t((uint8_t) std::stoi(value));
      break;
    case SHORT:
      return new short((short) std::stoi(value));
      break;
    case INT:
      return new int(std::stoi(value));
      break;
    case LONG:
      return new long(std::stol(value));
      break;
    case UNSIGNED_SHORT:
      return new unsigned short((unsigned short) std::stoul(value));
      break;
    case UNSIGNED_INT:
      return new unsigned int((unsigned int) std::stoul(value));
      break;
    case UNSIGNED_LONG:
      return new unsigned long(std::stoul(value));
      break;
    case FLOAT:
      return new float(std::stof(value));
      break;
    case DOUBLE:
      return new double(std::stod(value));
      break;
    case STRING:
      return new std::string(value);
      break;
    default:
      _log->error("Unknown representation basic type '%v' for value '%v'", type, value);
      break;
  }

  return nullptr;
}

void GContainerFactory::printReps()
{
  for (auto it = this->repMap.begin(); it != this->repMap.end(); it++)
  {
    this->printReps(it->second, 0);
  }
}

void GContainerFactory::printReps(std::shared_ptr<Representation> representation, int depth)
{
  std::cout << std::string(depth, ' ') << representation->name << std::endl;

  for (int i = 0; i < representation->dimensions.size(); ++i)
  {
    auto rep = representation->dimensions.at(i);
    std::cout << std::string(depth + 1, ' ') << representation->dimensionNames.at(i) << " " << rep->name << std::endl;
    if (rep->isBasic() == false)
      this->printReps(rep, depth + 2);
  }

}

bool GContainerFactory::addTransformation(std::string name, std::shared_ptr<Transformation> transformation)
{
  auto it = this->transformations.find(name);

  if (this->transformations.end() != it)
    return false;

  this->transformations[name] = transformation;

  return true;
}

std::shared_ptr<Transformation> GContainerFactory::getTransformation(std::string name)
{
  auto it = this->transformations.find(name);

  if (this->transformations.end() == it)
    return std::shared_ptr<Transformation>();

  return it->second;
}

}
