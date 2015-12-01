/*
 * ASPTransformationGeneration.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: sni
 */

#include "ice/representation/ASPTransformationGeneration.h"

#include <ros/package.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/ICEngine.h"

namespace ice
{

ASPTransformationGeneration::ASPTransformationGeneration() :
    groundingDirty(true)
{
  _log = el::Loggers::getLogger("ASPTransformationGeneration");
  _log->verbose(1, "Constructor called");
}

ASPTransformationGeneration::ASPTransformationGeneration(std::weak_ptr<ICEngine> engine) :
    engine(engine), groundingDirty(true)
{
  _log = el::Loggers::getLogger("ASPTransformationGeneration");
  _log->verbose(1, "Constructor called");
}

ASPTransformationGeneration::~ASPTransformationGeneration()
{
  //
}

void ASPTransformationGeneration::init()
{
  if (this->engine.expired())
    return;
  auto e = this->engine.lock();

  this->ontology = e->getOntologyInterface();
}

void ASPTransformationGeneration::cleanUp()
{
  this->ontology.reset();
}

void ASPTransformationGeneration::readInfoStructureFromOntology()
{
  _log->verbose(1, "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  const char* infoStructure = this->ontology->readInformationStructureAsASP();

  _log->debug("Extracted structure from ontology");
  _log->verbose(1, infoStructure);

  std::stringstream ss;
  std::string item;

  ss << infoStructure;
  //  delete infoStructure;

  while (std::getline(ss, item, '\n'))
  {
    if (std::find(this->entities.begin(), this->entities.end(), item) == this->entities.end())
    {
      this->entities.push_back(item);
    }
  }

  this->groundingDirty = true;
}

void ASPTransformationGeneration::extractTransformations()
{
  _log->verbose(1, "Extract transformations");

  if (this->groundingDirty == false)
  {
    // nothing to do here
    return;
  }

  std::string path = ros::package::getPath("ice") + "/asp/transformation/";

  // Initializing ASP
  supplementary::ClingWrapper asp;
  asp.addKnowledgeFile(path + "computing.lp");

  asp.setNoWarnings(true);
  asp.init();

  // Grounding ASP programm
  std::string programPart = "base";

  for (auto entity : this->entities)
  {
    asp.add(programPart, {}, entity);
  }
  asp.ground(programPart, {});

  // Solving asp programm
  auto solveResult = asp.solve();
  _log->info("Solving finished: %v", (Gringo::SolveResult::SAT == solveResult) ? "SAT" : "UNSAT");

  if (solveResult != Gringo::SolveResult::SAT)
  {
    _log->error("No model created by ASP solver, transformation extraction failed");
    return;
  }

  _log->debug("Resulting ASP Model %v", asp.toStringLastModel());
  std::shared_ptr<ProcessingModel> model = std::make_shared<ProcessingModel>();

  // query computed auto transformations
  // autoIRO(SCOPE,REP1,REP2)
  std::vector<Gringo::Value> values;
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value nodeQuery("autoIRO", values);
  auto queryResult = asp.queryAllTrue(&nodeQuery);

  for (auto trans : *queryResult)
  {
    auto scope = *trans.args()[0].name();
    auto representation1 = *trans.args()[1].name();
    auto representation2 = *trans.args()[2].name();

    _log->debug("Process transformation for '%v' (scope), '%v' (represetnation1), '%v' (representation2)", scope,
                representation1, representation2);

    auto rep1 = this->containerFactory->getRepresentation(representation1);
    auto rep2 = this->containerFactory->getRepresentation(representation1);

    if (false == rep1)
    {
      _log->error("Unknown source representation '%v' for auto transformation, transformation will be skipped",
                  representation1);
      continue;
    }

    if (false == rep2)
    {
      _log->error("Unknown target representation '%v' for auto transformation, transformation will be skipped",
                  representation2);
      continue;
    }

    std::string name = "autoTrans_" + scope + "-" + representation1 + "-" + representation2;

    auto t = this->containerFactory->getTransformation(name);

    if (t)
    {
      _log->debug("Transformation '%v' already exists and will be skipped", name);
      continue;
    }

    std::shared_ptr<Transformation> transformation = std::make_shared<Transformation>(this->containerFactory, name,
                                                                                      rep2);

    // set input representation
    transformation->getInputs().push_back(rep1);

    bool error = false;
    values.clear();
    values.push_back(Gringo::Value(scope));
    values.push_back(Gringo::Value(representation1));
    values.push_back(Gringo::Value(representation2));
    Gringo::Value simRep("simRep", values);

    // extract matches
    // match(simRep(SCOPE,REP1,REP2),SCOPE,REP)
    values.clear();
    values.push_back(simRep);
    values.push_back("?");
//    values.push_back("?");
    // TODO

    Gringo::Value matchQuery("match", values);
    auto matchQueryResult = asp.queryAllTrue(&matchQuery);

    for (auto match : *matchQueryResult)
    {
      auto matchingScope = *match.args()[1].name();
//      auto matchingRep = *match.args()[2].name();

      auto pathSource = rep1->accessPath( {matchingScope});

      if (nullptr == pathSource)
      {
        _log->error(
            "Unknown path '%v' to matching source dimension for transformation '%v', transformation can not be created",
            matchingScope, transformation->getName());
        error = true;

        break;
      }

      auto pathTarget = rep2->accessPath( {matchingScope});

      if (nullptr == pathTarget)
      {
        _log->error(
            "Unknown path '%v' to matching target dimension for transformation '%v', transformation can not be created",
            matchingScope, transformation->getName());
        error = true;

        break;
      }

      ice::TransformationOperation* o = new ice::TransformationOperation();
      o->sourceIndex = 0;
      o->sourceDimension = pathSource;
      o->type = ice::TransformationOperationType::USE;
      o->targetDimension = pathTarget;
      transformation->getOperations().push_back(o);
    }

    if (error)
      continue;

    std::vector<std::tuple<std::string, std::string>> diviations;

    // extract default
    diviations.clear();
    this->extractDeviation(asp, diviations, simRep, "default");

    for (auto div : diviations)
    {
      values.clear();
      std::string dim = std::get<0>(div);
      std::string rep = std::get<1>(div);
      int index = 1;

      values.push_back(Gringo::Value(dim));

      if (rep != "")
      {
        // complex
        // valueScopeDefault(SCOPE,REP,VALUE)
        index = 2;
        values.push_back(Gringo::Value(rep));
      }

      auto result = asp.queryAllTrue("valueScopeDefault", values);

      if (result->size() != 1)
      {
        _log->error(
            "Wrong size '%v' of default values for dimension '%v' of transformation '%v', transformation can not be created",
            result->size(), dim, transformation->getName());
        error = true;

        break;
      }

      auto defaultValue = result->at(0);
      auto pathTarget = rep2->accessPath({dim});

      if (nullptr == pathTarget)
      {
        _log->error(
            "Unknown path '%v' to matching target dimension for transformation '%v', transformation can not be created",
            dim, transformation->getName());
        error = true;

        break;
      }

      BasicRepresentationType type = rep2->dimensions.at(pathTarget->at(0))->type;

      if (type == BasicRepresentationType::NONE || type == BasicRepresentationType::UNSET)
      {
        _log->error(
            "Bad basic representation type '%v' for dimension '%v' of transformation '%v', transformation can not be created",
            type, dim, transformation->getName());
        error = true;

        break;
      }

      void* value = this->containerFactory->convertStringToBasic(type, *defaultValue.args()[1].name());

      if (value == nullptr)
      {
        _log->error(
            "Bad default value '%v' for dimension '%v' of transformation '%v', transformation can not be created",
            *defaultValue.args()[1].name(), dim, transformation->getName());
        error = true;

        break;
      }

      ice::TransformationOperation* o = new ice::TransformationOperation();
      o->type = ice::TransformationOperationType::DEFAULT;
      o->value = value;
      o->targetDimension = pathTarget;
      transformation->getOperations().push_back(o);
    }

    if (error)
      continue;

    // extract remove
    // nothing to do here, however the code was already written :D
//    diviations.clear();
//    this->extractDeviation(asp, diviations, simRep, "remove");
//
//    for (auto div : diviations)
//    {
//      //
//    }

    // TODO extract using IROs

    // TODO extract using auto IRO

    _log->info("Created transformation '%v' from ontology", name);
    this->containerFactory->addTransformation(name, transformation);
  }
}

void ASPTransformationGeneration::extractDeviation(supplementary::ClingWrapper &asp,
                                                   std::vector<std::tuple<std::string, std::string>> &deviations,
                                                   Gringo::Value &simRep, std::string type)
{
  // simple
  std::vector<Gringo::Value> values;
  values.push_back(simRep);
  values.push_back("?");
  values.push_back(Gringo::Value(type));

  Gringo::Value simple("fix", values);
  auto resultSimple = asp.queryAllTrue(&simple);

  for (auto fix : *resultSimple)
  {
    auto tup = std::make_tuple(*fix.args()[1].name(), "");
    deviations.push_back(tup);
  }

  // complex
  values.clear();
  values.push_back(simRep);
  values.push_back("?");
  values.push_back("?");
  values.push_back(Gringo::Value(type));

  Gringo::Value complex("fix", values);
  auto resultComplex = asp.queryAllTrue(&complex);

  for (auto fix : *resultComplex)
  {
    auto tup = std::make_tuple(*fix.args()[1].name(), *fix.args()[2].name());
    deviations.push_back(tup);
  }
}

void ASPTransformationGeneration::setOntology(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontology = ontology;
}

void ASPTransformationGeneration::setGContainerFactory(std::shared_ptr<GContainerFactory> factory)
{
  this->containerFactory = factory;
}

} /* namespace ice */
