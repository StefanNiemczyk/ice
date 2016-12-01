/*
 * ASPTransformationGeneration.cpp
 *
 *  Created on: Dec 1, 2015
 *      Author: sni
 */

#include <ice/representation/TransformationSynthese.h>
#include <ros/package.h>

#include "ice/ontology/OntologyInterface.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"
#include "ice/ICEngine.h"

namespace ice
{

TransformationSynthese::TransformationSynthese()
{
  _log = el::Loggers::getLogger("ASPTransformationGeneration");
}

TransformationSynthese::TransformationSynthese(std::weak_ptr<ICEngine> engine) :
    engine(engine)
{
  _log = el::Loggers::getLogger("ASPTransformationGeneration");
}

TransformationSynthese::~TransformationSynthese()
{
  //
}

void TransformationSynthese::init()
{
  if (this->engine.expired())
    return;
  auto e = this->engine.lock();

  this->ontology = e->getOntologyInterface();
  this->containerFactory = e->getGContainerFactory();
}

void TransformationSynthese::cleanUp()
{
  this->ontology.reset();
  this->containerFactory.reset();
}

void TransformationSynthese::readInfoStructureFromOntology()
{
  _log->verbose(1, "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  std::stringstream ss;
  ss.str(this->ontology->readInformationStructureAsASP());

  _log->debug("Extracted structure from ontology");
  _log->verbose(1, ss.str());

  std::string item;

  while (std::getline(ss, item, '\n'))
  {
    if (std::find(this->entities.begin(), this->entities.end(), item) == this->entities.end())
    {
      this->entities.push_back(item);
    }
  }
}

void TransformationSynthese::extractTransformations()
{
  _log->verbose(1, "Extract transformations");

  std::string path = ros::package::getPath("ice") + "/asp/transformation/";

  // Initializing ASP
  supplementary::ClingWrapper asp;
  asp.addKnowledgeFile(path + "computing.lp");

  asp.setNoWarnings(true);
  asp.init();

  // Grounding ASP programm
  std::string programPart = "base";

  this->readInfoStructureFromOntology();

  for (auto entity : this->entities)
  {
    asp.add(programPart, {}, this->ontology->toShortIriAll(entity));
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
  // autoTrans(SCOPE,REP1,REP2)
  std::vector<Gringo::Value> values;
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value nodeQuery("autoTrans", values);
  auto queryResult = asp.queryAllTrue(&nodeQuery);

  for (auto trans : *queryResult)
  {
    auto scope = this->ontology->toLongIri(*trans.args()[0].name());
    auto representation1 = this->ontology->toLongIri(*trans.args()[1].name());
    auto representation2 = this->ontology->toLongIri(*trans.args()[2].name());

    _log->debug("Process transformation for '%v' (scope), '%v' (represetnation1), '%v' (representation2)", scope,
                representation1, representation2);

    auto rep1 = this->containerFactory->getRepresentation(representation1);
    auto rep2 = this->containerFactory->getRepresentation(representation2);

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

    std::string name = "autoTrans_" + scope + "_" + representation1 + "_" + representation2;

    auto t = this->containerFactory->getTransformationByName(name);

    if (t)
    {
      _log->debug("Transformation '%v' already exists and will be skipped", name);
      continue;
    }

    auto transformation = std::make_shared<Transformation>(this->engine, name, scope, rep2);

    // set input representation
    transformation->getInputs().push_back(rep1);

    values.clear();
    values.push_back(trans.args()[0]);
    values.push_back(trans.args()[1]);
    values.push_back(trans.args()[2]);
    Gringo::Value simRep("simRep", values);

    std::vector<std::string> path;

    bool resultExtraction = this->extractOperations(asp, transformation, simRep, rep1, rep2, path);

    if (false == resultExtraction)
      continue;

    _log->info("Created transformation '%v' from ontology", name);
    this->containerFactory->addTransformation(name, transformation, true);
  }
}

void TransformationSynthese::extractDeviation(supplementary::ClingWrapper &asp,
                                                   std::vector<std::string> &deviations, Gringo::Value &simRep,
                                                   std::string type)
{
  std::vector<Gringo::Value> values;
  values.push_back(simRep);
  values.push_back("?");
  values.push_back("?");
  values.push_back(Gringo::Value(type));

  Gringo::Value simple("fix", values);
  auto resultSimple = asp.queryAllTrue(&simple);

  for (auto fix : *resultSimple)
  {
    deviations.push_back(*fix.args()[1].name());
  }
}

bool TransformationSynthese::extractOperations(supplementary::ClingWrapper &asp,
                                                 std::shared_ptr<Transformation> transformation, Gringo::Value simRep,
                                                 std::shared_ptr<Representation> rep1,
                                                 std::shared_ptr<Representation> rep2, std::vector<std::string> &path)
{
  std::vector<Gringo::Value> values;

  // extract matches
  // match(simRep(SCOPE,REP1,REP2),DIM,REP)
//  values.push_back(simRep);
//  values.push_back("?");
//  values.push_back("?");
//
//  Gringo::Value matchQuery("match", values);
//  auto matchQueryResult = asp.queryAllTrue(&matchQuery);
//
//  for (auto match : *matchQueryResult)
//  {
//    auto matchingScope = this->ontology->toLongIri(*match.args()[1].name());
//
//    path.push_back(matchingScope);
//    auto pathSource = rep1->accessPath(path);
//    path.pop_back();
//
//    if (nullptr == pathSource)
//    {
//      _log->error(
//          "Unknown path '%v' to matching source dimension for transformation '%v', transformation can not be created",
//          matchingScope, transformation->getName());
//      return false;
//    }
//
//    path.push_back(matchingScope);
//    auto pathTarget = rep2->accessPath(path);
//    path.pop_back();
//
//    if (nullptr == pathTarget)
//    {
//      _log->error(
//          "Unknown path '%v' to matching target dimension for transformation '%v', transformation can not be created",
//          matchingScope, transformation->getName());
//      return false;
//    }
//
//    ice::TransformationOperation* o = new ice::TransformationOperation();
//    o->sourceIndex = 0;
//    o->sourceDimension = pathSource;
//    o->type = ice::TransformationOperationType::USE;
//    o->targetDimension = pathTarget;
//    transformation->getOperations().push_back(o);
//  }

  std::vector<std::string> diviations;

  // extract default
  diviations.clear();
  this->extractDeviation(asp, diviations, simRep, "default");

  for (auto div : diviations)
  {
    values.clear();

    values.push_back(Gringo::Value(div));

    auto result = asp.queryAllTrue("valueScopeDefault", values);

    if (result->size() != 1)
    {
      _log->error(
          "Wrong size '%v' of default values for dimension '%v' of transformation '%v', transformation can not be created",
          result->size(), div, transformation->getName());
      return false;
    }

    auto defaultValue = result->at(0);
    path.push_back(this->ontology->toLongIri(div));
    auto pathTarget = rep2->accessPath(path);
    path.pop_back();

    if (nullptr == pathTarget)
    {
      _log->error(
          "Unknown path '%v' to matching target dimension for transformation '%v', transformation can not be created",
          div, transformation->getName());
      return false;
    }

    BasicRepresentationType type = rep2->dimensions.at(pathTarget->at(0))->type;

    if (type == BasicRepresentationType::NONE || type == BasicRepresentationType::UNSET)
    {
      _log->error(
          "Bad basic representation type '%v' for dimension '%v' of transformation '%v', transformation can not be created",
          type, div, transformation->getName());
      return false;
    }

    void* value = this->containerFactory->convertStringToBasic(type, *defaultValue.args()[1].name());

    if (value == nullptr)
    {
      _log->error("Bad default value '%v' for dimension '%v' of transformation '%v', transformation can not be created",
                  *defaultValue.args()[1].name(), div, transformation->getName());
      return false;
    }

    ice::TransformationOperation* o = new ice::TransformationOperation();
    o->type = ice::TransformationOperationType::DEFAULT;
    o->value = value;
    o->targetDimension = pathTarget;
    transformation->getOperations().push_back(o);
  }

  // TODO extract using Transformations


  // useAutoTrans(simRep(SCOPE,REP1,REP2),SCOPE,REP_SOURCE,REP_TARGET)
  values.clear();
  values.push_back(simRep);
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value autoTransQuery("useAutoTrans", values);
  auto autoTransResult = asp.queryAllTrue(&autoTransQuery);

  for (auto autoTrans : *autoTransResult)
  {
    std::string scope = *autoTrans.args()[1].name();
    std::string r1Str = *autoTrans.args()[2].name();
    std::string r2Str = *autoTrans.args()[3].name();

    _log->debug("Process used autoTrans for '%v' (scope), '%v' (represetnation1), '%v' (representation2)", scope, r1Str,
                r2Str);

    values.clear();
    values.push_back(Gringo::Value(scope));
    values.push_back(Gringo::Value(r1Str));
    values.push_back(Gringo::Value(r2Str));
    Gringo::Value simRep("simRep", values);

    path.push_back(this->ontology->toLongIri(scope));
    bool resultExtract = this->extractOperations(asp, transformation, simRep, rep1, rep2, path);
    path.pop_back();

    if (false == resultExtract)
      return false;
  }

  // use can only be applied to first level dimensions
  if (path.size() > 0)
    return true;

  // compute use operation
  for (auto dimension : rep2->dimensionNames)
  {
    bool found = false;
    auto &ops = transformation->getOperations();

    path.push_back(dimension);
    auto pathTarget = rep2->accessPath(path);
    path.pop_back();

    if (nullptr == pathTarget)
    {
      _log->error(
          "Unknown path '%v' to matching target dimension for transformation '%v', transformation can not be created",
          dimension, transformation->getName());
      return false;
    }

    for (auto op : ops)
    {
      if (op->targetDimension[0] == pathTarget[0])
      {
        found = true;
        break;
      }
    }

    if (found)
      continue;

    path.push_back(dimension);
    auto pathSource = rep1->accessPath(path);
    path.pop_back();

    if (nullptr == pathSource)
    {
      _log->error(
          "Unknown path '%v' to matching source dimension for transformation '%v', transformation can not be created",
          dimension, transformation->getName());
      return false;
    }

    ice::TransformationOperation* o = new ice::TransformationOperation();
    o->sourceIndex = 0;
    o->sourceDimension = pathSource;
    o->type = ice::TransformationOperationType::USE;
    o->targetDimension = pathTarget;
    ops.push_back(o);
  }

  return true;
}

void TransformationSynthese::setOntology(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontology = ontology;
}

void TransformationSynthese::setGContainerFactory(std::shared_ptr<GContainerFactory> factory)
{
  this->containerFactory = factory;
}

} /* namespace ice */
