/*
 * ASPCoordinator.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: sni
 */

#include "ice/model/aspModel/ASPModelGenerator.h"

#include <sstream>

#include "ice/ICEngine.h"
#include "ice/Entity.h"
#include "ice/EntityDirectory.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/StreamStore.h"
#include "ice/ontology/OntologyInterface.h"
#include "ice/processing/NodeStore.h"
#include "ice/representation/GContainerFactory.h"
#include "ice/representation/Transformation.h"

using namespace std;

namespace ice
{
std::mutex ASPModelGenerator::mtxModelGen_;

ASPModelGenerator::ASPModelGenerator(std::weak_ptr<ICEngine> engine) :
    ProcessingModelGenerator(engine)
{
  _log = el::Loggers::getLogger("ASPModelGenerator");

  this->queryIndex = 0;
  this->groundingDirty = true;
  this->subModelIndex = 0;
}

ASPModelGenerator::~ASPModelGenerator()
{
  // nothing to do here
}

void ASPModelGenerator::initInternal()
{
  std::string path = ros::package::getPath("ice") + "/asp/informationProcessing/";
  _log->debug("Default ASP path %v", path);

  std::lock_guard<std::mutex> guard(mtxModelGen_);
  auto en = this->engine.lock();
  this->gcontainerFactory = en->getGContainerFactory();

  // Initializing ASP
  this->asp = std::make_shared<supplementary::ClingWrapper>();
  this->asp->addKnowledgeFile(path + "processing.lp");
  this->asp->addKnowledgeFile(path + "searchBottomUp.lp");

  if (this->config.globalOptimization)
    this->asp->addKnowledgeFile(path + "globalOptimization.lp");
  else
    this->asp->addKnowledgeFile(path + "localOptimization.lp");

  this->asp->setNoWarnings(true);
  this->asp->init();

  // init self
  this->directory = en->getEntityDirector();
  this->self = en->getEntityDirector()->self;

  if (this->self->getExternal() == nullptr)
  {
    std::string iri;
    this->self->getId(EntityDirectory::ID_ONTOLOGY, iri);
    iri = this->ontology->toShortIri(iri);
    auto ext = this->asp->getExternal("system", {Gringo::Value(iri)}, "system", {Gringo::Value(iri)}, true);
    this->self->setExternal(ext);
  }
}

void ASPModelGenerator::cleanUpInternal()
{
  //
}

void ASPModelGenerator::readOntology()
{
  if (this->ontology->isLoadDirty())
  {
    _log->debug("Load flagged dirty, reload ontologies");
    this->ontology->loadOntologies();
  }

  this->readInfoStructureFromOntology();

  if (this->ontology->isSystemDirty())
  {
    _log->debug("System flagged dirty, reload systems");
    this->readSystemsFromOntology();
  }
}

std::shared_ptr<ProcessingModel> ASPModelGenerator::createProcessingModel()
{
  _log->verbose(1, "Start model creation");

  std::lock_guard<std::mutex> guard(mtxModelGen_);

  this->ontology->attachCurrentThread();

  this->readOntology();

  if (this->groundingDirty)
  {
    this->groundingDirty = false;
    _log->debug("Grounding flagged dirty, grounding asp program");

    if (this->lastQuery != nullptr)
    {
      this->lastQuery->assign(false);
      this->lastQuery->release();
    }

    this->lastQuery = this->asp->getExternal("query", {this->queryIndex}, "query",
                                             {this->queryIndex, this->config.maxHopCount,
                                              this->config.maxChainLength}, true);
  }

  // Transformations
  this->readTransformations();

  for (auto &trans : this->transformations)
  {
    if (trans.node->autoTransformation && this->config.useAutoTransformation)
    {
      trans.asp->external->assign(true);
    }
    else if (false == trans.node->autoTransformation && this->config.useXMLTransformation)
    {
      trans.asp->external->assign(true);
    }
    else
    {
      trans.asp->external->assign(false);
    }
  }

  // activate and deactivate systems
  std::vector<std::shared_ptr<Entity>> used;
  auto all = this->directory->allEntities();
  for (auto &entity : *all)
  {
    if (entity->isNodesExtracted() == false && entity->isCooperationPossible())
    {
      std::string ontSystem;
      entity->getId(EntityDirectory::ID_ONTOLOGY, ontSystem);
      auto nodes = this->ontology->readNodesAsASP(ontSystem);
      this->toASP(nodes, entity, this->ontology->toShortIri(ontSystem));
      entity->setNodesExtracted(true);
    }

    if (this->self != entity && entity->updateExternals(false))
    {
      used.push_back(entity);
    }
  }
  this->self->updateExternals(true);

  // Solving
  _log->debug("Start solving");
  auto solveResult = this->asp->solve();
  _log->info("Solving finished: %v", (Gringo::SolveResult::SAT == solveResult) ? "SAT" : "UNSAT");

  if (solveResult != Gringo::SolveResult::SAT)
  {
    _log->error("Optimizing failed, no processing was created");
    return nullptr;
  }

  _log->debug("Resulting ASP Model %v", this->asp->toStringLastModel());
  auto model = std::make_shared<ProcessingModel>();

  // Extract nodes that needs to be activated within own system
  if (false == this->extractNodes(model->getNodes(), this->self, true))
  {
    _log->error("Optimizing failed, error by extracting own processing model");
    return nullptr;
  }

  // Request information from other systems
  for (auto &entity : used)
  {
    _log->debug("Check if sub model exists for '%v'", entity->toString());

    std::shared_ptr<SubModel> subModel = std::make_shared<SubModel>();
    subModel->entity = entity;

    if (false == this->extractedSubModel(entity, subModel))
    {
      _log->error("Optimizing failed, error by extracting sub model for system '%v'", entity->toString());
      return nullptr;
    }

    if (subModel->model != nullptr)
    {
      model->getSubModels().push_back(subModel);
    }

    auto send = std::make_shared<StreamTransfer>();
    send->entity = entity;

    if (false == this->extractStreamTransfers(this->self, entity, send->transfer))
    {
      _log->error("Optimizing failed, error by extracting streams transfers from '%v' to '%v'", this->self->toString(),
                  entity->toString());
      return nullptr;
    }

    if (send->transfer.size() > 0)
      model->getSend().push_back(send);

    auto receive = std::make_shared<StreamTransfer>();
    receive->entity = entity;

    if (false == this->extractStreamTransfers(entity, this->self, receive->transfer))
    {
      _log->error("Optimizing failed, error by extracting streams transfers from '%v' to '%v'", entity->toString(),
                  this->self->toString());
      return nullptr;
    }

    if (receive->transfer.size() > 0)
      model->getReceive().push_back(receive);
  }

  _log->info("Model successfully created");

  return model;
}

bool ASPModelGenerator::extractedSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModel> &subModel)
{
  _log->debug("Look up ASP elements for system '%v'", entity->toString());
  auto model = std::make_shared<SubModelDesc>();
  model->index = this->subModelIndex;

  // extract nodes
  if (false == this->extractNodes(model->nodes, entity, false))
  {
    _log->error("Sub model extraction for system '%v' failed", entity->toString());
    subModel->model = nullptr;

    return false;
  }

  // identify streams send from self -> system
  if (false == this->extractStreamTransfers(entity, this->self, model->send))
  {
    _log->error("Sub model extraction for system '%v' failed", entity->toString());
    subModel->model = nullptr;

    return false;
  }

  // identify streams send from system -> self
  if (false == this->extractStreamTransfers(this->self, entity, model->receive))
  {
    _log->error("Sub model extraction for system '%v' failed", entity->toString());
    subModel->model = nullptr;

    return false;
  }

  if (model->nodes.size() == 0 && model->send.size() == 0 && model->receive.size() == 0)
  {
    _log->info("No Sub model extraction for system '%v'", entity->toString());
    subModel->model = nullptr;
  }
  else
  {
    _log->info("Sub model extraction for system '%v' successfully completed", entity->toString());
    // create sub model description
    subModel->model = model;
  }

  return true;
}

bool ASPModelGenerator::extractNodes(vector<NodeDesc> &nodes, std::shared_ptr<Entity> &entity, bool own)
{
  bool valid = true;
  std::string shortIri;
  entity->getId(EntityDirectory::ID_ONTOLOGY, shortIri);
  shortIri = this->ontology->toShortIri(shortIri);

  // node(QUERY_INDEX, SYSTEM, NODE, ENTITY, ENTITY2)
  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(shortIri));
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value nodeQuery("node", values);
  auto queryResult = this->asp->queryAllTrue(&nodeQuery);

  for (auto &nodeValue : *queryResult)
  {
    std::string nodeName = *nodeValue.args()[2].name();
    std::string nodeEntity = *nodeValue.args()[3].name();
    std::string nodeEntity2 = *nodeValue.args()[4].name();

    _log->debug("Look up node '%v' to process entity '%v'", nodeName, nodeEntity);

    auto aspNode = entity->getASPElementByName(nodeName);

    if (aspNode == nullptr && entity == this->self)
    {
      for (auto &transNode : this->transformations)
      {
        if (transNode.asp->name == nodeName)
        {
          aspNode = transNode.asp;
          break;
        }
      }
    }

    if (aspNode == nullptr)
    {
      _log->error("No node '%v' found, asp system description is invalid!", nodeName);
      valid = false;
      break;
    }

    NodeDesc nodeDesc;

    // connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP))
    std::vector<Gringo::Value> nodeValues;
    nodeValues.push_back(this->queryIndex);
    nodeValues.push_back(Gringo::Value(shortIri));
    nodeValues.push_back(Gringo::Value(aspNode->name));
    nodeValues.push_back(Gringo::Value(nodeEntity));
    nodeValues.push_back(Gringo::Value(nodeEntity2));

    std::vector<Gringo::Value> streamValues;
    streamValues.push_back(this->queryIndex);
    streamValues.push_back(Gringo::Value(shortIri));
    streamValues.push_back("?");
    streamValues.push_back("?");
    streamValues.push_back("?");

    std::vector<Gringo::Value> values;
    values.push_back(Gringo::Value("node", nodeValues));
    values.push_back(Gringo::Value("stream", streamValues));

    Gringo::Value connectQuery("connectToNode", values);
    auto connectResult = this->asp->queryAllTrue(&connectQuery);
    nodeDesc.inputs.resize(connectResult->size());

    for (int i=0; i < connectResult->size(); ++i)
    {
      _log->debug("Look up input stream for node '%v'", nodeName);
      auto &connect = connectResult->at(i);
      InputStreamDesc &input = nodeDesc.inputs.at(i);

      // get stream connected to node
      auto &streamValue = connect.args()[1];

      auto &node = streamValue.args()[2];
      auto &info = streamValue.args()[3];

      input.sourceSystem = this->ontology->toLongIri(*node.args()[1].name());
      input.nodeName = this->ontology->toLongIri(*node.args()[2].name());
      input.nodeEntity = this->ontology->toLongIri(*node.args()[3].name());
      input.nodeEntityRelated = *node.args()[4].name() == "none" ? "" : this->ontology->toLongIri(*node.args()[4].name());

      input.entity = this->ontology->toLongIri(*info.args()[0].name());
      input.scope = this->ontology->toLongIri(*info.args()[1].name());
      input.representation = this->ontology->toLongIri(*info.args()[2].name());
      input.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(input.metadata, streamValue);
    }

    // stream(k,SYSTEM,node(k,SOURCE,NODE,ENTITY,ENTITY2),INFO,STEP)
    values.clear();
    values.push_back(this->queryIndex);
    values.push_back(std::string(shortIri));
    values.push_back(Gringo::Value("node", nodeValues));
    values.push_back("?");
    values.push_back("?");

    Gringo::Value streamQuery("stream", values);
    auto streamResult = this->asp->queryAllTrue(&streamQuery);
    nodeDesc.outputs.resize(streamResult->size());

    for (int i=0; i < streamResult->size(); ++i)
    {
      _log->debug("Look up output stream for node '%v'", nodeName);
      auto &stream = streamResult->at(i);
      OutputStreamDesc &output = nodeDesc.outputs.at(i);

      auto &info = stream.args()[3];

      output.entity = this->ontology->toLongIri(*info.args()[0].name());
      output.scope = this->ontology->toLongIri(*info.args()[1].name());
      output.representation = this->ontology->toLongIri(*info.args()[2].name());
      output.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(output.metadata, stream);
    }

    nodeDesc.type = aspNode->getNodeType();
    nodeDesc.className = aspNode->className;
    nodeDesc.aspName = this->ontology->toLongIri(aspNode->name);
    nodeDesc.entity = (nodeEntity == "none" ? "" : this->ontology->toLongIri(nodeEntity));
    nodeDesc.relatedEntity = (nodeEntity2 == "none" ? "" : this->ontology->toLongIri(nodeEntity2));
    nodeDesc.config = aspNode->configAsString;

    if (nodeDesc.aspName == "")
      nodeDesc.aspName = aspNode->name;

    nodes.push_back(nodeDesc);

    if (false == valid)
      break;
  }

  // set node
  // setNode(QUERY_INDEX, SYSTEM, NODE, ENTITY_TYPE, ENTITY2)
  values.clear();
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(shortIri));
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value setNodeQuery("setNode", values);
  queryResult = this->asp->queryAllTrue(&setNodeQuery);

  for (auto &nodeValue : *queryResult)
  {
    std::string nodeName = *nodeValue.args()[2].name();
    std::string nodeEntityType = *nodeValue.args()[3].name();
    std::string nodeEntity2 = *nodeValue.args()[4].name();

    _log->debug("Look up set node '%v' to process entity type '%v'", nodeName, nodeEntityType);

    auto aspNode = entity->getASPElementByName(nodeName);

    if (aspNode == nullptr && entity == this->self)
    {
      for (auto &transNode : this->transformations)
      {
        if (transNode.asp->name == nodeName)
        {
          aspNode = transNode.asp;
          break;
        }
      }
    }

    if (aspNode == nullptr)
    {
      _log->error("No set node '%v' found, asp system description is invalid!", nodeName);
      valid = false;
      break;
    }

    NodeDesc nodeDesc;

    // streams

    // connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2), stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP))
    std::vector<Gringo::Value> nodeValues;
    nodeValues.push_back(this->queryIndex);
    nodeValues.push_back(Gringo::Value(shortIri));
    nodeValues.push_back(Gringo::Value(aspNode->name));
    nodeValues.push_back(Gringo::Value(nodeEntityType));
    nodeValues.push_back(Gringo::Value(nodeEntity2));

    std::vector<Gringo::Value> streamValues;
    streamValues.push_back(this->queryIndex);
    streamValues.push_back(Gringo::Value(shortIri));
    streamValues.push_back("?");
    streamValues.push_back("?");
    streamValues.push_back("?");

    std::vector<Gringo::Value> values;
    values.push_back(Gringo::Value("setNode", nodeValues));
    values.push_back(Gringo::Value("stream", streamValues));

    Gringo::Value connectQuery("connectToSet", values);
    auto connectResult = this->asp->queryAllTrue(&connectQuery);
    nodeDesc.inputs.resize(connectResult->size());

    for (int i=0; i < connectResult->size(); ++i)
    {
      _log->debug("Look up input stream for set node '%v'", nodeName);
      auto &connect = connectResult->at(i);
      InputStreamDesc &input = nodeDesc.inputs[i];

      // get stream connected to node
      auto &streamValue = connect.args()[1];

      auto &node = streamValue.args()[2];
      auto &info = streamValue.args()[3];

      input.sourceSystem = this->ontology->toLongIri(*node.args()[1].name());
      input.nodeName = this->ontology->toLongIri(*node.args()[2].name());
      input.nodeEntity = this->ontology->toLongIri(*node.args()[3].name());
      input.nodeEntityRelated = *node.args()[4].name() == "none" ? "" : this->ontology->toLongIri(*node.args()[4].name());

      input.entity = this->ontology->toLongIri(*info.args()[0].name());
      input.scope = this->ontology->toLongIri(*info.args()[1].name());
      input.representation = this->ontology->toLongIri(*info.args()[2].name());
      input.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(input.metadata, streamValue);
    }

    // stream(k,SYSTEM,setNode(k,SOURCE,NODE,ENTITY,ENTITY2),INFO,STEP)
    values.clear();
    values.push_back(this->queryIndex);
    values.push_back(std::string(shortIri));
    values.push_back(Gringo::Value("setNode", nodeValues));
    values.push_back("?");
    values.push_back("?");

    Gringo::Value streamQuery("stream", values);
    auto streamResult = this->asp->queryAllTrue(&streamQuery);
    nodeDesc.outputs.resize(streamResult->size());

    for (int i=0; i < streamResult->size(); ++i)
    {
      _log->debug("Look up output stream for set node '%v'", nodeName);
      auto &stream = streamResult->at(i);
      OutputStreamDesc &output = nodeDesc.outputs[i];

      auto &info = stream.args()[3];

      output.entity = this->ontology->toLongIri(*info.args()[0].name());
      output.scope = this->ontology->toLongIri(*info.args()[1].name());
      output.representation = this->ontology->toLongIri(*info.args()[2].name());
      output.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(output.metadata, stream);
    }

    // sets
    // connectToSet(setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),
    //              set(k,SYSTEM,setNode(k,SOURCE,PROVIDER,ENTITY_TYPE2,ENTITY4),INFO_TYPE,STEP))
    nodeValues.clear();
    nodeValues.push_back(this->queryIndex);
    nodeValues.push_back(Gringo::Value(shortIri));
    nodeValues.push_back(Gringo::Value(aspNode->name));
    nodeValues.push_back(Gringo::Value(nodeEntityType));
    nodeValues.push_back(Gringo::Value(nodeEntity2));

    streamValues.clear();
    streamValues.push_back(this->queryIndex);
    streamValues.push_back(Gringo::Value(shortIri));
    streamValues.push_back("?");
    streamValues.push_back("?");
    streamValues.push_back("?");

    values.clear();
    values.push_back(Gringo::Value("node", nodeValues));
    values.push_back(Gringo::Value("set", streamValues));

    Gringo::Value connectSetQuery("connectToSet", values);
    connectResult = this->asp->queryAllTrue(&connectSetQuery);
    nodeDesc.inputSets.resize(connectResult->size());

    for (int i=0; i < connectResult->size(); ++i)
    {
      _log->debug("Look up input set for set node '%v'", nodeName);
      auto &connect = connectResult->at(i);
      InputSetDesc &input = nodeDesc.inputSets[i];

      // get stream connected to node
      auto &streamValue = connect.args()[1];

      auto &node = streamValue.args()[2];
      auto &info = streamValue.args()[3];

      input.sourceSystem = this->ontology->toLongIri(*node.args()[1].name());
      input.nodeName = this->ontology->toLongIri(*node.args()[2].name());
      input.nodeEntity = this->ontology->toLongIri(*node.args()[3].name());
      input.nodeEntityRelated = *node.args()[4].name() == "none" ? "" : this->ontology->toLongIri(*node.args()[4].name());

      input.entityType = this->ontology->toLongIri(*info.args()[0].name());
      input.scope = this->ontology->toLongIri(*info.args()[1].name());
      input.representation = this->ontology->toLongIri(*info.args()[2].name());
      input.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(input.metadata, streamValue);
    }

    // set(k,SYSTEM,setNode(k,SYSTEM,NODE,ENTITY_TYPE,ENTITY2),ENTITY_TYPE,STEP)
    values.clear();
    values.push_back(this->queryIndex);
    values.push_back(std::string(shortIri));
    values.push_back(Gringo::Value("setNode", nodeValues));
    values.push_back("?");
    values.push_back("?");

    Gringo::Value setQuery("set", values);
    auto setResult = this->asp->queryAllTrue(&setQuery);
    nodeDesc.outputSets.resize(setResult->size());

    for (int i=0; i < setResult->size(); ++i)
    {
      _log->debug("Look up output set for set node '%v'", nodeName);
      auto &set = setResult->at(i);
      OutputSetDesc &output = nodeDesc.outputSets[i];

      auto &info = set.args()[3];

      output.entityType = this->ontology->toLongIri(*info.args()[0].name());
      output.scope = this->ontology->toLongIri(*info.args()[1].name());
      output.representation = this->ontology->toLongIri(*info.args()[2].name());
      output.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

      this->readMetadata(output.metadata, set);
    }

    nodeDesc.type = aspNode->getNodeType();
    nodeDesc.className = aspNode->className;
    nodeDesc.aspName = this->ontology->toLongIri(aspNode->name);
    nodeDesc.entity = (nodeEntityType == "none" ? "" : this->ontology->toLongIri(nodeEntityType));
    nodeDesc.relatedEntity = (nodeEntity2 == "none" ? "" : this->ontology->toLongIri(nodeEntity2));
    nodeDesc.config = aspNode->configAsString;

    if (nodeDesc.aspName == "")
      nodeDesc.aspName = aspNode->name;

    nodes.push_back(nodeDesc);

    if (false == valid)
      break;
  }


  // TODO selected stream

  return valid;
}

bool ASPModelGenerator::extractStreamTransfers(std::shared_ptr<Entity> &from, std::shared_ptr<Entity> &to,
                                           std::vector<TransferStreamDesc> &transfers)
{
  bool valid = true;
  std::string shortIriTo;
  to->getId(EntityDirectory::ID_ONTOLOGY, shortIriTo);
  shortIriTo = this->ontology->toShortIri(shortIriTo);

  std::string shortIriFrom;
  from->getId(EntityDirectory::ID_ONTOLOGY, shortIriFrom);
  shortIriFrom = this->ontology->toShortIri(shortIriFrom);

  _log->debug("Look up streams transfered from '%v' to '%v'", shortIriFrom, shortIriTo);

  // stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP)
  std::vector<Gringo::Value> values;
  std::vector<Gringo::Value> nodeValues;
  nodeValues.push_back(this->queryIndex);
  nodeValues.push_back(std::string(shortIriFrom));
  nodeValues.push_back("?");
  nodeValues.push_back("?");
  nodeValues.push_back("?");

  values.push_back(this->queryIndex);
  values.push_back(std::string(shortIriTo));
  values.push_back(Gringo::Value("node", nodeValues));
  values.push_back("?");
  values.push_back("?");

  Gringo::Value sendQuery("stream", values);
  auto results = this->asp->queryAllTrue(&sendQuery);
  transfers.resize(results->size());

  // get streams connected to the node
  for (int i=0; i < results->size(); ++i)
  {
    auto &streamValue = results->at(i);
    auto &transfer = transfers.at(i);

    auto &node = streamValue.args()[2];
    auto &info = streamValue.args()[3];

    transfer.entity = this->ontology->toLongIri(*info.args()[0].name());
    transfer.scope = this->ontology->toLongIri(*info.args()[1].name());
    transfer.representation = this->ontology->toLongIri(*info.args()[2].name());
    transfer.relatedEntity = *info.args()[3].name() == "none" ? "" : this->ontology->toLongIri(*info.args()[3].name());

    transfer.sourceSystem = this->ontology->toLongIri(*node.args()[1].name());
    transfer.nodeName = this->ontology->toLongIri(*node.args()[2].name());
    transfer.nodeEntity = this->ontology->toLongIri(*node.args()[3].name());
    transfer.nodeEntityRelated = *node.args()[4].name() == "none" ? "" : this->ontology->toLongIri(*node.args()[4].name());
  }

  return valid;
}

void ASPModelGenerator::readInfoStructureFromOntology()
{
  _log->verbose(1, "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  std::stringstream ss;
  ss.str(this->ontology->readInformationStructureAsASP());

  _log->debug("Extracted structure from ontology");
  _log->verbose(1, ss.str());

  std::string programPart = "ontology" + ++this->queryIndex;
  std::string item, noIri;

  while (std::getline(ss, item, '\n'))
  {
    noIri = this->ontology->toShortIriAll(item);
    if (std::find(this->entities.begin(), this->entities.end(), noIri) == this->entities.end())
    {
      this->entities.push_back(noIri);
      this->asp->add(programPart, {}, noIri);
    }
  }

  this->asp->ground(programPart, {});

  this->groundingDirty = true;
}

void ASPModelGenerator::readSystemsFromOntology()
{
  _log->verbose(1, "Read systems from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  if (false == this->ontology->isSystemDirty())
    return;

  auto ontSystems = this->ontology->getSystems();

  if (ontSystems == nullptr)
  {
    _log->error("Error occurred while reading systems");
    return;
  }

  std::string iriSelf;
  this->self->getId(EntityDirectory::ID_ONTOLOGY, iriSelf);
  iriSelf = this->ontology->toShortIri(iriSelf);

  std::shared_ptr<Entity> entity;

  for (auto ontSystem : *ontSystems)
  {
    _log->debug("Checking system " + std::string(ontSystem));
    entity = this->directory->lookup(EntityDirectory::ID_ONTOLOGY, ontSystem, true);
    std::string iri = this->ontology->toShortIri(ontSystem);

    if (entity->getExternal() == nullptr)
    {
      auto ext = this->asp->getExternal("system", {Gringo::Value(iri)}, "system", {Gringo::Value(iri)}, true);
      entity->setExternal(ext);

      this->asp->add("base", {}, "transfer(" + iri + "," + iriSelf + ") :- system(" + iri + ").");
    }

    if (entity == this->self || entity->isCooperationPossible())
    {
      auto nodes = this->ontology->readNodesAsASP(ontSystem);
      this->toASP(nodes, entity, iri);
      entity->setNodesExtracted(true);
    }
  }
}

void ASPModelGenerator::toASP(std::unique_ptr<std::vector<std::vector<std::string>>> &nodes,
                              std::shared_ptr<Entity> &entity, std::string entityIriShort)
{
  auto &types = nodes->at(0);
  auto &names = nodes->at(1);
  auto &strings = nodes->at(2);
  auto &aspStrings = nodes->at(3);
  auto &cppStrings = nodes->at(4);

  for (int i = 0; i < names.size(); ++i)
  {
    std::string &name = names.at(i);
    std::string &elementStr = strings.at(i);
    std::string &aspStr = aspStrings.at(i);
    std::string &cppStr = cppStrings.at(i);
    std::string &typeStr = types.at(i);
    ASPElementType type;

    if (typeStr == "" || name == "" || elementStr == "")
    {
      _log->error("Empty string for element '%v': '%v' (elementStr), '%v' (typeStr), element will be skipped",
                  name, elementStr, typeStr);

      continue;
    }

    if (typeStr == "COMPUTATION_NODE")
    {
      type = ASPElementType::ASP_COMPUTATION_NODE;
    }
    else if (typeStr == "SOURCE_NODE")
    {
      type = ASPElementType::ASP_SOURCE_NODE;
    }
    else if (typeStr == "REQUIRED_STREAM")
    {
      type = ASPElementType::ASP_REQUIRED_STREAM;
    }
    else if (typeStr == "SET_NODE")
    {
      type = ASPElementType::ASP_SET_NODE;
    }
    else if (typeStr == "TRANSFORMATION_NODE")
    {
      type = ASPElementType::ASP_TRANSFORMATION_NODE;
    }
    else if (typeStr == "REQUIRED_SET")
    {
      type = ASPElementType::ASP_REQUIRED_SET;
    }
    else
    {
      _log->error("Unknown asp element type '%v' for element '%v', element will be skipped", typeStr, name);
      continue;
    }

    auto node = entity->getASPElementByName(type, this->ontology->toShortIri(name));

    if (node == nullptr)
    {
      _log->info("ASP element '%v' not found, creating new element", std::string(name));
      auto element = std::make_shared<ASPElement>();
      element->aspString = this->ontology->toShortIriAll(aspStr);
      element->name = this->ontology->toShortIri(name);
      element->state = ASPElementState::ADDED_TO_ASP;
      element->type = type;

      if (cppStr != "")
      {
        int index = cppStr.find('\n');

        if (index == std::string::npos)
          continue;

        element->className = cppStr.substr(0, index);
        element->configAsString = cppStr.substr(index + 1);
        element->config = this->readConfiguration(element->configAsString);
      }

      std::string shortIris = this->ontology->toShortIriAll(elementStr);
      auto value = supplementary::ClingWrapper::stringToValue(shortIris.c_str());

      switch (type)
      {
        case ASPElementType::ASP_COMPUTATION_NODE:
        case ASPElementType::ASP_SOURCE_NODE:
        case ASPElementType::ASP_SET_NODE:
        case ASPElementType::ASP_TRANSFORMATION_NODE:
          if (false == this->nodeStore->existNodeCreator(element->className))
          {
            _log->warn("Missing creator for node '%v' of type '%v', cpp grounding '%v', asp external set to false",
                       element->name, ASPElementTypeNames[type],
                       element->className == "" ? "NULL" : element->className);
            continue;
          }
          break;
        default:
          break;
      }

      element->external = this->asp->getExternal(*value.name(), value.args());
      element->external->assign(false);

      std::string nameProgramPart = entityIriShort + "_" + element->name;
      this->asp->add(nameProgramPart, {}, element->aspString);
      this->asp->ground(nameProgramPart, {});

      entity->addASPElement(element);
      this->groundingDirty = true;
    }
  }
}

void ASPModelGenerator::readTransformations()
{
  std::string system;
  this->self->getId(EntityDirectory::ID_ONTOLOGY, system);
  system = this->ontology->toShortIri(system);

  for(auto &transNode : this->gcontainerFactory->getTransformations())
  {
    bool found = false;
    for (auto &t : this->transformations)
    {
      if (t.node->shortName == transNode.second->shortName)
      {
        found = true;
        break;
      }
    }

    if (found)
      continue;

    std::stringstream ss;
    auto &trans = transNode.second->transformation;
    auto &name = transNode.second->shortName;
    auto scope = this->ontology->toShortIri(trans->getScope());
    std::string autoTrans = "autoTrans(" + system + "," + name + ",any,none).";
    ss << autoTrans << std::endl;

    ss << "output(" << system << "," << name << "," << scope << ","
        << this->ontology->toShortIri(trans->getTargetRepresentation()->name) + ",none)." << std::endl;

    for (auto &input : trans->getInputs())
    {
      ss << "input(" << system << "," << name << "," << scope << ","
          << this->ontology->toShortIri(input->name) << ",none,1,1) :- " << autoTrans << std::endl;
    }

    ss << "metadataProcessing(cost," << system << "," << name << ",10)." << std::endl;
    ss << "metadataOutput(delay," << system << "," << name << ",max,1,0)." << std::endl;
    ss << "metadataOutput(accuracy," << system << "," << name << ",max,-5,0)." << std::endl;

    auto element = std::make_shared<ASPElement>();
    element->aspString = this->ontology->toShortIriAll(ss.str());
    element->name = transNode.second->shortName;
    element->className = transNode.second->name;
    element->state = ASPElementState::ADDED_TO_ASP;
    element->type = ASPElementType::ASP_TRANSFORMATION_NODE;

    auto value = supplementary::ClingWrapper::stringToValue(autoTrans.c_str());

    element->external = this->asp->getExternal(*value.name(), value.args());
    element->external->assign(false);

    this->asp->add(element->name, {}, element->aspString);
    this->asp->ground(element->name, {});

    this->groundingDirty = true;

    _log->info("Created external for transformation node %v: %v -> %v", element->name,
               transNode.second->transformation->getInputs().at(0)->name,
               transNode.second->transformation->getTargetRepresentation()->name);

    ASPTransformation node;
    node.asp = element;
    node.node = transNode.second;
    this->transformations.push_back(node);
  }
}

std::shared_ptr<supplementary::ClingWrapper> ASPModelGenerator::getClingWrapper()
{
  return this->asp;
}

std::map<std::string, std::string> ASPModelGenerator::readConfiguration(std::string const &config)
{
  std::map<std::string, std::string> configuration;
  std::stringstream ss(config);
  std::string item;

  while (std::getline(ss, item, ';'))
  {
    int index = item.find("=");

    if (index == std::string::npos)
    {
      _log->warn("Broken configuration '%v', skipped", item);
      continue;
    }

    configuration[item.substr(0, index)] = item.substr(index + 1);
  }

  return configuration;
}

void ASPModelGenerator::readMetadata(std::map<std::string, int>  &metadata, const Gringo::Value &element)
{
  // TODO read metadata from ontology
  this->readMetadata("delay", metadata, element);
  this->readMetadata("accuracy", metadata, element);
}

void ASPModelGenerator::readMetadata(std::string name, std::map<std::string, int> &metadata,
                                     const Gringo::Value &element)
{
  // metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SOURCE,NODE,ENTITY,ENTITY2),INFO,STEP),VALUE)
  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(name));
  values.push_back(element);
  values.push_back("?");

  Gringo::Value query("metadataStream", values);
  auto result = this->asp->queryAllTrue(&query);

  if (result->size() != 1)
  {
    std::stringstream o;
    o << element;
    _log->warn("Wrong size '%v' for metadata '%v' of stream '%v'", result->size(), name, o.str());
    return;
  }

  const Gringo::Value &value = result->at(0).args()[3];

  if (value.type() != Gringo::Value::Type::NUM)
  {
    std::stringstream o, o2;
    o << element;
    o2 << value;
    _log->warn("Wrong type '%v' of '%v' for metadata '%v' of stream '%v'", value.type(), o2.str(), name,
               o.str());
    return;
  }

  std::stringstream o;
  o << element;
  _log->debug("Metadata '%v' of stream '%v' has value '%v'", name, o.str(), value.num());

  metadata.insert(std::make_pair(this->ontology->toLongIri(name), value.num()));
}

std::string ASPModelGenerator::dataTypeForRepresentation(std::string &representation)
{
  // TODO
  return this->ontology->toLongIri(representation);
}

} /* namespace ice */
