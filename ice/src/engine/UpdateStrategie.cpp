/*
 * UpdateStrategie.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include <ice/model/updateStrategie/UpdateStrategie.h>

#include "ice/information/BaseInformationSet.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/SetStore.h"
#include "ice/information/StreamStore.h"
#include "ice/model/ProcessingModelGenerator.h"
#include "ice/processing/Node.h"
#include "ice/ICEngine.h"
#include "ice/Entity.h"
#include "ice/EntityDirectory.h"

namespace ice
{

UpdateStrategie::UpdateStrategie(std::weak_ptr<ICEngine> engine)
{
  this->_log = el::Loggers::getLogger("UpdateStrategie");
  this->engine = engine;
  this->running = false;
  this->established = false;
  this->valid = false;
}

UpdateStrategie::~UpdateStrategie()
{
  //
}

void UpdateStrategie::init()
{
  // create worker thread
  this->running = true;
  auto en = this->engine.lock();
  this->ontology = en->getOntologyInterface();
  this->nodeStore = en->getNodeStore();
  this->knowledgeBase = en->getKnowlegeBase();
  this->communication = en->getCommunicationInterface();
  this->modelGenerator = en->getProcessingModelGenerator();
  this->directory = en->getEntityDirector();
  this->worker = std::thread(&UpdateStrategie::workerTask, this);

  this->self = en->getSelf();

  this->initInternal();

  // register callback for new and vanisehd entities
  this->directory->disvoeredIceIdentity.registerCallback(this, &UpdateStrategie::onEntityDiscovered);
  this->directory->vanishedIceIdentity.registerCallback(this, &UpdateStrategie::onEntityVanished);

}

void UpdateStrategie::cleanUp()
{
  this->ontology.reset();
  this->nodeStore.reset();
  this->knowledgeBase.reset();
  this->communication.reset();
  this->modelGenerator.reset();

  {
    std::lock_guard<std::mutex> guard(mtx_);

    if (this->running == false)
      return;

    this->running = false;
  }

  this->cv.notify_all();
  this->worker.join();

  this->cleanUpInternal();
}

void UpdateStrategie::update(ModelUpdateEvent event, std::shared_ptr<void> object)
{
  if (false == this->running)
  {
    return;
  }

  bool trigger = false;
  bool synchrone = false;

  switch(event)
  {
    case MUE_INITIAL:
      trigger = true;
      synchrone = true;
      break;
    case MUE_NONE:
      // TODO
      break;
    case MUE_INSTANCE_NEW:
      trigger = true;
      break;
    case MUE_INSTANCE_VANISHED:
      // TODO
      trigger = true;
      break;
    case MUE_NODE_FAILURE:
      // TODO
      trigger = true;
      break;
    case MUE_INFORMATION_REQ:
      trigger = true;
      break;
    case MUE_RESOURCE_REQ:
      trigger = true;
      break;
    case MUE_OPTIMIZATION:
      trigger = true;
      break;
  }

  if (false == trigger)
  {
    return;
  }

  // trigger update
  if (synchrone)
  {
    auto model = this->modelGenerator->createProcessingModel();
    this->processModel(model);
  }
  else
  {
    std::lock_guard<std::mutex> guard(mtx_);
    this->cv.notify_all();
  }
}

void UpdateStrategie::processModel(std::shared_ptr<ProcessingModel> const &model)
{
  this->lastModel = this->model;
  this->model = model;
  this->valid = true;
  this->established = false;
}

bool UpdateStrategie::processSubModel(std::shared_ptr<Entity> &entity, std::shared_ptr<SubModelDesc> &subModel)
{
  _log->debug("Processing sub model description received from '%v'", entity->toString());
  bool valid = true;
  std::vector<std::shared_ptr<Node>> createdNodes;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived;
  std::vector<std::shared_ptr<BaseInformationSet>> setsSend;
  std::vector<std::shared_ptr<BaseInformationSet>> setsReceived;

  // TODO altes model prüfen und entsprechend handhaben
  entity->getReceivedSubModel().subModel = subModel;

  for (auto &nodeDesc : subModel->nodes)
  {
    auto node = this->activateNode(nodeDesc);

    if (node == nullptr)
    {
      valid = false;
      break;
    }

    createdNodes.push_back(node);
  }

  if (false == valid)
  {
    entity->clearReceived();
    this->nodeStore->cleanUpNodes();
    this->knowledgeBase->cleanUpStores();

    return false;
  }

  // streams
  for (auto &transferTo : subModel->send)
  {
    auto stream = this->getStream(transferTo);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", transferTo.nodeName);
      valid = false;
      break;
    }

    streamsSend.push_back(stream);
  }

  if (false == valid)
  {
    entity->clearReceived();
    this->nodeStore->cleanUpNodes();
    this->knowledgeBase->cleanUpStores();

    return false;
  }

  for (auto &transferFrom : subModel->receive)
  {
    auto stream = this->getStream(transferFrom);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", transferFrom.nodeName);
      valid = false;
      break;
    }

    streamsReceived.push_back(stream);
  }

  if (false == valid)
  {
    entity->clearReceived();
    this->nodeStore->cleanUpNodes();
    this->knowledgeBase->cleanUpStores();

    return false;
  }

  // sets
  for (auto &transferTo : subModel->sendSet)
  {
    auto set = this->getSet(transferTo);

    if (false == set)
    {
      _log->error("Set '%v' could not be found, sub model is invalid!", transferTo.nodeName);
      valid = false;
      break;
    }

    setsSend.push_back(set);
  }

  if (false == valid)
  {
    entity->clearReceived();
    this->nodeStore->cleanUpNodes();
    this->knowledgeBase->cleanUpStores();

    return false;
  }

  for (auto &transferFrom : subModel->receiveSet)
  {
    auto set = this->getSet(transferFrom);

    if (false == set)
    {
      _log->error("Set '%v' could not be found, sub model is invalid!", transferFrom.nodeName);
      valid = false;
      break;
    }

    setsReceived.push_back(set);
  }

  if (false == valid)
  {
    entity->clearReceived();
    this->nodeStore->cleanUpNodes();
    this->knowledgeBase->cleanUpStores();

    return false;
  }

  _log->info("Finished processing of received sub model from '%v'", entity->toString());
  entity->updateReceived(createdNodes, streamsSend, streamsReceived, setsSend, setsReceived);

  for (auto node : createdNodes)
  {
    node->activate();
  }

  this->nodeStore->cleanUpNodes();
  this->knowledgeBase->cleanUpStores();
  return true;
}

std::shared_ptr<Node> UpdateStrategie::activateNode(NodeDesc &nodeDesc)
{
  _log->debug("Look up node '%v' to process entity '%v'", nodeDesc.aspName, nodeDesc.entity);

  auto config = this->readConfiguration(nodeDesc.config);
  auto node = this->nodeStore->registerNode(static_cast<NodeType>(nodeDesc.type),
                                            nodeDesc.className,
                                            nodeDesc.aspName,
                                            nodeDesc.entity,
                                            nodeDesc.relatedEntity,
                                            config);

  if (node == nullptr)
  {
    _log->error("Node '%v' (%v) could not be created, sub model is invalid!", nodeDesc.aspName, nodeDesc.className);
    return nullptr;
  }

  // streams
  for (auto &input : nodeDesc.inputs)
  {
    _log->debug("Look up connected stream for node '%v'", nodeDesc.aspName);

    auto stream = this->getStream(input.nodeName, input.sourceSystem, input.entity, input.scope, input.representation,
                                  input.relatedEntity, input.metadata);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be created, sub model is invalid!", nodeDesc.aspName);
      return nullptr;
    }

    node->addInput(stream, true); // TODO stream is trigger?
  }

  for (auto &output : nodeDesc.outputs)
  {

    _log->debug("Look up output stream for node '%v'", nodeDesc.aspName);

    std::string iri;
    this->self->getId(EntityDirectory::ID_ONTOLOGY, iri);

    auto stream = this->getStream(nodeDesc.aspName, iri,
                                  output.entity, output.scope, output.representation,
                                  output.relatedEntity, output.metadata);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be created, sub model is invalid!", nodeDesc.aspName);
      return nullptr;
    }

    node->addOutput(stream);
  }

  // sets
  for (auto &input : nodeDesc.inputSets)
  {
    _log->debug("Look up connected set for node '%v'", nodeDesc.aspName);

    auto set = this->getSet(input.nodeName, input.sourceSystem, input.entityType, input.scope, input.representation,
                                  input.relatedEntity, input.metadata);

    if (false == set)
    {
      _log->error("Set '%v' could not be created, sub model is invalid!", nodeDesc.aspName);
      return nullptr;
    }

    node->addInputSet(set, true); // TODO stream is trigger?
  }

  for (auto &output : nodeDesc.outputSets)
  {

    _log->debug("Look up output set for node '%v'", nodeDesc.aspName);

    std::string iri;
    this->self->getId(EntityDirectory::ID_ONTOLOGY, iri);

    auto set = this->getSet(nodeDesc.aspName, iri,
                                  output.entityType, output.scope, output.representation,
                                  output.relatedEntity, output.metadata);

    if (false == set)
    {
      _log->error("Set '%v' could not be created, sub model is invalid!", nodeDesc.aspName);
      return nullptr;
    }

    node->addOutputSet(set);
  }

  return node;
}

bool UpdateStrategie::processSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex)
{
  auto subModel = entity->getSendSubModel().subModel;
  bool valid = true;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived;
  std::vector<std::shared_ptr<BaseInformationSet>> setsSend;
  std::vector<std::shared_ptr<BaseInformationSet>> setsReceived;

  _log->debug("Sub model accepted received from system '%v' with index '%v'", entity->toString(), modelIndex);

  if (subModel == nullptr)
  {
    _log->info("Sub model accepted received from system '%v' is empty", entity->toString());
    return false;
  }

  int index = subModel->index;

  if (index != modelIndex)
  {
    _log->info("Sub model accepted received from system '%v', wrong index '%v' expected '%v'", entity->toString(), index,
               modelIndex);
    return false;
  }

  // streams
  for (auto &transferTo : subModel->send)
  {
    auto stream = this->getStream(transferTo);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", transferTo.nodeName);
      valid = false;
      break;
    }

    streamsSend.push_back(stream);
  }

  if (false == valid)
  {
    return false;
  }

  for (auto &transferFrom : subModel->receive)
  {
    auto stream = this->getStream(transferFrom);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", transferFrom.nodeName);
      valid = false;
      break;
    }

    streamsReceived.push_back(stream);
  }

  if (false == valid)
  {
    return false;
  }

  // sets
  for (auto &transferTo : subModel->sendSet)
  {
    auto set = this->getSet(transferTo);

    if (false == set)
    {
      _log->error("Set '%v' could not be found, sub model is invalid!", transferTo.nodeName);
      valid = false;
      break;
    }

    setsSend.push_back(set);
  }

  if (false == valid)
  {
    return false;
  }

  for (auto &transferFrom : subModel->receiveSet)
  {
    auto set = this->getSet(transferFrom);

    if (false == set)
    {
      _log->error("Set '%v' could not be found, sub model is invalid!", transferFrom.nodeName);
      valid = false;
      break;
    }

    setsReceived.push_back(set);
  }

  if (false == valid)
  {
    return false;
  }

  return true;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(std::string &nodeName, std::string &source,
                                                                  std::string &entity, std::string &scope,
                                                                  std::string &rep, std::string &relatedEntity,
                                                                  std::map<std::string, int> &metadata)
{
  auto infoSpec = std::make_shared<InformationSpecification>(entity, this->knowledgeBase->getEntityType(entity),
                                                             scope, rep, relatedEntity);

  auto stream = this->knowledgeBase->streamStore->getBaseCollection(infoSpec.get(), nodeName, source);

  if (false == stream)
  {
    std::string dataType = this->dataTypeForRepresentation(rep);
    std::string name = entity + "_" + nodeName + "_" + source;
    std::replace(name.begin(), name.end(), '.', '_');
    std::replace(name.begin(), name.end(), '#', '_');
    std::replace(name.begin(), name.end(), '/', '_');
    std::replace(name.begin(), name.end(), ':', '_');
    std::replace(name.begin(), name.end(), '-', '_');
    stream = this->knowledgeBase->streamStore->registerBaseStream(dataType, infoSpec, name, 10, metadata, nodeName, source);
  }

  return stream;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(TransferStreamDesc &desc)
{
  std::map<std::string, int> metadata; //TODO
  return this->getStream(desc.nodeName, desc.sourceSystem, desc.entity, desc.scope, desc.representation, desc.relatedEntity, metadata);
}

std::shared_ptr<BaseInformationSet> UpdateStrategie::getSet(std::string &nodeName, std::string &source,
                                                                  std::string &entityType, std::string &scope,
                                                                  std::string &rep, std::string &relatedEntity,
                                                                  std::map<std::string, int> &metadata)
{
  auto infoSpec = std::make_shared<InformationSpecification>("", entityType,
                                                             scope, rep, relatedEntity);

  auto set = this->knowledgeBase->setStore->getBaseCollection(infoSpec.get(), nodeName, source);

  if (false == set)
  {
    std::string dataType = this->dataTypeForRepresentation(rep);
    std::string name = entityType + "_" + nodeName + "_" + source;
    std::replace(name.begin(), name.end(), '.', '_');
    std::replace(name.begin(), name.end(), '#', '_');
    std::replace(name.begin(), name.end(), '/', '_');
    std::replace(name.begin(), name.end(), ':', '_');
    std::replace(name.begin(), name.end(), '-', '_');
    set = this->knowledgeBase->setStore->registerBaseSet(dataType, infoSpec, name, metadata, nodeName, source);
  }

  return set;
}

std::shared_ptr<BaseInformationSet> UpdateStrategie::getSet(TransferSetDesc &desc)
{
  std::map<std::string, int> metadata; //TODO
  return this->getSet(desc.nodeName, desc.sourceSystem, desc.entityType, desc.scope, desc.representation, desc.relatedEntity, metadata);
}

std::map<std::string, std::string> UpdateStrategie::readConfiguration(std::string const config)
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

    configuration[item.substr(0, index)] = item.substr(index + 1, item.size());
  }

  return configuration;
}

std::string UpdateStrategie::dataTypeForRepresentation(std::string representation)
{
  // TODO
  return representation;
}

std::shared_ptr<SubModel> UpdateStrategie::getSubModelDesc(std::shared_ptr<Entity> &entity)
{
  if (this->model == nullptr)
    return nullptr;

  for (auto &sub : this->model->getSubModels())
  {
    if (sub->entity == entity)
      return sub;
  }

  return nullptr;
}

void UpdateStrategie::onEntityDiscovered(std::shared_ptr<Entity> entity)
{
  this->update(ModelUpdateEvent::MUE_INSTANCE_NEW, entity);
}

void UpdateStrategie::onEntityVanished(std::shared_ptr<Entity> entity)
{
  this->update(ModelUpdateEvent::MUE_INSTANCE_VANISHED, entity);
}

void UpdateStrategie::workerTask()
{
  while (this->running)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    this->cv.wait(lock);

    if (false == this->running)
      break;

    auto model = this->modelGenerator->createProcessingModel();
    this->processModel(model);
  }
}
} /* namespace ice */
