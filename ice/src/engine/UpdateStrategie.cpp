/*
 * UpdateStrategie.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include <ice/model/updateStrategie/UpdateStrategie.h>

#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationSpecification.h"
#include "ice/model/ProcessingModelGenerator.h"
#include "ice/processing/Node.h"
#include "ice/ICEngine.h"
#include "ice/Entity.h"

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
  this->streamStore = en->getStreamStore();
  this->communication = en->getCommunicationInterface();
  this->modelGenerator = en->getProcessingModelGenerator();
  this->worker = std::thread(&UpdateStrategie::workerTask, this);

  this->self = en->getSelf();

  this->initInternal();
}

void UpdateStrategie::cleanUp()
{
  this->ontology.reset();
  this->nodeStore.reset();
  this->streamStore.reset();
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

void UpdateStrategie::update(std::shared_ptr<ProcessingModel> &model)
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
    this->streamStore->cleanUpStreams();

    return false;
  }

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
    this->streamStore->cleanUpStreams();

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
    this->streamStore->cleanUpStreams();

    return false;
  }
  else
  {
    _log->info("Processed sub model received from system '%v'", entity->toString());

    entity->updateReceived(createdNodes, streamsSend, streamsReceived);

    for (auto node : createdNodes)
    {
      node->activate();
    }

    this->nodeStore->cleanUpNodes();
    this->streamStore->cleanUpStreams();
    return true;
  }
}

std::shared_ptr<Node> UpdateStrategie::activateNode(NodeDesc &nodeDesc)
{
  _log->debug("Look up node '%v' to process entity '%v'", nodeDesc.aspName, nodeDesc.entity);

  auto node = this->nodeStore->registerNode(static_cast<NodeType>(nodeDesc.type),
                                            nodeDesc.className,
                                            nodeDesc.aspName,
                                            nodeDesc.entity,
                                            nodeDesc.relatedEntity,
                                            this->readConfiguration(nodeDesc.config));

  if (node == nullptr)
  {
    _log->error("Node '%v' (%v) could not be created, sub model is invalid!", nodeDesc.aspName, nodeDesc.className);
    return nullptr;
  }

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
    std::string iriShort = this->ontology->toShortIri(iri);

    auto stream = this->getStream(nodeDesc.aspName, iriShort,
                                  output.entity, output.scope, output.representation,
                                  output.relatedEntity, output.metadata);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be created, sub model is invalid!", nodeDesc.aspName);
      return nullptr;
    }

    node->addOutput(stream);
  }

  return node;
}

bool UpdateStrategie::processSubModelResponse(std::shared_ptr<Entity> &entity, int modelIndex)
{
  auto subModel = entity->getSendSubModel().subModel;
  bool valid = true;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived;

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

  return true;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(std::string &nodeName, std::string &source,
                                                                  std::string &entity, std::string &scope,
                                                                  std::string &rep, std::string &relatedEntity,
                                                                  std::map<std::string, int> &metadata)
{
  auto infoSpec = std::make_shared<InformationSpecification>(entity, this->streamStore->getEntityType(entity),
                                                             scope, rep, relatedEntity);

  auto stream = this->streamStore->getBaseStream(infoSpec.get(), nodeName, source);

  if (false == stream)
  {
    std::string dataType = this->dataTypeForRepresentation(rep);
    std::string name = entity + "_" + nodeName + "_" + source;
    std::replace(name.begin(), name.end(), '.', '_');
    std::replace(name.begin(), name.end(), '#', '_');
    std::replace(name.begin(), name.end(), '/', '_');
    std::replace(name.begin(), name.end(), ':', '_');
    std::replace(name.begin(), name.end(), '-', '_');
    stream = this->streamStore->registerBaseStream(dataType, infoSpec, name, 10, metadata, nodeName, source);
  }

  return stream;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(TransferDesc &desc)
{
  std::map<std::string, int> metadata; //TODO
  return this->getStream(desc.nodeName, desc.sourceSystem, desc.entity, desc.scope, desc.representation, desc.relatedEntity, metadata);
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

void UpdateStrategie::triggerModelUpdate()
{
  std::lock_guard<std::mutex> guard(mtx_);
  this->cv.notify_all();
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
    this->update(model);
  }
}
} /* namespace ice */
