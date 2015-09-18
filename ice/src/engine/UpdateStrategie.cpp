/*
 * UpdateStrategie.cpp
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#include <ice/model/updateStrategie/UpdateStrategie.h>

#include "ice/coordination/EngineState.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationSpecification.h"
#include "ice/model/ProcessingModelGenerator.h"
#include "ice/processing/Node.h"
#include "ice/ICEngine.h"

namespace ice
{

UpdateStrategie::UpdateStrategie(std::weak_ptr<ICEngine> engine)
{
  this->_log = el::Loggers::getLogger("UpdateStrategie");
  this->engine = engine;
  this->running = false;
}

UpdateStrategie::~UpdateStrategie()
{
  // TODO Auto-generated destructor stub
}

void UpdateStrategie::init()
{
  // create worker thread
  this->running = true;
  auto en = this->engine.lock();
  this->ontology = en->getOntologyInterface();
  this->nodeStore = en->getNodeStore();
  this->informationStore = en->getInformationStore();
  this->coordinator = en->getCoordinator();
  this->communication = en->getCommunication();
  this->modelGenerator = en->getProcessingModelGenerator();
  this->worker = std::thread(&UpdateStrategie::workerTask, this);

  this->self = this->coordinator->getEngineStateNoMutex(en->getIri());

  this->initInternal();
}

void UpdateStrategie::cleanUp()
{
  this->ontology.reset();
  this->nodeStore.reset();
  this->informationStore.reset();
  this->coordinator.reset();
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

void UpdateStrategie::update(std::shared_ptr<ProcessingModel> model)
{
  this->lastModel = this->model;
  this->model = model;
  this->valid = true;
  this->established = false;
}

bool UpdateStrategie::processSubModel(std::shared_ptr<EngineState> engineState, SubModelDesc &subModel)
{
  const char* idCStr = IDGenerator::toString(engineState->getEngineId()).c_str();
  _log->debug("Processing sub model description received from engine '%v'", idCStr);
  bool valid = true;
  std::vector<std::shared_ptr<Node>> createdNodes;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived;

  engineState->getOffering()->subModel = std::make_shared<SubModelDesc>(subModel);

  for (auto &nodeDesc : std::get<1>(subModel))
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
    engineState->clearOffering();
    this->nodeStore->cleanUpNodes();
    this->informationStore->cleanUpStreams();

    return false;
  }

  for (auto transferTo : std::get<2>(subModel))
  {
    auto stream = this->getStream(transferTo);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", std::get<1>(transferTo).c_str());
      valid = false;
      break;
    }

    streamsSend.push_back(stream);
  }

  if (false == valid)
  {
    engineState->clearOffering();
    this->nodeStore->cleanUpNodes();
    this->informationStore->cleanUpStreams();

    return false;
  }

  for (auto transferFrom : std::get<3>(subModel))
  {
    auto stream = this->getStream(transferFrom);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", std::get<1>(transferFrom).c_str());
      valid = false;
      break;
    }

    streamsReceived.push_back(stream);
  }

  if (false == valid)
  {
    engineState->clearOffering();
    this->nodeStore->cleanUpNodes();
    this->informationStore->cleanUpStreams();

    return false;
  }
  else
  {
    _log->info("Processed sub model received from system '%v'",
               IDGenerator::toString(engineState->getEngineId()).c_str());

    engineState->updateOffering(&createdNodes, &streamsSend, &streamsReceived);

    for (auto node : createdNodes)
    {
      node->activate();
    }

    this->nodeStore->cleanUpNodes();
    this->informationStore->cleanUpStreams();
    return true;
  }
}

std::shared_ptr<Node> UpdateStrategie::activateNode(NodeDesc &nodeDesc)
{
  NodeType type = static_cast<NodeType>(std::get<0>(nodeDesc));
  string className = std::get<1>(nodeDesc);
  string nodeName = std::get<2>(nodeDesc);
  string entity = std::get<3>(nodeDesc);
  string entity2 = std::get<4>(nodeDesc);
  string cfg = std::get<5>(nodeDesc);
  vector<InputStreamDesc> inputs = std::get<6>(nodeDesc);
  vector<OutputStreamDesc> outputs = std::get<7>(nodeDesc);

  _log->debug("Look up node '%v' to process entity '%v'", nodeName.c_str(), entity.c_str());

  auto node = this->nodeStore->registerNode(type, className, nodeName, entity, entity2, this->readConfiguration(cfg));

  if (node == nullptr)
  {
    _log->error("Node '%v' (%v) could not be created, sub model is invalid!", nodeName.c_str(), className.c_str());
    return nullptr;
  }

  for (auto input : inputs)
  {
    _log->debug("Look up connected stream for node '%v'", nodeName.c_str());

    string sourceSystem = std::get<0>(input);
    string nodeName = std::get<1>(input);
    string nodeEntity = std::get<2>(input);
    string nodeEntity2 = std::get<3>(input);
    string entity = std::get<4>(input);
    string scope = std::get<5>(input);
    string rep = std::get<6>(input);
    string entity2 = std::get<7>(input);

    std::map<std::string, int> metadata; // TODO

    auto stream = this->getStream(nodeName, sourceSystem, entity, scope, rep, entity2, metadata);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be created, sub model is invalid!", nodeName.c_str());
      return nullptr;
    }

    node->addInput(stream, true); // TODO stream is trigger?
  }

  for (auto output : outputs)
  {
    string entity = std::get<0>(output);
    string scope = std::get<1>(output);
    string rep = std::get<2>(output);
    string entity2 = std::get<3>(output);

    std::map<std::string, int> metadata; // TODO

    _log->debug("Look up output stream for node '%v'", nodeName.c_str());
    auto stream = this->getStream(nodeName, this->self->getSystemIriShort(), entity, scope, rep, entity2, metadata);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be created, sub model is invalid!", nodeName.c_str());
      return nullptr;
    }

    node->addOutput(stream);
  }

  return node;
}

bool UpdateStrategie::processSubModelResponse(std::shared_ptr<EngineState> engineState, int modelIndex)
{
  const char* idCStr = IDGenerator::toString(engineState->getEngineId()).c_str();
  auto subModel = engineState->getRequesting()->subModel;
  bool valid = true;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend;
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived;

  _log->debug("Sub model accepted received from system '%v' with index '%v'", idCStr, modelIndex);

  if (subModel == nullptr)
  {
    _log->info("Sub model accepted received from system '%v' is empty", idCStr);
    // TODO
    return false;
  }

  int index = std::get<0>(*subModel);

  if (index != modelIndex)
  {
    _log->info("Sub model accepted received from system '%v', wrong index '%v' expected '%v'", idCStr, index,
               modelIndex);
    // TODO
    return false;
  }

  for (auto transferTo : std::get<3>(*subModel))
  {
    auto stream = this->getStream(transferTo);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", std::get<1>(transferTo).c_str());
      valid = false;
      break;
    }

    streamsSend.push_back(stream);
  }

  if (false == valid)
  {
    // TODO

    return false;
  }

  for (auto transferFrom : std::get<2>(*subModel))
  {
    auto stream = this->getStream(transferFrom);

    if (false == stream)
    {
      _log->error("Stream '%v' could not be found, sub model is invalid!", std::get<1>(transferFrom).c_str());
      valid = false;
      break;
    }

    streamsReceived.push_back(stream);
  }

  if (false == valid)
  {
    // TODO
    return false;
  }

  for (auto stream : streamsSend)
  {
    stream->registerSender(this->communication);
  }

  for (auto stream : streamsReceived)
  {
    stream->registerReceiver(this->communication);
  }

  return true;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(std::string nodeName, std::string source,
                                                                  std::string entity, std::string scope,
                                                                  std::string rep, std::string relatedEntity,
                                                                  std::map<std::string, int> metadata)
{
  auto infoSpec = std::make_shared<InformationSpecification>(entity, this->informationStore->getEntityType(entity),
                                                             scope, rep, relatedEntity);

  auto stream = this->informationStore->getBaseStream(infoSpec.get(), nodeName, source);

  if (false == stream)
  {

    //        std::shared_ptr<InformationSpecification> specification,
    //        const std::string name,
    //        const int streamSize,
    //        std::map<std::string, int> metadatas,
    //        std::string provider,
    //        std::string sourceSystem

    std::string dataType = this->dataTypeForRepresentation(rep);
    std::string name = entity + "-" + nodeName + "-" + source;
    stream = this->informationStore->registerBaseStream(dataType, infoSpec, name, 10, metadata, nodeName, source);
  }

  return stream;
}

std::shared_ptr<BaseInformationStream> UpdateStrategie::getStream(TransferDesc &desc)
{
  string source = std::get<0>(desc);
  string nodeName = std::get<1>(desc);
  string nodeEntity = std::get<2>(desc);
  string nodeEntity2 = std::get<3>(desc);
  string entity = std::get<4>(desc);
  string scope = std::get<5>(desc);
  string rep = std::get<6>(desc);
  string entity2 = std::get<7>(desc);

  _log->debug("Look up stream '%v'", nodeName.c_str());

  auto infoSpec = make_shared<InformationSpecification>(entity, this->informationStore->getEntityType(entity), scope,
                                                        rep, entity2);
  return this->informationStore->getBaseStream(infoSpec.get(), nodeName, source);
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
      _log->warn("Broken configuration '%v', skipped", item.c_str());
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

std::shared_ptr<SubModel> UpdateStrategie::getSubModelDesc(std::shared_ptr<EngineState> engineState)
{
  if (this->model == nullptr)
    return nullptr;

  for (auto sub : *this->model->getSubModels())
  {
    if (sub->engine == engineState)
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

    this->update(this->modelGenerator->createProcessingModel());
  }
}
} /* namespace ice */
