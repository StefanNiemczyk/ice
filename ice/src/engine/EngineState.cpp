/*
 * EngineState.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#include "ice/coordination/EngineState.h"

#include "ice/communication/Communication.h"
#include "ice/processing/Node.h"
#include "ice/Configuration.h"
#include "ice/ICEngine.h"
#include "ice/TimeFactory.h"

#include "easylogging++.h"

#include <algorithm>

namespace ice
{

EngineState::EngineState(identifier engineId, std::weak_ptr<ICEngine> engine) :
    engineId(engineId), systemIri("")
{
  this->engine = engine;
  this->offering = std::make_shared<CooperationContainer>();
  this->requesting = std::make_shared<CooperationContainer>();

  auto e = engine.lock();
  this->timeFactory = e->getTimeFactory();
  this->config = e->getConfig();
  this->communication = e->getCommunication();

  this->timeLastActivity = this->timeFactory->createTime();
  this->timeLastStateUpdate = NO_TIME;
  this->retryCounter = 0;
  this->nodesKnown = false;
  this->_log = el::Loggers::getLogger("EngineState");
}

EngineState::EngineState(std::string const systemIri, std::weak_ptr<ICEngine> engine) :
    systemIri(systemIri)
{
  this->engine = engine;
  this->offering = std::make_shared<CooperationContainer>();
  this->requesting = std::make_shared<CooperationContainer>();

  auto e = engine.lock();
  this->timeFactory = e->getTimeFactory();
  this->config = e->getConfig();

  this->timeLastActivity = this->timeFactory->createTime();
  this->timeLastStateUpdate = NO_TIME;
  this->retryCounter = 0;
  this->nodesKnown = false;
  this->_log = el::Loggers::getLogger("EngineState");
}

EngineState::~EngineState()
{
  //
}

const identifier EngineState::getEngineId() const
{
  return this->engineId;
}

void EngineState::setEngineId(const identifier id)
{
  this->engineId = id;
}

const std::string EngineState::getSystemIri() const
{
  return this->systemIri;
}

void EngineState::setSystemIri(std::string systemIri)
{
  this->systemIri = systemIri;
}

const std::string EngineState::getSystemIriShort()
{
  if (this->systemIri == "")
    return "";

  if (this->systemIriShort != "")
    return this->systemIriShort;

  auto e = this->engine.lock();
  this->systemIriShort = e->getOntologyInterface()->toShortIri(this->systemIri);

  return this->systemIriShort;

//  int index1 = this->systemIri.find("#");
//
//  if (index1 == std::string::npos)
//    return this->systemIri;
//
//  std::string name = this->systemIri.substr(index1 + 1, this->systemIri.size() - index1 - 1);
//  name[0] = std::tolower(name[0]);
//  return name;
}

time EngineState::getTimeLastActivity() const
{
  return this->timeLastActivity;
}

void EngineState::setTimeLastActivity(time timeLastActivity)
{
  this->timeLastActivity = timeLastActivity;
}

void EngineState::updateTimeLastActivity()
{
  this->timeLastActivity = this->timeFactory->createTime();
}

const time EngineState::getTimeLastStateUpdate() const
{
  return this->timeLastStateUpdate;
}

std::shared_ptr<CooperationContainer> EngineState::getOffering()
{
  return this->offering;
}

std::shared_ptr<CooperationContainer> EngineState::getRequesting()
{
  return this->requesting;
}

int EngineState::getRetryCounter() const
{
  return retryCounter;
}

void EngineState::setRetryCounter(int retryCounter)
{
  this->retryCounter = retryCounter;
}

int EngineState::increaseRetryCounter()
{
  this->retryCounter++;

  return this->retryCounter;
}

void EngineState::resetRetryCounter()
{
  this->retryCounter = 0;
}

bool EngineState::isCooperationPossible() const
{
  // no iri, so cooperation is not possible
  if (this->systemIri == "")
    return false;

  // No cooperation with this engine
  if (this->requesting->state == CooperationState::NO_COOPERATION
      || this->requesting->state == CooperationState::UNKNOWN)
    return false;

  // Connection lost
  if (this->timeFactory->checkTimeout(this->timeLastActivity, this->config->getHeartbeatTimeout()))
    return false;

  // Nodes of this engine are unknown, no cooperation is possible
  if (false == this->nodesKnown)
    return false;

  return true;
}

bool EngineState::isNodesKnown() const
{
  return this->nodesKnown;
}

void EngineState::setNodesKnown(bool value)
{
  this->nodesKnown = value;
}

void EngineState::updateOffering(std::vector<std::shared_ptr<Node>> *nodes,
                                 std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                                 std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived)
{
  this->updateContainer(this->offering, nodes, streamsSend, streamsReceived);
}

void EngineState::updateRequesting(std::vector<std::shared_ptr<Node>> *nodes,
                                  std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                                  std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived)
{
  this->updateContainer(this->requesting, nodes, streamsSend, streamsReceived);
}

std::vector<std::shared_ptr<EngineConnection>> EngineState::getConnections()
{
  return this->connections;
}

void EngineState::clearOffering()
{
  this->clearContainer(this->offering);
}

void EngineState::clearRequesting()
{
  this->clearContainer(this->requesting);
}

void EngineState::updateContainer(std::shared_ptr<CooperationContainer> container,
                                  std::vector<std::shared_ptr<Node>> *nodes,
                                  std::vector<std::shared_ptr<BaseInformationStream>> *streamsSend,
                                  std::vector<std::shared_ptr<BaseInformationStream>> *streamsReceived)
{
  // update nodes
  for (auto node : this->nodesActivated)
  {
    if (std::find(nodes->begin(), nodes->end(), node) == nodes->end())
    {
      node->unregisterEngine(this->shared_from_this());
    }
  }

  for (auto node : *nodes)
  {
    node->registerEngine(this->shared_from_this());
  }

  // update streams send
  for (int i = 0; i < container->streamsSend.size(); ++i)
  {
    auto stream = container->streamsSend[i];
    if (std::find(streamsSend->begin(), streamsSend->end(), stream) == streamsSend->end())
    {
      container->streamsSend.erase(container->streamsSend.begin() + i);
      --i;
      stream->unregisterEngineState(this->shared_from_this());
    }
  }

  for (auto stream : *streamsSend)
  {
    if (stream->registerEngineState(this->shared_from_this()) == 0)
    {
      container->streamsSend.push_back(stream);
    }

    stream->registerSender(this->communication);
  }

  // update streams received
  for (int i = 0; i < container->streamsReceived.size(); ++i)
  {
    auto stream = container->streamsReceived[i];
    if (std::find(streamsReceived->begin(), streamsReceived->end(), stream) == streamsReceived->end())
    {
      container->streamsReceived.erase(container->streamsReceived.begin() + i);
      --i;
      stream->dropReceiver();
    }
  }

  for (auto stream : *streamsReceived)
  {
    if (stream->registerEngineState(this->shared_from_this()) == 0)
    {
      container->streamsReceived.push_back(stream);
    }

    stream->registerReceiver(this->communication);
  }
}

void EngineState::clearContainer(std::shared_ptr<CooperationContainer> container)
{
  // clear sub model
  container->subModel.reset();

  // clear nodes
  for (auto node : this->nodesActivated)
  {
    node->unregisterEngine(this->shared_from_this());
  }

  // clear streams send
  for (int i = 0; i < container->streamsSend.size(); ++i)
  {
    auto stream = container->streamsSend[i];

    stream->unregisterEngineState(this->shared_from_this());
  }
  container->streamsSend.clear();

  // clear streams received
  for (int i = 0; i < container->streamsReceived.size(); ++i)
  {
    auto stream = container->streamsReceived[i];

    stream->dropReceiver();
  }
  container->streamsReceived.clear();
}

} /* namespace ice */
