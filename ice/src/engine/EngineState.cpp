/*
 * EngineState.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: sni
 */

#include <ice/coordination/CooperationRequest.h>
#include <ice/coordination/CooperationResponse.h>
#include <ice/coordination/EngineState.h>
#include <ice/ICEngine.h>
#include <ice/Logger.h>
#include <ice/TimeFactory.h>
#include "easylogging++.h"

namespace ice
{

EngineState::EngineState(identifier engineId, std::weak_ptr<ICEngine> engine) :
    engineId(engineId), systemIri(systemIri)
{
  this->engine = engine;
  this->cooperationState = CooperationState::UNKNOWN;
  this->timeFactory = engine.lock()->getTimeFactory();
  this->timeLastActivity = this->timeFactory->createTime();
  this->timeLastStateUpdate = NO_TIME;
  this->retryCounter = 0;
  this->_log = el::Loggers::getLogger("EngineState");
}

EngineState::EngineState(std::string const systemIri, std::weak_ptr<ICEngine> engine) :
    systemIri(systemIri)
{
  this->engine = engine;
  this->cooperationState = CooperationState::UNKNOWN;
  this->timeFactory = engine.lock()->getTimeFactory();
  this->timeLastActivity = this->timeFactory->createTime();
  this->timeLastStateUpdate = NO_TIME;
  this->retryCounter = 0;
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

const std::string EngineState::getSystemIriShort() const
{
  int index1 = this->systemIri.find("#");

  if (index1 == std::string::npos)
    return this->systemIri;

  std::string name = this->systemIri.substr(index1 + 1, this->systemIri.size() - index1 - 1);
  name[0] = std::tolower(name[0]);
  return name;
}

const std::shared_ptr<InformationModel> EngineState::getInformationModel() const
{
  return this->informationModel;
}

void EngineState::setInformationModel(const std::shared_ptr<InformationModel> informationModel)
{
  this->informationModel = informationModel;
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

const std::vector<std::shared_ptr<IntersectionInformationModel>>* EngineState::getIntersections() const
{
  return &this->intersections;
}

const CooperationState EngineState::getCooperationState() const
{
  return this->cooperationState;
}

void EngineState::setCooperationState(CooperationState cooperationState)
{
  _log->debug("Update cooperation state of engine %v from %v to %v",
              IDGenerator::toString(this->engineId).c_str(), this->cooperationState, cooperationState);
  this->cooperationState = cooperationState;
  this->timeLastStateUpdate = this->timeFactory->createTime();
}

const time EngineState::getTimeLastStateUpdate() const
{
  return this->timeLastStateUpdate;
}

const std::shared_ptr<CooperationRequest> EngineState::getCooperationRequest() const
{
  return cooperationRequest;
}

void EngineState::setCooperationRequest(const std::shared_ptr<CooperationRequest> cooperationRequest)
{
  this->cooperationRequest = cooperationRequest;
}

const std::shared_ptr<CooperationResponse> EngineState::getCooperationResponse() const
{
  return cooperationResponse;
}

void EngineState::setCooperationResponse(const std::shared_ptr<CooperationResponse> cooperationResponse)
{
  this->cooperationResponse = cooperationResponse;
}

std::vector<std::shared_ptr<BaseInformationStream>>* EngineState::getStreamsOffered()
{
  return &this->streamsOffered;
}

std::vector<std::shared_ptr<BaseInformationStream>>* EngineState::getStreamsRequested()
{
  return &this->streamsRequested;
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

std::shared_ptr<ASPElement> EngineState::getASPElementByName(ASPElementType type, std::string const name)
{
  switch (type)
  {
    case ASP_COMPUTATION_NODE:
      for (auto node : this->aspNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_SOURCE_NODE:
      for (auto node : this->aspSourceNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_IRO_NODE:
      for (auto node : this->aspIro)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_REQUIRED_STREAM:
      for (auto node : this->aspRequiredStreams)
      {
        if (node->name == name)
          return node;
      }
      break;
  }

  return nullptr;
}

std::shared_ptr<ASPElement> EngineState::getASPElementByName(std::string const name)
{
  for (auto node : this->aspNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspSourceNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspIro)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspRequiredStreams)
  {
    if (node->name == name)
      return node;
  }

  return nullptr;
}

void EngineState::addASPElement(std::shared_ptr<ASPElement> node)
{
  switch (node->type)
  {
    case ASP_COMPUTATION_NODE:
      this->aspNodes.push_back(node);
      break;
    case ASP_SOURCE_NODE:
      this->aspSourceNodes.push_back(node);
      break;
    case ASP_IRO_NODE:
      this->aspIro.push_back(node);
      break;
    case ASP_REQUIRED_STREAM:
      this->aspRequiredStreams.push_back(node);
      break;
  }
}

std::vector<std::shared_ptr<EngineConnection>> EngineState::getConnections()
{
  return this->connections;
}

} /* namespace ice */
