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

namespace ice
{

EngineState::EngineState(identifier engineId, std::weak_ptr<ICEngine> engine) :
    engineId(engineId)
{
  this->engine = engine;
  this->cooperationState = CooperationState::UNKNOWN;
  this->timeFactory = engine.lock()->getTimeFactory();
  this->timeLastActivity = this->timeFactory->createTime();
  this->timeLastStateUpdate = NO_TIME;
  this->retryCounter = 0;
  this->_log = Logger::get("EngineState");
}

EngineState::~EngineState()
{
  //
}

const identifier EngineState::getEngineId() const
{
  return this->engineId;
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
  _log->debug("setCooperationState", "Update cooperation state of engine %s from %i to %i",
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

} /* namespace ice */
