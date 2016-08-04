/*
 * CooperationRequest.cpp
 *
 *  Created on: 04.08.2016
 *      Author: sni
 */

#include "ice/communication/jobs/CooperationRequest.h"
#include "ice/communication/messages/SubModelMessage.h"

#include "ice/model/ProcessingModel.h"

namespace ice
{
int CooperationRequest::ID = 3;
int CooperationRequestCreator::val = IdentityRequestCreator::init();

CooperationRequest::CooperationRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity) :
        ComJob(IdentityRequest::ID, engine, entity, el::Loggers::getLogger("CooperationRequest")), tryCount(0),
        stateCR(CooperationRequestState::CRS_UNKNOWN)
{
  //
}

CooperationRequest::~CooperationRequest()
{
  //
}

void CooperationRequest::init()
{
  // call init from super class
  ComJobBase::init();
  this->sendSubModel();
}

void CooperationRequest::tick()
{
  if (this->timestampLastActive == 0 || this->state != CJState::CJ_WAITING)
  {
    return;
  }

  if (this->timeFactory->checkTimeout(this->timestampLastActive, 2000))
  {
    if (++this->tryCount < 3)
    {
      // TODO
      // retry
      this->sendSubModel();
    }
    else
    {
      entity->checkIce();
      this->abort();
    }
  }
}

void CooperationRequest::handleMessage(std::shared_ptr<Message> const &message)
{
  switch (message->getId())
  {
    case (IMI_SUBMODEL):
    {
      this->onSubModel(std::static_pointer_cast<SubModelMessage>(message));
      break;
    }
    case (IMI_SUBMODEL_RESPONSE):
    {

      break;
    }
    case (IMI_FINISH):
    {
      if (this->ownJob)
        this->finish();

      break;
    }
    default:
      ComJobBase::handleMessage(message);
      break;
  }
}

void CooperationRequest::setSubModelDesc(std::shared_ptr<SubModelDesc> &model)
{
  this->subModel = model;
}

void CooperationRequest::sendSubModel()
{
  _log->info("Send submodel to '%v'", entity->toString());
  this->stateCR = CooperationRequestState::CRS_SUBMODEL;

  auto m = std::make_shared<SubModelMessage>();
  m->setSubModel(*this->subModel);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

void CooperationRequest::onSubModel(std::shared_ptr<SubModelMessage> message)
{

//  _log->debug("Sub model request received from %v", IDGenerator::toString(engineId));
//
//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);
//
//  // check if current request is ok at current cooperation state
//  // TODO
//
//  // check if specification was already received
//  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RECEIVED)
//  {
//    _log->info("Duplicated sub model description received from engine %v", IDGenerator::toString(engineId));
//    // TODO again?
//  }
//
//  bool result = this->updateStrategie->handleSubModel(engineState, modelDesc);
//
//  if (result)
//  {
//    _log->info("Sub model from engine %v processed, sending acknowledgment", IDGenerator::toString(engineId));
//    this->communication->sendSubModelResponse(engineId, modelDesc.index, true);
//
//    return 0;
//  }
//  else
//  {
//    _log->info("Sub model from engine %v could not be processed", IDGenerator::toString(engineId));
//    engineState->getOffering()->state = CooperationState::NO_COOPERATION;
//    this->communication->sendSubModelResponse(engineId, modelDesc.index, false);
//
//    return 0;
//  }
}

int CooperationRequest::onSubModelResponse(identifier engineId, int index, bool accept)
{
//  if (false == this->running)
    return 5;

//  _log->debug("Sub model response received from '%v' with index '%v' and answer '%v'",
//              IDGenerator::toString(engineId), index, accept);
//
//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);
//
//  // check if engine is known
//  if (false == engineState)
//  {
//    _log->info("Sub model response received from unknown engine %v", IDGenerator::toString(engineId));
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    return 1;
//  }
//  engineState->updateTimeLastActivity();
//
//  // check if current request is ok at current cooperation state
//  // TODO
//
//  // check if specification was already received
//  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RESPONSE_RECEIVED)
//  {
//    _log->info("Duplicated sub model response received from engine %v", IDGenerator::toString(engineId));
//    // will be ignored
//
//    return 1;
//  }
//
//  if (accept)
//  {
//    auto result = this->updateStrategie->handleSubModelResponse(engineState, index);
//
//    if (result)
//    {
//      _log->info("Sub model response from engine %v processed, sending negotiation finished", IDGenerator::toString(engineId));
//      this->communication->sendNegotiationFinished(engineId);
//      engineState->getRequesting()->state = CooperationState::COOPERATION;
//
//      return 0;
//    }
//    else
//    {
//      _log->info("Sub model response from engine %v could not be processed, sending stop cooperation", IDGenerator::toString(engineId));
//      engineState->getRequesting()->state = CooperationState::NO_COOPERATION;
//      this->communication->sendStopCooperation(engineId);
//      // TODO create new model
//
//      return 1;
//    }
//  }
//  else
//  {
//    _log->info("Sub model request from engine '%v' was denied, sending stop cooperation",
//               IDGenerator::toString(engineId));
//    engineState->getRequesting()->state = CooperationState::NO_COOPERATION;
//    this->communication->sendStopCooperation(engineId);
//    // TODO create new model
//    return 1;
//  }
}

int CooperationRequest::onNegotiationFinished(identifier engineId)
{
//  if (false == this->running)
    return 5;

//  _log->debug("Negotiation finished received from '%v'", IDGenerator::toString(engineId));
//
//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);
//
//  // check if engine is known
//  if (false == engineState)
//  {
//    _log->info("Negotiation finished received from unknown engine %v", IDGenerator::toString(engineId));
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    return 1;
//  }
//
//  // check if current request is ok at current cooperation state
//  // TODO
//
//  // check if specification was already received
//  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RECEIVED)
//  {
//    _log->info("Duplicated negotiation finished received from engine %v", IDGenerator::toString(engineId));
//  }
//
//
//  _log->info("Negotiation finished received from engine '%v'", IDGenerator::toString(engineId));
////  this->communication->sendNegotiationFinished(engineId);
//  engineState->getOffering()->state = CooperationState::COOPERATION;
//
//  return 0;
}

} /* namespace ice */
