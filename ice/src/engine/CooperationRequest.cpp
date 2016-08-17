/*
 * CooperationRequest.cpp
 *
 *  Created on: 04.08.2016
 *      Author: sni
 */

#include "ice/communication/jobs/CooperationRequest.h"
#include "ice/communication/messages/SubModelMessage.h"
#include "ice/communication/messages/SubModelResponseMessage.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"

#include "ice/model/ProcessingModel.h"

namespace ice
{
int CooperationRequest::ID = 3;
int CooperationRequestCreator::val = CooperationRequestCreator::init();

CooperationRequest::CooperationRequest(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity) :
        ComJob(CooperationRequest::ID, engine, entity, el::Loggers::getLogger("CooperationRequest")), tryCount(0),
        stateCR(CooperationRequestState::CRS_UNKNOWN)
{
  auto e = engine.lock();
  this->updateStrategy = e->getUpdateStrategie();
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
      this->onSubModelResponse(std::static_pointer_cast<SubModelResponseMessage>(message));
      break;
    }
//    case (IMI_CANCLE_JOB):
//    {
//     //TODO
//      break;
//    }
    case (IMI_FINISH):
    {
      if (false == this->ownJob)
      {
        _log->info("Received finished from '%v'", entity->toString());
        this->finish();//onFinished(message);
      }

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
  m->setSubModel(this->subModel);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

void CooperationRequest::onSubModel(std::shared_ptr<SubModelMessage> message)
{
  _log->info("Submodel received from %v", entity->toString());

  if (this->stateCR != CooperationRequestState::CRS_UNKNOWN && this->stateCR != CooperationRequestState::CRS_SUBMODEL)
  {
    // received message in wrong state
    _log->warn("Received submodel message in wrong state '%v' from '%v'", this->stateCR, entity->toString());
    return;
  }
  else if (this->stateCR == CooperationRequestState::CRS_SUBMODEL &&
      false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    // duplicated message
    _log->debug("Received duplicated submodel message from '%v'", entity->toString());
    return;
  }

  this->stateCR = CooperationRequestState::CRS_SUBMODEL;
  auto &sharedSubModel = entity->getReceivedSubModel();

  // check if specification was already received
  if (sharedSubModel.subModel != nullptr
      && sharedSubModel.subModel->index == message->getSubModel()->index)
  {
    _log->info("Received duplicated submodel message from '%v'", entity->toString());

    auto m = std::make_shared<SubModelResponseMessage>(message->getSubModel()->index, sharedSubModel.accepted);
    this->send(m);
    this->state = CJState::CJ_WAITING;
    this->updateActiveTime();

    return;
  }

  sharedSubModel.accepted = this->updateStrategy->handleSubModel(entity, message->getSubModel());

  if (sharedSubModel.accepted)
  {
    _log->info("Submodel from %v processed, sending acknowledgment", entity->toString());
  }
  else
  {
    _log->info("Submodel from %v could not be processed", entity->toString());
  }

  auto m = std::make_shared<SubModelResponseMessage>(message->getSubModel()->index, sharedSubModel.accepted);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

void CooperationRequest::onSubModelResponse(std::shared_ptr<SubModelResponseMessage> message)
{
  _log->info("Submodel response received from %v", entity->toString());

  if (this->stateCR != CooperationRequestState::CRS_SUBMODEL
      && this->stateCR != CooperationRequestState::CRS_SUBMODEL_RESPONSE)
  {
    // received message in wrong state
    _log->warn("Received submodel response in wrong state '%v' from '%v'", this->stateCR, entity->toString());
    return;
  }
  else if (this->stateCR == CooperationRequestState::CRS_SUBMODEL_RESPONSE
      && false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    // duplicated message
    _log->debug("Received duplicated submodel message from '%v'", entity->toString());
    return;
  }

  auto &sharedSubModel = entity->getSendSubModel();

  // check index
  if(sharedSubModel.subModel != nullptr && sharedSubModel.subModel->index != message->getIndex())
  {
    _log->info("Received submodel response from '%v' for index '%v' instead of '%v'", entity->toString(),
               message->getIndex(), (sharedSubModel.subModel != nullptr ? sharedSubModel.subModel->index : -1));
    // will be ignored
    return;
  }

  this->stateCR = CooperationRequestState::CRS_SUBMODEL_RESPONSE;
  sharedSubModel.accepted = message->getResult();

  if (sharedSubModel.accepted)
  {
    bool result = this->updateStrategy->handleSubModelResponse(entity, index);

    if (result)
    {
      _log->info("Submodel response from '%v' processed, sending finished", entity->toString());
      this->finish();
    }
    else
    {
      _log->info("Submodel response from '%v' could not be processed, sending cancel", entity->toString());
      this->abort();
    }
  }
  else
  {
    _log->info("Submodel request from '%v' was denied, sending finished", entity->toString());
    this->finish();
    // TODO
  }
}

void CooperationRequest::onFinished(std::shared_ptr<SubModelResponseMessage> message)
{
  _log->info("Finished received from '%v'", entity->toString());

  // check if current request is ok at current cooperation state
  // TODO

  // check if specification was already received
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
