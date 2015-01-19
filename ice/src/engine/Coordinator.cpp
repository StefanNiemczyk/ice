/*
 * Coordinator.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/Coordinator.h"

#include "ice/Configuration.h"
#include "ice/ICEngine.h"
#include "ice/TimeFactory.h"
#include "ice/communication/Communication.h"
#include "ice/coordination/EngineState.h"
#include "ice/coordination/CooperationRequest.h"
#include "ice/coordination/CooperationResponse.h"
#include "ice/coordination/InformationModel.h"
#include "ice/coordination/IntersectionInformationModel.h"
#include "ice/coordination/ModelComperator.h"
#include "ice/information/InformationStore.h"
#include "easylogging++.h"

namespace ice
{

Coordinator::Coordinator(std::weak_ptr<ICEngine> engine)
{
  this->_log = el::Loggers::getLogger("Coordinator");
  this->engine = engine;
  this->running = false;

  _log->verbose(1, "Constructor called");
}

Coordinator::~Coordinator()
{
  _log->verbose(1, "Destructor called");
  this->running = false;
  this->worker.join();
}

void Coordinator::init()
{
  _log->verbose(1, "Init called");

  auto e = engine.lock();
  this->communication = e->getCommunication();
  this->informationStore = e->getInformationStore();
  this->config = e->getConfig();
  this->timeFactory = e->getTimeFactory();

  // create worker thread
  this->running = true;
  this->worker = std::thread(&Coordinator::workerTask, this);
}

void Coordinator::cleanUp()
{
  _log->verbose(1, "Clean up called");
  std::lock_guard<std::mutex> guard(mtx_);

  for (auto engineState : this->engineStates)
  {
    this->stopCooperationWithEngine(engineState, true);
  }

  this->running = false;
  this->communication.reset();
  this->informationStore.reset();

  this->cv.notify_all();
}

std::shared_ptr<EngineState> Coordinator::getEngineState(identifier engineId)
{
  //std::lock_guard<std::mutex> guard(mtx_);

  for (auto engineState : this->engineStates)
  {
    if (engineState->getEngineId() == engineId)
      return engineState;
  }

  std::shared_ptr<EngineState> ptr;
  return ptr;
}

int Coordinator::onEngineHeartbeat(identifier engineId, time timestamp)
{
  if (false == this->running)
    return 5;

  _log->verbose(1, "New heartbeat from %v with %v", IDGenerator::toString(engineId).c_str(),
                timestamp);

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (engineState)
  {
    if (engineState->getCooperationState() == CooperationState::UNKNOWN)
    {
      _log->info("Engine rediscovered %v, sending information model request",
                 IDGenerator::toString(engineId).c_str());

      engineState->setTimeLastActivity(timestamp);
      engineState->setCooperationState(CooperationState::INFORMATION_MODEL_REQUESTED);

      this->communication->sendInformationRequest(engineId);

      return 0;
    }

    engineState->setTimeLastActivity(timestamp);
    _log->verbose(1, "Update lastActiveTime of %v with %v", IDGenerator::toString(engineId).c_str(),
                  timestamp);
    return 0;
  }

  _log->info("New engine discovered %v", IDGenerator::toString(engineId).c_str());

  engineState = std::make_shared<EngineState>(engineId, this->engine);
  this->engineStates.push_back(engineState);

  engineState->setTimeLastActivity(timestamp);
  engineState->setCooperationState(CooperationState::INFORMATION_MODEL_REQUESTED);

  this->communication->sendInformationRequest(engineId);

  return 0;
}

int Coordinator::onInformationModelRequest(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Information model request from %v",
              IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Information model request from unknown engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    engineState->setCooperationState(CooperationState::UNKNOWN);
  }

  if (engineState->getCooperationState() == CooperationState::INFORMATION_MODEL_SEND)
  {
    _log->info("Duplicated information model request received from engine %v",
               IDGenerator::toString(engineId).c_str());

    auto model = this->engine.lock()->getInformationModel();
    this->communication->sendInformationModel(engineId, model);
    engineState->updateTimeLastActivity();
    return 0;
  }

  if (engineState->getCooperationState() != CooperationState::INFORMATION_MODEL_SEND
      && engineState->getCooperationState() != CooperationState::UNKNOWN
      && engineState->getCooperationState() != CooperationState::INFORMATION_MODEL_REQUESTED
      && engineState->getCooperationState() != CooperationState::RETRY_NEGOTIATION)
  {
    _log->warn("Information model request from engine %v in wrong state %v",
                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());

    return 1;
  }

  if (engineState->getCooperationState() == CooperationState::INFORMATION_MODEL_REQUESTED)
  {
    if (IDGenerator::getInstance()->compare(this->engine.lock()->getId(), engineId) < 0)
    {
      _log->info("Overlapping information model request from %v discarded because of lower id",
                 IDGenerator::toString(engineId).c_str());
      return 2;
    }
  }

  auto model = this->engine.lock()->getInformationModel();
  this->communication->sendInformationModel(engineId, model);
  engineState->updateTimeLastActivity();
  engineState->setCooperationState(CooperationState::INFORMATION_MODEL_SEND);

  return 0;
}

int Coordinator::onInformationModel(identifier engineId, std::shared_ptr<InformationModel> informationModel)
{
  if (false == this->running)
    return 5;

  _log->debug("Information model received from %v", IDGenerator::toString(engineId).c_str());

//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto engineState = this->getEngineState(engineId);
//
//  if (false == engineState)
//  {
//    _log->info("Information model received from unknown engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  if (engineState->getCooperationState() == CooperationState::COOPERATION_REQUEST_SEND)
//  {
//    _log->info("Duplicated information model received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    this->communication->sendCooperationRequest(engineId, engineState->getCooperationRequest());
//    engineState->updateTimeLastActivity();
//    return 0;
//  }
//
//  if (engineState->getCooperationState() != CooperationState::INFORMATION_MODEL_REQUESTED)
//  {
//    _log->warn("Information model received from engine %v in wrong state %v",
//                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  // computing model intersections
//  auto engineSptr = this->engine.lock();
//  auto ownModel = engineSptr->getInformationModel();
//  auto request = std::make_shared<CooperationRequest>(engineSptr->getId());
//
//  this->modelComperator.findOfferesAndRequests(ownModel, informationModel, request->getOffers(),
//                                               request->getRequests());
//  auto compareResult = this->modelComperator.findModelMatches(ownModel, informationModel);
//
//  engineState->setCooperationRequest(request);
//
//  if (request->isEmpty())
//  {
//    _log->info("Empty Cooperation request, no cooperation with engine %v, refuse send",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->setCooperationState(CooperationState::COOPERATION_REFUSE_SEND);
//    engineState->updateTimeLastActivity();
//
//    this->communication->sendCooperationRefuse(engineId);
//
//    return 0;
//  }
//
//  this->communication->sendCooperationRequest(engineId, request);
//
//  engineState->setCooperationState(CooperationState::COOPERATION_REQUEST_SEND);
//  engineState->updateTimeLastActivity();
//
//  _log->info("Cooperation request sends to engine %v", IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onCooperationRequest(identifier engineId, std::shared_ptr<CooperationRequest> request)
{
  if (false == this->running)
    return 5;

  _log->debug("Cooperation request received from %v", IDGenerator::toString(engineId).c_str());

//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto engineState = this->getEngineState(engineId);
//
//  if (false == engineState)
//  {
//    _log->info("Cooperation request received from unknown engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  if (engineState->getCooperationState() == CooperationState::COOPERATION_RESPONSE_SEND)
//  {
//    _log->info("Duplicated cooperation request received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->updateTimeLastActivity();
//    this->communication->sendCooperationResponse(engineId, engineState->getCooperationResponse());
//    return 0;
//  }
//  else if (engineState->getCooperationState() == CooperationState::COOPERATION_REFUSE_SEND)
//  {
//    _log->info("Duplicated cooperation request received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->updateTimeLastActivity();
//    this->communication->sendCooperationRefuse(engineId);
//    return 0;
//  }
//
//  if (engineState->getCooperationState() != CooperationState::INFORMATION_MODEL_SEND)
//  {
//    _log->warn("Cooperation request received from engine %v in wrong state %v",
//                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  // save the request
//  engineState->setCooperationRequest(request);
//
//  // Accepting cooperation request
//  auto engineSptr = this->engine.lock();
//  auto response = std::make_shared<CooperationResponse>(engineSptr->getId());
//
//  // Check requests
//  for (auto requestedStream : *request->getRequests())
//  {
//    auto stream = this->informationStore->getBaseStream(requestedStream);
//
//    if (false == stream)
//    {
//      _log->error("Unknown stream requested %v from engine %v, request partial ignored",
//                  requestedStream->getId(), IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    response->getRequestsAccepted()->push_back(requestedStream);
//  }
//
//  // Check offers
//  for (auto offeredStream : *request->getOffers())
//  {
//    auto type = this->informationStore->getInformationType(offeredStream->getId());
//
//    if (false == type->existsStreamTemplate(offeredStream))
//    {
//      _log->error("Unknown stream offered %v from engine %v, offer partial ignored",
//                  offeredStream->getId(), IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    response->getOffersAccepted()->push_back(offeredStream);
//  }
//
//  // Check if no cooperation will be established
//  if (response->isEmpty())
//  {
//    _log->info("Empty Cooperation response, no cooperation with engine %v, refuse send",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->setCooperationState(CooperationState::COOPERATION_REFUSE_SEND);
//    engineState->updateTimeLastActivity();
//
//    this->communication->sendCooperationRefuse(engineId);
//
//    return 0;
//  }
//
//  this->communication->sendCooperationResponse(engineId, response);
//
//  engineState->setCooperationResponse(response);
//  engineState->setCooperationState(CooperationState::COOPERATION_RESPONSE_SEND);
//  engineState->updateTimeLastActivity();
//
//  _log->info("Cooperation response sends to engine %v",
//             IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onCooperationResponse(identifier engineId, std::shared_ptr<CooperationResponse> response)
{
  if (false == this->running)
    return 5;

  _log->debug("Cooperation response received from %v",
              IDGenerator::toString(engineId).c_str());

//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto engineState = this->getEngineState(engineId);
//
//  if (false == engineState)
//  {
//    _log->info("Cooperation response received from unknown engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  if (engineState->getCooperationState() == CooperationState::COOPERATION_ACCEPT_SEND
//      && (response->getOffersAccepted()->size() > 0 || response->getRequestsAccepted()->size() > 0))
//  {
//    _log->info("Duplicated cooperation response received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->updateTimeLastActivity();
//    this->communication->sendCooperationAccept(engineId);
//    return 0;
//  }
//  else if (engineState->getCooperationState() == CooperationState::NO_COOPERATION
//      && response->getOffersAccepted()->size() == 0 && response->getRequestsAccepted()->size() == 0)
//  {
//    _log->info("Duplicated cooperation response received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->updateTimeLastActivity();
//    this->communication->sendNegotiationFinished(engineId);
//    return 0;
//  }
//
//  if (engineState->getCooperationState() != CooperationState::COOPERATION_REQUEST_SEND)
//  {
//    _log->warn("Cooperation response received from engine %v in wrong state %v",
//                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  // save the response
//  engineState->setCooperationResponse(response);
//
//  // Accepting cooperation request
//  auto engineSptr = this->engine.lock();
//
//  // No cooperation
//  if (response->getOffersAccepted()->size() == 0 && response->getRequestsAccepted()->size() == 0)
//  {
//    _log->info("Empty cooperation response received from engine %v results in no cooperation",
//               IDGenerator::toString(engineId).c_str());
//
//    this->communication->sendNegotiationFinished(engineId);
//    engineState->setCooperationState(CooperationState::NO_COOPERATION);
//    engineState->updateTimeLastActivity();
//
//    return 1;
//  }
//
//  // Check accepted requests
//  for (auto requestedStreameAccepted : *response->getRequestsAccepted())
//  {
//    auto type = this->informationStore->getInformationType(requestedStreameAccepted->getId());
//    auto stream = type->createStreamFromTemplate(requestedStreameAccepted, IDGenerator::toString(engineId));
//
//    if (false == stream)
//    {
//      _log->error("Unknown stream %v from this engine requested from engine %v, request partial ignored",
//                  IDGenerator::toString(requestedStreameAccepted->getId()).c_str(),
//                  IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    engineState->getStreamsRequested()->push_back(stream);
//    stream->registerReceiver(this->communication);
//  }
//
//  // Check accepted offers
//  for (auto offeredStreamsAccepted : *response->getOffersAccepted())
//  {
//    auto stream = this->informationStore->getBaseStream(offeredStreamsAccepted);
//
//    if (false == stream)
//    {
//      _log->error("Unknown stream  %v from this engine offered to engine %v, offer partial ignored",
//                  IDGenerator::toString(offeredStreamsAccepted->getId()).c_str(),
//                  IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    if (stream->registerEngineState(engineState) == 0)
//    {
//      engineState->getStreamsOffered()->push_back(stream);
//    }
//
//    stream->registerSender(this->communication);
//  }
//
//  this->communication->sendCooperationAccept(engineId);
//
//  engineState->setCooperationState(CooperationState::COOPERATION_ACCEPT_SEND);
//  engineState->updateTimeLastActivity();
//
//  _log->info("Cooperation accept sends to engine %v", IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onCooperationAccept(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Cooperation accept received from %v", IDGenerator::toString(engineId).c_str());

//  std::lock_guard<std::mutex> guard(mtx_);
//
//  auto engineState = this->getEngineState(engineId);
//
//  if (false == engineState)
//  {
//    _log->info("Cooperation accept received from unknown engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState = std::make_shared<EngineState>(engineId, this->engine);
//    this->engineStates.push_back(engineState);
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  if (engineState->getCooperationState() == CooperationState::COOPERATION)
//  {
//    _log->info("Duplicated cooperation accept received from engine %v",
//               IDGenerator::toString(engineId).c_str());
//
//    engineState->updateTimeLastActivity();
//    this->communication->sendNegotiationFinished(engineId);
//    return 0;
//  }
//
//  if (engineState->getCooperationState() != CooperationState::COOPERATION_RESPONSE_SEND)
//  {
//    _log->warn("Cooperation accept received from engine %v in wrong state %v",
//                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());
//
//    this->communication->sendRetryNegotiation(engineId);
//    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//
//    return 1;
//  }
//
//  // adding stream sending and listening
//  auto response = engineState->getCooperationResponse();
//
//  // Check requests
//  for (auto requestedStream : *response->getRequestsAccepted())
//  {
//    auto stream = this->informationStore->getBaseStream(requestedStream);
//
//    if (false == stream)
//    {
//      _log->error("Unknown stream requested %v from engine %v, request part ignored",
//                  requestedStream->getId(), IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    if (stream->registerEngineState(engineState) == 0)
//    {
//      engineState->getStreamsOffered()->push_back(stream);
//    }
//
//    stream->registerSender(this->communication);
//  }
//
//  // Check offers
//  for (auto offeredStream : *response->getOffersAccepted())
//  {
//    auto type = this->informationStore->getInformationType(offeredStream->getId());
//    auto stream = type->createStreamFromTemplate(offeredStream, IDGenerator::toString(engineId));
//
//    if (false == stream)
//    {
//      _log->error("Unknown stream offer %v from engine %v, request part ignored",
//                  offeredStream->getId(), IDGenerator::toString(engineId).c_str());
//      continue;
//    }
//
//    engineState->getStreamsRequested()->push_back(stream);
//    stream->registerReceiver(this->communication);
//  }
//
//  engineState->setCooperationState(CooperationState::COOPERATION);
//  engineState->updateTimeLastActivity();
//  this->communication->sendNegotiationFinished(engineId);
//
//  _log->info("Cooperation accept received to engine %v",
//             IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onCooperationRefuse(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Cooperation refuse received from %v", IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Cooperation refuse received from unknown engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 1;
  }

  if (engineState->getCooperationState() == CooperationState::NO_COOPERATION)
  {
    _log->info("Duplicated cooperation refuse received from engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState->updateTimeLastActivity();
    return 0;
  }

  if (engineState->getCooperationState() != CooperationState::COOPERATION_RESPONSE_SEND
      && engineState->getCooperationState() != CooperationState::COOPERATION_REQUEST_SEND
      && engineState->getCooperationState() != CooperationState::INFORMATION_MODEL_SEND)
  {
    _log->warn("Cooperation refuse received from engine %v in wrong state %v",
                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 1;
  }

  engineState->setCooperationState(CooperationState::NO_COOPERATION);
  engineState->updateTimeLastActivity();
  this->communication->sendNegotiationFinished(engineId);

  _log->info("Cooperation refuse received to engine %v",
             IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onNegotiationFinished(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Negotiation finished received from %v",
              IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Negotiation finished received from unknown engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 1;
  }

  if (engineState->getCooperationState() == CooperationState::NO_COOPERATION
      || engineState->getCooperationState() == CooperationState::COOPERATION)
  {
    _log->info("Duplicated negotiation finished received from engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState->updateTimeLastActivity();
    return 0;
  }

  if (engineState->getCooperationState() != CooperationState::COOPERATION_REFUSE_SEND
      && engineState->getCooperationState() != CooperationState::COOPERATION_ACCEPT_SEND)
  {
    _log->warn("Negotiation finished received from engine %v in wrong state %v",
                  IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 1;
  }

  if (engineState->getCooperationState() == CooperationState::COOPERATION_REFUSE_SEND)
    engineState->setCooperationState(CooperationState::NO_COOPERATION);
  else
    engineState->setCooperationState(CooperationState::COOPERATION);
  engineState->updateTimeLastActivity();

  _log->info("Negotiation finished received from engine %v",
             IDGenerator::toString(engineId).c_str());

  return 0;
}

int Coordinator::onStopCooperation(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->info("Stop cooperation received from engine %v", IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Stop cooperation received from unknown engine %v, message discarded",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 0;
  }

  this->stopCooperationWithEngine(engineState, false);
  engineState->setCooperationState(CooperationState::NO_COOPERATION);
  engineState->updateTimeLastActivity();

  this->communication->sendCooperationStopped(engineId);

  return 0;
}

int Coordinator::onCooperationStopped(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->info("Cooperation stopped received from engine %v",
             IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Cooperation stopped received from unknown engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    this->communication->sendRetryNegotiation(engineId);
    engineState->setCooperationState(CooperationState::RETRY_NEGOTIATION);

    return 0;
  }

  if (engineState->getCooperationState() == CooperationState::NO_COOPERATION)
  {
    _log->info("Duplicated cooperation stopped received from engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState->updateTimeLastActivity();
    return 0;
  }

  if (engineState->getCooperationState() != CooperationState::STOP_COOPERATION_SEND)
  {
    _log->info("Cooperation stopped received from engine %v in wrong state %v",
               IDGenerator::toString(engineId).c_str(), engineState->getCooperationState());

    return 0;
  }

  engineState->setCooperationState(CooperationState::NO_COOPERATION);
  engineState->updateTimeLastActivity();

  return 0;
}

int Coordinator::onRetryNegotiation(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Retry negotiation with engine %v", IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("Retry negotiation from unknown engine %v",
               IDGenerator::toString(engineId).c_str());

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    engineState->setCooperationState(CooperationState::UNKNOWN);
  }

  std::shared_ptr<CooperationRequest> requestPtr;
  engineState->setCooperationRequest(requestPtr);

  std::shared_ptr<CooperationResponse> responsePtr;
  engineState->setCooperationResponse(responsePtr);

  engineState->setCooperationState(CooperationState::INFORMATION_MODEL_REQUESTED);
  engineState->updateTimeLastActivity();

  this->communication->sendInformationRequest(engineId);

  return 0;
}

int Coordinator::stopCooperationWith(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Stop cooperation with engine %v", IDGenerator::toString(engineId).c_str());

  std::lock_guard<std::mutex> guard(mtx_);

  auto engineState = this->getEngineState(engineId);

  if (false == engineState)
  {
    _log->info("No engine with engine id %v", IDGenerator::toString(engineId).c_str());

    return 1;
  }

  this->stopCooperationWithEngine(engineState, true);

  return 1;
}

void Coordinator::stopCooperationWithEngine(std::shared_ptr<EngineState> engineState, bool sendStop)
{
  if (engineState->getCooperationState() == CooperationState::STOP_COOPERATION_SEND
      || engineState->getCooperationState() == CooperationState::NO_COOPERATION)
  {
    return;
  }

  _log->debug("Stop cooperation with engine %v",
              IDGenerator::toString(engineState->getEngineId()).c_str());

  // sender
  for (auto stream : *engineState->getStreamsOffered())
  {
    stream->unregisterEngineState(engineState);
  }

  // receiver
  for (auto stream : *engineState->getStreamsRequested())
  {
    stream->dropReceiver();
  }

  std::shared_ptr<CooperationRequest> requestPtr;
  engineState->setCooperationRequest(requestPtr);

  std::shared_ptr<CooperationResponse> responsePtr;
  engineState->setCooperationResponse(responsePtr);

  if (sendStop)
  {
    engineState->setCooperationState(CooperationState::STOP_COOPERATION_SEND);
    engineState->updateTimeLastActivity();

    this->communication->sendStopCooperation(engineState->getEngineId());
  }
}

void Coordinator::workerTask()
{
  // sleep to enalbe the engine to initialize
  {
    std::unique_lock<std::mutex> lock(mtx_);
    this->cv.wait_for(lock, std::chrono::milliseconds(1000));
  }

  int counter = 1;

  while (this->running)
  {
    {
      auto e = this->engine.lock();
      if (e)
        _log->verbose("Sending heartbeat %v, %v", counter, IDGenerator::toString(e->getId()).c_str());
    }

    {
      std::lock_guard<std::mutex> guard(mtx_);
      if (this->running)
        this->communication->sendHeartbeat();

      for (auto engine : this->engineStates)
      {
        _log->verbose("Checking engine %v in engine state %v since %v",
                      IDGenerator::toString(engine->getEngineId()).c_str(), engine->getCooperationState(),
                      engine->getTimeLastStateUpdate());

        if (engine->getCooperationState() == CooperationState::UNKNOWN)
        {
          continue;
        }

        if (engine->getCooperationState() == CooperationState::NO_COOPERATION
            || engine->getCooperationState() == CooperationState::COOPERATION)
        {
          if (this->timeFactory->checkTimeout(engine->getTimeLastActivity(), this->config->getHeartbeatTimeout()))
          {
            _log->info("Timeout of last activity of engine %v in state, stop cooperation",
                       engine->getCooperationState(), IDGenerator::toString(engine->getEngineId()).c_str(),
                       engine->getCooperationState());

            this->stopCooperationWithEngine(engine, false);
            engine->setCooperationState(CooperationState::UNKNOWN);
          }

          continue;
        }

        // check timeout, continue if timeout not reached
        if (false
            == this->timeFactory->checkTimeout(engine->getTimeLastStateUpdate(),
                                               this->config->getCoordinationMessageTimeout()))
        {
          continue;
        }

        engine->increaseRetryCounter();

        if (engine->getRetryCounter() >= this->config->getMaxRetryCount())
        {
          _log->info("Timeout in engine state %v with engine %v, reached max retry count",
                     engine->getCooperationState(), IDGenerator::toString(engine->getEngineId()).c_str(),
                     engine->getCooperationState());

          this->stopCooperationWithEngine(engine, false);
          engine->setCooperationState(CooperationState::UNKNOWN);
          continue;
        }
        else
        {
          _log->info("Timeout in engine state %v with engine %v, %v retry", engine->getCooperationState(),
                     IDGenerator::toString(engine->getEngineId()).c_str(), engine->getCooperationState(),
                     engine->getRetryCounter());
        }

        switch (engine->getCooperationState())
        {
          case (RETRY_NEGOTIATION):
            engine->setCooperationState(CooperationState::RETRY_NEGOTIATION);
            this->communication->sendRetryNegotiation(engine->getEngineId());
            break;
          case (INFORMATION_MODEL_REQUESTED):
            engine->setCooperationState(CooperationState::INFORMATION_MODEL_REQUESTED);
            this->communication->sendInformationRequest(engine->getEngineId());
            break;
          case (INFORMATION_MODEL_SEND):
            engine->setCooperationState(CooperationState::INFORMATION_MODEL_SEND);
            this->communication->sendInformationModel(engine->getEngineId(),
                                                      this->engine.lock()->getInformationModel());
            break;
          case (COOPERATION_REQUEST_SEND):
            engine->setCooperationState(CooperationState::COOPERATION_REQUEST_SEND);
            this->communication->sendCooperationRequest(engine->getEngineId(), engine->getCooperationRequest());
            break;
          case (COOPERATION_RESPONSE_SEND):
            engine->setCooperationState(CooperationState::COOPERATION_RESPONSE_SEND);
            this->communication->sendCooperationResponse(engine->getEngineId(), engine->getCooperationResponse());
            break;
          case (COOPERATION_REFUSE_SEND):
            engine->setCooperationState(CooperationState::COOPERATION_REFUSE_SEND);
            this->communication->sendCooperationRefuse(engine->getEngineId());
            break;
          case (COOPERATION_ACCEPT_SEND):
            engine->setCooperationState(CooperationState::COOPERATION_ACCEPT_SEND);
            this->communication->sendCooperationAccept(engine->getEngineId());
            break;
          case (NEGOTIATION_FINISHED_SEND):
            engine->setCooperationState(CooperationState::NEGOTIATION_FINISHED_SEND);
            this->communication->sendNegotiationFinished(engine->getEngineId());
            break;
          case (STOP_COOPERATION_SEND):
            engine->setCooperationState(CooperationState::STOP_COOPERATION_SEND);
            this->communication->sendStopCooperation(engine->getEngineId());
            break;
          default:
            _log->warn("Unknown cooperation state %v", engine->getCooperationState());
        }
      }
    }

    std::unique_lock<std::mutex> lock(mtx_);
    this->cv.wait_for(lock, std::chrono::milliseconds(2000));
    ++counter;
  }
}

} /* namespace ice */

