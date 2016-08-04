/*
 * Coordinator.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/coordination/Coordinator.h"

#include <tuple>

#include "ice/Configuration.h"
#include "ice/ICEngine.h"
#include "ice/communication/Communication.h"
#include "ice/coordination/EngineState.h"
#include "ice/coordination/CooperationRequest.h"
#include "ice/coordination/CooperationResponse.h"
#include "ice/coordination/InformationModel.h"
#include "ice/coordination/IntersectionInformationModel.h"
#include "ice/coordination/ModelComperator.h"
#include "ice/information/StreamStore.h"
#include "ice/model/updateStrategie/UpdateStrategie.h"
#include "ice/ontology/OntologyInterface.h"
#include "ice/processing/NodeStore.h"

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
}

void Coordinator::init()
{
  _log->verbose(1, "Init called");

  auto e = engine.lock();
  this->communication = e->getCommunication();
  this->streamStore = e->getStreamStore();
  this->config = e->getConfig();
  this->timeFactory = e->getTimeFactory();
  e->getSelf()->getId(EntityDirectory::ID_ONTOLOGY, this->engineIri);
  this->nodeStore = e->getNodeStore();
  this->ontologyInterface = e->getOntologyInterface();
  this->modelGenerator = e->getProcessingModelGenerator();
  this->updateStrategie = e->getUpdateStrategie();

//  std::string id;
//  e->getSelf()->getId(EntityDirectory::ID_ICE, id);
  this->self = e->getSelf();

  // create worker thread
  this->running = true;
  this->worker = std::thread(&Coordinator::workerTask, this);
}

void Coordinator::cleanUp()
{
  _log->verbose(1, "Clean up called");

  {
    std::lock_guard<std::mutex> guard(mtx_);

    if (this->running == false)
      return;

    this->running = false;

    for (auto engineState : this->entityDirectory)
    {
      this->stopCooperationWithEngine(engineState, true);
    }

    this->communication.reset();
    this->streamStore.reset();
    this->config.reset();
    this->timeFactory.reset();
    this->nodeStore.reset();
    this->ontologyInterface.reset();
    this->modelGenerator.reset();
    this->updateStrategie.reset();
    this->entityDirectory.reset();
    this->self.reset();
  }

  this->cv.notify_all();
  this->worker.join();
}

int Coordinator::onEngineHeartbeat(identifier engineId, time timestamp)
{
  if (false == this->running)
    return 5;

  _log->verbose(1, "New heartbeat from %v with %v", IDGenerator::toString(engineId),
                timestamp);

  std::lock_guard<std::mutex> guard(mtx_);

  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);

  _log->verbose(1, "Update lastActiveTime of %v with %v", IDGenerator::toString(engineId),
                timestamp);

  if (this->timeFactory->checkTimeout(entity->getActiveTimestamp(), this->config->getHeartbeatTimeout()))
  {
    _log->info("Engine rediscovered %v", IDGenerator::toString(engineId));

    if (entity->isIceIdentity())
    {
      // TODO hook!
      this->updateStrategie->onEngineDiscovered(entity);
    }
  }

  entity->setActiveTimestamp(timestamp);

  return 0;
}



int Coordinator::onSubModelRequest(identifier engineId, SubModelDesc modelDesc)
{
  if (false == this->running)
    return 5;

  _log->debug("Sub model request received from %v", IDGenerator::toString(engineId));

  std::lock_guard<std::mutex> guard(mtx_);

  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);

  // check if current request is ok at current cooperation state
  // TODO

  // check if specification was already received
  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RECEIVED)
  {
    _log->info("Duplicated sub model description received from engine %v", IDGenerator::toString(engineId));
    // TODO again?
  }

  bool result = this->updateStrategie->handleSubModel(engineState, modelDesc);

  if (result)
  {
    _log->info("Sub model from engine %v processed, sending acknowledgment", IDGenerator::toString(engineId));
    this->communication->sendSubModelResponse(engineId, modelDesc.index, true);

    return 0;
  }
  else
  {
    _log->info("Sub model from engine %v could not be processed", IDGenerator::toString(engineId));
    engineState->getOffering()->state = CooperationState::NO_COOPERATION;
    this->communication->sendSubModelResponse(engineId, modelDesc.index, false);

    return 0;
  }
}

int Coordinator::onSubModelResponse(identifier engineId, int index, bool accept)
{
  if (false == this->running)
    return 5;

  _log->debug("Sub model response received from '%v' with index '%v' and answer '%v'",
              IDGenerator::toString(engineId), index, accept);

  std::lock_guard<std::mutex> guard(mtx_);

  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);

  // check if engine is known
  if (false == engineState)
  {
    _log->info("Sub model response received from unknown engine %v", IDGenerator::toString(engineId));

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    return 1;
  }
  engineState->updateTimeLastActivity();

  // check if current request is ok at current cooperation state
  // TODO

  // check if specification was already received
  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RESPONSE_RECEIVED)
  {
    _log->info("Duplicated sub model response received from engine %v", IDGenerator::toString(engineId));
    // will be ignored

    return 1;
  }

  if (accept)
  {
    auto result = this->updateStrategie->handleSubModelResponse(engineState, index);

    if (result)
    {
      _log->info("Sub model response from engine %v processed, sending negotiation finished", IDGenerator::toString(engineId));
      this->communication->sendNegotiationFinished(engineId);
      engineState->getRequesting()->state = CooperationState::COOPERATION;

      return 0;
    }
    else
    {
      _log->info("Sub model response from engine %v could not be processed, sending stop cooperation", IDGenerator::toString(engineId));
      engineState->getRequesting()->state = CooperationState::NO_COOPERATION;
      this->communication->sendStopCooperation(engineId);
      // TODO create new model

      return 1;
    }
  }
  else
  {
    _log->info("Sub model request from engine '%v' was denied, sending stop cooperation",
               IDGenerator::toString(engineId));
    engineState->getRequesting()->state = CooperationState::NO_COOPERATION;
    this->communication->sendStopCooperation(engineId);
    // TODO create new model
    return 1;
  }
}

int Coordinator::onNegotiationFinished(identifier engineId)
{
  if (false == this->running)
    return 5;

  _log->debug("Negotiation finished received from '%v'", IDGenerator::toString(engineId));

  std::lock_guard<std::mutex> guard(mtx_);

  auto entity = this->entityDirectory->lookup(EntityDirectory::ID_ICE, std::to_string(engineId), true);

  // check if engine is known
  if (false == engineState)
  {
    _log->info("Negotiation finished received from unknown engine %v", IDGenerator::toString(engineId));

    engineState = std::make_shared<EngineState>(engineId, this->engine);
    this->engineStates.push_back(engineState);

    return 1;
  }

  // check if current request is ok at current cooperation state
  // TODO

  // check if specification was already received
  if (engineState->getOffering()->state == CooperationState::SUB_MODEL_RECEIVED)
  {
    _log->info("Duplicated negotiation finished received from engine %v", IDGenerator::toString(engineId));
  }


  _log->info("Negotiation finished received from engine '%v'", IDGenerator::toString(engineId));
//  this->communication->sendNegotiationFinished(engineId);
  engineState->getOffering()->state = CooperationState::COOPERATION;

  return 0;
}

int Coordinator::onStopCooperation(identifier engineId)
{
  if (false == this->running)
     return 5;

   _log->debug("Stop cooperation received from '%v'", IDGenerator::toString(engineId));

   std::lock_guard<std::mutex> guard(mtx_);

   auto engineState = this->getEngineStateNoMutex(engineId, false);

   // check if engine is known
   if (false == engineState)
   {
     _log->info("Stop cooperation received from unknown engine %v", IDGenerator::toString(engineId));

     engineState = std::make_shared<EngineState>(engineId, this->engine);
     this->engineStates.push_back(engineState);
     engineState->getOffering()->state = CooperationState::NO_COOPERATION;

     return 1;
   }
   engineState->updateTimeLastActivity();

   // check if current request is ok at current cooperation state
   // TODO

   // check if specification was already received
   if (engineState->getOffering()->state == CooperationState::NO_COOPERATION)
   {
     _log->info("Duplicated stop cooperation received from engine %v", IDGenerator::toString(engineId));
     return 0;
   }

   _log->info("Stop cooperation received from unknown engine '%v'", IDGenerator::toString(engineId));
   engineState->getOffering()->state = CooperationState::NO_COOPERATION;
   engineState->clearOffering();
   this->nodeStore->cleanUpNodes();
   this->streamStore->cleanUpStreams();

   return 0;
}

void Coordinator::workerTask()
{
  // sleep to enable the engine to initialize
  {
    std::unique_lock<std::mutex> lock(threadMtx_);
    this->cv.wait_for(lock, std::chrono::milliseconds(1000));
  }

  int counter = 1;

  while (this->running)
  {
    {
      std::unique_lock<std::mutex> lock(threadMtx_);
      this->cv.wait_for(lock, std::chrono::milliseconds(1000));

      auto e = this->engine.lock();
      if (e == nullptr || false == e->isRunning())
      {
        continue;
      }
    }

    _log->info("Sending heartbeat '%v', '%v'", counter, engineIri);

    {
      std::lock_guard<std::mutex> guard(mtx_);
      if (this->running)
        this->communication->sendHeartbeat();

      for (auto engine : this->engineStates)
      {
        _log->verbose(1, "Checking engine '%v' in engine state '%v'/'%v' since '%v'",
                      IDGenerator::toString(engine->getEngineId()), engine->getRequesting()->state,
                      engine->getOffering()->state, engine->getTimeLastStateUpdate());

        if (engine->getRequesting()->state == CooperationState::UNKNOWN)
        {
          continue;
        }

        if (engine->getRequesting()->state == CooperationState::NO_COOPERATION
            || engine->getRequesting()->state == CooperationState::COOPERATION)
        {
          if (this->timeFactory->checkTimeout(engine->getTimeLastActivity(), this->config->getHeartbeatTimeout()))
          {
            _log->info("Timeout of last activity of engine '%v' in state '%v'/'%v', stop cooperation",
                       IDGenerator::toString(engine->getEngineId()),
                       engine->getRequesting()->state, engine->getOffering()->state);

            this->stopCooperationWithEngine(engine, false);
            engine->getRequesting()->state = CooperationState::UNKNOWN;
            engine->getOffering()->state = CooperationState::UNKNOWN;
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
          _log->info("Timeout in communication with engine %v in state %v/%v, reached max retry count",
                     IDGenerator::toString(engine->getEngineId()), engine->getRequesting()->state,
                     engine->getOffering()->state);

          this->stopCooperationWithEngine(engine, false);
          engine->getRequesting()->state = CooperationState::UNKNOWN;
          engine->getOffering()->state = CooperationState::UNKNOWN;
          continue;
        }
        else
        {
          _log->info("Timeout in communication with engine %v in state %v/%v, %v retry",
                     IDGenerator::toString(engine->getEngineId()), engine->getRequesting()->state,
                     engine->getOffering()->state, engine->getRetryCounter());
        }

//        switch (engine->getCooperationState())
//        {
//          case (RETRY_NEGOTIATION):
//            engine->setCooperationState(CooperationState::RETRY_NEGOTIATION);
//            this->communication->sendRetryNegotiation(engine->getEngineId());
//            break;
//          case (INFORMATION_MODEL_REQUESTED):
//            engine->setCooperationState(CooperationState::INFORMATION_MODEL_REQUESTED);
//            this->communication->sendInformationRequest(engine->getEngineId());
//            break;
//          case (INFORMATION_MODEL_SEND):
//            engine->setCooperationState(CooperationState::INFORMATION_MODEL_SEND);
//            this->communication->sendInformationModel(engine->getEngineId(),
//                                                      this->engine.lock()->getInformationModel());
//            break;
//          case (COOPERATION_REQUEST_SEND):
//            engine->setCooperationState(CooperationState::COOPERATION_REQUEST_SEND);
////            this->communication->sendCooperationRequest(engine->getEngineId(), engine->getCooperationRequest());
//            break;
//          case (COOPERATION_RESPONSE_SEND):
//            engine->setCooperationState(CooperationState::COOPERATION_RESPONSE_SEND);
////            this->communication->sendCooperationResponse(engine->getEngineId(), engine->getCooperationResponse());
//            break;
//          case (COOPERATION_REFUSE_SEND):
//            engine->setCooperationState(CooperationState::COOPERATION_REFUSE_SEND);
//            this->communication->sendCooperationRefuse(engine->getEngineId());
//            break;
//          case (COOPERATION_ACCEPT_SEND):
//            engine->setCooperationState(CooperationState::COOPERATION_ACCEPT_SEND);
//            this->communication->sendCooperationAccept(engine->getEngineId());
//            break;
//          case (NEGOTIATION_FINISHED_SEND):
//            engine->setCooperationState(CooperationState::NEGOTIATION_FINISHED_SEND);
//            this->communication->sendNegotiationFinished(engine->getEngineId());
//            break;
//          case (STOP_COOPERATION_SEND):
//            engine->setCooperationState(CooperationState::STOP_COOPERATION_SEND);
//            this->communication->sendStopCooperation(engine->getEngineId());
//            break;
//          default:
//            _log->warn("Unknown cooperation state %v", engine->getCooperationState());
//        }
      }
    }

    std::unique_lock<std::mutex> lock(threadMtx_);
    this->cv.wait_for(lock, std::chrono::milliseconds(2000));
    ++counter;
  }
}

} /* namespace ice */

