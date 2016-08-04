/*
 * IdentityRequest.cpp
 *
 *  Created on: Jul 15, 2016
 *      Author: sni
 */

#include <ice/communication/jobs/IdentityRequest.h>

#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/messages/IdMessage.h"
#include "ice/communication/messages/OntologyIdMessage.h"

namespace ice
{
int IdentityRequest::ID = 1;
int IdentityRequestCreator::val = IdentityRequestCreator::init();

IdentityRequest::IdentityRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity) :
    ComJob(IdentityRequest::ID, engine, entity, el::Loggers::getLogger("IdentityRequest")), tryCount(0)
{
}

IdentityRequest::~IdentityRequest()
{
}

void IdentityRequest::init()
{
  // call init from super class
  ComJobBase::init();
  this->sendRequestIds();
}

void IdentityRequest::tick()
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
      this->sendRequestIds();
    }
    else
    {
      entity->checkIce();
      this->abort();
    }
  }
}

void IdentityRequest::handleMessage(std::shared_ptr<Message> const &message)
{
  switch (message->getId())
  {
    case (IMI_IDS_REQUEST):
    {
      this->onRequestIds(message);
      break;
    }
    case (IMI_IDS_RESPONSE):
    {
      this->onRequestIds(std::static_pointer_cast<IdMessage>(message));
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

void IdentityRequest::sendRequestIds()
{
  _log->info("Requesting Ids from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceMessageIds::IMI_IDS_REQUEST);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

void IdentityRequest::onRequestIds(std::shared_ptr<Message> const &message)
{
  _log->info("Sending Ids to '%v'", entity->toString());

  auto m = std::make_shared<IdMessage>();
  this->self->pushIds(m->getIds());
  this->send(m);
}

void IdentityRequest::onResponsIds(std::shared_ptr<IdMessage> const &message)
{
  _log->debug("Received ids from %v", this->entity->toString());

  auto entity = message->getEntity();

  entity->fuse(message->getIds());
  entity->checkIce();

  // if not an ice engine stop here
  if (false == entity->isIceIdentity())
  {
    this->finish();
    return;
  }

  // ontology iris known?
  if (entity->getOntologyIds().size() > 0)
  {
    // done
    this->sendCommand(IceMessageIds::IMI_FINISH);
    this->finish();
  }

  // request ontology ids
  this->sendCommand(IceMessageIds::IMI_ONTOLOGY_IDS_REQUEST);
}

void IdentityRequest::onRequestOntologyIds(std::shared_ptr<Message> const &message)
{
  _log->debug("Sending ontology ids to %v", this->entity->toString());

  // check if request already received
  // TODO

  // create and send system specification
  auto msg = std::make_shared<OntologyIdMessage>();
  this->engine->getOntologyInterface()->getOntologyIDs(msg->getIds());

  this->send(msg);
}

void IdentityRequest::onResponseOntologyIds(std::shared_ptr<OntologyIdMessage> const &message)
{
  _log->debug("Received ontology ids to %v", this->entity->toString());

  if (this->entity->getOntologyIds().size() > 0)
  {
    _log->info("Duplicated ontology ids received from engine %v", this->entity->toString());
    return;
  }

  this->entity->getOntologyIds() = message->getIds();

   auto result = this->engine->getOntologyInterface()->compareOntologyIDs(message->getIds());

   if (result->size() != 0)
   {
     _log->info("Ontology ids do not match, received from %v", this->entity->toString());
     // TODO implement requesting ontologies
     engineState->getRequesting()->state = CooperationState::NO_COOPERATION;

     return;
   }

  // check if system is known in ontology
  if (this->ontologyInterface->isSystemKnown(std::get<0>(spec)))
  {
    // system is known, cooperation is possible
    _log->info("Entity '%v' known by ontology", this->entity->toString());


    // trigger processing model update
    engineState->setNodesKnown(true);
    this->updateStrategie->onEngineDiscovered(engineState);

    // done
    this->sendCommand(IceMessageIds::IMI_FINISH);
    this->finish();
  }
  else
  {
    // system is unknown, request nodes
    _log->info("System' %v' identified by id '%v' is unknown", std::get<0>(spec),
               IDGenerator::toString(engineId));
    // TODO request nodes
    engineState->getRequesting()->state = CooperationState::NO_COOPERATION;
  }

  return 0;
}


} /* namespace ice */
