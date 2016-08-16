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

IdentityRequest::IdentityRequest(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity) :
    ComJob(IdentityRequest::ID, engine, entity, el::Loggers::getLogger("IdentityRequest")), tryCount(0),
    stateIR(IdentityRequestState::IRS_UNKNOWN)
{
  auto e = engine.lock();
  this->ontologyInterface = e->getOntologyInterface();
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
      this->onResponsIds(std::static_pointer_cast<IdMessage>(message));
      break;
    }
    case (IMI_ONTOLOGY_IDS_REQUEST):
    {
      this->onRequestOntologyIds(message);
      break;
    }
    case (IMI_ONTOLOGY_IDS_RESPONSE):
    {
      this->onResponseOntologyIds(std::static_pointer_cast<OntologyIdMessage>(message));
      break;
    }
    case (IMI_FINISH):
    {
      if (false == this->ownJob)
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
  this->stateIR = IdentityRequestState::IRS_REQUEST_IDS;

  auto m = std::make_shared<CommandMessage>(IceMessageIds::IMI_IDS_REQUEST);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

void IdentityRequest::onRequestIds(std::shared_ptr<Message> const &message)
{
  if (this->stateIR != IdentityRequestState::IRS_UNKNOWN && this->stateIR != IdentityRequestState::IRS_REQUEST_IDS)
  {
    // received message in wrong state
    _log->warn("Received requestIds message in wrong state '%v' from '%v'", this->stateIR, entity->toString());
    return;
  }
  else if (this->stateIR == IdentityRequestState::IRS_REQUEST_IDS &&
      false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    // duplicated message
    _log->debug("Received duplicated requestIds message from '%v'", entity->toString());
    return;
  }

  _log->info("Sending Ids to '%v'", entity->toString());
  this->updateActiveTime();
  this->stateIR = IdentityRequestState::IRS_REQUEST_IDS;

  auto m = std::make_shared<IdMessage>();
  this->self->pushIds(m->getIds());
  this->send(m);
}

void IdentityRequest::onResponsIds(std::shared_ptr<IdMessage> const &message)
{
  if (this->stateIR != IdentityRequestState::IRS_REQUEST_IDS && this->stateIR != IdentityRequestState::IRS_RESPONS_IDS)
  {
    _log->warn("Received ids message in wrong state '%v' from '%v'", this->stateIR, entity->toString());
    return;
  }

  // duplicated message
  if (this->stateIR == IdentityRequestState::IRS_RESPONS_IDS
      && false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    _log->debug("Received duplicated ids message from '%v'", entity->toString());
    return;
  }

  _log->info("Received ids from %v", this->entity->toString());
  this->updateActiveTime();
  this->stateIR = IdentityRequestState::IRS_RESPONS_IDS;

  auto entity = message->getEntity();

  entity->fuse(message->getIds());
  entity->checkDirectory();

  bool hasIri = entity->getId(EntityDirectory::ID_ONTOLOGY, this->iri);

  // if not an ice engine stop here
  if (false == hasIri)
  {
    this->finish();
    return;
  }

  // ontology iris known?
  if (entity->getOntologyIds().size() > 0)
  {
    this->checkOntologyIris();
    return;
  }

  // request ontology ids
  this->stateIR = IdentityRequestState::IRS_REQUEST_ONT_IRI;
  this->sendCommand(IceMessageIds::IMI_ONTOLOGY_IDS_REQUEST);
}

void IdentityRequest::onRequestOntologyIds(std::shared_ptr<Message> const &message)
{
  if (this->stateIR == IdentityRequestState::IRS_UNKNOWN)
  {
    _log->warn("Received ids message in wrong state '%v' from '%v'", this->stateIR, entity->toString());
    return;
  }

  // duplicated message
  if (this->stateIR == IdentityRequestState::IRS_REQUEST_ONT_IRI && false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    _log->debug("Received duplicated ids message from '%v'", entity->toString());
    return;
  }

  _log->info("Sending ontology ids to %v", this->entity->toString());
  this->updateActiveTime();
  this->stateIR = IdentityRequestState::IRS_REQUEST_ONT_IRI;

  // create and send system specification
  auto msg = std::make_shared<OntologyIdMessage>();
  this->ontologyInterface->getOntologyIDs(msg->getIds());

  this->send(msg);
}

void IdentityRequest::onResponseOntologyIds(std::shared_ptr<OntologyIdMessage> const &message)
{
  if (this->stateIR != IdentityRequestState::IRS_REQUEST_ONT_IRI && this->stateIR != IdentityRequestState::IRS_RESPONS_ONT_IRI)
  {
    _log->warn("Received ontologyIds message in wrong state '%v' from '%v'", this->stateIR, entity->toString());
    return;
  }

  // duplicated message
  if (this->stateIR == IdentityRequestState::IRS_RESPONS_ONT_IRI
      && false == this->timeFactory->checkTimeout(this->timestampLastActive, 100))
  {
    _log->debug("Received duplicated ontologyIds message from '%v'", entity->toString());
    return;
  }

  _log->info("Received ontology ids to %v", this->entity->toString());
  this->updateActiveTime();
  this->stateIR = IdentityRequestState::IRS_RESPONS_ONT_IRI;

  this->entity->getOntologyIds() = message->getIds();
  this->checkOntologyIris();
}

void IdentityRequest::checkOntologyIris()
{
  // check diff
  this->ontologyInterface->compareOntologyIDs(this->entity->getOntologyIds(),
                                              this->entity->getOntologyIriDiff());

  if (this->entity->getOntologyIriDiff().size() != 0)
  {
    _log->info("Ontology ids do not match, received from %v", this->entity->toString());

    // done
    this->sendCommand(IceMessageIds::IMI_FINISH);
    this->finish();
    return;
  }

    // check if system is known in ontology
    if (this->entity->initializeFromOntology(this->ontologyInterface) >= 0)
    {
      // system is known, cooperation is possible
      _log->info("Entity '%v' known by ontology", this->entity->toString());
    }
    else
    {
      // system is unknown, request nodes
      _log->info("Entity' %v' is unknown in ontology", this->entity->toString());
      // TODO request nodes
    }

    // done
    this->sendCommand(IceMessageIds::IMI_FINISH);
    this->finish();
    entity->checkIce();
}

} /* namespace ice */
