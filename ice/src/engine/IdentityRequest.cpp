/*
 * IdentityRequest.cpp
 *
 *  Created on: Jul 15, 2016
 *      Author: sni
 */

#include <ice/communication/jobs/IdentityRequest.h>

#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/messages/IdMessage.h"

namespace ice
{
int IdentityRequestCreator::val = IdentityRequestCreator::init();

IdentityRequest::IdentityRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity) :
    ComJob(1, engine, entity, el::Loggers::getLogger("IdentityRequest"))
{

}

IdentityRequest::~IdentityRequest()
{
  //
}

void IdentityRequest::init()
{
  // call init from super class
  ComJobBase::init();

  _log->info("Requesting Ids from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_IDS_REQUEST);
  this->send(m);
  this->state = CJState::CJ_WAITING;
}

void IdentityRequest::init(std::shared_ptr<Message> const &message)
{
  this->handleMessage(message);
}

void IdentityRequest::handleMessage(std::shared_ptr<Message> const &message)
{
  auto entity = message->getEntity();

  switch (message->getId())
  {
    case (SCMD_IDS_REQUEST):
    {
      // sending ids
      _log->info("Sending Ids to '%v'", entity->toString());
      auto m = std::make_shared<IdMessage>();
      this->self->pushIds(m->getIds());

      this->send(m);
      this->state = CJState::CJ_FINISHED;
      break;
    }
    case (SCMD_IDS_RESPONSE):
    {
      entity->fuse(std::static_pointer_cast<IdMessage>(message)->getIds());
      entity->checkIce();
      this->state = CJState::CJ_FINISHED;
      break;
    }
    default:
      _log->error("Unknown command '%v', message will be skipped", std::to_string(message->getId()));
      break;
  }
}

} /* namespace ice */
