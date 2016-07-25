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
  this->requestIds();
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
      // retry
      this->requestIds();
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
  auto entity = message->getEntity();

  switch (message->getId())
  {
    case (IMI_IDS_REQUEST):
    {
      // sending ids
      _log->info("Sending Ids to '%v'", entity->toString());
      auto m = std::make_shared<IdMessage>();
      this->self->pushIds(m->getIds());
      this->send(m);
      this->finish();
      break;
    }
    case (IMI_IDS_RESPONSE):
    {
      entity->fuse(std::static_pointer_cast<IdMessage>(message)->getIds());
      entity->checkIce();
      this->state = CJState::CJ_FINISHED;
      break;
    }
    default:
      ComJobBase::handleMessage(message);
      break;
  }
}

void IdentityRequest::requestIds()
{
  _log->info("Requesting Ids from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceMessageIds::IMI_IDS_REQUEST);
  this->send(m);
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
}

} /* namespace ice */
