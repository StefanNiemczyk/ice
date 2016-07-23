/*
 * InformationRequest.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: sni
 */

#include <ice/communication/jobs/InformationRequest.h>

#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/IntMessage.h"
#include "ice/communication/messages/RequestMessage.h"

namespace ice
{

int InformationRequest::ID = 2;
int InformationRequestCreator::val = InformationRequestCreator::init();

InformationRequest::InformationRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity) :
    ComJob(InformationRequest::ID, engine, entity, el::Loggers::getLogger("InformationRequest")), currentIndex(-1), tryCount(
        0)
{
  this->timeout = 2000;
}

InformationRequest::~InformationRequest()
{
  //
}

void InformationRequest::init()
{
  // call init from super class
  ComJobBase::init();
  this->requestInformation();
  this->state = CJState::CJ_WAITING;
}

void InformationRequest::tick()
{
  if (this->ownJob)
  {
    // not initialized
    if (this->state == CJState::CJ_CREATED)
    {
      return;
    }

    // check timestamp since last message
    if (false == this->engine->getTimeFactory()->checkTimeout(this->timestampLastActive, this->timeout))
    {
      return;
    }

    this->updateActiveTime();
    ++this->tryCount;

    // request ack again
    if (true)
    {
      this->requestInformation();

      return;
    }

    // check missing
    int count = 0;

    for (int i=0; i < this->received.size(); ++i)
    {
      if (this->received.at(i))
      {
        ++count;
      }
      else
      {
        this->requestInformation(i);
      }
    }

    if (count == 0)
    {
      this->finish();
      return;
    }

    return;
  }

  if (this->currentIndex < 0)
  {
    return;
  }

  if (this->currentIndex >= this->information.size())
  {
    this->state == CJState::CJ_WAITING;
    return;
  }

  this->sendInformation(this->currentIndex++);
}

void InformationRequest::handleMessage(std::shared_ptr<Message> const &message)
{
  this->updateActiveTime();
  auto entity = message->getEntity();

  switch (message->getId())
  {
    case (IMI_INFORMATION_REQUEST):
    {
      this->onRequestInformation(std::static_pointer_cast<RequestMessage>(message));
      break;
    }
    case (IMI_INFORMATION_REQUEST_INDEX):
    {
      this->sendInformation(std::static_pointer_cast<IntMessage>(message)->getValue());
      break;
    }
    case (IMI_ACK):
    {
      this->onAcc(std::static_pointer_cast<IntMessage>(message));
      break;
    }
    case (IMI_INFORMATION_RESPONSE):
    {
      this->onInformation(std::static_pointer_cast<InformationMessage>(message));
      break;
    }
    default:
      ComJobBase::handleMessage(message);
      break;
  }
}

void InformationRequest::requestInformation()
{
  _log->info("Requesting information from '%v'", entity->toString());
  auto m = std::make_shared<RequestMessage>();

  auto &vec = m->getRequests();
  for (auto info : requests)
  {
    vec.push_back(info);
  }

  this->send(m);
}

void InformationRequest::requestInformation(int index)
{
  _log->info("Requesting information for index '%v' from '%v'", entity->toString());
  auto m = std::make_shared<IntMessage>(IceMessageIds::IMI_INFORMATION_REQUEST_INDEX);
  m->setValue(index);
  this->send(m);
}

void InformationRequest::onRequestInformation(std::shared_ptr<RequestMessage> const &message)
{
  _log->info("Information request received from '%v'", entity->toString());

  if (this->information.size() == 0)
  {
    for (auto &request : message->getRequests())
    {
      this->engine->getInformationStore()->getInformation(request, this->information);
    }

    this->currentIndex = 0;
  }

  int count = this->information.size();
  _log->info("Sending acc for '%v' information to '%v'", count, entity->toString());

  auto msg = std::make_shared<IntMessage>(IceMessageIds::IMI_ACK);
  msg->setValue(count);
  this->send(msg);

  if (count == 0)
  {
    this->finish();
  }
}

void InformationRequest::onAcc(std::shared_ptr<IntMessage> const &message)
{
  int value = message->getValue();
  if (value == 0)
  {
    _log->info("No information received from engine '%v'", entity->toString());
    this->finish();
  }

  _log->info("'%v' information will be send by engine '%v'", std::to_string(value), entity->toString());

  this->received.resize(value, false);
}

void InformationRequest::sendInformation(int index)
{
  if (index >= this->information.size())
  {
    _log->error("Trying to send information with to high index to engine '%v'", entity->toString());
    this->abort();
  }

  _log->info("Send information '%v' to engine '%v'", std::to_string(index), entity->toString());

  auto m = std::make_shared<InformationMessage>();
  m->getInformations().push_back(std::make_pair(index, this->information.at(index)));
  this->send(m);
}

void InformationRequest::onInformation(std::shared_ptr<InformationMessage> const &message)
{
  _log->info("Information received from '%v'", entity->toString());

  for (auto &info : message->getInformations())
  {
    if (this->received.size() < info.first)
    {
      this->received.resize(info.first * 2, false);
    }

    if (this->received.at(info.first))
    {
      // information is duplicated
      continue;
    }

    this->received.at(info.first) = true;
    this->engine->getInformationStore()->addInformation(info.second);
    this->information.push_back(info.second);
  }
}

std::vector<std::shared_ptr<InformationSpecification>>& InformationRequest::getRequests()
{
  return this->requests;
}

} /* namespace ice */
