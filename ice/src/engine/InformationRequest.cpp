/*
 * InformationRequest.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: sni
 */

#include <ice/communication/jobs/InformationRequest.h>

#include <algorithm>

#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/IntMessage.h"
#include "ice/communication/messages/RequestMessage.h"
#include "ice/information/InformationStore.h"

namespace ice
{

int InformationRequest::ID = 2;
int InformationRequestCreator::val = InformationRequestCreator::init();

InformationRequest::InformationRequest(std::weak_ptr<ICEngine> engine, std::shared_ptr<Entity> const &entity) :
    ComJob(InformationRequest::ID, engine, entity, el::Loggers::getLogger("InformationRequest")), currentIndex(-1), tryCount(
        0), receivedAck(false), resendCount(0)
{
  this->timeout = 2000;
  auto e = this->engine.lock();
  this->informationStore = e->getInformationStore();
}

InformationRequest::~InformationRequest()
{
  //
}

void InformationRequest::init()
{
  // call init from super class
  ComJobBase::init();
  this->currentIndex = 0;
  this->requestInformation();
  this->state = CJState::CJ_WAITING;
  this->updateActiveTime();
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
    if (false == this->timeFactory->checkTimeout(this->timestampLastActive, this->timeout))
    {
      return;
    }

    this->updateActiveTime();
    ++this->tryCount;

    // request ack again
    if (false == this->receivedAck)
    {
      if (this->tryCount >= 5)
      {
        _log->warn("Abort requesting information from '%v', max try count reached", entity->toString());
        this->abort();
        return;
      }

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

    if (count == this->received.size())
    {
      this->finish();
      return;
    }
    else if (this->tryCount >= 10)
    {
      _log->warn("Abort requesting information from '%v', max try count reached", entity->toString());
      this->abort();
      return;
    }

    return;
  }
  else
  {
    if (this->currentIndex < 0)
    {
      return;
    }

    if (this->currentIndex >= this->information.size())
    {
      this->state == CJState::CJ_WAITING;
      return;
    }

    int send = 10;
    int max = std::min((int)(this->currentIndex + send), (int) this->information.size());
    for (int i=this->currentIndex; i < max; ++i)
    {
      this->sendInformation(i);
    }
    this->currentIndex += send;
  }
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
    case (IMI_FINISH):
    {
      if (this->ownJob == false)
      {
        _log->info("Job finished from '%v'", entity->toString());
       // this->finish();
        this->state = CJState::CJ_FINISHED;
      }
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
  _log->info("Requesting information for index '%v' from '%v'", index, entity->toString());
  ++this->resendCount;
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
      this->informationStore->getInformation(request, this->information);
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
  receivedAck = true;

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
  _log->info("Information message received from '%v'", entity->toString());

  for (auto &info : message->getInformations())
  {
    _log->info("Information received for index '%v' from '%v'", info.first, entity->toString());
    if (this->received.size() < info.first)
    {
      this->received.resize(info.first * 2, false);
    }

    if (this->received.at(info.first))
    {
      // information is duplicated
      continue;
    }

    this->received[info.first] = true;
    this->informationStore->addInformation(info.second);
    this->information.push_back(info.second);
    this->currentIndex++;
  }

  if (this->receivedAck && this->currentIndex == this->received.size())
  {
    this->finish();
  }
}

std::vector<std::shared_ptr<InformationSpecification>>& InformationRequest::getRequests()
{
  return this->requests;
}

int InformationRequest::getResendCount()
{
  return this->resendCount;
}

} /* namespace ice */
