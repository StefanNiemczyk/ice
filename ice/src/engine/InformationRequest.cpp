/*
 * InformationRequest.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: sni
 */

#include <ice/communication/jobs/InformationRequest.h>

#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/RequestMessage.h"

namespace ice
{

int InformationRequest::ID = 2;
int InformationRequestCreator::val = InformationRequestCreator::init();

InformationRequest::InformationRequest(ICEngine* const engine, std::shared_ptr<Entity> const &entity) :
    ComJob(InformationRequest::ID, engine, entity, el::Loggers::getLogger("InformationRequest"))
{

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

void InformationRequest::handleMessage(std::shared_ptr<Message> const &message)
{
  auto entity = message->getEntity();

  switch (message->getId())
  {
    case (IMI_INFORMATION_REQUEST):
    {
      this->onRequestInformation(std::static_pointer_cast<RequestMessage>(message));
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
  m->setEntity(entity);

  auto &vec = m->getRequests();
  for (auto info : requests)
  {
    vec.push_back(info);
  }

  this->send(m);
}

void InformationRequest::onRequestInformation(std::shared_ptr<RequestMessage> const &message)
{
  _log->info("Information request received from '%v'", entity->toString());
  std::vector<std::shared_ptr<InformationElement<GContainer>>>infos;

  for (auto &request : message->getRequests())
  {
    this->engine->getInformationStore()->getInformation(request, infos);
  }

  int count = infos.size();
  if (count == 0)
  {
    _log->warn("No information found for request from '%v'", entity->toString());
    // TODO
    return;
  }

  _log->info("Sending '%v' information to '%v'", count, entity->toString());
  for (auto &info : infos)
  {
    auto m = std::make_shared<InformationMessage>();
    m->setEntity(entity);
    m->getInformations().push_back(info);
    this->send(m);
  }
}

void InformationRequest::onInformation(std::shared_ptr<InformationMessage> const &message)
{
  _log->info("Information received from '%v'", entity->toString());

  for (auto &info : message->getInformations())
  {
    this->engine->getInformationStore()->addInformation(info);
  }
}

std::vector<std::shared_ptr<InformationSpecification>>& InformationRequest::getRequests()
{
  return this->requests;
}

} /* namespace ice */
