/*
 * CommunicationInterface.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "ice/communication/CommunicationInterface.h"

#include <iostream>
#include <map>

#include "ice/communication/messages/Message.h"
#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/messages/IdMessage.h"
#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/OffersMessage.h"
#include "ice/communication/messages/RequestMessage.h"
#include "ice/communication/jobs/ComJobBase.h"
#include "ice/communication/jobs/IdentityRequest.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationStore.h"
#include "ice/representation/GContainer.h"
#include "ice/Entity.h"
#include "ice/ICEngine.h"
#include "ice/Time.h"

namespace ice
{

CommunicationInterface::CommunicationInterface(ICEngine * engine) :
    running(false), engine(engine)
{
  _log = el::Loggers::getLogger("CommunicationInterface");
}

CommunicationInterface::~CommunicationInterface()
{
  if (this->running && this->worker.joinable())
  {
    this->running = false;
    this->worker.join();
  }
}

void CommunicationInterface::init()
{
  this->directory = this->engine->getEntityDirector();
  this->self = this->engine->getSelf();
  this->timeFactory = this->engine->getTimeFactory();
  this->initInternal();

  this->running = true;
  this->worker = std::thread(&CommunicationInterface::workerTask, this);
}

void CommunicationInterface::cleanUp()
{
  this->running = false;
  this->worker.join();

  this->cleanUpInternal();
}

void CommunicationInterface::send(std::shared_ptr<Message> message)
{
  std::lock_guard<std::mutex> guard(this->_messageMtx);

  this->messages.push_back(message);
}

void CommunicationInterface::addComJob(std::shared_ptr<ComJobBase> const &job)
{
  std::lock_guard<std::mutex> guard(this->_jobMtx);
  this->comJobsOwn.push_back(job);
}

void CommunicationInterface::removeComJob(std::shared_ptr<ComJobBase> const &job)
{
  std::lock_guard<std::mutex> guard(this->_jobMtx);
  auto j = std::find(this->comJobsOwn.begin(), this->comJobsOwn.end(), job);

  if (j != this->comJobsOwn.end())
    this->comJobsOwn.erase(j);
}

void CommunicationInterface::discoveredEntity(std::shared_ptr<Entity> const &entity)
{
  std::lock_guard<std::mutex> guard(this->_jobMtx);
  _log->info("Requesting information from discovered entity '%v'", entity->toString());

  auto request = std::make_shared<IdentityRequest>(this->engine, entity);
  this->comJobsOwn.push_back(request);
  request->init();
}

void CommunicationInterface::requestIds(std::shared_ptr<Entity> const &entity)
{
  _log->info("Requesting Ids from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_IDS_REQUEST);
  m->setEntity(entity);

  this->send(m);
}

void CommunicationInterface::onRequestIds(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending Ids to '%v'", entity->toString());
  auto m = std::make_shared<IdMessage>();
  m->setEntity(entity);
  this->self->pushIds(m->getIds());

  this->send(m);
}

void CommunicationInterface::requestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Requesting offered information from '%v'", entity->toString());
  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_OFFERS_REQUEST);
  m->setEntity(entity);

  this->send(m);
}

void CommunicationInterface::onRequestOffers(std::shared_ptr<Entity> const &entity)
{
  _log->info("Sending offered information to '%v'", entity->toString());
  auto m = std::make_shared<OffersMessage>();
  m->setEntity(entity);

  auto &vec = m->getOfferes();
//  for (auto info : this->bridge->getOfferedInfos())
//  {
//    vec.push_back(info->infoSpec);
//  }

  this->send(m);
}

void CommunicationInterface::requestInformation(std::shared_ptr<Entity> const &entity,
                                                std::vector<std::shared_ptr<InformationSpecification>> const &requests)
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

void CommunicationInterface::onRequestInformation(
    std::shared_ptr<Entity> const &entity, std::vector<std::shared_ptr<InformationSpecification>> const &requests)
{
  _log->info("Information request received from '%v'", entity->toString());
  std::vector<std::shared_ptr<InformationElement<GContainer>>>infos;

//  for (auto &request : requests)
//  {
//    this->bridge->informationStore->getInformation(request,infos);
//  }

  int count = infos.size();
  if (count == 0)
  {
    _log->warn("No information found for request from '%v'", entity->toString());
    return;
  }

  _log->info("Sending '%v' information to '%v'", count, entity->toString());
  for (auto &info : infos)
  {
    auto m = std::make_shared<InformationMessage>();
    m->setEntity(entity);
    m->getInformations().push_back(info);
    this->sendMessage(m);
  }
}

void CommunicationInterface::onInformation(std::shared_ptr<Entity> const &entity,
                                           std::vector<std::shared_ptr<InformationElement<GContainer>>>&information)
{
  _log->info("Information received from '%v'", entity->toString());

  for (auto &info : information)
  {
    this->informationStore->addInformation(info);
  }
}

void CommunicationInterface::handleMessage(std::shared_ptr<Message> message)
{
  auto entity = message->getEntity();
  _log->info("Received Message with id '%v', job id '%v' and index '%v' from %v", std::to_string(message->getId()),
             std::to_string(message->getJobId()), std::to_string(message->getJobIndex()), entity->toString());
  entity->setActiveTimestamp();

  // message for own job
  if (message->getJobId() <= 127)
  {
    std::lock_guard<std::mutex> guard(this->_jobMtx);
    for (auto &job : this->comJobsOwn)
    {
      if (job->match(message->getJobId(), message->getJobIndex()))
      {
        job->handleMessage(message);
        return;
      }
    }

    _log->warn("Received answer for missing job with id '%v' and index '%v' from %v",
               std::to_string(message->getJobId()), std::to_string(message->getJobIndex()), entity->toString());
    // received answer for missing request

    if (message->getId() == IceCmd::SCMD_CANCLE_JOB)
    {
      return;
    }
    auto msg = std::make_shared<CommandMessage>(IceCmd::SCMD_CANCLE_JOB);
    msg->setJobIndex(message->getJobIndex());
    msg->setJobId(message->getJobId());
    msg->setEntity(entity);
    this->send(msg);

    return;
  }

  // message for incomming job
  uint8_t id = message->getJobId() - 127;
  for (auto &job : this->comJobsIncomming)
  {
    if (job->match(id, message->getJobIndex()))
    {
      job->handleMessage(message);
      return;
    }
  }

  // no matching job, creating one
  auto job = ComJobRegistry::makeInstance(id, this->engine, entity);
  job->setIndex(message->getJobIndex());
  job->setOwnJob(false);
  this->comJobsOwn.push_back(job);
  job->handleMessage(message);

//  switch (message->getId())
//  {
//    case (SCMD_IDS_REQUEST):
//      this->onRequestIds(entity);
//      break;
//
//    case (SCMD_IDS_RESPONSE):
//    {
//      entity->fuse(std::static_pointer_cast<IdMessage>(message)->getIds());
//      entity->checkIce();
//      break;
//    }
//
//    case (SCMD_OFFERS_REQUEST):
//      this->onRequestOffers(entity);
//      break;
//
//    case (SCMD_OFFERS_RESPONSE):
//        entity->addOfferedInformation(std::static_pointer_cast<OffersMessage>(message)->getOfferes());
//      break;
//
//    case (SCMD_INFORMATION_REQUEST):
//        this->onRequestInformation(entity, std::static_pointer_cast<RequestMessage>(message)->getRequests());
//    break;
//
//    case (SCMD_INFORMATION_RESPONSE):
//        this->onInformation(entity, std::static_pointer_cast<InformationMessage>(message)->getInformations());
//    break;
//
//    default:
//      _log->error("Unknown command '%v', message will be skipped", std::to_string(message->getId()));
//      break;
//  }
}

void CommunicationInterface::workerTask()
{
  int counter = 0;
  std::vector<std::shared_ptr<Message>> msgs;

  while (this->running)
  {
    // check all n iterations for new peers
    if (counter >= 100)
    {
      this->discover();
      this->directory->checkTimeout();

      counter = 0;
    }

    // check for new messages
    msgs.clear();
    int count = this->readMessage(msgs);
    for (auto &msg : msgs)
    {
      this->handleMessage(msg);
    }

    // handle own jobs
    {
      std::lock_guard<std::mutex> guard(this->_jobMtx);
      for (int i = 0; i < this->comJobsOwn.size(); ++i)
      {
        auto &job = this->comJobsOwn.at(i);
        time t = this->timeFactory->createTime();
        switch (job->getState())
        {
          case (CJ_CREATED):
            job->init();
            break;
          case (CJ_INITIALIZED):
          case (CJ_ACTIVE):
          case (CJ_WAITING):
            job->tick(t);
            break;
          case (CJ_FINISHED):
            job->cleanUp();
            this->comJobsOwn.erase(this->comJobsOwn.begin() + i);
            --i;
            break;
        }
      }

      for (int i = 0; i < this->comJobsIncomming.size(); ++i)
      {
        auto &job = this->comJobsIncomming.at(i);

        if (job->getState() == CJState::CJ_FINISHED)
        {
          job->cleanUp();
          this->comJobsIncomming.erase(this->comJobsIncomming.begin() + i);
          --i;
        }
      }
    }

    // send messages
    {
      std::lock_guard<std::mutex> guard(this->_messageMtx);

      for (auto &msg : this->messages)
      {
        if (false == this->running)
        {
          return;
        }

        this->sendMessage(msg);
      }

      this->messages.clear();
    }

    ++counter;
    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

std::shared_ptr<GContainerFactory> CommunicationInterface::getGContainerFactory()
{
  return this->containerFactory;
}

void CommunicationInterface::setGContainerFactory(std::shared_ptr<GContainerFactory> factory)
{
  this->containerFactory = factory;
}

std::shared_ptr<InformationStore> CommunicationInterface::getInformationStore()
{
  return this->informationStore;
}

void CommunicationInterface::setInformationStore(std::shared_ptr<InformationStore> store)
{
  this->informationStore = store;
}

std::shared_ptr<OntologyInterface> CommunicationInterface::getOntologyInterface()
{
  return this->ontology;
}

void CommunicationInterface::setOntologyInterface(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontology = ontology;
}

} /* namespace ice */
