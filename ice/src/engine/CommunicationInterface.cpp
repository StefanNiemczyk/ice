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
  std::lock_guard<std::mutex> guard(this->_jobAddMtx);
  this->comJobsOwnNew.push_back(job);
  job->init();
}

void CommunicationInterface::removeComJob(std::shared_ptr<ComJobBase> const &job)
{
  std::lock_guard<std::mutex> guard(this->_jobAddMtx);
  auto j = std::find(this->comJobsOwnNew.begin(), this->comJobsOwnNew.end(), job);

  if (j != this->comJobsOwnNew.end())
    this->comJobsOwnNew.erase(j);

  std::lock_guard<std::mutex> guard2(this->_jobMtx);
  j = std::find(this->comJobsOwn.begin(), this->comJobsOwn.end(), job);

  if (j != this->comJobsOwn.end())
    this->comJobsOwn.erase(j);
}

void CommunicationInterface::discoveredEntity(std::shared_ptr<Entity> const &entity)
{
  _log->info("Requesting information from discovered entity '%v'", entity->toString());

  auto request = std::make_shared<IdentityRequest>(this->engine, entity);
  this->addComJob(request);
}

//void CommunicationInterface::requestOffers(std::shared_ptr<Entity> const &entity)
//{
//  _log->info("Requesting offered information from '%v'", entity->toString());
//  auto m = std::make_shared<CommandMessage>(IceCmd::SCMD_OFFERS_REQUEST);
//  m->setEntity(entity);
//
//  this->send(m);
//}
//
//void CommunicationInterface::onRequestOffers(std::shared_ptr<Entity> const &entity)
//{
//  _log->info("Sending offered information to '%v'", entity->toString());
//  auto m = std::make_shared<OffersMessage>();
//  m->setEntity(entity);
//
//  auto &vec = m->getOfferes();
////  for (auto info : this->bridge->getOfferedInfos())
////  {
////    vec.push_back(info->infoSpec);
////  }
//
//  this->send(m);
//}

void CommunicationInterface::handleMessage(std::shared_ptr<Message> message)
{
  auto entity = message->getEntity();
  entity->setActiveTimestamp();
  _log->info("Received Message with id '%v', job id '%v' and index '%v' from %v", std::to_string(message->getId()),
             std::to_string(message->getJobId()), std::to_string(message->getJobIndex()), entity->toString());

  // message for own job
  if (message->getJobId() <= 127)
  {
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
    }

    _log->warn("Received answer for missing job with id '%v' and index '%v' from %v",
               std::to_string(message->getJobId()), std::to_string(message->getJobIndex()), entity->toString());
    // received answer for missing request

    if (message->getId() == IceMessageIds::IMI_CANCLE_JOB)
    {
      return;
    }
    auto msg = std::make_shared<CommandMessage>(IceMessageIds::IMI_CANCLE_JOB);
    msg->setJobIndex(message->getJobIndex());
    msg->setJobId(message->getJobId() + 127);
    msg->setEntity(entity);
    this->send(msg);

    return;
  }

  // message for incomming job
  uint8_t id = message->getJobId() - 127;
  std::lock_guard<std::mutex> guard(this->_jobMtx);
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
  job->setTimeout(120000);
  job->init(message);

  this->comJobsIncomming.push_back(job);
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

    {
      std::lock_guard<std::mutex> guard(this->_jobMtx);

      {
        std::lock_guard<std::mutex> guard(this->_jobAddMtx);

        for (auto &job : this->comJobsOwnNew)
        {
          this->comJobsOwn.push_back(job);
        }

        this->comJobsOwnNew.clear();
      }

      // handle own jobs
      for (int i = 0; i < this->comJobsOwn.size(); ++i)
      {
        auto &job = this->comJobsOwn.at(i);
        switch (job->getState())
        {
          case (CJ_CREATED):
            job->init();
            break;
          case (CJ_INITIALIZED):
          case (CJ_ACTIVE):
          case (CJ_WAITING):
            job->tick();
            break;
          case (CJ_FINISHED):
          case (CJ_ABORTED):
            job->cleanUp();
            this->comJobsOwn.erase(this->comJobsOwn.begin() + i);
            --i;
            break;
        }
      }

      for (int i = 0; i < this->comJobsIncomming.size(); ++i)
      {
        auto &job = this->comJobsIncomming.at(i);

        switch (job->getState())
        {
          case (CJ_INITIALIZED):
          case (CJ_ACTIVE):
          case (CJ_WAITING):
            job->tick();
            break;
          case (CJ_FINISHED):
          case (CJ_ABORTED):
            job->cleanUp();
            this->comJobsIncomming.erase(this->comJobsIncomming.begin() + i);
            --i;
            break;
          default:
            if (job->checkTimeout())
            {
              job->abort();
              this->comJobsIncomming.erase(this->comJobsIncomming.begin() + i);
              --i;
            }
            break;
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
