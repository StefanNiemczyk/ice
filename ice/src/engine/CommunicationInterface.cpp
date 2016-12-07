/*
 * CommunicationInterface.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "ice/communication/CommunicationInterface.h"

#include <iostream>
#include <map>

#include "ice/communication/BaseInformationSender.h"
#include "ice/communication/InformationReceiver.h"
#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/messages/IdMessage.h"
#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/IntMessage.h"
#include "ice/communication/messages/OffersMessage.h"
#include "ice/communication/messages/OntologyIdMessage.h"
#include "ice/communication/messages/RequestMessage.h"
#include "ice/communication/messages/SubModelMessage.h"
#include "ice/communication/messages/SubModelResponseMessage.h"
#include "ice/communication/jobs/ComJobBase.h"
#include "ice/communication/jobs/IdentityRequest.h"
#include "ice/information/InformationCollection.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationStore.h"
#include "ice/representation/GContainer.h"
#include "ice/Entity.h"
#include "ice/ICEngine.h"
#include "ice/Time.h"

namespace ice
{

CommunicationInterface::CommunicationInterface(std::weak_ptr<ICEngine> engine) :
    running(false), engine(engine), maxMessageSend(10), discoveryInterval(100)
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
  auto e = this->engine.lock();
  this->directory = e->getEntityDirector();
  this->self = e->getSelf();
  this->timeFactory = e->getTimeFactory();
  this->containerFactory = e->getGContainerFactory();
  this->ontology = e->getOntologyInterface();

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
  this->messages.push(message);
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

std::shared_ptr<BaseInformationSender> CommunicationInterface::registerCollectionAsSender(
    std::shared_ptr<InformationCollection> collection)
{
  std::shared_ptr<BaseInformationSender> ptr;

  ptr = this->createSender(collection);

  if (ptr == nullptr)
  {
    _log->error("Sender could not be created for stream %v with type string %v",
                collection->getName(), collection->getSpecification()->getTypeString());
  }

  return ptr;
}

std::shared_ptr<InformationReceiver> CommunicationInterface::registerCollectionAsReceiver(
    std::shared_ptr<InformationCollection> collection)
{
  std::shared_ptr<InformationReceiver> ptr = this->createReceiver(collection);

  if (ptr == nullptr)
  {
    _log->error("Receiver could not be created for stream %v with type string %v",
                collection->getName(), collection->getSpecification()->getTypeString());
  }

  return ptr;
}

void CommunicationInterface::handleMessage(std::shared_ptr<Message> message)
{
  auto entity = message->getEntity();
  entity->setActiveTimestamp();
  _log->info("Received Message '%v', job id '%v' and index '%v' from %v",
             IceMessageIdsString((IceMessageIds) message->getId()),
             std::to_string(message->getJobId()), std::to_string(message->getJobIndex()), entity->toString());

  // message for own job
  if (message->getJobId() <= 127)
  {
    {
      std::lock_guard<std::mutex> guard(this->_jobMtx);
      for (auto &job : this->comJobsOwn)
      {
        if (job->getEntity() == message->getEntity() && job->match(message->getJobId(), message->getJobIndex()))
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

std::shared_ptr<Message> CommunicationInterface::parse(uint8_t id, std::string &jsonString,
                                                       std::shared_ptr<Entity> &entity)
{

 std::shared_ptr<Message> message;

 switch(id)
 {
   case(IMI_IDS_REQUEST):
   case(IMI_ID_REQUEST):
   case(IMI_ONTOLOGY_IDS_REQUEST):
   case(IMI_OFFERS_REQUEST):
   case(IMI_FINISH):
   case(IMI_CANCLE_JOB):
       message =  std::make_shared<CommandMessage>(id);
       break;
   case(IMI_IDS_RESPONSE):
       message = std::make_shared<IdMessage>();
       break;
   case(IMI_ONTOLOGY_IDS_RESPONSE):
       message = std::make_shared<OntologyIdMessage>();
       break;
   case(IMI_OFFERS_RESPONSE):
       message = std::make_shared<OffersMessage>();
       break;
   case(IMI_INFORMATION_REQUEST):
       message = std::make_shared<RequestMessage>();
       break;
   case(IMI_INFORMATION_RESPONSE):
       message = std::make_shared<InformationMessage>();
       break;
   case(IMI_ACK):
   case(IMI_INFORMATION_REQUEST_INDEX):
       message = std::make_shared<IntMessage>(id);
       break;
   case(IMI_SUBMODEL):
       message = std::make_shared<SubModelMessage>();
       break;
   case(IMI_SUBMODEL_RESPONSE):
       message = std::make_shared<SubModelResponseMessage>();
       break;

   default:
     _log->error("Message could not be parsed, unknown ID '%v'", id);
     return nullptr;
     break;
 }

 message->setEntity(entity);

 if (message->isPayload())
 {
   rapidjson::Document document;

   document.Parse(jsonString.c_str());

   if (document.GetType() == 0)
   {
     _log->error("Message '%v' could not be parsed, Json is broken '%v'", std::to_string(id), jsonString.c_str());
     return nullptr;
   }

   if (false == message->parsePayload(document, this->containerFactory))
   {
     _log->error("Message could not be parsed, Error while parsing payload for Message ID '%v'", std::to_string(id));
     return nullptr;
   }
 }

 return message;
}

void CommunicationInterface::workerTask()
{
  int counter = 0;
  std::vector<std::shared_ptr<Message>> msgs;

  while (this->running)
  {
    // check all n iterations for new peers
    if (counter >= discoveryInterval)
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

        if (false == job->getEntity()->isAvailable())
        {
          // pause job
          auto time = job->getEntity()->getActiveTimestamp();
          if (this->timeFactory->checkTimeout(time, 60000))
          {
            job->abort();
          }
        }

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

        if (false == job->getEntity()->isAvailable())
        {
          // pause job
          auto time = job->getEntity()->getActiveTimestamp();
          if (this->timeFactory->checkTimeout(time, 60000))
          {
            job->abort();
          }
        }

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

      int count = std::min(this->maxMessageSend, (int) this->messages.size());
      for (int i = 0; i < count; ++i)
      {
        if (false == this->running)
        {
          return;
        }

        this->sendMessage(this->messages.front());
        this->messages.pop();
      }
    }

    ++counter;
    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

std::shared_ptr<OntologyInterface> CommunicationInterface::getOntologyInterface()
{
  return this->ontology;
}

void CommunicationInterface::setOntologyInterface(std::shared_ptr<OntologyInterface> ontology)
{
  this->ontology = ontology;
}

Traffic& CommunicationInterface::getTraffic()
{
  return this->traffic;
}

void CommunicationInterface::resetTraffic()
{
  this->traffic.sendBytes = 0;
  this->traffic.receivedBytes = 0;
  this->traffic.messageSendCount = 0;
  this->traffic.messageReceivedCount = 0;
}

int CommunicationInterface::getMaxMessageSend()
{
  return this->maxMessageSend;
}

} /* namespace ice */
