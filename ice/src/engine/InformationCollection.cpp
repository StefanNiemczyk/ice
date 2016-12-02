/*
 * InformationCollection.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#include <ice/information/InformationCollection.h>

#include <sstream>

#include "ice/processing/EventHandler.h"
#include "ice/Entity.h"
#include "easylogging++.h"
#include "fnv.h"

namespace ice
{
int InformationCollection::IDENTIFIER_COUNTER = 0;

InformationCollection::InformationCollection(std::shared_ptr<CollectionDescription> streamDescription,
                                             std::shared_ptr<EventHandler> eventHandler) :
    streamDescription(streamDescription), iid(IDENTIFIER_COUNTER++), eventHandler(eventHandler), hash(0)
{
  _log = nullptr;
}

InformationCollection::~InformationCollection()
{
  // nothing to do here
}

std::shared_ptr<InformationSpecification> InformationCollection::getSpecification() const
{
  return this->streamDescription->getInformationSpecification();
}

const int InformationCollection::getIID() const
{
  return this->iid;
}

const std::string InformationCollection::getName() const
{
  return this->streamDescription->getName();
}

const uint32_t InformationCollection::getHash()
{
  if (this->hash == 0)
  {
    this->hash = FNV::fnv1a(this->streamDescription->getName());
  }

  return this->hash;
}

const std::string InformationCollection::getProvider() const
{
  return this->streamDescription->getProvider();
}


std::shared_ptr<CollectionDescription> InformationCollection::getStreamDescription()
{
  return this->streamDescription;
}

int ice::InformationCollection::registerTaskAsync(std::shared_ptr<AsynchronousTask> task)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (auto taskItr : this->taskAsynchronous)
  {
    if (taskItr == task)
      return 1;
  }

  this->taskAsynchronous.push_back(task);

  return 0;
}

int InformationCollection::unregisterTaskAsync(std::shared_ptr<AsynchronousTask> task)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (int i = 0; i < this->taskAsynchronous.size(); ++i)
  {
    auto taskItr = this->taskAsynchronous[i];

    if (taskItr == task)
    {
      this->taskAsynchronous.erase(this->taskAsynchronous.begin() + i);
      return 0;
    }
  }

  return 1;
}

int InformationCollection::registerTaskSync(std::shared_ptr<AsynchronousTask> task)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (auto taskItr : this->taskSynchronous)
  {
    if (taskItr == task)
      return 1;
  }

  this->taskSynchronous.push_back(task);

  return 0;
}

int InformationCollection::unregisterTaskSync(std::shared_ptr<AsynchronousTask> task)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (int i = 0; i < this->taskSynchronous.size(); ++i)
  {
    auto taskItr = this->taskSynchronous[i];

    if (taskItr == task)
    {
      this->taskSynchronous.erase(this->taskSynchronous.begin() + i);
      return 0;
    }
  }

  return 1;
}

int InformationCollection::registerRemoteListener(std::shared_ptr<Entity> &entity,
                                                  std::shared_ptr<CommunicationInterface> &communication)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  if (std::find(this->remoteListeners.begin(), this->remoteListeners.end(), entity) != this->remoteListeners.end())
  {
    return 1;
  }

  this->remoteListeners.push_back(entity);

  if (this->remoteListeners.size() > 0)
  {
    _log->info("Register sender for stream '%v'", this->toString());
    this->registerSender(communication);
  }

  return 0;
}

int InformationCollection::unregisterRemoteListener(std::shared_ptr<Entity> &entity)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  auto ent = std::find(this->remoteListeners.begin(), this->remoteListeners.end(), entity);

  if (ent != this->remoteListeners.end())
  {
    this->remoteListeners.erase(ent);
    if (this->remoteListeners.empty())
    {
      this->dropSender();
    }
    return 0;
  }

  return 1;
}

int InformationCollection::setRemoteSource(std::shared_ptr<Entity> entity,
                                           std::shared_ptr<CommunicationInterface> &communication)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  if (this->remoteSource == entity)
    return 1;

  if (this->remoteSource != nullptr)
    this->dropReceiver();

  this->remoteSource = entity;
  if (entity != nullptr)
    this->registerReceiver(communication);

  return 0;
}

void InformationCollection::dropReceiver()
{
}

int InformationCollection::getSharingCount() const
{
  return this->remoteListeners.size();
}

std::string InformationCollection::toString()
{
  std::stringstream ss;

  ss << "stream(" << this->streamDescription->toString() << "," << this->iid << ")";

  return ss.str();
}

void InformationCollection::destroy()
{
  this->dropSender();
  this->dropReceiver();
}

} /* namespace ice */
