/*
 * BaseInformationStream.cpp
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#include "ice/information/BaseInformationStream.h"

#include <sstream>

#include "ice/information/InformationType.h"
#include "ice/processing/EventHandler.h"
#include "ice/Entity.h"
#include "easylogging++.h"

namespace ice
{

int BaseInformationStream::IDENTIFIER_COUNTER = 0;

BaseInformationStream::BaseInformationStream(std::shared_ptr<StreamDescription> streamDescription,
                                             std::shared_ptr<EventHandler> eventHandler,
                                             int sharingMaxCount) :
    streamDescription(streamDescription), iid(IDENTIFIER_COUNTER++), eventHandler(eventHandler),
    sharingMaxCount(sharingMaxCount)
{
  this->_log = el::Loggers::getLogger("InformationStream");
}

BaseInformationStream::~BaseInformationStream()
{
  // currently nothing to do here
}

std::shared_ptr<InformationSpecification> BaseInformationStream::getSpecification() const
{
  return this->streamDescription->getInformationSpecification();
}

const int BaseInformationStream::getIID() const
{
  return this->iid;
}

const std::string BaseInformationStream::getName() const
{
  return this->streamDescription->getName();
}

const std::string BaseInformationStream::getProvider() const
{
  return this->streamDescription->getProvider();
}


std::shared_ptr<StreamDescription> BaseInformationStream::getStreamDescription()
{
  return this->streamDescription;
}

int ice::BaseInformationStream::registerTaskAsync(std::shared_ptr<AsynchronousTask> task)
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

int BaseInformationStream::unregisterTaskAsync(std::shared_ptr<AsynchronousTask> task)
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

int BaseInformationStream::registerTaskSync(std::shared_ptr<AsynchronousTask> task)
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

int BaseInformationStream::unregisterTaskSync(std::shared_ptr<AsynchronousTask> task)
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

int BaseInformationStream::registerRemoteListener(std::shared_ptr<Entity> &entity,
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

int BaseInformationStream::unregisterRemoteListener(std::shared_ptr<Entity> &entity)
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

int BaseInformationStream::setRemoteSource(std::shared_ptr<Entity> entity,
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

void BaseInformationStream::dropReceiver()
{
}

bool BaseInformationStream::canBeShared()
{
  return this->remoteListeners.size() < this->sharingMaxCount -1 && this->streamDescription->isShared();
}

int BaseInformationStream::getSharingCount() const
{
  return this->remoteListeners.size();
}

int BaseInformationStream::getSharingMaxCount() const
{
  return sharingMaxCount;
}

void BaseInformationStream::setSharingMaxCount(int sharingMaxCount)
{
  this->sharingMaxCount = sharingMaxCount;
}

std::string BaseInformationStream::toString()
{
  std::stringstream ss;

  ss << "stream(" << this->streamDescription->toString() << ",";
  ss << this->iid;
  ss << ")";

  return ss.str();
}

void BaseInformationStream::destroy()
{
  this->dropSender();
  this->dropReceiver();
}

} /* namespace ice */
