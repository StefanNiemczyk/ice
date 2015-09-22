/*
 * BaseInformationStream.cpp
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#include "ice/information/BaseInformationStream.h"

#include <sstream>

#include "ice/coordination/EngineState.h"
#include "ice/information/InformationType.h"
#include "ice/processing/EventHandler.h"
#include "easylogging++.h"

namespace ice
{

int BaseInformationStream::IDENTIFIER_COUNTER = 0;

BaseInformationStream::BaseInformationStream(std::shared_ptr<StreamDescription> streamDescription,
                                             std::shared_ptr<EventHandler> eventHandler,
                                             int sharingMaxCount) :
    streamDescription(streamDescription), iid(IDENTIFIER_COUNTER++)
{
  this->eventHandler = eventHandler;
//  this->description = description;
  this->sharingMaxCount = sharingMaxCount;
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

//void BaseInformationStream::setProvider(std::string provider)
//{
//  this->provider = provider;
//}

//const std::string BaseInformationStream::getDescription() const
//{
//  return this->description;
//}
//
//void BaseInformationStream::setDescription(std::string description)
//{
//  this->description = description;
//}

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

int ice::BaseInformationStream::unregisterTaskAsync(std::shared_ptr<AsynchronousTask> task)
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

int ice::BaseInformationStream::registerTaskSync(std::shared_ptr<AsynchronousTask> task)
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

int ice::BaseInformationStream::unregisterTaskSync(std::shared_ptr<AsynchronousTask> task)
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

std::shared_ptr<StreamDescription> BaseInformationStream::getStreamDescription()
{
//  if (this->streamDescription)
//    return this->streamDescription;
//
//  std::lock_guard<std::mutex> guard(this->_mtx);
//
//  if (this->streamDescription)
//    return this->streamDescription;
//
//  this->streamDescription = std::make_shared<StreamDescription>(this->getSpecification(), this->shared);

  return this->streamDescription;
}

int BaseInformationStream::registerEngineState(std::shared_ptr<EngineState> engineState)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (auto itr : this->remoteListeners)
  {
    if (itr == engineState)
      return 1;
  }

  this->remoteListeners.push_back(engineState);

  return 0;
}

//const std::weak_ptr<InformationStreamTemplate> BaseInformationStream::getStreamTemplate() const
//{
//  return streamTemplate;
//}
//
//void BaseInformationStream::setStreamTemplate(const std::weak_ptr<InformationStreamTemplate> streamTemplate)
//{
//  this->streamTemplate = streamTemplate;
//}

int BaseInformationStream::unregisterEngineState(std::shared_ptr<EngineState> engineState)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  for (int i = 0; i < this->remoteListeners.size(); ++i)
  {
    auto itr = this->remoteListeners[i];

    if (itr == engineState)
    {
      this->remoteListeners.erase(this->remoteListeners.begin() + i);

      this->allEngineStatesUnregistered();

      return 0;
    }
  }

  return 1;
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
  this->allEngineStatesUnregistered();
  this->dropReceiver();
}

} /* namespace ice */
