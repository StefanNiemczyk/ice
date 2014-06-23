/*
 * EventHandler.cpp
 *
 *  Created on: May 21, 2014
 *      Author: sni
 */

#include "ice/processing/EventHandler.h"
#include "ice/ICEngine.h"

namespace ice
{

EventHandler::EventHandler(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
  std::shared_ptr<Configuration> config;

  if (engine.expired())
    return;

  config = engine.lock()->getConfig();

  this->taskBuffer = std::unique_ptr<Buffer<AsynchronousTask> >(new Buffer<AsynchronousTask>(config->getEventHandlerBufferSize(), false));
  this->running = true;

  //initialize threads
  for (int i = 0; i < config->getEventHandlerThreadCount(); ++i)
  {
    this->threadPool.push_back(std::thread(&EventHandler::performTask, this, i));
  }
}

EventHandler::EventHandler(const int numThreads, const int bufferSize)
{
  this->taskBuffer = std::unique_ptr<Buffer<AsynchronousTask> >(new Buffer<AsynchronousTask>(bufferSize, false));
  this->running = true;

  //initialize threads
  for (int i = 0; i < numThreads; ++i)
  {
    this->threadPool.push_back(std::thread(&EventHandler::performTask, this, i));
  }
}

EventHandler::~EventHandler()
{
  //stopping all threads
  this->running = false;

  this->cv.notify_all();

  for (auto &t : this->threadPool)
  {
    t.join();
  }
}

const int EventHandler::addTask(std::shared_ptr<AsynchronousTask> event)
{
  std::unique_lock<std::mutex> lock(mtx_);

  std::shared_ptr<AsynchronousTask> returnValue = this->taskBuffer->add(event);

  if (returnValue)
  {
    std::cerr << "Error, Event Buffer full, dropping event" << returnValue << std::endl;

    return 1;
  }

  cv.notify_one();

  return 0;
}

const int EventHandler::addTimerTaks(std::shared_ptr<AsynchronousTask> task, long time)
{
  // /TODO implement timer task
  return 0;
}

std::shared_ptr<AsynchronousTask> EventHandler::popTask()
{
  std::shared_ptr<AsynchronousTask> ptr;

  std::unique_lock<std::mutex> lock(mtx_);
  this->cv.wait(lock, [&]()
  { return false == this->taskBuffer->isEmpty() || false == this->running;});

  if (false == this->running)
  {
    return ptr;
  }

  ptr = this->taskBuffer->popFirst();

  return ptr;
}

const int EventHandler::getThreadPoolSize() const
{
  return this->threadPool.size();
}

const int EventHandler::getBufferSize() const
{
  return this->taskBuffer->getBufferSize();
}

void EventHandler::performTask(const int id)
{
  std::shared_ptr<AsynchronousTask> ptr;

//  {
//    std::lock_guard<std::mutex> guard(this->mtx_);
//    std::cerr << id << ": Starting" << std::endl;
//  }

  while (this->running)
  {
    // get next event
    ptr = this->popTask();

    //no event exists
    if (false == ptr)
      continue;

    //perform event
    ptr->performTask();
  }

//  {
//    std::lock_guard<std::mutex> guard(this->mtx_);
//    std::cerr << id << ": Stopped" << std::endl;
//  }
}

} /* namespace ice */

