/*
 * EventHandler.h
 *
 *  Created on: May 21, 2014
 *      Author: sni
 */

#ifndef EVENTHANDLER_H_
#define EVENTHANDLER_H_

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ice/Configuration.h"
#include "ice/container/Buffer.h"
#include "ice/processing/AsynchronousTask.h"
#include "ice/processing/InformationEvent.h"

namespace ice
{

class ICEngine;

class EventHandler
{
public:
  /*!
   * \brief This constructor creates a buffer and initialize a thread pool based on the
   * configuration of the engine.
   *
   * This constructor creates a buffer and initialize a thread pool based on the configuration
   * of the engine.
   *
   * \param engine A weak pointer to the icengine.
   */
  EventHandler(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief This constructor creates a buffer of size bufferSize and initialize a thread pool
   * with numThreads threads.
   *
   * This constructor creates a buffer of size bufferSize and initialize a thread pool with
   * numThreads threads.
   *
   * \param numThreads The number of threads used by this pool.
   * \param bufferSize The size of the event buffer.
   */
  EventHandler(const int numThreads, const int bufferSize);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~EventHandler();

  void init();

  void cleanUp();

  /*!
   * \brief Adds a new task to perform.
   *
   * Adds a new task to perform into the event buffer.
   *
   * \param event The task to add.
   */
  const int addTask(std::shared_ptr<AsynchronousTask> task);

  /*!
   * \brief Adds a task which will be executed in time milliseconds.
   *
   * Adds a task which will be executed in time milliseconds.
   *
   * @param task The task which should be executed.
   * @param doIn The time in milliseconds until the task should be executed.
   */
  const int addTimerTask(std::shared_ptr<AsynchronousTask> task, long time);

  /*!
   * \brief Returns the number of threads within the thread pool.
   *
   * Returns the number of threads within the thread pool.
   */
  const int getThreadPoolSize() const;

  /*!
   * \brief Return the event buffer size.
   *
   * Return the event buffer size.
   */
  const int getBufferSize() const;

protected:

  /*!
   * \brief Returns a asynchronous task from the buffer.
   *
   * Returns a asynchronous task from the buffer. The buffer access is thread safe.
   */
  std::shared_ptr<AsynchronousTask> popTask();

  /*!
   * \brief Method executed by a single thread.
   *
   * Method executed by a single thread.
   *
   * \param id The id of the thread within the thread pool.
   */
  void performTask(const int id);

private:
  std::weak_ptr<ICEngine> engine; /**< Weak pointer to the engine */
  std::unique_ptr<Buffer<AsynchronousTask>> taskBuffer; /**< buffer to store asynchronous tasks */
  std::mutex mtx_; /**< Mutex to synchronize the event operations */
  std::vector<std::thread> threadPool; /**< Pool of threads to handle asynchronous tasks */
  std::condition_variable cv; /**< Condition variable for synchronizing threads */
  bool running;
};

} /* namespace ice */

#endif /* EVENTHANDLER_H_ */
