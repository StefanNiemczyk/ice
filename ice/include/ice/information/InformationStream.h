/*
 * InformationStream.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef INFORMATIONSTREAM_H_
#define INFORMATIONSTREAM_H_

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <typeinfo>
#include <vector>

#include "ice/Time.h"
#include "ice/container/RingBuffer.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/InformationEvent.h"

//Forward declaration
namespace ice
{
class BaseInformationSender;
template<typename T>
  class AbstractInformationListener;
class Communication;
class InformationReceiver;
template<typename T>
  class InformationSender;
class InformationType;
} /* namespace ice */

namespace ice
{
//* InformationStream
/**
 * The class stored information for a given type. The stream is initialized with a
 * fixed size, which enables to access old information. Access the stored information is thread save.
 *
 */
template<typename T>
  class InformationStream : public BaseInformationStream, public std::enable_shared_from_this<InformationStream<T>>
  {

  public:
    /*!
     * \brief The constructor initialize the stream with a given stream size.
     *
     * The constructor initialize the stream with a given stream size.
     *
     * \param name The name of the stream.
     * \param informationType The information type which holds this stream.
     * \param eventHandler Handler to execute events asynchronously.
     * \param specification The specification of the stored information.
     * \param streamSize The count of information elements within this stream.
     */
    // InformationStream(const std::string name, std::weak_ptr<InformationType> informationType,
    //                     std::shared_ptr<EventHandler> eventHandler,
    //                  std::shared_ptr<InformationSpecification> specification, int streamSize);
    /*!
     * \brief The constructor initialize the stream with a given stream size.
     *
     * The constructor initialize the stream with a given stream size.
     *
     * \param name The name of the stream.
     * \param informationType The information type which holds this stream.
     * \param eventHandler Handler to execute events asynchronously.
     * \param specification The specification of the stored information.
     * \param streamSize The count of information elements within this stream.
     * \param provider The provider of the information stored in this stream.
     * \param description The description of this stream.
     * \param shared True if the stream is shared, else false.
     */
    InformationStream(const std::string name, std::weak_ptr<InformationType> informationType,
                      std::shared_ptr<EventHandler> eventHandler,
                      std::shared_ptr<InformationSpecification> specification, int streamSize,
                      std::string provider = "", std::string description = "", bool shared = false);

    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~InformationStream();

    /*!
     * \brief Returns the last stream element or NULL if no element exists.
     *
     * Returns the last stream element or NULL if no element exists.
     */
    std::shared_ptr<InformationElement<T>> getLast();

    /*!
     * \brief Returns the n-th last stream element or NULL.
     *
     * Returns the n-th last stream element. Returns NULL if no element exists or n > maxBuffer.
     *
     * \param n The n-th last information element, 0 will return the newest one.
     */
    std::shared_ptr<InformationElement<T>> getLast(int n);

    /*!
     * \brief Adds a new information to the stream and returns the identifier.
     *
     * Adds a new information to the stream and returns the identifier. An asynchronous event
     * is triggered for each registered listener.
     *
     * \param information The Information to add.
     */
    //int add(std::shared_ptr<InformationElement<T>> information);
    /*!
     * \brief Creates and adds a new information to the stream and returns the identifier.
     *
     * Creates and adds a new information to the stream and returns the identifier. An asynchronous
     * event is triggered for each registered listener.
     *
     * \param information The Information to add.
     * \param timeValidity validity time of the information.
     * \param timeObservation observation time of the information.
     * \param timeProcessed time of the processing of the information.
     */
    int add(std::unique_ptr<T> information, time timeValidity = NO_TIME, time timeObservation = NO_TIME,
            time timeProcessed = NO_TIME);

    /*!
     * \brief Return the buffer size.
     *
     * Return the buffer size.
     */
    int getStreamSize() const;

    /*!
     * \brief Clears the stream.
     *
     * Clears the stream. If cleanBuffer is false only the index structure is reseted, but the
     * buffer still exists (old information are not accessible). If cleanBuffer is true the
     * pointers from the buffer are cleared as well.
     *
     * \param cleanBuffer True to clear the buffer.
     */
    int clear(bool cleanBuffer);

    /*!
     * \brief Registers this stream in the communication class as sending stream.
     *
     * Registers this stream in the communication class as sending stream.
     */
    virtual std::shared_ptr<BaseInformationSender> registerSender(std::shared_ptr<Communication> communication);

    /*!
     * \brief Registers this stream in the communication class as receiving stream.
     *
     * Registers this stream in the communication class as receiving stream.
     */
    virtual std::shared_ptr<InformationReceiver> registerReceiver(std::shared_ptr<Communication> communication);

    /*!
     *\brief Returns the type_info of the used template type.
     *
     * Returns the type_info of the used template type.
     */
    const std::type_info* getTypeInfo() const;

    /*!
     * \brief Registered a listener which is asynchronous triggered if a new information element is added.
     *
     *  Registered a listener which is asynchronous triggered if a new information element is added.
     */
    int registerListenerAsync(std::shared_ptr<AbstractInformationListener<T>> listener);

    /*!
     * \brief Registered a listener which is synchronous triggered if a new information element is added.
     *
     *  Registered a listener which is synchronous triggered if a new information element is added.
     */
    int registerListenerSync(std::shared_ptr<AbstractInformationListener<T>> listener);

    /*!
     * \brief Filters the existing elements within the stream and add these to filteredList.
     *
     * Filters the existing elements within the stream and add these to filteredList. The function
     * func is used to decide if an element is filtered out (return false) or added to the list
     * (return true).
     *
     * \param filteredList List of resulting filtered elements
     * \param func The filter function
     */
    const int getFilteredList(std::shared_ptr<std::vector<std::shared_ptr<InformationElement<T> > > > filteredList,
                              std::function<bool(std::shared_ptr<InformationElement<T> >)> func);

    /*!
     * \brief Removes the receiver of this stream.
     *
     * Removes the receiver of this stream.
     */
    virtual void dropReceiver();

  protected:
    /*!
     * \brief This method is calls if the last engine state will be unregistered.
     *
     * This method is calls if the last engine state will be unregistered.
     */
    virtual void allEngineStatesUnregistered();

  private:
    std::unique_ptr<RingBuffer<InformationElement<T>>> ringBuffer; /**< Ring buffer of information elements */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>> listenersAsynchronous; /**< List of asynchronous triggered listeners */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>> listenersSynchronous; /**< List of synchronous triggered listeners */
    std::shared_ptr<InformationSender<T>> sender; /**< Sender to send information elements */
    std::shared_ptr<InformationReceiver> receiver; /**< Receiver to receive information elements */
  };

}
/* namespace ice */

//Include after forward declaration
#include "ice/communication/Communication.h"
#include "ice/communication/InformationSender.h"
#include "ice/coordination/EngineState.h"
#include "ice/information/AbstractInformationListener.h"
#include "ice/information/InformationType.h"

//Implementing methods here
/*
 template<typename T>
 ice::InformationStream<T>::InformationStream(const std::string name,
 std::weak_ptr<InformationType> informationType,
 std::shared_ptr<EventHandler> eventHandler,
 std::shared_ptr<InformationSpecification> specification,
 int streamSize) :
 BaseInformationStream(name, informationType, eventHandler, specification)
 {
 this->ringBuffer = std::unique_ptr<RingBuffer<InformationElement<T> > >(
 new RingBuffer<InformationElement<T> >(streamSize));
 }*/

template<typename T>
  ice::InformationStream<T>::InformationStream(const std::string name, std::weak_ptr<InformationType> informationType,
                                               std::shared_ptr<EventHandler> eventHandler,
                                               std::shared_ptr<InformationSpecification> specification, int streamSize,
                                               std::string provider, std::string description, bool shared) :
      BaseInformationStream(name, informationType, eventHandler, specification, provider, description, shared)
  {
    this->ringBuffer = std::unique_ptr<RingBuffer<InformationElement<T>>>(
    new RingBuffer<InformationElement<T>>(streamSize));
  }

template<typename T>
  ice::InformationStream<T>::~InformationStream()
  {
    // currently nothing to do
  }

template<typename T>
  std::shared_ptr<ice::InformationElement<T> > ice::InformationStream<T>::getLast()
  {
    return this->getLast(0);
  }

template<typename T>
  std::shared_ptr<ice::InformationElement<T> > ice::InformationStream<T>::getLast(int n)
  {
    std::lock_guard<std::mutex> guard(_mtx);
    return this->ringBuffer->getLast(n);
  }

template<typename T>
  int ice::InformationStream<T>::add(std::unique_ptr<T> information, time timeValidity, time timeObservation,
                                     time timeProcessed)
  {
    std::lock_guard<std::mutex> guard(_mtx);
    auto informationElement = std::make_shared<InformationElement<T>>(this->specification, std::move(information),
                                                                      timeValidity, timeObservation, timeProcessed);

    int returnValue = this->ringBuffer->add(informationElement);

    // notify listeners
    for (auto listener : this->listenersAsynchronous)
    {
      auto event = std::make_shared<InformationEvent<T> >(listener, informationElement, this->shared_from_this());

      this->eventHandler->addTask(event);
    }

    for (auto listener : this->listenersSynchronous)
    {
      listener->newEvent(informationElement, this->shared_from_this());
    }

    // execute tasks
    for (auto task : this->taskAsynchronous)
    {
      this->eventHandler->addTask(task);
    }

    for (auto task : this->taskSynchronous)
    {
      task->performTask();
    }

    // send information
    if (this->remoteListeners.size() > 0)
    {
      std::shared_ptr<std::vector<identifier>> sendTo = std::make_shared<std::vector<identifier>>(
          this->remoteListeners.size());

      for (auto engine : this->remoteListeners)
      {
        sendTo->push_back(engine->getEngineId());
      }

      if (this->sender)
      {
        this->sender->sendInformationElement(sendTo, informationElement);
      }
      else
      {
        _log->error("add", "No sender for stream %s", &this->name);
      }
    }

    return returnValue;
  }

template<typename T>
  int ice::InformationStream<T>::getStreamSize() const
  {
    return this->ringBuffer->getBufferSize();
  }

template<typename T>
  int ice::InformationStream<T>::clear(bool cleanBuffer)
  {
    std::lock_guard<std::mutex> guard(_mtx);
    return ringBuffer->clear(cleanBuffer);
  }

template<typename T>
  std::shared_ptr<ice::BaseInformationSender> ice::InformationStream<T>::registerSender(
      std::shared_ptr<Communication> communication)
  {
    if (this->sender)
    {
      return this->sender;
    }

    std::lock_guard<std::mutex> guard(_mtx);
    if (this->sender)
    {
      return this->sender;
    }

    auto comResult = communication->registerStreamAsSender(this->shared_from_this());

    if (false == comResult)
    {
      _log->error("registerSender", "No sender returned for stream %s", std::string(this->name).c_str());
      std::shared_ptr<ice::BaseInformationSender> ptr;
      return ptr;
    }

    if (typeid(T) == *comResult->getTypeInfo())
    {
      this->sender = std::static_pointer_cast<InformationSender<T> >(comResult);
      return comResult;
    }
    else
    {
      _log->error("registerSender", "Incorrect type of sender %s for stream %s", comResult->getTypeInfo(),
                  std::string(this->name).c_str());
      std::shared_ptr<ice::BaseInformationSender> ptr;
      return ptr;
    }
  }

template<typename T>
  std::shared_ptr<ice::InformationReceiver> ice::InformationStream<T>::registerReceiver(
      std::shared_ptr<Communication> communication)
  {
    std::lock_guard<std::mutex> guard(_mtx);
    auto comResult = communication->registerStreamAsReceiver(this->shared_from_this());

    if (false == comResult)
    {
      _log->error("registerReceiver", "No receiver returned for stream %s", std::string(this->name).c_str());
      std::shared_ptr<ice::InformationReceiver> ptr;
      return ptr;
    }

    this->receiver = comResult;

    return comResult;
  }

template<typename T>
  const std::type_info* ice::InformationStream<T>::getTypeInfo() const
  {
    return &typeid(T);
  }

template<typename T>
  int ice::InformationStream<T>::registerListenerAsync(std::shared_ptr<AbstractInformationListener<T> > listener)
  {
    this->listenersAsynchronous.push_back(listener);

    return 0;
  }

template<typename T>
  int ice::InformationStream<T>::registerListenerSync(std::shared_ptr<AbstractInformationListener<T> > listener)
  {
    this->listenersSynchronous.push_back(listener);

    return 0;
  }

template<typename T>
  inline const int ice::InformationStream<T>::getFilteredList(
      std::shared_ptr<std::vector<std::shared_ptr<InformationElement<T> > > > filteredList,
      std::function<bool(std::shared_ptr<InformationElement<T> >)> func)
  {
    std::lock_guard<std::mutex> guard(_mtx);
    int count = 0;
    std::shared_ptr<InformationElement<T> > ptr;

    for (int i = this->ringBuffer->getSize() - 1; i >= 0; --i)
    {
      ptr = this->ringBuffer->getLast(i);
      if (ptr && func(ptr))
      {
        filteredList->push_back(ptr);
        ++count;
      }
    }

    return count;
  }

template<typename T>
  inline void ice::InformationStream<T>::dropReceiver()
  {
  this->receiver.reset();
  }

template<typename T>
  inline void ice::InformationStream<T>::allEngineStatesUnregistered()
  {
  this->sender.reset();
  }

#endif /* INFORMATIONSTREAM_H_ */
