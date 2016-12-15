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
#include "ice/communication/CommunicationInterface.h"
#include "ice/communication/InformationSender.h"
#include "ice/container/RingBuffer.h"
#include "ice/information/AbstractInformationListener.h"
#include "ice/information/CollectionDescription.h"
#include "ice/information/BaseInformationStream.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/InformationEvent.h"
#include "ice/representation/GContainer.h"
#include "easylogging++.h"

//Forward declaration
namespace ice
{
class BaseInformationSender;
class InformationReceiver;
} /* namespace ice */

namespace ice
{
//* InformationStream
/**
 * The class stores information for a given type. The stream is initialized with a
 * fixed size, which enables to access old information. Access the stored information is thread save.
 *
 */
template<typename T>
  class InformationStream : public BaseInformationStream, public std::enable_shared_from_this<InformationStream<T>>,
                            public AbstractInformationListener<T>
  {

  public:

    /*!
     * \brief The constructor initialize the stream with a given stream size.
     *
     * The constructor initialize the stream with a given stream size.
     *
     * \param streamDescription The description of this stream.
     * \param eventHandler Handler to execute events asynchronously.
     * \param streamSize The count of information elements within this stream.
     */
    InformationStream(std::shared_ptr<CollectionDescription> streamDescription,
                          std::shared_ptr<EventHandler> eventHandler, int streamSize)
            : BaseInformationStream(streamDescription, eventHandler)
    {
      this->ringBuffer = std::unique_ptr<RingBuffer<InformationElement<T>>>(
      new RingBuffer<InformationElement<T>>(streamSize));
    }

    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~InformationStream() {};

    /*!
     * \brief Returns the last stream element or NULL if no element exists.
     *
     * Returns the last stream element or NULL if no element exists.
     */
    std::shared_ptr<InformationElement<T>> getLast()
    {
      return this->getLast(0);
    }

    /*!
     * \brief Returns the n-th last stream element or NULL.
     *
     * Returns the n-th last stream element. Returns NULL if no element exists or n > maxBuffer.
     *
     * \param n The n-th last information element, 0 will return the newest one.
     */
    std::shared_ptr<InformationElement<T>> getLast(int n)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      return this->ringBuffer->getLast(n);
    }

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
    int add(std::shared_ptr<T> information, time timeValidity = NO_TIME, time timeObservation = NO_TIME,
            time timeProcessed = NO_TIME)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      auto informationElement = std::make_shared<InformationElement<T>>(
          this->description->getInformationSpecification(), information, timeValidity, timeObservation,
          timeProcessed);

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
        if (this->sender)
        {
          this->sender->sendInformationElement(this->remoteListeners, informationElement);
        }
        else if (this->genericSender)
        {
          auto informationElement = std::make_shared<InformationElement<GContainer>>(
              this->description->getInformationSpecification(), information, timeValidity, timeObservation,
              timeProcessed);

            this->genericSender->sendInformationElement(this->remoteListeners, informationElement);
        }
        else
        {
          _log->error("No sender for stream %v", this->description->getName());
        }
      }

      return returnValue;
    }

   virtual const int newEvent(std::shared_ptr<InformationElement<T>> element,
                              std::shared_ptr<InformationCollection> stream)
   {
     this->add(element->getInformation(), element->getTimeValidity(),
                    element->getTimeObservation(), element->getTimeProcessed());

     return 0;
   }

    /*!
     * \brief Return the buffer size.
     *
     * Return the buffer size.
     */
    int getStreamSize() const
    {
      return this->ringBuffer->getBufferSize();
    }

    int getSize() const
    {
      return this->ringBuffer->getSize();
    }

    /*!
     * \brief Clears the stream.
     *
     * Clears the stream. If cleanBuffer is false only the index structure is reseted, but the
     * buffer still exists (old information are not accessible). If cleanBuffer is true the
     * pointers from the buffer are cleared as well.
     *
     * \param cleanBuffer True to clear the buffer.
     */
    int clear(bool cleanBuffer)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      return ringBuffer->clear(cleanBuffer);
    }

    /*!
     *\brief Returns the type_info of the used template type.
     *
     * Returns the type_info of the used template type.
     */
    const std::type_info* getTypeInfo() const
    {
      return &typeid(T);
    }

    /*!
     * \brief Registered a listener which is asynchronous triggered if a new information element is added.
     *
     *  Registered a listener which is asynchronous triggered if a new information element is added.
     */
    int registerListenerAsync(std::shared_ptr<AbstractInformationListener<T>> listener)
    {
    std::lock_guard<std::mutex> guard(this->_mtxRegister);
      this->listenersAsynchronous.push_back(listener);

      return 0;
    }

    int unregisterListenerAsync(std::shared_ptr<AbstractInformationListener<T>> listener)
    {
      std::lock_guard<std::mutex> guard(this->_mtxRegister);

      for (int i = 0; i < this->listenersAsynchronous.size(); ++i)
      {
        if (this->listenersAsynchronous[i] == listener)
        {
          this->listenersAsynchronous.erase(this->listenersAsynchronous.begin() + i);

          return 0;
        }
      }

      return 1;
    }

    /*!
     * \brief Registered a listener which is synchronous triggered if a new information element is added.
     *
     *  Registered a listener which is synchronous triggered if a new information element is added.
     */
    int registerListenerSync(std::shared_ptr<AbstractInformationListener<T>> listener)
    {
    std::lock_guard<std::mutex> guard(this->_mtxRegister);
      this->listenersSynchronous.push_back(listener);

      return 0;
    }

    int unregisterListenerSync(std::shared_ptr<AbstractInformationListener<T>> listener)
    {
      std::lock_guard<std::mutex> guard(this->_mtxRegister);

      for (int i = 0; i < this->listenersSynchronous.size(); ++i)
      {
        if (this->listenersSynchronous[i] == listener)
        {
          this->listenersSynchronous.erase(this->listenersSynchronous.begin() + i);

          return 0;
        }
      }

      return 1;
    }

    virtual int registerBaseListenerSync(std::shared_ptr<BaseInformationStream> listener)
    {
      if (typeid(T).hash_code() != listener->getTypeInfo()->hash_code())
        throw std::bad_cast();

      auto stream = std::static_pointer_cast<InformationStream<T>>(listener);
      return this->registerListenerSync(stream);
    }

    virtual int unregisterBaseListenerSync(std::shared_ptr<BaseInformationStream> listener)
    {
      if (typeid(T).hash_code() != listener->getTypeInfo()->hash_code())
        throw std::bad_cast();

      auto stream = std::static_pointer_cast<InformationStream<T>>(listener);
      return this->unregisterListenerSync(stream);
    }

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
    const int getFilteredList(std::shared_ptr<std::vector<std::shared_ptr<InformationElement<T>>>> filteredList,
                              std::function<bool(std::shared_ptr<InformationElement<T>>&)> func)
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


  protected:
    /*!
     * \brief Registers this stream in the communication class as sending stream.
     *
     * Registers this stream in the communication class as sending stream.
     */
    virtual std::shared_ptr<BaseInformationSender> registerSender(std::shared_ptr<CommunicationInterface> &communication)
    {
      if (this->sender)
      {
        return this->sender;
      }
      if (this->genericSender)
      {
        return this->genericSender;
      }

      auto comResult = communication->registerCollectionAsSender(this->shared_from_this());

      if (comResult == nullptr)
      {
        _log->error("No sender returned for stream %v", this->description->getName());
        return nullptr;
      }

      if (typeid(T).hash_code() == comResult->getTypeInfo()->hash_code())
      {
        this->sender = std::static_pointer_cast<InformationSender<T>>(comResult);
        this->sender->init();
        return comResult;
      }
      else if(typeid(GContainer).hash_code() == comResult->getTypeInfo()->hash_code())
      {
        this->genericSender = std::static_pointer_cast<InformationSender<GContainer>>(comResult);
        this->genericSender->init();
        return comResult;
      }
      else
      {
        _log->error("Incorrect type of sender '%v' for stream '%v'", comResult->getTypeInfo()->name(),
                    this->description->getName());
        comResult->cleanUp();
        return nullptr;
      }
    }

    /*!
     * \brief Registers this stream in the communication class as receiving stream.
     *
     * Registers this stream in the communication class as receiving stream.
     */
    virtual std::shared_ptr<InformationReceiver> registerReceiver(std::shared_ptr<CommunicationInterface> &communication)
    {
      auto comResult = communication->registerCollectionAsReceiver(this->shared_from_this());

      if (comResult == nullptr)
      {
        _log->error("No receiver returned for stream %v", this->description->getName());
        return nullptr;
      }

      this->receiver = comResult;
      this->receiver->init();

      return comResult;
    }

    /*!
     * \brief Removes the receiver of this stream.
     *
     * Removes the receiver of this stream.
     */
    virtual void dropReceiver()
    {
      if (this->receiver)
        this->receiver->cleanUp();

      this->receiver.reset();
    }

    /*!
     * \brief This method is calls if the last engine state will be unregistered.
     *
     * This method is calls if the last engine state will be unregistered.
     */
    virtual void dropSender()
    {
      if (this->sender)
        this->sender->cleanUp();
      if (this->genericSender)
        this->genericSender->cleanUp();

      this->sender.reset();
      this->genericSender.reset();
    }

  private:
    std::unique_ptr<RingBuffer<InformationElement<T>>>  ringBuffer;             /**< Ring buffer of information elements */
    std::vector<std::shared_ptr<
            AbstractInformationListener<T>>>            listenersAsynchronous;  /**< List of asynchronous triggered listeners */
    std::vector<std::shared_ptr<
            AbstractInformationListener<T>>>            listenersSynchronous;   /**< List of synchronous triggered listeners */
    std::shared_ptr<InformationSender<T>>               sender;                 /**< Sender to send information elements */
    std::shared_ptr<InformationSender<GContainer>>      genericSender;          /**< Sender to send information elements */
    std::shared_ptr<InformationReceiver>                receiver;               /**< Receiver to receive information elements */
  };

}
/* namespace ice */

#endif /* INFORMATIONSTREAM_H_ */
