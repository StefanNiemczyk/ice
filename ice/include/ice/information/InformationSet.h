/*
 * InformationSet.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_INFORMATIONSET_H_
#define INCLUDE_ICE_INFORMATION_INFORMATIONSET_H_

#include <functional>
#include <iostream>
#include <map>
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
#include "ice/information/BaseInformationSet.h"
#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"
#include "ice/processing/InformationEvent.h"
#include "easylogging++.h"

namespace ice
{
//Forward declaration
class BaseInformationSender;
class InformationReceiver;

//* InformationSet
/**
 * The class stores information for a given type.
 *
 */
template<typename T>
  class InformationSet : public BaseInformationSet, public std::enable_shared_from_this<InformationSet<T>>
  {

  public:

    /*!
     * \brief The constructor initialize the set.
     *
     * The constructor initialize the set.
     *
     * \param description The description of this stream.
     * \param eventHandler Handler to execute events asynchronously.
     */
    InformationSet(std::shared_ptr<CollectionDescription> description, std::shared_ptr<EventHandler> eventHandler) :
      BaseInformationSet(description, eventHandler)
    {
      // currently nothing to do
    }

    /*!
     * \brief Default destructor
     *
     * Default destructor
     */
    virtual ~InformationSet()
    {
    }

    std::shared_ptr<InformationElement<T>> get(ont::entity entity)
    {
      auto ele = this->map.find(entity);
      if (ele == this->map.end())
      {
        return nullptr;
      }

      return ele->second;
    }

    /*!
     * \brief Creates and adds a new information to the set and returns the identifier.
     *
     * Creates and adds a new information to the set and returns the identifier. An asynchronous
     * event is triggered for each registered listener.
     *
     * \param entity The entity described by the information.
     * \param information The Information to add.
     * \param timeValidity validity time of the information.
     * \param timeObservation observation time of the information.
     * \param timeProcessed time of the processing of the information.
     */
    int add(ont::entity entity, std::shared_ptr<T> information, time timeValidity = NO_TIME, time timeObservation = NO_TIME,
            time timeProcessed = NO_TIME)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      auto tempSpec = this->streamDescription->getInformationSpecification();
      auto spec = std::make_shared<InformationSpecification>(entity, tempSpec->getEntityType(),
                                                             tempSpec->getScope(), tempSpec->getRepresentation(),
                                                             tempSpec->getRelatedEntity());
      auto informationElement = std::make_shared<InformationElement<T>>(
          spec, information, timeValidity, timeObservation, timeProcessed);

      this->entities.insert(entity);
      this->map[entity] = informationElement;

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
        else
        {
          _log->error("No sender for stream %v", this->streamDescription->getName());
        }
      }

      return 0;
    }

    /*!
     * \brief Return the buffer size.
     *
     * Return the buffer size.
     */
    int getSize() const
    {
      return this->map.size();
    }

    /*!
     * \brief Clears the set.
     *
     * Clears the set.
     *
     * \param cleanBuffer True to clear the buffer.
     */
    int clear()
    {
      std::lock_guard<std::mutex> guard(_mtx);
      return this->map.clear();
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
      this->listenersAsynchronous.push_back(listener);

      return 0;
    }

    /*!
     * \brief Registered a listener which is synchronous triggered if a new information element is added.
     *
     *  Registered a listener which is synchronous triggered if a new information element is added.
     */
    int registerListenerSync(std::shared_ptr<AbstractInformationListener<T>> listener)
    {
      this->listenersSynchronous.push_back(listener);

      return 0;
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
    const int getFilteredList(std::shared_ptr<std::vector<std::shared_ptr<InformationElement<T> > > > filteredList,
                              std::function<bool(std::shared_ptr<InformationElement<T>>&)> func)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      int count = 0;
      std::shared_ptr<InformationElement<T> > ptr;

      for (auto &element : this->map)
      {
        if (func(element.second))
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
    virtual std::shared_ptr<BaseInformationSender> registerSender(
        std::shared_ptr<CommunicationInterface> &communication)
    {
      if (this->sender)
      {
        return this->sender;
      }

      auto comResult = communication->registerCollectionAsSender(this->shared_from_this());

      if (false == comResult)
      {
        _log->error("No sender returned for stream %v", this->streamDescription->getName());
        std::shared_ptr<ice::BaseInformationSender> ptr;
        return ptr;
      }

      if (typeid(T) == *comResult->getTypeInfo())
      {
        this->sender = std::static_pointer_cast<InformationSender<T>>(comResult);
        return comResult;
      }
      else
      {
        _log->error("Incorrect type of sender %s for stream %s", comResult->getTypeInfo(),
                    this->streamDescription->getName());
        return nullptr;
      }
    }

    /*!
     * \brief Registers this stream in the communication class as receiving stream.
     *
     * Registers this stream in the communication class as receiving stream.
     */
    virtual std::shared_ptr<InformationReceiver> registerReceiver(
        std::shared_ptr<CommunicationInterface> &communication)
    {
      auto comResult = communication->registerCollectionAsReceiver(this->shared_from_this());

      if (false == comResult)
      {
        _log->error("No receiver returned for stream %s", this->streamDescription->getName());
        return nullptr;
      }

      this->receiver = comResult;

      return comResult;
    }

    /*!
     * \brief Removes the receiver of this stream.
     *
     * Removes the receiver of this stream.
     */
    virtual void dropReceiver()
    {
      this->receiver.reset();
    }

    /*!
     * \brief This method is calls if the last engine state will be unregistered.
     *
     * This method is calls if the last engine state will be unregistered.
     */
    virtual void dropSender()
    {
      this->sender.reset();
    }

  private:
    std::map<ont::entity, std::shared_ptr<ice::InformationElement<T>>> map; /**< Ring buffer of information elements */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>>listenersAsynchronous; /**< List of asynchronous triggered listeners */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>> listenersSynchronous; /**< List of synchronous triggered listeners */
    std::shared_ptr<InformationSender<T>> sender; /**< Sender to send information elements */
    std::shared_ptr<InformationReceiver> receiver; /**< Receiver to receive information elements */
  };

}
/* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_INFORMATIONSET_H_ */
