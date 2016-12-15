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
#include "ice/communication/InformationReceiver.h"
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
  class InformationSet : public BaseInformationSet, public std::enable_shared_from_this<InformationSet<T>>,
                         public AbstractInformationListener<T>
  {

  public:

    /*!
     * \brief The constructor initialize the set.
     *
     * The constructor initialize the set.
     *
     * \param description The description of this set.
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
      auto tempSpec = this->description->getInformationSpecification();
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
        else if (this->genericSender)
        {
          auto informationElement = std::make_shared<InformationElement<GContainer>>(
              spec, information, timeValidity, timeObservation, timeProcessed);

            this->genericSender->sendInformationElement(this->remoteListeners, informationElement);
        }
        else
        {
          _log->error("No sender for set %v", this->description->getName());
        }
      }

      return 0;
    }

    virtual const int newEvent(std::shared_ptr<InformationElement<T>> element,
                               std::shared_ptr<InformationCollection> set)
    {
      this->add(element->getSpecification()->getEntity(), element->getInformation(), element->getTimeValidity(),
                     element->getTimeObservation(), element->getTimeProcessed());

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

    virtual int registerBaseListenerSync(std::shared_ptr<BaseInformationSet> listener)
    {
      if (typeid(T).hash_code() != listener->getTypeInfo()->hash_code())
        throw std::bad_cast();

      auto set = std::static_pointer_cast<InformationSet<T>>(listener);
      return this->registerListenerSync(set);
    }

    virtual int unregisterBaseListenerSync(std::shared_ptr<BaseInformationSet> listener)
    {
      if (typeid(T).hash_code() != listener->getTypeInfo()->hash_code())
        throw std::bad_cast();

      auto set = std::static_pointer_cast<InformationSet<T>>(listener);
      return this->unregisterListenerSync(set);
    }

    /*!
     * \brief Filters the existing elements within the set and add these to filteredList.
     *
     * Filters the existing elements within the set and add these to filteredList. The function
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

      for (auto &element : this->map)
      {
        if (func(element.second))
        {
          filteredList->push_back(element.second);
          ++count;
        }
      }

      return count;
    }

    std::shared_ptr<InformationElement<T>> getOptimal(std::function<bool(std::shared_ptr<InformationElement<T>>&, std::shared_ptr<InformationElement<T>>&)> func)
    {
      std::lock_guard<std::mutex> guard(_mtx);
      int count = 0;
      std::shared_ptr<InformationElement<T>> optimal = nullptr;

      for (auto &element : this->map)
      {
        if (optimal == nullptr)
          optimal = element.second;

        if (func(optimal, element.second))
        {
          optimal = element.second;
          ++count;
        }
      }

      return optimal;
    }

  protected:
    /*!
     * \brief Registers this set in the communication class as sending set.
     *
     * Registers this set in the communication class as sending set.
     */
    virtual std::shared_ptr<BaseInformationSender> registerSender(
        std::shared_ptr<CommunicationInterface> &communication)
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

      if (false == comResult)
      {
        _log->error("No sender returned for set %v", this->description->getName());
        std::shared_ptr<ice::BaseInformationSender> ptr;
        return ptr;
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
        _log->error("Incorrect type of sender '%v' for set '%v'", comResult->getTypeInfo()->name(),
                    this->description->getName());
        comResult->cleanUp();
        return nullptr;
      }
    }

    /*!
     * \brief Registers this set in the communication class as receiving set.
     *
     * Registers this set in the communication class as receiving set.
     */
    virtual std::shared_ptr<InformationReceiver> registerReceiver(
        std::shared_ptr<CommunicationInterface> &communication)
    {
      auto comResult = communication->registerCollectionAsReceiver(this->shared_from_this());

      if (false == comResult)
      {
        _log->error("No receiver returned for set %v", this->description->getName());
        return nullptr;
      }

      this->receiver = comResult;
      this->receiver->init();

      return comResult;
    }

    /*!
     * \brief Removes the receiver of this set.
     *
     * Removes the receiver of this set.
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
    std::map<ont::entity, std::shared_ptr<ice::InformationElement<T>>> map; /**< Ring buffer of information elements */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>>listenersAsynchronous; /**< List of asynchronous triggered listeners */
    std::vector<std::shared_ptr<AbstractInformationListener<T>>> listenersSynchronous; /**< List of synchronous triggered listeners */
    std::shared_ptr<InformationSender<T>> sender; /**< Sender to send information elements */
    std::shared_ptr<InformationSender<GContainer>> genericSender; /**< Sender to send information elements */
    std::shared_ptr<InformationReceiver> receiver; /**< Receiver to receive information elements */
  };

}
/* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_INFORMATIONSET_H_ */
