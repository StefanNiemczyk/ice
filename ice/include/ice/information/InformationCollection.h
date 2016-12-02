/*
 * InformationCollection.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_INFORMATIONCOLLECTION_H_
#define INCLUDE_ICE_INFORMATION_INFORMATIONCOLLECTION_H_


#include <memory>
#include <mutex>
#include <vector>

#include "ice/information/InformationSpecification.h"
#include "ice/information/CollectionDescription.h"
#include "ice/processing/AsynchronousTask.h"

// Forward declaration
namespace ice
{
class BaseInformationSender;
class CommunicationInterface;
class InformationReceiver;
class Entity;
class EventHandler;
}
namespace el {
class Logger;
}

namespace ice
{

class InformationCollection
{
public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param streamDescription The description of this stream.
   * \param eventHandler Handler to execute events asynchronously.
   */
  InformationCollection(std::shared_ptr<CollectionDescription> streamDescription,
                        std::shared_ptr<EventHandler> eventHandler);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~InformationCollection();

  /*!
   * \brief Returns the type_info of the template type.
   *
   * Returns the type_info of the template type.
   */
  virtual const std::type_info* getTypeInfo() const = 0;

  /*!
   * \brief Returns the specification of the information stored in this container.
   *
   * Returns the specification of the information stored in this container.
   */
  std::shared_ptr<InformationSpecification> getSpecification() const;

  /*!
   * \brief Returns the internal identifier.
   *
   * Returns the internal identifier.
   */
  const int getIID() const;

  /*!
   * \brief Returns the name of this stream.
   *
   * Returns the name of this stream.
   */
  const std::string getName() const;

  const uint32_t getHash();

  /*!
   * \brief Returns the provider of the information stored in this stream.
   *
   * Returns the provider of the information stored in this stream.
   */
  const std::string getProvider() const;

  /*!
   * \brief Returns the description of this collection.
   *
   * Returns the description of this collection.
   */
  virtual std::shared_ptr<CollectionDescription> getDescription();

  /*!
   * \brief Register a task which is executed asynchronous if a new information element is added.
   *
   *  Registered a task which is executed asynchronous if a new information element is added.
   *  Returns 1 if the task is already registered, else 0.
   */
  int registerTaskAsync(std::shared_ptr<AsynchronousTask> task);

  /*!
   * \brief Unregisters a task which is executed asynchronous if a new information element is added.
   *
   *  Registered a task which is executed asynchronous if a new information element is added.
   *  Returns 1 if the task was not registered before, else 0.
   */
  int unregisterTaskAsync(std::shared_ptr<AsynchronousTask> task);

  /*!
   * \brief Registered a task which is executed synchronous if a new information element is added.
   *
   *  Registered a listener which is executed synchronous if a new information element is added.
   *  Returns 1 if the task is already registered, else 0.
   */
  int registerTaskSync(std::shared_ptr<AsynchronousTask> task);

  /*!
   * \brief Unregisters a task which is executed synchronous if a new information element is added.
   *
   *  Registered a task which is executed synchronous if a new information element is added.
   *  Returns 1 if the task was not registered before, else 0.
   */
  int unregisterTaskSync(std::shared_ptr<AsynchronousTask> task);

  /*!
   * \brief Registered an engine state.
   *
   *  Registers an engine state. Returns 1 if the task is already registered, else 0.
   */
  int registerRemoteListener(std::shared_ptr<Entity> &entity, std::shared_ptr<CommunicationInterface> &communication);

  /*!
   * \brief Unregisters a engine state.
   *
   *  Unregisters an engine state. Returns 1 if the task is already registered, else 0.
   */
  int unregisterRemoteListener(std::shared_ptr<Entity> &entity);

  int setRemoteSource(std::shared_ptr<Entity> entity, std::shared_ptr<CommunicationInterface> &communication);

  /*!
   * \brief Returns the current number of sharing.
   *
   * Returns the current number of sharing.
   */
  int getSharingCount() const;

  /*!
   * \brief Returns the stream as string.
   *
   * Returns the stream as string.
   */
  std::string toString();

  void destroy();

protected:
  /*!
   * \brief Registers this stream in the communication class as sending stream.
   *
   * Registers this stream in the communication class as sending stream.
   */
  virtual std::shared_ptr<BaseInformationSender> registerSender(std::shared_ptr<CommunicationInterface> &communication) = 0;

  /*!
   * \brief Registers this stream in the communication class as receiving stream.
   *
   * Registers this stream in the communication class as receiving stream.
   */
  virtual std::shared_ptr<InformationReceiver> registerReceiver(std::shared_ptr<CommunicationInterface> &communication) = 0;

  /*!
   * \brief Removes the receiver of this stream.
   *
   * Removes the receiver of this stream.
   */
  virtual void dropReceiver();

  /*!
   * \brief This method is calls if the last engine state will be unregistered.
   *
   * This method is calls if the last engine state will be unregistered.
   */
  virtual void dropSender() = 0;

protected:
  const long                                            iid;                    /**< The internal id */
  uint32_t                                              hash;                   /**< Hash of this stream */
  std::shared_ptr<EventHandler>                         eventHandler;           /**< Handler to execute events asynchronously */
  std::vector<std::shared_ptr<AsynchronousTask>>        taskAsynchronous;       /**< List of events which are fired asynchronous if a new element is addes */
  std::vector<std::shared_ptr<AsynchronousTask>>        taskSynchronous;        /**< List of events which are executed synchronous if a new element is addes */
  std::vector<std::shared_ptr<Entity>>                  remoteListeners;        /**< List of engine states of remote engines registered */
  std::shared_ptr<Entity>                               remoteSource;           /**< Remote source of this stream */
  const std::shared_ptr<CollectionDescription>              streamDescription;      /**< Description of this stream used for information coordination */
  el::Logger*                                           _log;                   /**< Logger */
  std::mutex                                            _mtx;                   /**< Mutex */

private:
  static int IDENTIFIER_COUNTER;
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_INFORMATIONCOLLECTION_H_ */
