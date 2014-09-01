/*
 * BaseInformationStream.h
 *
 *  Created on: May 15, 2014
 *      Author: sni
 */

#ifndef BASEINFORMATIONSTREAM_H_
#define BASEINFORMATIONSTREAM_H_

#include <memory>
#include <mutex>

#include "ice/Logger.h"
#include "ice/coordination/StreamDescription.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/AsynchronousTask.h"

// Forward declaration
namespace ice
{
class BaseInformationSender;
class Communication;
class InformationReceiver;
class InformationStreamTemplate;
class InformationType;
class EngineState;
class EventHandler;
}

namespace ice
{

//* BaseInformationStream
/**
 * This class provides the default interface of the information container.
 */
class BaseInformationStream
{
public:
  /*!
   * \brief Default constructor
   *
   * Default constructor
   *
   * \param name The name of the stream
   * \param informationType The information type which holds this stream.
   * \param eventHandler Handler to execute events asynchronously.
   * \param specification The specification of the stored information.
   * \param provider The provider of the information stored in this stream.
   * \param description The description of this stream.
   * \param shared True if the stream is shared, else false.
   * \param sharingMaxCount Max number of sharing this stream.
   */
  BaseInformationStream(const std::string name, std::weak_ptr<InformationType> informationType,
                        std::shared_ptr<EventHandler> eventHandler,
                        std::shared_ptr<InformationSpecification> specification, std::string provider = "",
                        std::string description = "", bool shared = false, int sharingMaxCount = 0);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~BaseInformationStream();

  /*!
   * \brief Registers this stream in the communication class as sending stream.
   *
   * Registers this stream in the communication class as sending stream.
   */
  virtual std::shared_ptr<BaseInformationSender> registerSender(std::shared_ptr<Communication> communication) = 0;

  /*!
   * \brief Registers this stream in the communication class as receiving stream.
   *
   * Registers this stream in the communication class as receiving stream.
   */
  virtual std::shared_ptr<InformationReceiver> registerReceiver(std::shared_ptr<Communication> communication) = 0;

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

  /*!
   * \brief Returns the provider of the information stored in this stream.
   *
   * Returns the provider of the information stored in this stream.
   */
  const std::string getProvider() const;

  /*!
   * \brief Sets the provider of the information stored in this stream.
   *
   * Sets the provider of the information stored in this stream.
   *
   * \param provider The new provider.
   */
  void setProvider(std::string provider);

  /*!
   * \brief Returns the description of the information stored in this stream.
   *
   * Returns the description of the information stored in this stream.
   */
  const std::string getDescription() const;

  /*!
   * \brief Sets the description of the information stored in this stream.
   *
   * Sets the description of the information stored in this stream.
   *
   * \param description The new description.
   */
  void setDescription(std::string description);

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
  int registerEngineState(std::shared_ptr<EngineState> engineState);

  /*!
   * \brief Unregisters a engine state.
   *
   *  Unregisters an engine state. Returns 1 if the task is already registered, else 0.
   */
  int unregisterEngineState(std::shared_ptr<EngineState> engineState);

  /*!
   * \brief Returns the description of this stream.
   *
   * Returns the description of this stream.
   */
  virtual std::shared_ptr<StreamDescription> getStreamDescription();

  /*!
   *\brief Returns the stream template used to create this stream, else NULL.
   *
   * Returns the stream template used to create this stream, else NULL.
   */
  const std::weak_ptr<InformationStreamTemplate> getStreamTemplate() const;

  /*!
   * \brief Sets the stream template used to create this stream.
   *
   * Sets the stream template used to create this stream.
   */
  void setStreamTemplate(const std::weak_ptr<InformationStreamTemplate> streamTemplate);

  /*!
   * \brief Removes the receiver of this stream.
   *
   * Removes the receiver of this stream.
   */
  virtual void dropReceiver();

  /*!
   * \brief Returns true if this stream can be shared, else false.
   *
   * Returns true if this stream can be shared, else false.
   */
  bool canBeShared();

  /*!
   * \brief Returns the current number of sharing.
   *
   * Returns the current number of sharing.
   */
  int getSharingCount() const;

  /*!
   * \brief Returns the max count of sharing.
   *
   * Returns the max count of sharing.
   */
  int getSharingMaxCount() const;

  /*!
   * \brief Sets the max count of sharing.
   *
   * Sets the max count of sharing.
   *
   * \brief sharingMaxCount Max number of sharing this stream;
   */
  void setSharingMaxCount(int sharingMaxCount);

protected:
  /*!
   * \brief This method is calls if the last engine state will be unregistered.
   *
   * This method is calls if the last engine state will be unregistered.
   */
  virtual void allEngineStatesUnregistered() = 0;

protected:
  const long iid; /**< The internal id */
  const std::string name; /**< The name of the stream */
  bool shared; /**< True if the stream can be shared with others */
  int sharingMaxCount; /**< Max number of sharing this stream */
  std::string provider; /**< The provider of the information stored in this stream */
  std::string description; /**< The description of this stream */
  std::weak_ptr<InformationStreamTemplate> streamTemplate; /**< The template used to create this stream */
  std::weak_ptr<InformationType> informationType; /**< The information type */
  std::shared_ptr<InformationSpecification> specification; /**< Specification of the information stored in this container */
  std::shared_ptr<EventHandler> eventHandler; /**< Handler to execute events asynchronously */
  std::vector<std::shared_ptr<AsynchronousTask>> taskAsynchronous; /**< List of events which are fired asynchronous if a new element is addes */
  std::vector<std::shared_ptr<AsynchronousTask>> taskSynchronous; /**< List of events which are executed synchronous if a new element is addes */
  std::vector<std::shared_ptr<EngineState>> remoteListeners; /**< List of engine states of remote engines registered */
  std::shared_ptr<StreamDescription> streamDescription; /**< Description of this stream used for information coordination */
  Logger* _log; /**< Logger */
  std::mutex _mtx; /**< Mutex */

private:
  static int IDENTIFIER_COUNTER;
};

} /* namespace ice */

#endif /* BASEINFORMATIONSTREAM_H_ */
