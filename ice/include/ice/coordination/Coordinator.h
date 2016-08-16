/*
 * Coordinator.h
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ice/Identifier.h"
#include "ice/Time.h"
#include "ice/model/ProcessingModel.h"

#include "easylogging++.h"

//Forward declaration
namespace ice
{
class Communication;
class Configuration;
class Entity;
class EntityDirectory;
class ICEngine;
class StreamStore;
class NodeStore;
class OntologyInterface;
class ProcessingModelGenerator;
class TimeFactory;
class UpdateStrategie;

class CooperationRequest;
class CooperationResponse;
class InformationModel;
class IntersectionInformationModel;
} /* namespace ice */

namespace ice
{
//* Coordinator
/**
 * This class coordinates the communication flow between multiple engines.
 *
 */
class Coordinator
{
public:
  /*!
   * \brief Creates a new uninitialized object. init() needs to be called before using.
   *
   * Creates a new uninitialized object. init() needs to be called before using.
   *
   * \param engine Weak pointer to the main engine object.
   */
  Coordinator(std::weak_ptr<ICEngine> engine);

  /*!
   * \brief Default destructor
   *
   * Default destructor
   */
  virtual ~Coordinator();

  /*!
   * \brief Initialize the coordinator.
   *
   * Initialize the coordinator.
   */
  void init();

  /*!
   * \brief Cleanup the coordinator.
   *
   * Cleanup the coordinator.
   */
  void cleanUp();

  /*!
   * \brief This method should be called if a heartbeat from another engine is received.
   *
   * This method should be called if a heartbeat from another engine is received.
   *
   * \param engineId Identifier of the sending engine.
   * \param timestamp Time stamp of the receiving of the heartbeat.
   */
  int onEngineHeartbeat(identifier engineId, time timestamp);

  int onSystemSpecRequest(identifier engineId);

  int onSystemSpec(identifier engineId, std::tuple<std::string, std::vector<std::string>, std::vector<std::string>> spec);

  int onSubModelRequest(identifier engineId, SubModelDesc modelDesc);

  int onSubModelResponse(identifier engineId, int index, bool accept);

  /*!
   * \brief This method should be called if a negotiation finished from another engine is received.
   *
   * This method should be called if a negotiation finished from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onNegotiationFinished(identifier engineId);

  /*!
   * \brief This method should be called if a stop cooperation from another engine is received.
   *
   * This method should be called if a stop cooperation from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onStopCooperation(identifier engineId);

private:

  /*!
   * \brief This message is executed by the worker thread.
   *
   * This message is executed by the worker thread.
   */
  void workerTask();

private:
  bool                                          running; /**< True if the communication is running, else false */
  std::thread                                   worker; /**< Thread which sends the heart beat and cyclic tests the engine states */
  std::weak_ptr<ICEngine>                       engine; /**< Weak pointer to the engine */
  std::string                                   engineIri; /**< Iri of the main engine */
  std::shared_ptr<OntologyInterface>            ontologyInterface; /**< The interface to access the ontology */
  std::shared_ptr<TimeFactory>                  timeFactory; /**< Time factory to create and compare time stamps */
  std::shared_ptr<Configuration>                config; /**< Configuration object */
  std::shared_ptr<Communication>                communication; /**< Communication interface to send messages */
  std::shared_ptr<StreamStore>                  streamStore; /**< The stream store */
  std::shared_ptr<ProcessingModelGenerator>     modelGenerator; /**< The model generator */
  std::shared_ptr<NodeStore>                    nodeStore; /**< The node store */
  std::shared_ptr<EntityDirectory>              entityDirectory; /**< Directory for known entities */
  std::shared_ptr<Entity>                       self; /**< Own entity object */
  std::shared_ptr<UpdateStrategie>              updateStrategie; /**< Update strategie to change the processing model */

  el::Logger*                                   _log; /**< Logger */
  std::mutex                                    mtx_; /**< Mutex */
  std::mutex                                    threadMtx_; /**< Mutex */
  std::condition_variable                       cv; /**< Condition variable for synchronizing threads */
};

} /* namespace ice */

#endif /* COORDINATOR_H_ */
