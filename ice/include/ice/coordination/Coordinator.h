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
class CooperationRequest;
class CooperationResponse;
class EngineState;
class ICEngine;
class InformationModel;
class InformationStore;
class IntersectionInformationModel;
class NodeStore;
class OntologyInterface;
class ProcessingModelGenerator;
class TimeFactory;
class UpdateStrategie;
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
   * \brief Returns the engine state object for a given identifier or NULL.
   *
   * Returns the engine state object for a given identifier from the list of known engines.
   * If no engine for the given identifier exists NULL will be returned
   *
   * \param engineId The identifier of the searched engine.
   */
  std::shared_ptr<EngineState> getEngineState(identifier engineId);

  std::shared_ptr<EngineState> getEngineState(std::string p_iri);

  std::shared_ptr<EngineState> getEngineStateNoMutex(identifier engineId, bool returnNew);

  std::shared_ptr<EngineState> getEngineStateNoMutex(std::string p_iri);

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

  //---------------------------------------------------------------------------
  /*!
   * \brief This method should be called if an information model request from another engine is received.
   *
   * This method should be called if an information model request from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onInformationModelRequest(identifier engineId);

  /*!
   * \brief This method should be called if an information model from another engine is received.
   *
   * This method should be called if an information model from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onInformationModel(identifier engineId, std::shared_ptr<InformationModel> informationModel);

  /*!
   * \brief This method should be called if a cooperation request from another engine is received.
   *
   * This method should be called if a cooperation request from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onCooperationRequest(identifier engineId, std::shared_ptr<CooperationRequest> request);

  /*!
   * \brief This method should be called if a cooperation response from another engine is received.
   *
   * This method should be called if a cooperation response from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onCooperationResponse(identifier engineId, std::shared_ptr<CooperationResponse> response);

  /*!
   * \brief This method should be called if a cooperation accepted from another engine is received.
   *
   * This method should be called if a cooperation accepted from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onCooperationAccept(identifier engineId);

  /*!
   * \brief This method should be called if a cooperation refuse from another engine is received.
   *
   * This method should be called if a cooperation refuse from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onCooperationRefuse(identifier engineId);

  /*!
   * \brief This method should be called if a cooperation stopped from another engine is received.
   *
   * This method should be called if a cooperation stopped from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onCooperationStopped(identifier engineId);

  /*!
   * \brief This method should be called if a retry negotiation from another engine is received.
   *
   * This method should be called if a retry negotiation from another engine is received.
   *
   * @param engineId Identifier of the sending engine.
   */
  int onRetryNegotiation(identifier engineId);

  /*!
   * \brief Stops the cooperation with an engine identified by the given identifier.
   *
   * Stops the cooperation with an engine identified by the given identifier.
   *
   * @param engineId Identifier of engine to stop the cooperation.
   */
  int stopCooperationWith(identifier engineId);

private:

  /*!
   * \brief Stops the cooperation with the given engine.
   *
   * Stops the cooperation with the given engine. Sends a stop message if sendStop is true.
   *
   * @param engineState The engine to stop the cooperation with.
   * @param sendStop True if a stop cooperation should be send.
   */
  void stopCooperationWithEngine(std::shared_ptr<EngineState> engineState, bool sendStop);

  /*!
   * \brief This message is executed by the worker thread.
   *
   * This message is executed by the worker thread.
   */
  void workerTask();

private:
  bool running; /**< True if the communication is running, else false */
  std::thread worker; /**< Thread which sends the heart beat and cyclic tests the engine states */
  std::weak_ptr<ICEngine> engine; /**< Weak pointer to the engine */
  std::string engineIri; /**< Iri of the main engine */
  std::shared_ptr<OntologyInterface> ontologyInterface; /**< The interface to access the ontology */
  std::shared_ptr<TimeFactory> timeFactory; /**< Time factory to create and compare time stamps */
  std::shared_ptr<Configuration> config; /**< Configuration object */
  std::shared_ptr<Communication> communication; /**< Communication interface to send messages */
  std::shared_ptr<InformationStore> informationStore; /**< The information store */
  std::shared_ptr<ProcessingModelGenerator> modelGenerator; /**< The model generator */
  std::shared_ptr<NodeStore> nodeStore; /**< The node store */
  std::vector<std::shared_ptr<EngineState>> engineStates; /**< List of known engines */
  std::shared_ptr<UpdateStrategie> updateStrategie; /**< Update strategie to change the processing model */
  std::shared_ptr<EngineState> self; /**< Engine state object of the main engine */
  el::Logger* _log; /**< Logger */
  std::mutex mtx_; /**< Mutex */
  std::mutex threadMtx_; /**< Mutex */
  std::condition_variable cv; /**< Condition variable for synchronizing threads */
};

} /* namespace ice */

#endif /* COORDINATOR_H_ */
