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
#include "ice/Logger.h"
#include "ice/Time.h"
#include "ice/coordination/ModelComperator.h"

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
class TimeFactory;
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
  Coordinator(std::weak_ptr<ICEngine> engine);
  virtual ~Coordinator();

  void init();

  /*!
   * \brief Cleanup the coordinator.
   *
   * Cleanup the coordinator.
   */
  void cleanUp();

  std::shared_ptr<EngineState> getEngineState(identifier engineId);

  int onEngineHeartbeat(identifier engineId, time timestamp);

  int onInformationModelRequest(identifier engineId);

  int onInformationModel(identifier engineId, std::shared_ptr<InformationModel> informationModel);

  int onCooperationRequest(identifier engineId, std::shared_ptr<CooperationRequest> request);

  int onCooperationResponse(identifier engineId, std::shared_ptr<CooperationResponse> response);

  int onCooperationAccept(identifier engineId);

  int onCooperationRefuse(identifier engineId);

  int onNegotiationFinished(identifier engineId);

  int onStopCooperation(identifier engineId);

  int onCooperationStopped(identifier engineId);

  int onRetryNegotiation(identifier engineId);

  int stopCooperationWith(identifier engineId);

private:
  void stopCooperationWithEngine(std::shared_ptr<EngineState> engineState, bool sendStop);

  void workerTask();

private:
  bool running; /**< True if the communication is running, else false */
  std::thread worker; /**< Thread which sends the heartbeat and cyclic tests the engine states */
  std::weak_ptr<ICEngine> engine; /**< Weak pointer to the engine */
  std::shared_ptr<TimeFactory> timeFactory; /**< Time factory to create and compare time stamps */
  std::shared_ptr<Configuration> config; /**< Configuration object */
  std::shared_ptr<Communication> communication; /**< Communication interface to send messages */
  std::shared_ptr<InformationStore> informationStore; /**< The information store */
  std::vector<std::shared_ptr<EngineState>> engineStates; /**< List of known engines */
  ModelComperator modelComperator; /**< Comparator to find intersections in information models */
  Logger* _log; /**< Logger */
  std::mutex mtx_; /**< Mutex */
  std::condition_variable cv; /**< Condition variable for synchronizing threads */
};

} /* namespace ice */

#endif /* COORDINATOR_H_ */
