/*
 * Communication.h
 *
 *  Created on: June 12, 2014
 *      Author: sni
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <memory>
#include <vector>

#include "ice/Identifier.h"
#include "ice/model/ProcessingModelGenerator.h"

// Forward declarations
namespace ice
{
class BaseInformationSender;
class BaseInformationStream;
class CooperationRequest;
class CooperationResponse;
class Coordinator;
class EventHandler;
template<typename T>
  class InformationSender;
template<typename T>
  class InformationStream;
class InformationModel;
class InformationReceiver;
class ICEngine;
} /* namespace ice */
namespace el
{
class Logger;
}

namespace ice
{

class Communication
{
public:
  Communication(std::weak_ptr<ICEngine> engine);
  virtual ~Communication();

  virtual void init();

  virtual void cleanUp();

  virtual void sendHeartbeat() = 0;

  virtual void sendSystemSpecRequest(identifier receiverId) = 0;

  virtual void sendSystemSpecResponse(identifier receiverId, std::tuple<std::string, std::vector<std::string>, std::vector<std::string>> &content) = 0;

//  virtual void sendNodeListRequest(identifier receiverId) = 0;
//
//  virtual void sendNodeListResponse(identifier receiverId, std::vector<std::string> nodeIris) = 0;
//
//  virtual void sendNodeSpecRequest(identifier receiverId, std::vector<std::string> nodeIris) = 0;
//
//  virtual void sendNodeSpecResponse(identifier receiverId, std::vector<std::string> nodeIris) = 0; // TODO

  virtual void sendSubModelRequest(identifier receiverId, SubModelDesc &modelDesc) = 0;

  virtual void sendSubModelResponse(identifier receiverId, int index, bool accept) = 0;

  virtual void sendNegotiationFinished(identifier receiverId) = 0;

  virtual void sendStopCooperation(identifier receiverId) = 0;

//---------------------------------------------------------------------------------------------------
  // OLD

  virtual void sendInformationRequest(identifier receiverId) = 0;

  virtual void sendInformationModel(identifier receiverId, std::shared_ptr<InformationModel> informationModel) = 0;

  virtual void sendCooperationRequest(identifier receiverId, std::shared_ptr<CooperationRequest> request) = 0;

  virtual void sendCooperationResponse(identifier receiverId, std::shared_ptr<CooperationResponse> request) = 0;

  virtual void sendCooperationRefuse(identifier receiverId) = 0;

  virtual void sendCooperationAccept(identifier receiverId) = 0;

//  virtual void sendNegotiationFinished(identifier receiverId) = 0;

  virtual void sendRetryNegotiation(identifier receiverId) = 0;

  virtual void sendCooperationStopped(identifier receiverId) = 0;

//---------------------------------------------------------------------------------------------------

  virtual std::shared_ptr<BaseInformationSender> registerStreamAsSender(std::shared_ptr<BaseInformationStream> stream) = 0;

  virtual std::shared_ptr<InformationReceiver> registerStreamAsReceiver(std::shared_ptr<BaseInformationStream> stream) = 0;

protected:
  std::weak_ptr<ICEngine> engine; /**< The ice engine */
  std::shared_ptr<EventHandler> eventHandler; /**< The event handler */
  identifier engineId; /**< identifier of the main engine */
  std::shared_ptr<Coordinator> coordinator; /**< Coordinator which coordinates die communication between engines */
  el::Logger* _log; /**< Logger for communication */
};

} /* namespace ice */

#endif /* COMMUNICATION_H_ */
