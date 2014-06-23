/*
 * RosCommunication.h
 *
 *  Created on: Jun 16, 2014
 *      Author: sni
 */

#ifndef ROSCOMMUNICATION_H_
#define ROSCOMMUNICATION_H_

#include <iostream>
#include <memory>
#include <thread>

#include "ros/ros.h"

#include "ice/Identifier.h"
#include "ice/communication/Communication.h"
#include "ice/information/InformationStream.h"
#include "ice/ros/RosInformationReceiver.h"
#include "ice/ros/RosInformationSender.h"
#include "ice/ros/RosMessageTransform.h"

#include "ice_msgs/CooperationRequest.h"
#include "ice_msgs/CooperationResponse.h"
#include "ice_msgs/Heartbeat.h"
#include "ice_msgs/ICECoordination.h"
#include "ice_msgs/InformationModel.h"

namespace ice
{
enum RosCoordinationCommand
{
  REQUEST_INFORMATION_MODEL, //< Command to request an information model
  COOPERATION_ACCEPT, //< Command to accept the cooperation response
  COOPERATION_REFUSE, //< Command to refuse the cooperation response
  NEGOTIATION_FINISHED, //< Command to finish the negotiation process
  NEGOTIATION_RETRY, //< Command to restart the negotiation process
  STOP_COOPERATION, //< Command to stop the cooperation
  COOPERATION_STOPPED //< Command that the cooperation has stopped
};

class RosCommunication : public Communication
{
public:
  RosCommunication(std::weak_ptr<ICEngine> engine);
  virtual ~RosCommunication();

  virtual void init();

  virtual void cleanUp();

  virtual void sendHeartbeat();

  virtual void sendInformationRequest(identifier receiverId);

  virtual void sendInformationModel(identifier receiverId, std::shared_ptr<InformationModel> informationModel);

  virtual void sendCooperationRequest(identifier receiverId, std::shared_ptr<CooperationRequest> cooperationRequest);

  virtual void sendCooperationResponse(identifier receiverId, std::shared_ptr<CooperationResponse> cooperationResponse);

  virtual void sendCooperationRefuse(identifier receiverId);

  virtual void sendCooperationAccept(identifier receiverId);

  virtual void sendNegotiationFinished(identifier receiverId);

  virtual void sendRetryNegotiation(identifier receiverId);

  virtual void sendStopCooperation(identifier receiverId);

  virtual void sendCooperationStopped(identifier receiverId);

  virtual std::shared_ptr<BaseInformationSender> registerStreamAsSender(std::shared_ptr<BaseInformationStream> type);

  virtual std::shared_ptr<InformationReceiver> registerStreamAsReceiver(std::shared_ptr<BaseInformationStream> stream);

  virtual std::shared_ptr<BaseInformationSender> createSender(std::shared_ptr<BaseInformationStream> stream);

  virtual std::shared_ptr<InformationReceiver> createReceiver(std::shared_ptr<BaseInformationStream> stream);

  template<typename ICEType, typename ROSType>
    std::shared_ptr<BaseInformationSender> _createSender(std::shared_ptr<BaseInformationStream> stream,
                                                         transformC2M<ICEType, ROSType> &messageTransform);

  template<typename ICEType, typename ROSType>
    std::shared_ptr<InformationReceiver> _createReceiver(std::shared_ptr<BaseInformationStream> stream,
                                                         transformM2C<ICEType, ROSType> &messageTransform);

  void sendCommand(const identifier receiverId, const RosCoordinationCommand command) const;

  void workerTask();

  // ROS listener methods
  void onHeartbeat(const ice_msgs::Heartbeat::ConstPtr& msg);

  void onCoordination(const ice_msgs::ICECoordination::ConstPtr& msg);

  void onInformationModel(const ice_msgs::InformationModel::ConstPtr& msg);

  void onCooperationRequest(const ice_msgs::CooperationRequest::ConstPtr& msg);

  void onCooperationResponse(const ice_msgs::CooperationResponse::ConstPtr& msg);

private:
  bool checkReceiverIds(ice_msgs::ICEHeader header);

private:
  ros::NodeHandle nodeHandel; /**< Ros node handle */
  ros::Publisher heartbeatPublisher; /**< Publisher on the heartbeat channel */
  ros::Subscriber heartbeatSubscriber; /**< Subscriber on the heartbeat channel */
  ros::Publisher coordinationPublisher; /**< Publisher on the coordination channel */
  ros::Subscriber coordinationSubscriber; /**< Subscriber on the coordination channel */
  ros::Publisher informationModelPublisher; /**< Publisher on the information model channel */
  ros::Subscriber informationModelSubscriber; /**< Subscriber on the information model channel */
  ros::Publisher cooperationRequestPublisher; /**< Publisher on the cooperation request channel */
  ros::Subscriber cooperationRequestSubscriber; /**< Subscriber on the  cooperation request channel */
  ros::Publisher cooperationResponsePublisher; /**< Publisher on the  cooperation response channel */
  ros::Subscriber cooperationResponseSubscriber; /**< Subscriber on the  cooperation response channel */
  bool running; /**< True if the worker thread is running, else false */
  std::thread worker; /**< Worker thread to perform the spin */
  Logger* _log; /**< Logger for communication */
};

template<typename ICEType, typename ROSType>
  inline std::shared_ptr<BaseInformationSender> RosCommunication::_createSender(
      std::shared_ptr<BaseInformationStream> stream, transformC2M<ICEType, ROSType> &messageTransform)
  {
    // identifier engineId
    // ros::NodeHandle& nodeHandel
    // std::string topic
    // int bufferSize
    // transform<ICEType, ROSType> &messageTransform
    return std::make_shared<RosInformationSender<ICEType, ROSType>>(this->engineId, &this->nodeHandel,
                                                                    "/ice" + stream->getSpecification()->getName(),
                                                                    100, messageTransform);
  }

template<typename ICEType, typename ROSType>
  inline std::shared_ptr<InformationReceiver> RosCommunication::_createReceiver(
      std::shared_ptr<BaseInformationStream> stream, transformM2C<ICEType, ROSType> &messageTransform)
  {
    // identifier engineId
    // std::shared_ptr<BaseInformationStream> stream
    // ros::NodeHandle* nodeHandel
    // const std::string topic
    // int bufferSize,
    // transformM2C<ICEType, ROSType> &messageTransform
    return std::make_shared<RosInformationReceiver<ICEType, ROSType>>(this->engineId, stream, &this->nodeHandel,
                                                                      "/ice" + stream->getSpecification()->getName(),
                                                                      100, messageTransform);
  }

} /* namespace ice */

#endif /* ROSCOMMUNICATION_H_ */
