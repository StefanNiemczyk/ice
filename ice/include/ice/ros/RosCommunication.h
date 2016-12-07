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
#include "ice/communication/CommunicationInterface.h"
#include "ice/information/InformationStream.h"
#include "ice/ros/RosInformationReceiver.h"
#include "ice/ros/RosInformationSender.h"
#include "ice/ros/RosMessageTransform.h"

#include "ice_msgs/Heartbeat.h"
#include "ice_msgs/ICECoordination.h"
#include "easylogging++.h"

namespace ice
{

class GContainerFactory;

class RosCommunication : public CommunicationInterface
{
public:
  RosCommunication(std::weak_ptr<ICEngine> engine);
  virtual ~RosCommunication();

  // ROS listener methods
  void onHeartbeat(const ice_msgs::Heartbeat::ConstPtr& msg);
  void onCoordination(const ice_msgs::ICECoordination::ConstPtr& msg);

protected:
  virtual void initInternal();
  virtual void cleanUpInternal();
  virtual void discover();
  virtual int readMessage(std::vector<std::shared_ptr<Message>> &outMessages);
  virtual void sendMessage(std::shared_ptr<Message> msg);
  virtual std::shared_ptr<BaseInformationSender> createSender(std::shared_ptr<InformationCollection> collection);
  virtual std::shared_ptr<InformationReceiver> createReceiver(std::shared_ptr<InformationCollection> collection);

private:
  std::string createTopic(std::shared_ptr<InformationCollection> &collection);
  template<typename ICEType, typename ROSType>
    std::shared_ptr<BaseInformationSender> _createSender(std::shared_ptr<InformationCollection> collection,
                                                         transformC2M<ICEType, ROSType> &messageTransform);
  template<typename ICEType, typename ROSType>
    std::shared_ptr<InformationReceiver> _createReceiver(std::shared_ptr<InformationCollection> collection,
                                                         transformM2C<ICEType, ROSType> &messageTransform);
  void spin();
  bool checkReceiverIds(ice_msgs::ICEHeader header);

private:
  ros::NodeHandle                       nodeHandel;                     /**< Ros node handle */
  ros::Publisher                        heartbeatPublisher;             /**< Publisher on the heartbeat channel */
  ros::Subscriber                       heartbeatSubscriber;            /**< Subscriber on the heartbeat channel */
  ros::Publisher                        coordinationPublisher;          /**< Publisher on the coordination channel */
  ros::Subscriber                       coordinationSubscriber;         /**< Subscriber on the coordination channel */
  int                                   iceId;                          /**< own ice id */
  bool                                  running;                        /**< True if the worker thread is running, else false */
  std::thread                           worker;                         /**< Worker thread to perform the spin */
  el::Logger*                           _log;                           /**< Logger for communication */
};

template<typename ICEType, typename ROSType>
  inline std::shared_ptr<BaseInformationSender> RosCommunication::_createSender(
      std::shared_ptr<InformationCollection> collection, transformC2M<ICEType, ROSType> &messageTransform)
  {
    // identifier engineId
    // ros::NodeHandle& nodeHandel
    // std::string topic
    // int bufferSize
    // transform<ICEType, ROSType> &messageTransform
    return std::make_shared<RosInformationSender<ICEType, ROSType>>(collection, this->iceId, &this->nodeHandel,
                                                                    this->createTopic(collection),
                                                                    100, messageTransform);
  }

template<typename ICEType, typename ROSType>
  inline std::shared_ptr<InformationReceiver> RosCommunication::_createReceiver(
      std::shared_ptr<InformationCollection> collection, transformM2C<ICEType, ROSType> &messageTransform)
  {
    // identifier engineId
    // std::shared_ptr<BaseInformationStream> stream
    // ros::NodeHandle* nodeHandel
    // const std::string topic
    // int bufferSize,
    // transformM2C<ICEType, ROSType> &messageTransform
    return std::make_shared<RosInformationReceiver<ICEType, ROSType>>(collection, this->iceId, &this->nodeHandel,
                                                                      this->createTopic(collection),
                                                                      100, messageTransform);
  }

} /* namespace ice */

#endif /* ROSCOMMUNICATION_H_ */
