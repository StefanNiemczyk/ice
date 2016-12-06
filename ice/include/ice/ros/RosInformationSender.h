/*
 * RosInformationSender.h
 *
 *  Created on: Jun 18, 2014
 *      Author: sni
 */

#ifndef ROSINFORMATIONSENDER_H_
#define ROSINFORMATIONSENDER_H_

#include <iostream>
#include <typeinfo>

#include "ros/ros.h"

#include "ice/communication/InformationSender.h"
#include "ice/Entity.h"
#include "ice/EntityDirectory.h"

#include "ice_msgs/Identifier.h"

namespace ice
{

template<typename ICEType, typename ROSType>
  using transformC2M = typename std::shared_ptr<ROSType> (*)(std::shared_ptr<InformationElement<ICEType> >);

template<typename ICEType, typename ROSType>
  class RosInformationSender : public InformationSender<ICEType>
  {
  public:
    RosInformationSender(std::shared_ptr<InformationCollection> collection, identifier engineId,
                         ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize,
                         transformC2M<ICEType, ROSType> &messageTransform) :
                           InformationSender<ICEType>(collection), topic(topic)
    {
     this->engineId = engineId;
     this->bufferSize = bufferSize;
     this->publisher = nodeHandel->advertise<ROSType>(topic, bufferSize);
     this->messageTransform = messageTransform;
    }

    virtual ~RosInformationSender() {}

    virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                        std::shared_ptr<InformationElement<ICEType>> informationElement)
    {
      auto msg = this->messageTransform(informationElement);

      msg->header.senderId.value = this->engineId;
      msg->header.entity = informationElement->getSpecification()->getEntity();

      for (auto &entity : sendTo)
      {
        ice_msgs::Identifier receiver;
        std::string id;
        entity->getId(EntityDirectory::ID_ICE, id);
        receiver.value = std::stoi(id);
        msg->header.receiverIds.push_back(receiver);
      }

      this->publisher.publish(*msg);
    }

  private:
    identifier engineId; /**< ID of this engine, used as sender id */
    const std::string topic; /**< Topic of the ros channel */
    int bufferSize; /**< Size of the buffer */
    transformC2M<ICEType, ROSType> messageTransform; /**< Function pointer to message Transform method */
    ros::Publisher publisher; /**< Publisher */
  };
} /* namespace ice */

#endif /* ROSINFORMATIONSENDER_H_ */
