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

#include <ice_msgs/InformationHeader.h>
#include <ice_msgs/GContainer.h>
#include <ros/ros.h>

#include "ice/communication/InformationSender.h"
#include "ice/representation/GContainer.h"
#include "ice/Entity.h"
#include "ice/EntityDirectory.h"

#include "ice_msgs/Identifier.h"

namespace ice
{
class RosGContainerSender : public InformationSender<GContainer>
  {
  public:
  RosGContainerSender(std::shared_ptr<InformationCollection> collection, identifier ownId,
                         ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize) :
                           InformationSender<GContainer>(collection), topic(topic)
    {
     this->ownId = ownId;
     this->publisher = nodeHandel->advertise<ice_msgs::GContainer>(topic, bufferSize);
    }

    virtual ~RosGContainerSender() {}

    virtual void init()
    {

    }
    virtual void cleanUp()
    {

    }

    virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                        std::shared_ptr<InformationElement<GContainer>> informationElement)
    {
      auto msg = std::make_shared<ice_msgs::GContainer>();

      msg->info.header.senderId.value = this->ownId;
      if (this->collection->getCollectionType() == CollectionType::CT_SET)
        msg->info.entity = informationElement->getSpecification()->getEntity();

      msg->info.timestamp = informationElement->getTimeObservation();
      msg->info.validityTime = informationElement->getTimeValidity();

      for (auto &entity : sendTo)
      {
        ice_msgs::Identifier receiver;
        std::string id;
        entity->getId(EntityDirectory::ID_ICE, id);
        receiver.value = std::stoi(id);
        msg->info.header.receiverIds.push_back(receiver);
      }

      std::string json = informationElement->getInformation()->toJSON();
      msg->bytes.resize(json.length());
      std::copy(json.begin(), json.end(), msg->bytes.begin());

      this->publisher.publish(*msg);
    }

  private:
    identifier          ownId;          /**< ID of this engine, used as sender id */
    const std::string   topic;          /**< Topic of the ros channel */
    ros::Publisher      publisher;      /**< Publisher */
  };


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

    virtual void init()
    {

    }
    virtual void cleanUp()
    {

    }

    virtual void sendInformationElement(std::vector<std::shared_ptr<Entity>> &sendTo,
                                        std::shared_ptr<InformationElement<ICEType>> informationElement)
    {
      auto msg = this->messageTransform(informationElement);

      msg->info.header.senderId.value = this->engineId;
      if (this->collection->getCollectionType() == CollectionType::CT_SET)
        msg->info.entity = informationElement->getSpecification()->getEntity();

      msg->info.timestamp = informationElement->getTimeObservation();
      msg->info.validityTime = informationElement->getTimeValidity();

      for (auto &entity : sendTo)
      {
        ice_msgs::Identifier receiver;
        std::string id;
        entity->getId(EntityDirectory::ID_ICE, id);
        receiver.value = std::stoi(id);
        msg->info.header.receiverIds.push_back(receiver);
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
