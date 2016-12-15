/*
 * RosInformationReceiver.h
 *
 *  Created on: Jun 19, 2014
 *      Author: sni
 */

#ifndef ROSINFORMATIONRECEIVER_H_
#define ROSINFORMATIONRECEIVER_H_

#include <iostream>
#include <memory>

#include <ice_msgs/InformationHeader.h>
#include <ice_msgs/ICEHeader.h>
#include <ice_msgs/Identifier.h>
#include <ice_msgs/GContainer.h>
#include <ros/ros.h>

#include "ice/communication/InformationReceiver.h"
#include "ice/information/InformationCollection.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"
#include "ice/representation/GContainerFactory.h"


namespace ice
{
class RosGContainerReceiver : public InformationReceiver
{
public:
  RosGContainerReceiver(std::shared_ptr<InformationCollection> collection,
                        std::shared_ptr<TimeFactory> const &timeFactory, identifier ownId,
                        ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize,
                        std::shared_ptr<GContainerFactory> factory) :
      InformationReceiver(collection, timeFactory), topic(topic), factory(factory)

  {
    this->ownId = ownId;
    this->subscriber = nodeHandel->subscribe(topic, bufferSize, &RosGContainerReceiver::onMessage, this);

    if (collection->getCollectionType() == CollectionType::CT_STREAM)
      this->stream = std::static_pointer_cast<InformationStream<GContainer>>(collection);
    else
      this->set = std::static_pointer_cast<InformationSet<GContainer>>(collection);
  }
  virtual ~RosGContainerReceiver() {}

  virtual void init()
  {

  }
  virtual void cleanUp()
  {

  }

  virtual void onMessage(const boost::shared_ptr<ice_msgs::GContainer const> msg)
  {
    if (false == this->checkReceiverIds(msg->info.header))
      return;

    std::string json(msg->bytes.begin(), msg->bytes.end());
    auto container = this->factory->fromJSON(json);

    if (container == nullptr)
    {
      // TODO
      return;
    }

    if (stream)
      stream->add(container, msg->info.validityTime, msg->info.timestamp, this->timeFactory->createTime());
    else
      set->add(msg->info.entity, container, msg->info.validityTime, msg->info.timestamp, this->timeFactory->createTime());
  }

  bool checkReceiverIds(ice_msgs::ICEHeader header)
  {
    for (auto id : header.receiverIds)
    {
      identifier receiverId = id.value;

      if (receiverId == this->ownId)
        return true;
    }

    return false;
  }

private:
  identifier                                            ownId;          /**< ID of this engine, used as sender id */
  const std::string                                     topic;          /**< Topic of the ros channel */
  ros::Subscriber                                       subscriber;     /**< Subscriber */
  std::shared_ptr<GContainerFactory>                    factory;        /**< Factory to create GContainer instances */
  std::shared_ptr<InformationStream<GContainer>>        stream;         /**< The stream which listens to the channel */
  std::shared_ptr<InformationSet<GContainer>>           set;            /**< The set which listens to the channel */
};

template<typename ICEType, typename ROSType>
  using transformM2C = typename std::shared_ptr<ICEType> (*)(const boost::shared_ptr<ROSType const>);

template<typename ICEType, typename ROSType>
  class RosInformationReceiver : public InformationReceiver
  {
  public:
    RosInformationReceiver(std::shared_ptr<InformationCollection> collection,
                           std::shared_ptr<TimeFactory> const &timeFactory, identifier engineId,
                           ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize,
                           transformM2C<ICEType, ROSType> &messageTransform) :
        InformationReceiver(collection, timeFactory), topic(topic)

    {
      this->engineId = engineId;
      this->bufferSize = bufferSize;
      this->subscriber = nodeHandel->subscribe(topic, bufferSize, &RosInformationReceiver::onMessage, this);
      this->messageTransform = messageTransform;

      if (collection->getCollectionType() == CollectionType::CT_STREAM)
        this->stream = std::static_pointer_cast<InformationStream<ICEType>>(collection);
      else
        this->set = std::static_pointer_cast<InformationSet<ICEType>>(collection);
    }
    virtual ~RosInformationReceiver() {}

    virtual void init()
    {

    }
    virtual void cleanUp()
    {

    }

    virtual void onMessage(const boost::shared_ptr<ROSType const> msg)
    {
      if (false == this->checkReceiverIds(msg->info.header))
        return;

      auto contrainer = this->messageTransform(msg);
      if (stream)
        stream->add(contrainer, msg->info.validityTime, msg->info.timestamp, this->timeFactory->createTime());
      else
        set->add(msg->info.entity, contrainer, msg->info.validityTime, msg->info.timestamp, this->timeFactory->createTime());
    }

    bool checkReceiverIds(ice_msgs::ICEHeader header)
    {
      for (auto id : header.receiverIds)
      {
        identifier receiverId = id.value;

        if (receiverId == this->engineId)
          return true;
      }

      return false;
    }

  private:
    identifier                                  engineId;               /**< ID of this engine, used as sender id */
    const std::string                           topic;                  /**< Topic of the ros channel */
    int                                         bufferSize;             /**< Size of the buffer */
    transformM2C<ICEType, ROSType>              messageTransform;       /**< Function pointer to message Transform method */
    ros::Subscriber                             subscriber;             /**< Subscriber */
    std::shared_ptr<InformationStream<ICEType>> stream;                 /**< The stream which listens to the channel */
    std::shared_ptr<InformationSet<ICEType>>    set;                    /**< The set which listens to the channel */
  };

} /* namespace ice */

#endif /* ROSINFORMATIONRECEIVER_H_ */
