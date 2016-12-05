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

#include "boost/shared_ptr.hpp"

#include "ros/ros.h"

#include "ice/communication/InformationReceiver.h"
#include "ice/information/InformationCollection.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationStream.h"

#include "ice_msgs/ICEHeader.h"
#include "ice_msgs/Identifier.h"

namespace ice
{

template<typename ICEType, typename ROSType>
  using transformM2C = typename std::shared_ptr<ICEType> (*)(const boost::shared_ptr<ROSType const>);

template<typename ICEType, typename ROSType>
  class RosInformationReceiver : public InformationReceiver
  {
  public:
    RosInformationReceiver(identifier engineId, std::shared_ptr<InformationCollection> collection,
                           ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize,
                           transformM2C<ICEType, ROSType> &messageTransform);
    virtual ~RosInformationReceiver();

    virtual void onMessage(const boost::shared_ptr<ROSType const> msg);

    bool checkReceiverIds(ice_msgs::ICEHeader header);

  private:
    identifier                                  engineId;               /**< ID of this engine, used as sender id */
    const std::string                           topic;                  /**< Topic of the ros channel */
    int                                         bufferSize;             /**< Size of the buffer */
    transformM2C<ICEType, ROSType>              messageTransform;       /**< Function pointer to message Transform method */
    ros::Subscriber                             subscriber;             /**< Subscriber */
    std::shared_ptr<InformationStream<ICEType>> stream;                 /**< The stream which listenen to the channel */
    std::shared_ptr<InformationSet<ICEType>>    set;                    /**< The stream which listenen to the channel */
  };

template<typename ICEType, typename ROSType>
  inline RosInformationReceiver<ICEType, ROSType>::RosInformationReceiver(
      identifier engineId, std::shared_ptr<InformationCollection> collection, ros::NodeHandle* nodeHandel,
      const std::string topic, int bufferSize, transformM2C<ICEType, ROSType> &messageTransform) :
      InformationReceiver(collection), topic(topic)
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

template<typename ICEType, typename ROSType>
  inline RosInformationReceiver<ICEType, ROSType>::~RosInformationReceiver()
  {
    //
  }

template<typename ICEType, typename ROSType>
  inline void RosInformationReceiver<ICEType, ROSType>::onMessage(const boost::shared_ptr<ROSType const> msg)
  {
    if (false == this->checkReceiverIds(msg->header))
      return;

    auto contrainer = this->messageTransform(msg);
    if (stream)
      stream->add(contrainer);
    else
      set->add(msg->header.entity, contrainer);
  }

template<typename ICEType, typename ROSType>
  inline bool RosInformationReceiver<ICEType, ROSType>::checkReceiverIds(ice_msgs::ICEHeader header)
  {
    for (auto id : header.receiverIds)
    {
      identifier receiverId = id.value;//IDGenerator::getInstance()->getIdentifier(id.id);

      if (receiverId == this->engineId)
        return true;
    }

    return false;
  }

} /* namespace ice */

#endif /* ROSINFORMATIONRECEIVER_H_ */
