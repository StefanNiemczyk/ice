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

#include "ice/Identifier.h"
#include "ice/communication/InformationSender.h"

#include "ice_msgs/Identifier.h"

namespace ice
{

template<typename ICEType, typename ROSType>
  using transformC2M = typename std::unique_ptr<ROSType> (*)(std::shared_ptr<InformationElement<ICEType> >);

template<typename ICEType, typename ROSType>
  class RosInformationSender : public InformationSender<ICEType>
  {
  public:
    RosInformationSender(identifier engineId, ros::NodeHandle* nodeHandel, const std::string topic, int bufferSize,
                         transformC2M<ICEType, ROSType> &messageTransform);
    virtual ~RosInformationSender();

    virtual void sendInformationElement(std::shared_ptr<std::vector<identifier>> sendTo,
                                        std::shared_ptr<InformationElement<ICEType>> informationElement);

    /*!
     * \brief Returns the type_info of the template type.
     *
     * Returns the type_info of the template type.
     */
    virtual const std::type_info* getTypeInfo() const;

  private:
    identifier engineId; /**< ID of this engine, used as sender id */
    const std::string topic; /**< Topic of the ros channel */
    int bufferSize; /**< Size of the buffer */
    transformC2M<ICEType, ROSType> messageTransform; /**< Function pointer to message Transform method */
    ros::Publisher publisher; /**< Publisher */
  };

template<typename ICEType, typename ROSType>
  inline RosInformationSender<ICEType, ROSType>::RosInformationSender(identifier engineId, ros::NodeHandle* nodeHandel,
                                                                      const std::string topic, int bufferSize,
                                                                      transformC2M<ICEType, ROSType> &messageTransform) :
      topic(topic)
  {
    this->engineId = engineId;
    this->bufferSize = bufferSize;
    this->publisher = nodeHandel->advertise<ROSType>(topic, bufferSize);
    this->messageTransform = messageTransform;
  }

template<typename ICEType, typename ROSType>
  inline RosInformationSender<ICEType, ROSType>::~RosInformationSender()
  {
    //
  }

template<typename ICEType, typename ROSType>
  inline void RosInformationSender<ICEType, ROSType>::sendInformationElement(
      std::shared_ptr<std::vector<identifier> > sendTo,
      std::shared_ptr<InformationElement<ICEType> > informationElement)
  {
    auto msg = this->messageTransform(informationElement);

    msg->header.senderId.value = this->engineId;
//    msg->header.senderId.id.resize(16);
//    std::copy(this->engineId.begin(), this->engineId.end(), msg->header.senderId.id.begin());

    for (auto identifier : *sendTo)
    {
      ice_msgs::Identifier receiver;
      receiver.value = identifier;
     // std::copy(identifier.begin(), identifier.end(), receiver.id.begin());
      msg->header.receiverIds.push_back(receiver);
    }

    this->publisher.publish(*msg);
  }

template<typename ICEType, typename ROSType>
  const std::type_info* RosInformationSender<ICEType, ROSType>::getTypeInfo() const
  {
    return &typeid(ICEType);
  }
} /* namespace ice */

#endif /* ROSINFORMATIONSENDER_H_ */
