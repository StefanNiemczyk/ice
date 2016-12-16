/*
 * RosMessageTransform.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: sni
 */

//#define LOG_MESSAGE_TRANSFORM

#include "ice/ros/RosMessageTransform.h"

namespace ice
{

std::shared_ptr<ice_msgs::Position> RosMessageTransform::transformC2MPosition(
    std::shared_ptr<InformationElement<Position> > informationElement)
{
  auto pos = std::make_shared<ice_msgs::Position>();

  auto info = informationElement->getInformation();

#ifdef LOG_MESSAGE_TRANSFORM
  std::cout << "transformC2MPosition: " << info.x << ", " << info.y << ", " << info.z << std::endl;
#endif

  pos->x = info->x;
  pos->y = info->y;
  pos->z = info->z;

  return pos;
}

std::shared_ptr<Position> RosMessageTransform::transformM2CPosition(
    const boost::shared_ptr<ice_msgs::Position const> msg)
{
  auto pos = std::make_shared<Position>();

#ifdef LOG_MESSAGE_TRANSFORM
  std::cout << "transformM2CPosition: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
#endif

  pos->x = msg->x;
  pos->y = msg->y;
  pos->z = msg->z;

  return pos;
}

} /* namespace ice */
