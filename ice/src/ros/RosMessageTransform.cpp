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

std::unique_ptr<ice_msgs::Position> RosMessageTransform::transformC2MPosition(
    std::shared_ptr<InformationElement<Position> > informationElement)
{
  std::unique_ptr<ice_msgs::Position> pos(new ice_msgs::Position());

  auto info = informationElement->getInformation();

#ifdef LOG_MESSAGE_TRANSFORM
  std::cout << "transformC2MPosition: " << info.x << ", " << info.y << ", " << info.z << std::endl;
#endif

  pos->x = info.x;
  pos->y = info.y;
  pos->z = info.z;

  return pos;
}

std::unique_ptr<Position> RosMessageTransform::transformM2CPosition(
    const boost::shared_ptr<ice_msgs::Position const> msg)
{
  std::unique_ptr<Position> pos(new Position());

#ifdef LOG_MESSAGE_TRANSFORM
  std::cout << "transformM2CPosition: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
#endif

  pos->x = msg->x;
  pos->y = msg->y;
  pos->z = msg->z;

  return pos;
}

std::unique_ptr<ice_msgs::Positions> RosMessageTransform::transformC2MPositions(std::shared_ptr<InformationElement<std::vector<Position>>> informationElement)
{
  auto infoList = informationElement->getInformation();
  std::unique_ptr<ice_msgs::Positions> positions(new ice_msgs::Positions());

  for (auto infoPos : infoList)
  {
    ice_msgs::Position pos;

    pos.x = infoPos.x;
    pos.y = infoPos.y;
    pos.z = infoPos.z;

    positions->positions.push_back(pos);
  }

  return positions;
}

std::unique_ptr<std::vector<Position>> RosMessageTransform::transformM2CPositions(const boost::shared_ptr<ice_msgs::Positions const> msg)
{
  std::unique_ptr<std::vector<Position>> positions(new std::vector<Position>());

  for (auto msgPos : msg->positions)
  {
    Position pos;

    pos.x = msgPos.x;
    pos.y = msgPos.y;
    pos.z = msgPos.z;

    positions->push_back(pos);
  }

  return positions;
}

} /* namespace ice */
