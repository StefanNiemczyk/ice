/*
 * RosTimeFactory.cpp
 *
 *  Created on: Jun 17, 2014
 *      Author: sni
 */

#include "ice/ros/RosTimeFactory.h"

namespace ice
{

RosTimeFactory::RosTimeFactory()
{
  // TODO Auto-generated constructor stub

}

RosTimeFactory::~RosTimeFactory()
{
  // TODO Auto-generated destructor stub
}

time RosTimeFactory::createTime()
{
  ros::Time rosTime = ros::Time::now();
  time iceTime = rosTime.sec * 1000000000 + rosTime.nsec;
  return iceTime;
}

time RosTimeFactory::createTime(ros::Time rosTime)
{
  time iceTime = (rosTime.sec * 1000000000 + rosTime.nsec);
  return iceTime;
}

bool RosTimeFactory::checkTimeout(time timestamp, long milliseconds)
{
  time t = this->createTime();

  return (timestamp + milliseconds * 1000000) < t;
}

} /* namespace ice */
