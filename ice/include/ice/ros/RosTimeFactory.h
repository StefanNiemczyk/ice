/*
 * RosTimeFactory.h
 *
 *  Created on: Jun 17, 2014
 *      Author: sni
 */

#ifndef ROSTIMEFACTORY_H_
#define ROSTIMEFACTORY_H_

#include "ros/ros.h"

#include "ice/Time.h"
#include "ice/TimeFactory.h"

namespace ice
{

class RosTimeFactory : public TimeFactory
{
public:
  static time createTime(ros::Time rosTime);

public:
  RosTimeFactory();
  virtual ~RosTimeFactory();

  /*!
   * \brief Creates and returns a new time stamp.
   *
   * Creates and returns a new time stamp.
   */
  virtual time createTime();

  /*!
   * \brief Checks if the timestamp is more than millisecond in past.
   *
   * Checks if the timestamp is more than millisecond in past.
   *
   * \param timestamp The timestamp to check.
   * \param millisecond The timeout duration in milliseconds.
   */
  virtual bool checkTimeout(time timestamp, long milliseconds);
};

} /* namespace ice */

#endif /* ROSTIMEFACTORY_H_ */
