/*
 * TimeFactory.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef TIMEFACTORY_H_
#define TIMEFACTORY_H_

#include <memory>

#include "ice/Time.h"

namespace ice
{

//* TimeFactory
/**
 * Factory to create new time stamps.
 *
 */
class TimeFactory
{
public:
  TimeFactory();
  virtual ~TimeFactory();

  /*!
   * \brief Creates and returns a new time stamp.
   *
   * Creates and returns a new time stamp.
   */
  virtual time createTime() = 0;

  /*!
   * \brief Checks if the timestamp is more than millisecond in past.
   *
   * Checks if the timestamp is more than millisecond in past.
   *
   * \param timestamp The timestamp to check.
   * \param millisecond The timeout duration in milliseconds.
   */
  virtual bool checkTimeout(time timestamp, long milliseconds) = 0;
};

} /* namespace ice */

#endif /* TIMEFACTORY_H_ */
