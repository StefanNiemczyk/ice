/*
 * time.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef TIME_H_
#define TIME_H_

#include <memory>
#include <chrono>

namespace ice
{
/** Type definition of the time data type */
typedef unsigned long long time;

/** Value definition of not specified time fields */
static time NO_TIME = 0;

//* TimeFactory
/**
 * Factory to create new time stamps.
 *
 */
class TimeFactory
{
public:
  TimeFactory() {};
  virtual ~TimeFactory() {};

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


class SimpleTimeFactory : public TimeFactory
{
public:
  SimpleTimeFactory() {}

  virtual ~SimpleTimeFactory() {}

  virtual ice::time createTime()
  {
    auto now = std::chrono::system_clock::now();
    auto tms = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = tms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);

    return value.count();
  }

  virtual bool checkTimeout(ice::time timestamp, long millisecond)
  {
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    auto tms = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = tms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);

    return (timestamp + (millisecond * 1000000)) < value.count();
  }
};

} /* namespace ice */

#endif /* TIME_H_ */
