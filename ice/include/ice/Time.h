/*
 * time.h
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#ifndef TIME_H_
#define TIME_H_

#include <memory>


namespace ice
{
/** Type definition of the time data type */
typedef unsigned long long time;

/** Value definition of not specified time fields */
static time NO_TIME = 0;





// OLD #########################################################################################
class Time
{
public:
  Time();
  virtual ~Time();

  virtual bool isBefore(Time* otherTime) const = 0;
  virtual bool isBeforeOrEqual(Time* otherTime) const = 0;
  virtual bool isAfter(Time* otherTime) const = 0;
  virtual bool isAfterOrEqual(Time* otherTime) const = 0;

  virtual bool isBefore(std::shared_ptr<Time> otherTime) const = 0;
  virtual bool isBeforeOrEqual(std::shared_ptr<Time> otherTime) const = 0;
  virtual bool isAfter(std::shared_ptr<Time> otherTime) const = 0;
  virtual bool isAfterOrEqual(std::shared_ptr<Time> otherTime) const = 0;

  virtual bool isBeforeCurrent() const = 0;
  virtual bool isAfterCurrent() const = 0;

  virtual std::shared_ptr<Time> clone(std::shared_ptr<Time> otherTime) const = 0;

  virtual Time* clone(Time* otherTime) const = 0;
};

} /* namespace ice */

#endif /* TIME_H_ */
