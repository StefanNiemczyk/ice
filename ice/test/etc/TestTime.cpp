#include <iostream>
#include <sys/time.h>

#include "ice/TimeFactory.h"
#include "ice/Time.h"

class TestTime : public ice::Time
{
public:
  TestTime()
  {
    time = 0;
  }

  TestTime(int time)
  {
    this->time = time;
  }

  TestTime(std::shared_ptr<ice::Time> otherTime)
  {
    time = 0;
  }

  virtual ~TestTime()
  {
  }

  virtual bool isBefore(ice::Time* otherTime) const
  {
    TestTime* t2 = (TestTime*)otherTime;
    return this->time < t2->time;
  }

  virtual bool isBeforeOrEqual(ice::Time* otherTime) const
  {
    return false;
  }

  virtual bool isAfter(ice::Time* otherTime) const
  {
    return false;
  }
  virtual bool isAfterOrEqual(ice::Time* otherTime) const
  {
    return false;
  }

  virtual bool isBefore(std::shared_ptr<ice::Time> otherTime) const
  {
    std::shared_ptr<TestTime> t2 = std::static_pointer_cast<TestTime>(otherTime);

    return this->time < t2->time;
  }
  virtual bool isBeforeOrEqual(std::shared_ptr<ice::Time> otherTime) const
  {
    return false;
  }
  virtual bool isAfter(std::shared_ptr<ice::Time> otherTime) const
  {
    return false;
  }
  virtual bool isAfterOrEqual(std::shared_ptr<ice::Time> otherTime) const
  {
    return false;
  }

  virtual bool isBeforeCurrent() const
  {
    return false;
  }
  virtual bool isAfterCurrent() const
  {
    return false;
  }

  std::shared_ptr<Time> clone(std::shared_ptr<Time> otherTime) const
  {
    return std::make_shared<TestTime>(this->time);
  }

  Time* clone(Time* otherTime) const
  {
    return new TestTime(this->time);
  }

  long getTime() const
  {
    return time;
  }

  void setTime(long time)
  {
    this->time = time;
  }

private:
  long time;
};

class TestTimeFactory : public ice::TimeFactory
{
public:
  TestTimeFactory()
  {
  }
  virtual ~TestTimeFactory()
  {
  }

  virtual ice::time createTime() {
    timeval val;
    gettimeofday(&val, NULL);

    return val.tv_usec;
  }

  virtual bool checkTimeout(ice::time timestamp, long millisecond)
  {
    timeval val;
    gettimeofday(&val, NULL);

    return (timestamp + (millisecond * 1000)) < val.tv_usec;
  }

 /* virtual std::shared_ptr<ice::Time> createTimeSharedPtr() {
    return std::make_shared<TestTime>();
  }*/
};
