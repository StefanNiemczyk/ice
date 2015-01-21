/*
 * test_EventHandler.cpp
 *
 *  Created on: May 22, 2014
 *      Author: sni
 */

#include <iostream>
#include <memory>
#include <string>
#include <time.h>
#include <typeinfo>
#include <vector>

#include "ice/processing/AsynchronousTask.h"
#include "ice/processing/EventHandler.h"

#include "gtest/gtest.h"

namespace
{

class SimpleAsyncEvent : public ice::AsynchronousTask
{
public:
  SimpleAsyncEvent(int value, int expected)
  {
    this->value = value;
    this->expected = expected;
  }
  virtual ~SimpleAsyncEvent()
  {

  }
  virtual int performTask()
  {
    this->value = this->expected;
    return 0;
  }

public:
  int value;
  int expected;
};

class EventHandlerTest : public ::testing::Test
{
protected:
  time_t start_time_;

  virtual void SetUp()
  {
    start_time_ = time(NULL);
  }

  virtual void TearDown()
  {
    const time_t end_time = time(NULL);

  //  EXPECT_TRUE(end_time - start_time_ <= 5000) << "The test took too long.";
  }
};

TEST_F(EventHandlerTest, create)
{
  ice::EventHandler handler(3, 10);

  EXPECT_EQ(3, handler.getThreadPoolSize());

  std::this_thread::sleep_for(std::chrono::milliseconds {100});
}

TEST_F(EventHandlerTest, handle_one_event)
{
  ice::EventHandler handler(3, 1);
  std::shared_ptr<ice::AsynchronousTask> event = std::make_shared<SimpleAsyncEvent>(0, 5);
  SimpleAsyncEvent* e = (SimpleAsyncEvent*) event.get();

  EXPECT_NE(e->value, e->expected);

  handler.addTask(event);

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  EXPECT_EQ(e->value, e->expected);
}

TEST_F(EventHandlerTest, handle_10000_event)
{
  ice::EventHandler handler(3, 10000);
  std::vector<std::shared_ptr<SimpleAsyncEvent> > vec;


  for (int i=0; i < 10000; ++i)
  {
    auto event = std::make_shared<SimpleAsyncEvent>(-i, i);
    handler.addTask(event);
    vec.push_back(event);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  for (auto e : vec)
  {
    EXPECT_EQ(e->value, e->expected);
  }
}

TEST_F(EventHandlerTest, drop_event)
{
  ice::EventHandler handler(0, 10);

  for (int i=0; i < 20; ++i)
  {
    auto event = std::make_shared<SimpleAsyncEvent>(-i, i);
    int result = handler.addTask(event);

    if (i < handler.getBufferSize())
    {
      EXPECT_EQ(0, result);
    } else {
      EXPECT_EQ(1, result);
    }
  }

}

TEST_F(EventHandlerTest, timertask)
{
  ice::EventHandler handler(3, 1);
  std::shared_ptr<ice::AsynchronousTask> event = std::make_shared<SimpleAsyncEvent>(0, 5);
  SimpleAsyncEvent* e = (SimpleAsyncEvent*) event.get();

  EXPECT_NE(e->value, e->expected);

  handler.addTimerTask(event, 1000);
  cout << "Waiting a second for the task to start" << endl;

  std::this_thread::sleep_for(std::chrono::milliseconds {500});

  // As the task should start after around 500 ms the test values should still be diffrerent
  EXPECT_NE(e->value, e->expected);

  std::this_thread::sleep_for(std::chrono::milliseconds {600});

  // Now after total 1100 ms the task should be done and the test values equal
  EXPECT_EQ(e->value, e->expected);
}


/*
TEST_F(EventHandlerTest, handle_10_sec_full_automatic_fire)
{
  ice::EventHandler handler(4, 5000);
  std::vector<std::shared_ptr<SimpleAsyncEvent> > vec;

  const time_t start_time = time(NULL);
  time_t end_time = time(NULL);
  int i = 0;

  while(end_time - start_time <= 10)
  {
    auto event = std::make_shared<SimpleAsyncEvent>(-1, i);
    handler.addTask(event);
    vec.push_back(event);

    ++i;
    end_time = time(NULL);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds {100});

  for (auto e : vec)
  {
    EXPECT_EQ(e->value, e->expected);
  }
}
*/
}

