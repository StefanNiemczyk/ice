/*
 * test_Logger.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "ice/Logger.h"

#include "gtest/gtest.h"

namespace
{

class TestLogger : public ::testing::Test
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

    EXPECT_TRUE(end_time - start_time_ <= 5) << "The test took too long.";
  }
};

TEST_F(TestLogger, test_verbose)
{
  auto _log = ice::Logger::get("test");

  _log->verbose("test_verbose", "Simple Test");
  _log->verbose("test_verbose", "Simple Test %s", "string");
  _log->verbose("test_verbose", "Simple Test %i", 5);
  _log->verbose("test_verbose", "Simple Test %f", 1.8);
  _log->verbose("test_verbose", "Complex Test %s, %i, %f", "muhkuh", 128, 1.8);
}

}


