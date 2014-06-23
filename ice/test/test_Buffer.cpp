/*
 * test_Buffer.cpp
 *
 *  Created on: May 21, 2014
 *      Author: sni
 */

#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "ice/container/Buffer.h"

#include "gtest/gtest.h"

namespace
{

class BufferTest : public ::testing::Test
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


TEST_F(BufferTest, create_buffer)
{
  ice::Buffer<int> bufferInt(10, false);
  EXPECT_EQ(10, bufferInt.getBufferSize());
  EXPECT_EQ(typeid(int), *bufferInt.getTypeInfo());
  EXPECT_EQ(false, bufferInt.isThreadSafe());

  ice::Buffer<double> bufferDouble(10, true);
  EXPECT_EQ(10, bufferDouble.getBufferSize());
  EXPECT_EQ(typeid(double), *bufferDouble.getTypeInfo());
  EXPECT_EQ(true, bufferDouble.isThreadSafe());

  ice::Buffer<std::string> bufferString(10, false);
  EXPECT_EQ(10, bufferString.getBufferSize());
  EXPECT_EQ(typeid(std::string), *bufferString.getTypeInfo());
  EXPECT_EQ(false, bufferString.isThreadSafe());

  ice::Buffer<bool> bufferBool(10, true);
  EXPECT_EQ(10, bufferBool.getBufferSize());
  EXPECT_EQ(typeid(bool), *bufferBool.getTypeInfo());
  EXPECT_EQ(true, bufferBool.isThreadSafe());

  EXPECT_NE(typeid(bool), *bufferInt.getTypeInfo());
}

TEST_F(BufferTest, add)
{
  ice::Buffer<int> bufferInt(10, false);
  std::shared_ptr<int> intPtr;
  std::shared_ptr<int> returnPtr;

  for (int i=0; i < 15; ++i) {
    intPtr = std::make_shared<int>(i);
    returnPtr = bufferInt.add(intPtr);

    if (i < bufferInt.getBufferSize()) {
      EXPECT_EQ(i+1, bufferInt.getSize());
    EXPECT_FALSE((returnPtr ? true : false));
    } else {
      EXPECT_EQ(bufferInt.getBufferSize(), bufferInt.getSize());
      EXPECT_TRUE((returnPtr ? true : false));
    }
  }
}


TEST_F(BufferTest, add_getFirst)
{
  ice::Buffer<int> bufferInt(10, false);
  std::shared_ptr<int> intPtr;

  for (int i=0; i < 15; ++i) {
    intPtr = std::make_shared<int>(i);
    bufferInt.add(intPtr);
  }

  for (int i=0; i < 15; ++i){
    intPtr = bufferInt.popFirst();

    if (i < bufferInt.getBufferSize()) {
      EXPECT_EQ (10 - 1 - i, bufferInt.getSize());
      EXPECT_TRUE((intPtr ? true : false));
      EXPECT_EQ(15-10 + i, *intPtr);
    } else {
      EXPECT_EQ (0, bufferInt.getSize());
      EXPECT_FALSE((intPtr ? true : false));
    }
  }
}


TEST_F(BufferTest, add_getLast)
{
  ice::Buffer<int> bufferInt(10, false);
  std::shared_ptr<int> intPtr;

  for (int i=0; i < 15; ++i) {
    intPtr = std::make_shared<int>(i);
    bufferInt.add(intPtr);
  }

  for (int i=0; i < 15; ++i){
    intPtr = bufferInt.popLast();

    if (i < bufferInt.getBufferSize()) {
      EXPECT_EQ (10 - 1 - i, bufferInt.getSize());
      EXPECT_TRUE((intPtr ? true : false));
      EXPECT_EQ(14 - i, *intPtr);
    } else {
      EXPECT_EQ (0, bufferInt.getSize());
      EXPECT_FALSE((intPtr ? true : false));
    }
  }
}


}

