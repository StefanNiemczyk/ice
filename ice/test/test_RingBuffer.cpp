#include <iostream>
#include <string>
#include <time.h>
#include <typeinfo>
#include <memory>

#include "ice/container/RingBuffer.h"

#include "gtest/gtest.h"

namespace
{

class RingBufferTest : public ::testing::Test
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


TEST_F(RingBufferTest, create_buffer)
{
  ice::RingBuffer<int> bufferInt(10);
  EXPECT_EQ(10, bufferInt.getBufferSize());
  EXPECT_EQ(typeid(int), *bufferInt.getTypeInfo());

  ice::RingBuffer<double> bufferDouble(10);
  EXPECT_EQ(10, bufferDouble.getBufferSize());
  EXPECT_EQ(typeid(double), *bufferDouble.getTypeInfo());

  ice::RingBuffer<std::string> bufferString(10);
  EXPECT_EQ(10, bufferString.getBufferSize());
  EXPECT_EQ(typeid(std::string), *bufferString.getTypeInfo());

  ice::RingBuffer<bool> bufferBool(10);
  EXPECT_EQ(10, bufferBool.getBufferSize());
  EXPECT_EQ(typeid(bool), *bufferBool.getTypeInfo());

  ice::RingBuffer<std::shared_ptr<std::string> > bufferSharedPtr(10);
  EXPECT_EQ(10, bufferSharedPtr.getBufferSize());
  EXPECT_EQ(typeid(std::shared_ptr<std::string>), *bufferSharedPtr.getTypeInfo());


  EXPECT_NE(typeid(bool), *bufferInt.getTypeInfo());
}

TEST_F(RingBufferTest, read_write1)
{
  ice::RingBuffer<int> bufferInt(10);
  std::shared_ptr<int> intPtr;

  for (int i=0; i < bufferInt.getBufferSize() + 5; ++i) {
    intPtr = std::make_shared<int>(i);
    int result = bufferInt.add(intPtr);

    EXPECT_EQ(i, result);
  }

  for (int i=0; i < bufferInt.getBufferSize(); ++i) {
    intPtr = bufferInt.getLast(i);

    EXPECT_TRUE((intPtr ? true : false));
    EXPECT_EQ(bufferInt.getBufferSize() - 1 + 5 - i, *intPtr);
  }
}

TEST_F(RingBufferTest, clean)
{
  ice::RingBuffer<int> bufferInt(10);
  std::shared_ptr<int> intPtr;

  for (int i=0; i < bufferInt.getBufferSize() + 5; ++i) {
    intPtr = std::make_shared<int>(i);
    int result = bufferInt.add(intPtr);

    EXPECT_EQ(i, result);
  }

  bufferInt.clear(false);

  intPtr = bufferInt.getLast();

  EXPECT_FALSE((intPtr ? true : false));
}

}
/*
int main(int argc, char **argv)
{
  try
  {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }
  catch (std::exception &e)
  {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}*/
