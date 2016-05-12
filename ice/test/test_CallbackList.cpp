#include <iostream>
#include <jni.h>
#include <vector>
#include <string>


#include "ice/processing/CallbackList.h"

#include "gtest/gtest.h"

namespace
{

class CBTest1
{
public:
  void cb(int value)
  {
    val = value;
  }

  int val;
};

class CBTest2
{
public:
  void cb(int value)
  {
    val = value;
  }

  int val;
};

typedef void (*testFunction)(int);

static int val = 0;
void simpleTestFunction(int value)
{
  val = value;
}

TEST(CallbackListTest, functionCallback)
{
  ice::CallbackList<int> cl;

  cl.registerCallback(*simpleTestFunction);

  cl.trigger(5);
  ASSERT_EQ(5, val);

  ASSERT_TRUE(cl.unregisterCallback(*simpleTestFunction));

  cl.trigger(10);
  ASSERT_EQ(5, val);

  ASSERT_FALSE(cl.unregisterCallback(*simpleTestFunction));
}

TEST(CallbackListTest, classCallback)
{
  ice::CallbackList<int> cl;
  CBTest1 cb1;
  CBTest2 cb2;

  cl.registerCallback(&cb1, &CBTest1::cb);
  cl.registerCallback(&cb2, &CBTest2::cb);

  cl.trigger(5);
  ASSERT_EQ(5, cb1.val);
  ASSERT_EQ(5, cb2.val);

  ASSERT_TRUE(cl.unregisterCallback(&cb1, &CBTest1::cb));

  cl.trigger(10);
  ASSERT_EQ(5, cb1.val);
  ASSERT_EQ(10, cb2.val);

  ASSERT_FALSE(cl.unregisterCallback(&cb1, &CBTest1::cb));
}

}
