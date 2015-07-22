/*
 * test_OI.cpp
 *
 *  Created on: 10.06.2015
 *      Author: paspartout
 */

#include "gtest/gtest.h"
#include "ice/representation/Representation.h"


namespace
{

TEST(RepresentationTest, BooleanRep)
{
  ice::Representation r;

  const int res = r.fromCSV("1;true");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::BooleanRep, r.type);
  ASSERT_TRUE(*(r.get<bool*>()));
}

TEST(RepresentationTest, StringRep)
{
  ice::Representation r;

  const int res = r.fromCSV("3;Hello World");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::StringRep, r.type);
  ASSERT_TRUE(strcmp("Hello World", r.get<const char*>()) == 0);
}

}



