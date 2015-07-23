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
  ice::Representation rt;
  ice::Representation rf;
  int res;

  res = rt.fromCSV("1;true");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::BooleanRep, rt.type);
  ASSERT_TRUE(*(rt.get<bool*>()));

  res = rf.fromCSV("1;false");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::BooleanRep, rf.type);
  ASSERT_FALSE(*(rf.get<bool*>()));
}

TEST(RepresentationTest, ByteRep)
{
  ice::Representation r;

  const int res = r.fromCSV("2;A");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::ByteRep, r.type);
  ASSERT_EQ('A', *r.get<char*>());
}

TEST(RepresentationTest, StringRep)
{
  ice::Representation r;

  const int res = r.fromCSV("3;Hello World");

  ASSERT_EQ(0, res);
  ASSERT_EQ(ice::StringRep, r.type);
  ASSERT_STREQ("Hello World", r.get<const char*>());
}



}



