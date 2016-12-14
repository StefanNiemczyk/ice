/*
 * test_Containers.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: sni
 */

#include <gtest/gtest.h>
#include <fstream>

#include <ice/representation/GContainer.h>

#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"
#include "container/WGS84.h"

#include "TBKnowledgeBase.h"

int runs = 10000;

void gen_random(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZÖÄÜ"
        "abcdefghijklmnopqrstuvwxyzöäüß"
        ":#_-+!'\"§$%&/()=?";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}

TEST(CustomContainer, Pos3D)
{
  auto tbKnowledgeBase = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  tbKnowledgeBase->init();
  auto factory = tbKnowledgeBase->getGContainerFactory();
  auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#CoordinatePositionRep");

  for (int i=0; i < runs; ++i)
  {
    auto element = std::make_shared<ice::Pos3D>(rep);

    element->x = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->y = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->z = (double)rand() / 2.0 + (double)rand() / RAND_MAX;

    auto deserialized = factory->fromJSON(element->toJSON());
    ASSERT_TRUE((deserialized != nullptr));
    auto element2 = std::static_pointer_cast<ice::Pos3D>(deserialized);

    ASSERT_DOUBLE_EQ(element->x, element2->x);
    ASSERT_DOUBLE_EQ(element->y, element2->y);
    ASSERT_DOUBLE_EQ(element->z, element2->z);
  }
}

TEST(CustomContainer, PositionOrientation3D)
{
  auto tbKnowledgeBase = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  tbKnowledgeBase->init();
  auto factory = tbKnowledgeBase->getGContainerFactory();
  auto rep = factory->getRepresentation("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D");

  for (int i=0; i < runs; ++i)
  {
    auto element = std::make_shared<ice::PositionOrientation3D>(rep);

    element->alpha = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->x = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->y = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->z = (double)rand() / 2.0 + (double)rand() / RAND_MAX;

    auto deserialized = factory->fromJSON(element->toJSON());
    ASSERT_TRUE((deserialized != nullptr));
    auto element2 = std::static_pointer_cast<ice::PositionOrientation3D>(deserialized);

    ASSERT_DOUBLE_EQ(element->alpha, element2->alpha);
    ASSERT_DOUBLE_EQ(element->x, element2->x);
    ASSERT_DOUBLE_EQ(element->y, element2->y);
    ASSERT_DOUBLE_EQ(element->z, element2->z);
  }
}

TEST(CustomContainer, RTLandmark)
{
  auto tbKnowledgeBase = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  tbKnowledgeBase->init();
  auto factory = tbKnowledgeBase->getGContainerFactory();
  auto rep = factory->getRepresentation("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark");

  for (int i=0; i < runs; ++i)
  {
    auto element = std::make_shared<ice::RTLandmark>(rep);

    element->x = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->y = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->z = (double)rand() / 2.0 + (double)rand() / RAND_MAX;

    char str[51];
    gen_random(str, 50);
    element->landmark = std::string(str);

    auto deserialized = factory->fromJSON(element->toJSON());
    ASSERT_TRUE((deserialized != nullptr));
    auto element2 = std::static_pointer_cast<ice::RTLandmark>(deserialized);

    ASSERT_DOUBLE_EQ(element->x, element2->x);
    ASSERT_DOUBLE_EQ(element->y, element2->y);
    ASSERT_DOUBLE_EQ(element->z, element2->z);
    ASSERT_EQ(element->landmark, element2->landmark);
  }
}


TEST(CustomContainer, WGS84)
{
  auto tbKnowledgeBase = std::make_shared<ice::TBKnowledgeBase>("Leonardo");
  tbKnowledgeBase->init();
  auto factory = tbKnowledgeBase->getGContainerFactory();
  auto rep = factory->getRepresentation("http://vs.uni-kassel.de/Ice#WGS84Rep");

  for (int i=0; i < runs; ++i)
  {
    auto element = std::make_shared<ice::WGS84>(rep);

    element->altitude = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->latitude = (double)rand() / 2.0 + (double)rand() / RAND_MAX;
    element->longitude = (double)rand() / 2.0 + (double)rand() / RAND_MAX;

    auto deserialized = factory->fromJSON(element->toJSON());
    ASSERT_TRUE((deserialized != nullptr));
    auto element2 = std::static_pointer_cast<ice::WGS84>(deserialized);

    ASSERT_DOUBLE_EQ(element->altitude, element2->altitude);
    ASSERT_DOUBLE_EQ(element->latitude, element2->latitude);
    ASSERT_DOUBLE_EQ(element->longitude, element2->longitude);
  }
}
