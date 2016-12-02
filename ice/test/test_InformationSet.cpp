#include <gtest/gtest.h>

#include "ice/information/CollectionDescription.h"
#include "ice/information/InformationSet.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"

TEST(InformationSet, simpleTest)
{
  auto handler = std::make_shared<ice::EventHandler>(2, 100);
  auto spec = std::make_shared<ice::InformationSpecification>("", "type", "scope", "rep");
  std::map<std::string, int> metadatas;
  auto description = std::make_shared<ice::CollectionDescription>(spec, "name,", "provider", "source", metadatas);
  auto set = std::make_shared<ice::InformationSet<int>>(description, handler);

  set->add("test1", std::make_shared<int>(1));
  set->add("test2", std::make_shared<int>(2));
  ASSERT_EQ(2, set->getSize());

  set->add("test2", std::make_shared<int>(3));
  ASSERT_EQ(2, set->getSize());

  ASSERT_EQ(1, *set->get("test1")->getInformation());
  ASSERT_EQ(3, *set->get("test2")->getInformation());
}
