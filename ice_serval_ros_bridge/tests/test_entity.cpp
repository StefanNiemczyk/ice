#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "Entity.h"

TEST(Entity, checkMatching)
{
   std::shared_ptr<ice::Entity> id1(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}}));

   std::shared_ptr<ice::Entity> idTest(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}}));
   ASSERT_EQ(ice::entity_match::FULL_MATCH, id1->checkMatching(idTest));

   idTest.reset(new ice::Entity(nullptr, {{"id2", "value_id2"}, {"id3", "value_id3"}}));
   ASSERT_EQ(ice::entity_match::PARTIAL_MATCH, id1->checkMatching(idTest));

   idTest.reset(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}, {"id3", "value_id3"}}));
   ASSERT_EQ(ice::entity_match::INCLUDED, id1->checkMatching(idTest));

   idTest.reset(new ice::Entity(nullptr, {{"id1", "value_id1"}}));
   ASSERT_EQ(ice::entity_match::INCLUDING, id1->checkMatching(idTest));

   idTest.reset(new ice::Entity(nullptr, {{"id1", "value_id1_1"}}));
   ASSERT_EQ(ice::entity_match::NO_MATCH, id1->checkMatching(idTest));

   idTest.reset(new ice::Entity(nullptr, {{"id1", "value_id1_1"}, {"id2", "value_id2"}}));
   ASSERT_EQ(ice::entity_match::CONFLICTING, id1->checkMatching(idTest));
}

TEST(Entity, checkMatchingList)
{
   std::shared_ptr<ice::Entity> id1(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}}));

   ASSERT_EQ(ice::entity_match::FULL_MATCH, id1->checkMatching({{"id1", "value_id1"}, {"id2", "value_id2"}}));

   ASSERT_EQ(ice::entity_match::PARTIAL_MATCH, id1->checkMatching({{"id2", "value_id2"}, {"id3", "value_id3"}}));

   ASSERT_EQ(ice::entity_match::INCLUDED, id1->checkMatching({{"id1", "value_id1"}, {"id2", "value_id2"}, {"id3", "value_id3"}}));

   ASSERT_EQ(ice::entity_match::INCLUDING, id1->checkMatching({{"id1", "value_id1"}}));

   ASSERT_EQ(ice::entity_match::NO_MATCH, id1->checkMatching({{"id1", "value_id1_1"}}));

   ASSERT_EQ(ice::entity_match::CONFLICTING, id1->checkMatching({{"id1", "value_id1_1"}, {"id2", "value_id2"}}));
}

TEST(Entity, checkMatchingSingle)
{
  std::shared_ptr<ice::Entity> id1(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}}));

  std::string key = "id1";
  std::string value = "value_id1";
  ASSERT_EQ(ice::entity_match::FULL_MATCH, id1->checkMatching(key, value));

  key = "id3";
  value = "value_id3";
  ASSERT_EQ(ice::entity_match::NO_MATCH, id1->checkMatching(key, value));

  key = "id1";
  value = "value_id1_1";
  ASSERT_EQ(ice::entity_match::CONFLICTING, id1->checkMatching(key, value));
}

TEST(Entity, connectionQuality)
{
  std::shared_ptr<ice::Entity> id(new ice::Entity(nullptr, {{"id1", "value_id1"}, {"id2", "value_id2"}}));

  std::string key = "cq1";
  double value = 12.5;
  id->addConnectionQuality(key, value);

  double out = 0;
  ASSERT_TRUE(id->getConnectionQuality(key, out));
  ASSERT_EQ(out, value);

  key = "cq2";
  ASSERT_FALSE(id->getConnectionQuality(key, out));
}
