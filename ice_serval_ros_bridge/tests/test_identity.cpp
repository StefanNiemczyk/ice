#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "Identity.h"

TEST(Identity, checkMatching)
{
   std::shared_ptr<ice::Identity> id1(new ice::Identity({{"id1", "value_id1"}, {"id2", "value_id2"}}));


   std::shared_ptr<ice::Identity> idTest(new ice::Identity({{"id1", "value_id1"}, {"id2", "value_id2"}}));
   ASSERT_EQ(ice::identity_match::FULL_MATCH, id1->checkMatching(idTest));

   idTest(new ice::Identity({{"id2", "value_id2"}, {"id3", "value_id3"}}));
   ASSERT_EQ(ice::identity_match::PARTIAL_MATCH, id1->checkMatching(idTest));

   idTest(new ice::Identity({{"id1", "value_id1"}, {"id2", "value_id2"}, {"id3", "value_id3"}}));
   ASSERT_EQ(ice::identity_match::INCLUDED, id1->checkMatching(idTest));

   idTest(new ice::Identity({{"id1", "value_id1"}}));
   ASSERT_EQ(ice::identity_match::INCLUDING, id1->checkMatching(idTest));

   idTest(new ice::Identity({{"id1", "value_id1_1"}}));
   ASSERT_EQ(ice::identity_match::NO_MATCH, id1->checkMatching(idTest));

   idTest(new ice::Identity({{"id1", "value_id1_1"}, {"id2", "value_id2"}}));
   ASSERT_EQ(ice::identity_match::CONFLICTING, id1->checkMatching(idTest));
}
