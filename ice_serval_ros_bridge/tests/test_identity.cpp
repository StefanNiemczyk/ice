#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "Identity.h"

TEST(Identity, checkMatching)
{
   std::shared_ptr<ice::Identity> id1(new ice::Identity({{"id1", "value_id1"}, {"id2", "value_id2"}}));
   std::shared_ptr<ice::Identity> id2(new ice::Identity({{"id1", "value_id1"}, {"id2", "value_id2"}}));

   ASSERT_EQ(id1->checkMatching(id2), ice::identity_match::FULL_MATCH);
}
