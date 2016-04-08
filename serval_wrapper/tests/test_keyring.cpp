#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "serval_interface.h"

TEST(JNITest, get_identities)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto ids = si.getIdentities();

  ASSERT_TRUE(ids != nullptr);

  for (auto &id : * ids)
  {
    std::cout << id.toString() << std::endl;
  }
}

TEST(JNITest, add_identity)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto id = si.addIdentity();

  ASSERT_TRUE(id != nullptr);
}
