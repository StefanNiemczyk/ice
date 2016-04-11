#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "serval_interface.h"

TEST(keyring, get_identities)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto ids = si.keyring.getIdentities();
  ASSERT_TRUE(ids != nullptr);
}

TEST(keyring, add_identity)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto id = si.keyring.addIdentity();
  ASSERT_TRUE(id != nullptr);

  auto ids = si.keyring.getIdentities();
  ASSERT_TRUE(ids != nullptr);

  ASSERT_GT(ids->size(), 0);

  // add name
}
