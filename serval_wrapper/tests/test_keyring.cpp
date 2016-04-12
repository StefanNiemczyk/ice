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

}

TEST(keyring, set_identity)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto id = si.keyring.addIdentity();
  ASSERT_TRUE(id != nullptr);

  auto ids = si.keyring.getIdentities();
  ASSERT_TRUE(ids != nullptr);

  ASSERT_GT(ids->size(), 0);

  // add name
  auto updatedId = si.keyring.setIdentity(id->sid, "testname", "098766725");
  ASSERT_NE(updatedId, nullptr);
  ASSERT_EQ(updatedId->sid, id->sid);
  ASSERT_EQ(updatedId->did, "098766725");
  ASSERT_EQ(updatedId->name, "testname");
}

TEST(keyring, get_identities_cmd)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto list = si.keyring.getSelf();
  ASSERT_NE(list, nullptr);
  ASSERT_GT(list->size(), 0);

//  for (auto i : *list)
//    std::cout << i.toString() << std::endl;
}
