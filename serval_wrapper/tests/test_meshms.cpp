#include <memory>

#include "gtest/gtest.h"
#include "serval_interface.h"

static ice::serval_interface si("localhost", 4110, "peter", "venkman");
static std::unique_ptr<ice::serval_identity> sid1 = si.keyring.addIdentity();
static std::unique_ptr<ice::serval_identity> sid2 = si.keyring.addIdentity();

TEST(meshms, get_conversation_lists)
{
  auto list = si.meshms.getConversationList(sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->size(), 0);

  list = si.meshms.getConversationList("sdf");
  ASSERT_TRUE(list == nullptr);
}

TEST(meshms, get_message_list)
{
  auto list = si.meshms.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si.meshms.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si.meshms.getMessageList(sid2->sid, "");
  ASSERT_TRUE(list == nullptr);
}

TEST(meshms, post_message)
{
  auto result = si.meshms.postMessage(sid1->sid, sid2->sid, "Funky test message");
  ASSERT_TRUE(result);

  auto list = si.meshms.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 1);

  auto token = list->messages[0].token;
  result = si.meshms.postMessage(sid2->sid, sid1->sid, "Hammer Time!");
  ASSERT_TRUE(result);

  list = si.meshms.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 2);

  list = si.meshms.getMessageList(sid1->sid, sid2->sid, token);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 3);
}
