#include <memory>

#include "gtest/gtest.h"
#include "serval_interface.h"

static ice::serval_interface si1("/tmp/instance1", "localhost", 4110, "peter", "venkman");
static ice::serval_interface si2("/tmp/instance2", "localhost", 4110, "peter", "venkman");
static std::unique_ptr<ice::serval_identity> sid1 = si1.keyring.addIdentity();
static std::unique_ptr<ice::serval_identity> sid2 = si2.keyring.addIdentity();

TEST(meshms, get_conversation_lists)
{
  auto list = si1.meshms.getConversationList(sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->size(), 0);

  list = si1.meshms.getConversationList("sdf");
  ASSERT_TRUE(list == nullptr);
}

TEST(meshms, get_message_list)
{
  auto list = si1.meshms.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si1.meshms.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si1.meshms.getMessageList(sid2->sid, "");
  ASSERT_TRUE(list == nullptr);
}

TEST(meshms, post_message)
{
  auto result = si1.meshms.postMessage(sid1->sid, sid2->sid, "Funky test message");
  ASSERT_TRUE(result);

  auto list = si1.meshms.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 1);

  auto token = list->messages[0].token;
  result = si2.meshms.postMessage(sid2->sid, sid1->sid, "Hammer Time!");
  ASSERT_TRUE(result);

  list = si2.meshms.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 2);

  list = si1.meshms.getMessageList(sid1->sid, sid2->sid, token);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 2);
}

TEST(meshms, mark_read)
{
  auto list = si1.meshms.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 3);

  for (auto &msg : list->messages)
  {
    ASSERT_FALSE(msg.read);
  }

  si1.meshms.markMessagesAsRead(sid2->sid);

  list = si1.meshms.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);

  for (auto &msg : list->messages)
  {
    if (msg.type != "<")
      continue;

    ASSERT_FALSE(msg.read);
  }
}
