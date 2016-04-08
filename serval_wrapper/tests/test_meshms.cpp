#include <memory>

#include "gtest/gtest.h"
#include "serval_interface.h"

static ice::serval_interface si("localhost", 4110, "peter", "venkman");
static std::unique_ptr<ice::serval_identity> sid1 = si.addIdentity();
static std::unique_ptr<ice::serval_identity> sid2 = si.addIdentity();

TEST(JNITest, get_conversation_lists)
{
  auto list = si.getConversationList(sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->size(), 0);

  list = si.getConversationList("sdf");
  ASSERT_TRUE(list == nullptr);
}

TEST(JNITest, get_message_list)
{
  auto list = si.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 0);

  list = si.getMessageList(sid2->sid, "");
  ASSERT_TRUE(list == nullptr);
}

TEST(JNITest, post_message)
{
  auto result = si.postMessage(sid1->sid, sid2->sid, "Funky test message");
  ASSERT_TRUE(result);

  auto list = si.getMessageList(sid1->sid, sid2->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 1);

  std::cout << list->toString() << std::endl;

  for (auto m : list->messages)
  {
    std::cout << m.toString() << std::endl;
  }
  auto token = list->messages[0].token;

  result = si.postMessage(sid2->sid, sid1->sid, "Hammer Time!");
  ASSERT_TRUE(result);

  list = si.getMessageList(sid2->sid, sid1->sid);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 2);

  std::cout << list->toString() << std::endl;

  for (auto m : list->messages)
  {
    std::cout << m.toString() << std::endl;
  }

  list = si.getMessageList(sid1->sid, sid2->sid, token);
  ASSERT_TRUE(list != nullptr);
  ASSERT_EQ(list->messages.size(), 3);

  std::cout << list->toString() << std::endl;

  for (auto m : list->messages)
  {
    std::cout << m.toString() << std::endl;
  }
}
