#include <iostream>
#include <cstring>

#include <mdp_cpp.h>

#include "gtest/gtest.h"
#include "serval_interface.h"

#define PAYLOAD_LENGTH 16
#define PORT 8042

static ice::serval_interface si("/tmp/instance1", "localhost", 4110, "peter", "venkman");
static std::unique_ptr<ice::serval_identity> sid1 = si.keyring.addIdentity();
static std::unique_ptr<ice::serval_identity> sid2 = si.keyring.addIdentity();

TEST(mdp, send_receive)
{
  auto sock1 = si.createSocket(sid2->sid, 8045, sid1->sid);
  auto sock2 = si.createSocket(sid1->sid, 8045, sid2->sid);

  sleep(5);

  uint8_t buff[10];
  uint8_t buff2[10];
  for(int i=0; i < 10; ++i) buff[i] = i;

  sock1->send(buff, 10);
  int recCount = sock2->receive(buff2, 10);

  ASSERT_EQ(10, recCount);

  for (int i=0; i < 10; ++i)
  {
    ASSERT_EQ(buff[i], buff2[i]);
  }

  sock1->close();
  sock2->close();
}
