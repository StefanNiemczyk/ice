/*
 * meshms.h
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#ifndef MESHMS_H_
#define MESHMS_H_

#include <memory>
#include <vector>

namespace ice
{

class serval_interface;
struct serval_conversation;
struct serval_message_list;

namespace serval_wrapper {

const std::string SERVAL_REST_GET_CONVERSATION_LIST = "/restful/meshms/$RECIPIENTSID/conversationlist.json";
const std::string SERVAL_REST_GET_MESSAGE_LIST = "/restful/meshms/$SENDERSID/$RECIPIENTSID/messagelist.json";
const std::string SERVAL_REST_GET_MESSAGE_LIST_BY_TOKEN = "/restful/meshms/$SENDERSID/$RECIPIENTSID/newsince/$TOKEN/messagelist.json";
const std::string SERVAL_REST_POST_MESSAGE = "/restful/meshms/$SENDERSID/$RECIPIENTSID/sendmessage";

class meshms
{
public:
  meshms(serval_interface *interface);
  virtual ~meshms();

  std::unique_ptr<std::vector<serval_conversation>> getConversationList(std::string recipientSid);
  std::unique_ptr<serval_message_list> getMessageList(std::string recipientSid, std::string senderSid,
                                                      std::string token = "");
  bool postMessage(std::string recipientSid, std::string senderSid, std::string msg);

private:
  serval_interface *interface;
};

}
} /* namespace ice */

#endif /* MESHMS_H_ */
