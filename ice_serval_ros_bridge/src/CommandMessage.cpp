/*
 * CommandMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include <messages/CommandMessage.h>

namespace ice
{

CommandMessage::CommandMessage(int id) : Message(id, false)
{
  // TODO Auto-generated constructor stub

}

CommandMessage::~CommandMessage()
{
  // TODO Auto-generated destructor stub
}

rapidjson::Value CommandMessage::payloadToJson(rapidjson::Document &document)
{
  return rapidjson::Value("");
}

bool CommandMessage::parsePayload(rapidjson::Value& value)
{
  return true;
}
} /* namespace ice */
