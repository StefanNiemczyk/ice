/*
 * CommandMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include "ice/communication/messages/CommandMessage.h"

namespace ice
{

CommandMessage::CommandMessage(int id) : Message(id, false)
{
  //

}

CommandMessage::~CommandMessage()
{
  //
}

rapidjson::Value CommandMessage::payloadToJson(rapidjson::Document &document)
{
  return rapidjson::Value("");
}

bool CommandMessage::parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory)
{
  return true;
}
} /* namespace ice */
