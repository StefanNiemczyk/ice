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

void CommandMessage::payloadToJson(rapidjson::Document &document)
{
  //
}

bool CommandMessage::parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory)
{
  return true;
}
} /* namespace ice */
