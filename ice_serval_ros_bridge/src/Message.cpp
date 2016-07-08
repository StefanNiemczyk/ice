/*
 * Message.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include <messages/Message.h>

#include <string.h>

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "messages/CommandMessage.h"
#include "messages/IdMessage.h"
#include "messages/InformationMessage.h"
#include "messages/OffersMessage.h"
#include "messages/RequestMessage.h"
#include "Entity.h"

namespace ice
{

el::Logger* Message::_logFactory = el::Loggers::getLogger("Message");

std::shared_ptr<Message> Message::parse(std::string &jsonString, IceServalBridge* bridge)
{
  std::cout << jsonString << std::endl;
  rapidjson::Document document;

  document.Parse(jsonString.c_str());

  if (document.GetType() == 0)
  {
    _logFactory->error("Message could not be parsed, Json is broken '%v'", jsonString.c_str());
    return nullptr;
  }

  auto id = document.FindMember("id");
  auto payload = document.FindMember("pl");

  if (false == id->value.IsInt())
  {
    _logFactory->error("Message could not be parsed, ID is missig in JSON '%v'", jsonString.c_str());
    return nullptr;
  }

  int command = id->value.GetInt();
  std::shared_ptr<Message> message;

  switch(command)
  {
    case(SCMD_IDS_REQUEST):
    case(SCMD_ID_REQUEST):
    case(SCMD_OFFERS_REQUEST):
        return std::make_shared<CommandMessage>(command);
        break;
    case(SCMD_IDS_RESPONSE):
        message = std::make_shared<IdMessage>();
        break;
    case(SCMD_OFFERS_RESPONSE):
        message = std::make_shared<OffersMessage>();
        break;
    case(SCMD_INFORMATION_REQUEST):
        message = std::make_shared<RequestMessage>();
        break;
    case(SCMD_INFORMATION_RESPONSE):
        message = std::make_shared<InformationMessage>();
        break;

    default:
      _logFactory->error("Message could not be parsed, unknown ID '%v'", command);
      return nullptr;
      break;
  }

  if (false == message->parsePayload(payload->value, bridge))
  {
    _logFactory->error("Message could not be parsed, Error while parsing payload for Message ID '%v'", command);
    return nullptr;
  }

  return message;
}

Message::Message(int id, bool payload) : id(id), payload(payload), _log(nullptr)
{

}

Message::~Message()
{

}

std::string Message::toJson()
{
  rapidjson::Document document;
  rapidjson::Value name("id");
  rapidjson::Value value(this->id);

  // set id
  document.SetObject();
  document.AddMember(name, value, document.GetAllocator());

  // add payload
  if (this->payload)
  {
    rapidjson::Value name("pl");
    rapidjson::Value value = this->payloadToJson(document);
    document.AddMember(name, value, document.GetAllocator());
  }

  // to string
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

  document.Accept(writer);

  const char* output = buffer.GetString();
  return std::string(output);
}

int Message::getId()
{
  return this->id;
}

std::shared_ptr<Entity> Message::getEntity()
{
  return this->entity;
}

void Message::setEntity(std::shared_ptr<Entity> entity)
{
  this->entity = entity;
}

} /* namespace ice */
