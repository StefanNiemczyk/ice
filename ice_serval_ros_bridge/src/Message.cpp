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
#include "messages/OffersMessage.h"
#include "messages/RequestMessage.h"
#include "Entity.h"

namespace ice
{
std::shared_ptr<Message> Message::parse(std::string &jsonString)
{
  std::cout << jsonString << std::endl;
  rapidjson::Document document;

  document.Parse(jsonString.c_str());

  auto id = document.FindMember("id");
  auto payload = document.FindMember("pl");

  if (false == id->value.IsInt())
  {

    return nullptr;
  }

  int command = id->value.GetInt();
  // check if command message
  if (command % 2 == 0)
  {
    return std::make_shared<CommandMessage>(command);
  }

  std::shared_ptr<Message> message;

  switch(command)
  {
    case(SCMD_IDS_RESPONSE):
        message = std::make_shared<IdMessage>();
        break;
    case(SCMD_OFFERS_RESPONSE):
        message = std::make_shared<OffersMessage>();
        break;
    case(SCMD_INFORMATION_REQUEST):
        message = std::make_shared<RequestMessage>();
        break;

    default:
      // TODO
      break;
  }

  if (message == nullptr)
  {
    // TODO
    return nullptr;
  }

  if (false == message->parsePayload(payload->value))
  {
    // TODO
    return nullptr;
  }

  return message;
}

Message::Message(int id, bool payload) :
    id(id), payload(payload)
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
