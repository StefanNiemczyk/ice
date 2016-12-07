/*
 * Message.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include "ice/communication/messages/Message.h"

#include <string.h>

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "ice/Entity.h"

INITIALIZE_EASYLOGGINGPP

namespace ice
{

el::Logger* Message::_logFactory = el::Loggers::getLogger("Message");

Message::Message(int id, bool payload) : id(id), payload(payload), _log(nullptr), jobId(0), jobIndex(0)
{

}

Message::~Message()
{

}

std::string Message::toJson()
{

  // add payload
  if (this->payload == false)
  {
    return "";
  }

  rapidjson::Document document;
  this->payloadToJson(document);
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

uint8_t Message::getJobId()
{
  return jobId;
}

void Message::setJobId(uint8_t jobId)
{
  this->jobId = jobId;
}

uint8_t Message::getJobIndex()
{
  return jobIndex;
}

void Message::setJobIndex(uint8_t jobIndex)
{
  this->jobIndex = jobIndex;
}

std::shared_ptr<Entity> Message::getEntity()
{
  return this->entity;
}

void Message::setEntity(std::shared_ptr<Entity> entity)
{
  this->entity = entity;
}

bool Message::isPayload()
{
  return this->payload;
}


} /* namespace ice */
