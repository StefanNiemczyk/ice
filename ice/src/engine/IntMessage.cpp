/*
 * IntMessage.cpp
 *
 *  Created on: Jul 23, 2016
 *      Author: sni
 */

#include <ice/communication/messages/IntMessage.h>

namespace ice
{

IntMessage::IntMessage(int id): Message(id, true), value(0)
{
  _log = el::Loggers::getLogger("IntMessage");
}

IntMessage::~IntMessage()
{

}

int IntMessage::getValue()
{
  return this->value;
}

void IntMessage::setValue(int &value)
{
  this->value = value;
}

rapidjson::Value IntMessage::payloadToJson(rapidjson::Document &document)
{
  rapidjson::Value value;
  value.SetInt(this->value);
  return value;
}

bool IntMessage::parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory)
{
  if (false == value.IsInt())
  {
    return false;
  }

  this->value = value.GetInt();

  return true;
}

} /* namespace ice */
