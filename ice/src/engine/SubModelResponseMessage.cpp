/*
 * SubModelResponseMessage.cpp
 *
 *  Created on: 15.08.2016
 *      Author: sni
 */

#include "ice/communication/messages/SubModelResponseMessage.h"

namespace ice
{

SubModelResponseMessage::SubModelResponseMessage(int index, bool result) :
    Message(IceMessageIds::IMI_SUBMODEL_RESPONSE, true), index(index), result(result)
{
  _log = el::Loggers::getLogger("SubModelResponseMessage");
}

SubModelResponseMessage::~SubModelResponseMessage()
{

}

int SubModelResponseMessage::getIndex()
{
  return this->index;
}

void SubModelResponseMessage::setIndex(int &index)
{
  this->index = index;
}

bool SubModelResponseMessage::getResult()
{
  return this->result;
}

void SubModelResponseMessage::setResult(bool &result)
{
  this->result = result;
}

rapidjson::Value SubModelResponseMessage::payloadToJson(rapidjson::Document &document)
{
  rapidjson::Value value, index, result;
  value.SetArray();
  index.SetInt(this->index);
  result.SetBool(this->result);

  value.PushBack(index, document.GetAllocator());
  value.PushBack(result, document.GetAllocator());

  return value;
}

bool SubModelResponseMessage::parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory)
{
  if (false == value.IsArray())
  {
    return false;
  }

  if (value.Size() != 2)
  {
    return false;
  }

  auto index = value.Begin();
  auto result = value.Begin() + 1;

  if (false == index->IsInt())
  {
    return false;
  }

  if (false == result->IsBool())
  {
    return false;
  }

  this->index = index->GetInt();
  this->result = result->GetBool();

  return true;
}

} /* namespace ice */
