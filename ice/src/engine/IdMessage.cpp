/*
 * IdMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include "ice/communication/messages/IdMessage.h"

namespace ice
{

IdMessage::IdMessage() : Message(IceMessageIds::IMI_IDS_RESPONSE, true)
{
  _log = el::Loggers::getLogger("IdMessage");
}

IdMessage::~IdMessage()
{

}

std::vector<std::tuple<std::string, std::string>>& IdMessage::getIds()
{
  return this->ids;
}

void IdMessage::payloadToJson(rapidjson::Document &document)
{
  document.SetObject();

  for (auto &id : this->ids)
  {
    rapidjson::Value n, v;

    n.SetString(std::get<0>(id).c_str(), document.GetAllocator());
    v.SetString(std::get<1>(id).c_str(), document.GetAllocator());

    document.AddMember(n, v, document.GetAllocator());
  }
}

bool IdMessage::parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory)
{
  if (false == value.IsObject())
  {
    _log->error("Payload could not be parsed: Is not an object");
    return false;
  }

  for (auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
  {
    if (false == it->name.IsString() || false == it->value.IsString())
    {
      _log->error("Payload could not be parsed: Id is not a string");
      return false;
    }

    this->ids.push_back(std::make_tuple(it->name.GetString(), it->value.GetString()));
  }

  return true;
}

} /* namespace ice */
