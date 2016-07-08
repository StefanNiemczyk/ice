/*
 * OffersMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include <messages/OffersMessage.h>

#include <ice/information/InformationSpecification.h>

namespace ice
{

OffersMessage::OffersMessage() : Message(IceCmd::SCMD_OFFERS_RESPONSE, true)
{
  _log = el::Loggers::getLogger("OffersMessage");
}

OffersMessage::~OffersMessage()
{

}

std::vector<InformationSpecification>& OffersMessage::getOfferes()
{
  return this->offeres;
}

rapidjson::Value OffersMessage::payloadToJson(rapidjson::Document &document)
{
  rapidjson::Value value;
  value.SetArray();

  for (auto &offer : this->offeres)
  {
    rapidjson::Value n, v;

    n.SetArray();
    rapidjson::Value e(offer.getEntity().c_str(), document.GetAllocator());
    rapidjson::Value et(offer.getEntityType().c_str(), document.GetAllocator());
    rapidjson::Value s(offer.getScope().c_str(), document.GetAllocator());
    rapidjson::Value r(offer.getRepresentation().c_str(), document.GetAllocator());
    rapidjson::Value re(offer.getRelatedEntity().c_str(), document.GetAllocator());

    n.PushBack(e, document.GetAllocator());
    n.PushBack(et, document.GetAllocator());
    n.PushBack(s, document.GetAllocator());
    n.PushBack(r, document.GetAllocator());
    n.PushBack(re, document.GetAllocator());

    value.PushBack(n, document.GetAllocator());
  }

  return value;
}

bool OffersMessage::parsePayload(rapidjson::Value& value, IceServalBridge* bridge)
{
  if (false == value.IsArray())
  {
    _log->error("Payload could not be parsed: Is not an array");
    return false;
  }

  std::string entity, entityType, scope, rep, relatedEntity;
  int count = 0;

  for (auto it = value.Begin(); it != value.End(); ++it)
  {
    if (false == it->IsArray())
    {
      _log->error("Payload could not be parsed: Id is not an array");
      return false;
    }

    count = 0;

    for (auto it2 = it->Begin(); it2 != it->End(); ++it2)
    {
      if (false == it2->IsString())
      {
        _log->error("Payload could not be parsed: Field is not a string");
        return false;
      }

      switch(count)
      {
        case 0:
          entity = it2->GetString();
          break;
        case 1:
          entityType = it2->GetString();
          break;
        case 2:
          scope = it2->GetString();
          break;
        case 3:
          rep = it2->GetString();
          break;
        case 4:
          relatedEntity = it2->GetString();
          break;
      }

      ++count;
    }

    if (count != 5)
    {
      _log->error("Payload could not be parsed: Wrong number of fields for offer");
      return false;
    }

    InformationSpecification spec(entity, entityType, scope, rep, relatedEntity);
    this->offeres.push_back(spec);
  }

  return true;
}

} /* namespace ice */
