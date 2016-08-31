/*
 * InformationMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include "ice/communication/messages/InformationMessage.h"

#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/representation/GContainer.h"

namespace ice
{

InformationMessage::InformationMessage() : Message(IceMessageIds::IMI_INFORMATION_RESPONSE, true)
{
  _log = el::Loggers::getLogger("InformationMessage");

}

InformationMessage::~InformationMessage()
{

}

std::vector<std::pair<int,std::shared_ptr<InformationElement<GContainer>>>>& InformationMessage::getInformations()
{
  return this->informations;
}

void InformationMessage::payloadToJson(rapidjson::Document &document)
{
  document.SetArray();

  for (auto &infoElement : this->informations)
  {
    rapidjson::Value element, spec, info, infoValue;
    rapidjson::Value indexName("index", document.GetAllocator());
    rapidjson::Value specName("spec", document.GetAllocator());
    rapidjson::Value infoName("info", document.GetAllocator());

    element.SetObject();

    rapidjson::Value index;
    index.SetInt(infoElement.first);
    element.AddMember(indexName, index, document.GetAllocator());

    auto &ie = infoElement.second;
    // spec
    spec.SetArray();
    auto sp = ie->getSpecification();
    rapidjson::Value e(sp->getEntity().c_str(), document.GetAllocator());
    rapidjson::Value et(sp->getEntityType().c_str(), document.GetAllocator());
    rapidjson::Value s(sp->getScope().c_str(), document.GetAllocator());
    rapidjson::Value r(sp->getRepresentation().c_str(), document.GetAllocator());
    rapidjson::Value re(sp->getRelatedEntity().c_str(), document.GetAllocator());

    spec.PushBack(e, document.GetAllocator());
    spec.PushBack(et, document.GetAllocator());
    spec.PushBack(s, document.GetAllocator());
    spec.PushBack(r, document.GetAllocator());
    spec.PushBack(re, document.GetAllocator());

    element.AddMember(specName, spec, document.GetAllocator());

    // Information
    infoValue = ie->getInformation()->toJSONValue(document);

    rapidjson::Value rep(ie->getInformation()->representation->name.c_str(), document.GetAllocator());
    info.SetObject();
    info.AddMember(rep, infoValue, document.GetAllocator());

    element.AddMember(infoName, info, document.GetAllocator());

    document.PushBack(element, document.GetAllocator());
  }
}

bool InformationMessage::parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory)
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
    if (false == it->IsObject())
    {
      _log->error("Payload could not be parsed: InformationElement is not an object");
      return false;
    }

    auto index = it->FindMember("index");
    auto spec = it->FindMember("spec");
    auto info = it->FindMember("info");

    if (index == it->MemberEnd())
    {
      _log->error("Payload could not be parsed: Index not found");
      return false;
    }

    if (spec == it->MemberEnd())
    {
      _log->error("Payload could not be parsed: Specification not found");
      return false;
    }

    if (false == spec->value.IsArray())
    {
      _log->error("Payload could not be parsed: Specification is not an array");
      return false;
    }


    if (false == index->value.IsInt())
    {
      _log->error("Payload could not be parsed: Index is not an int");
      return false;
    }

    int indexValue = index->value.GetInt();

    count = 0;

    for (auto it2 = spec->value.Begin(); it2 != spec->value.End(); ++it2)
    {

      if (false == it2->IsString())
      {
        _log->error("Payload could not be parsed: Field is not a string");
        return false;
      }

      switch (count)
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

    auto specification = std::make_shared<InformationSpecification>(entity, entityType, scope, rep, relatedEntity);

    if (info == it->MemberEnd())
    {
      _log->error("Payload could not be parsed: Specification not");
      return false;
    }

    auto information = factory->fromJSON(info->value);

    if (information == nullptr)
    {
      _log->error("Payload could not be parsed: Error while extracting information");
      return false;
    }

    auto informationElement = std::make_shared<InformationElement<GContainer>>(specification, information);
    auto pair = std::make_pair(indexValue, informationElement);
    this->informations.push_back(pair);
  }

  return true;
}

} /* namespace ice */
