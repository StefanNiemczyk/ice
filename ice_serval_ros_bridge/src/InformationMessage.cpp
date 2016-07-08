/*
 * InformationMessage.cpp
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#include <messages/InformationMessage.h>

#include "ice/information/InformationElement.h"
#include "ice/information/InformationSpecification.h"
#include "ice/representation/GContainer.h"
#include "IceServalBridge.h"

namespace ice
{

InformationMessage::InformationMessage() : Message(IceCmd::SCMD_INFORMATION_RESPONSE, true)
{
  _log = el::Loggers::getLogger("InformationMessage");

}

InformationMessage::~InformationMessage()
{

}

std::vector<std::shared_ptr<InformationElement<GContainer>>>& InformationMessage::getInformations()
{
  return this->informations;
}

rapidjson::Value InformationMessage::payloadToJson(rapidjson::Document &document)
{
  rapidjson::Value value;
  value.SetArray();

  for (auto &ie : this->informations)
  {
    rapidjson::Value element, spec, info, infoValue;
    rapidjson::Value specName("spec", document.GetAllocator());
    rapidjson::Value infoName("info", document.GetAllocator());

    element.SetObject();
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

    value.PushBack(element, document.GetAllocator());
  }

  return value;
}

bool InformationMessage::parsePayload(rapidjson::Value& value, IceServalBridge* bridge)
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

     auto spec = it->FindMember("spec");
     auto info = it->FindMember("info");

     if (spec == it->MemberEnd())
     {
       _log->error("Payload could not be parsed: Specification not");
       return false;
     }

     if (false == spec->value.IsArray())
     {
       _log->error("Payload could not be parsed: Specification is not an array");
       return false;
     }

     count = 0;

     for (auto it2 = spec->value.Begin(); it2 != spec->value.End(); ++it2)
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

     auto specification = std::make_shared<InformationSpecification>(entity, entityType, scope, rep, relatedEntity);



     if (info == it->MemberEnd())
     {
       _log->error("Payload could not be parsed: Specification not");
       return false;
     }

     auto information = bridge->gcontainerFactory->fromJSON(info->value);

     if (information == nullptr)
     {
       _log->error("Payload could not be parsed: Error while extracting information");
       return false;
     }

     auto informationElement = std::make_shared<InformationElement<GContainer>>(specification, information);
     this->informations.push_back(informationElement);
   }

  return true;
}

} /* namespace ice */
