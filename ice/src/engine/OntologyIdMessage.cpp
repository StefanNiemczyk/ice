/*
 * OntologyIdMessage.cpp
 *
 *  Created on: 30.07.2016
 *      Author: sni
 */

#include "ice/communication/messages/OntologyIdMessage.h"

namespace ice
{

OntologyIdMessage::OntologyIdMessage() : Message(IceMessageIds::IMI_ONTOLOGY_IDS_RESPONSE, true)
{
  _log = el::Loggers::getLogger("OntologyIdMessage");
}

OntologyIdMessage::~OntologyIdMessage()
{
  //
}


std::vector<std::pair<std::string, std::string>>& OntologyIdMessage::getIds()
{
  return this->ids;
}

void OntologyIdMessage::payloadToJson(rapidjson::Document &document)
{
  document.SetArray();

  for (auto &p : this->ids)
  {
    rapidjson::Value pair, id, version;

    pair.SetArray();

    id.SetString(p.first.c_str(), document.GetAllocator());
    version.SetString(p.second.c_str(), document.GetAllocator());

    pair.PushBack(id, document.GetAllocator());
    pair.PushBack(version, document.GetAllocator());

    document.PushBack(pair, document.GetAllocator());
  }
}

bool OntologyIdMessage::parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory)
{
  if (false == value.IsArray())
  {
    _log->error("Payload could not be parsed: Is not an array");
    return false;
  }

  for (auto it = value.Begin(); it != value.End(); ++it)
  {
    if (false == it->IsArray())
    {
      _log->error("Payload could not be parsed: Id is not an array");
      return false;
    }

    if (it->Size() != 2)
    {
      _log->error("Payload could not be parsed: Id has wrong size '%v'", it->Size());
      return false;
    }

    auto id = it->Begin();
    auto version = it->Begin()+1;


    if (false == id->IsString())
    {
      _log->error("Payload could not be parsed: Id is not a string");
      return false;
    }

    if (false == version->IsString())
    {
      _log->error("Payload could not be parsed: Version is not a string");
      return false;
    }

    this->ids.push_back(std::make_pair(id->GetString(), version->GetString()));
  }

  return true;
}

} /* namespace ice */
