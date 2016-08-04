/*
 * SubModelMessage.cpp
 *
 *  Created on: 04.08.2016
 *      Author: sni
 */

#include "ice/communication/messages/SubModelMessage.h"

#include <boost/serialization/string.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace ice
{

SubModelMessage::SubModelMessage() : Message(IceMessageIds::IMI_SUBMODEL, true)
{
  _log = el::Loggers::getLogger("SubModelMessage");
}

SubModelMessage::~SubModelMessage()
{
  //
}

SubModelDesc& SubModelMessage::getSubModel()
{
  return this->subModel;
}

void SubModelMessage::setSubModel(SubModelDesc &desc)
{
  this->subModel = desc;
}

rapidjson::Value SubModelMessage::payloadToJson(rapidjson::Document &document)
{
  std::stringstream ss;
  boost::archive::text_oarchive ar(ss);
  ar << this->subModel;

  rapidjson::Value value;
  value.SetString(ss.str().c_str(), document.GetAllocator());

  return value;
}

bool SubModelMessage::parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory)
{
  if (false == value.IsString())
  {
    _log->error("Payload could not be parsed: Is not a string");
    return false;
  }

  boost::archive::text_iarchive ar(value.GetString());
  ar >> this->subModel;

  return true;
}

} /* namespace ice */
