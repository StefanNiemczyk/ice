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

#include "ice/communication/messages/CommandMessage.h"
#include "ice/communication/messages/IdMessage.h"
#include "ice/communication/messages/InformationMessage.h"
#include "ice/communication/messages/IntMessage.h"
#include "ice/communication/messages/OffersMessage.h"
#include "ice/communication/messages/OntologyIdMessage.h"
#include "ice/communication/messages/RequestMessage.h"
#include "ice/communication/messages/SubModelMessage.h"
#include "ice/communication/messages/SubModelResponseMessage.h"
#include "ice/Entity.h"

INITIALIZE_EASYLOGGINGPP

namespace ice
{

el::Logger* Message::_logFactory = el::Loggers::getLogger("Message");

std::shared_ptr<Message> Message::parse(uint8_t id, std::string &jsonString, std::shared_ptr<Entity> &entity,
                                        std::shared_ptr<GContainerFactory> factory)
{

  std::shared_ptr<Message> message;

  switch(id)
  {
    case(IMI_IDS_REQUEST):
    case(IMI_ID_REQUEST):
    case(IMI_ONTOLOGY_IDS_REQUEST):
    case(IMI_OFFERS_REQUEST):
    case(IMI_FINISH):
    case(IMI_CANCLE_JOB):
        message =  std::make_shared<CommandMessage>(id);
        break;
    case(IMI_IDS_RESPONSE):
        message = std::make_shared<IdMessage>();
        break;
    case(IMI_ONTOLOGY_IDS_RESPONSE):
        message = std::make_shared<OntologyIdMessage>();
        break;
    case(IMI_OFFERS_RESPONSE):
        message = std::make_shared<OffersMessage>();
        break;
    case(IMI_INFORMATION_REQUEST):
        message = std::make_shared<RequestMessage>();
        break;
    case(IMI_INFORMATION_RESPONSE):
        message = std::make_shared<InformationMessage>();
        break;
    case(IMI_ACK):
    case(IMI_INFORMATION_REQUEST_INDEX):
        message = std::make_shared<IntMessage>(id);
        break;
    case(IMI_SUBMODEL):
        message = std::make_shared<SubModelMessage>();
        break;
    case(IMI_SUBMODEL_RESPONSE):
        message = std::make_shared<SubModelResponseMessage>();
        break;

    default:
      _logFactory->error("Message could not be parsed, unknown ID '%v'", id);
      return nullptr;
      break;
  }

  message->setEntity(entity);

  if (message->isPayload())
  {
    rapidjson::Document document;

    document.Parse(jsonString.c_str());

    if (document.GetType() == 0)
    {
      _logFactory->error("Message '%v' could not be parsed, Json is broken '%v'", std::to_string(id), jsonString.c_str());
      return nullptr;
    }

    if (false == message->parsePayload(document, factory))
    {
      _logFactory->error("Message could not be parsed, Error while parsing payload for Message ID '%v'", std::to_string(id));
      return nullptr;
    }
  }

  return message;
}

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
