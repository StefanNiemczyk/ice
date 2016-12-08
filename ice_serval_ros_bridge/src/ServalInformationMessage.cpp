/*
 * ServalRawMessage.cpp
 *
 *  Created on: Dec 7, 2016
 *      Author: sni
 */

#include <ServalInformationMessage.h>

namespace ice
{

ServalInformationMessage::ServalInformationMessage(std::shared_ptr<GContainer> container, ont::entity ontEntity,
                                                   uint32_t collectionHash, time timeValidity, time timeObservation,
                                                   time timeProcessed) : Message(100, true), container(container),
                                                       ontEntity(ontEntity), collectionHash(collectionHash),
                                                       timeValidity(timeValidity), timeObservation(timeObservation),
                                                       timeProcessed(timeProcessed)
{
  _log = el::Loggers::getLogger("ServalInformationMessage");
}

ServalInformationMessage::~ServalInformationMessage()
{
  // nothing to do here
}


std::shared_ptr<GContainer> ServalInformationMessage::getContainer()
{
  return this->container;
}

ont::entity ServalInformationMessage::getOntEntity()
{
  return this->ontEntity;
}

uint32_t ServalInformationMessage::getCollectionHash()
{
  return this->collectionHash;
}

time ServalInformationMessage::getTimeValidity()
{
  return timeValidity;
}

time ServalInformationMessage::getTimeObservation()
{
  return timeObservation;
}

time ServalInformationMessage::getTimeProcessed()
{
  return timeProcessed;
}

void ServalInformationMessage::payloadToJson(rapidjson::Document &document)
{

}

bool ServalInformationMessage::parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory)
{
  return true;
}

} /* namespace ice */
