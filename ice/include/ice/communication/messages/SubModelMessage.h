/*
 * SubModelMessage.h
 *
 *  Created on: 04.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELMESSAGE_H_
#define INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELMESSAGE_H_

#include "ice/communication/messages/Message.h"

namespace ice
{

class SubModelDesc;

class SubModelMessage : public Message
{
public:
  SubModelMessage();
  virtual ~SubModelMessage();

  std::shared_ptr<SubModelDesc>& getSubModel();
  void setSubModel(std::shared_ptr<SubModelDesc> &desc);

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory);

private:
  std::shared_ptr<SubModelDesc> subModel;
};

} /* namespace ice */

#endif /* INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELMESSAGE_H_ */
