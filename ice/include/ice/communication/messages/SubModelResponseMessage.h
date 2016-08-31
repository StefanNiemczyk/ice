/*
 * SubModelResponseMessage.h
 *
 *  Created on: 15.08.2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELRESPONSEMESSAGE_H_
#define INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELRESPONSEMESSAGE_H_

#include "ice/communication/messages/Message.h"

namespace ice
{

class SubModelResponseMessage : public Message
{
public:
  SubModelResponseMessage(int index = -1, bool result = false);
  virtual ~SubModelResponseMessage();

  int getIndex();
  void setIndex(int &index);
  bool getResult();
  void setResult(bool &value);

protected:
  virtual void payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory);

private:
  int index;
  bool result;
};

} /* namespace ice */

#endif /* INCLUDE_ICE_COMMUNICATION_MESSAGES_SUBMODELRESPONSEMESSAGE_H_ */
