/*
 * IntMessage.h
 *
 *  Created on: Jul 23, 2016
 *      Author: sni
 */

#ifndef INTMESSAGE_H_
#define INTMESSAGE_H_

#include "ice/communication/messages/Message.h"

namespace ice
{

class IntMessage : public Message
{
public:
  IntMessage(int id);
  virtual ~IntMessage();

  int getValue();
  void setValue(int &value);

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory);

private:
  int value;
};

} /* namespace ice */

#endif /* INTMESSAGE_H_ */
