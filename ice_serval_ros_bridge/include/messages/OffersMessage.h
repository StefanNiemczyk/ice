/*
 * OffersMessage.h
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#ifndef OFFERSMESSAGE_H_
#define OFFERSMESSAGE_H_

#include <vector>

#include "messages/Message.h"

namespace ice
{
class InformationSpecification;

class OffersMessage : public Message
{
public:
  OffersMessage();
  virtual ~OffersMessage();

  std::vector<InformationSpecification>& getOfferes();

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value, IceServalBridge* bridge);

private:
  std::vector<InformationSpecification> offeres;
};

} /* namespace ice */

#endif /* OFFERSMESSAGE_H_ */
