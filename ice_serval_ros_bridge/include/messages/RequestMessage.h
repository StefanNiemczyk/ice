/*
 * RequestMessage.h
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#ifndef REQUESTMESSAGE_H_
#define REQUESTMESSAGE_H_

#include "messages/Message.h"

namespace ice
{
class InformationSpecification;

class RequestMessage : public Message
{
public:
  RequestMessage();
  virtual ~RequestMessage();

  std::vector<std::shared_ptr<InformationSpecification>>& getRequests();

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value);

private:
  std::vector<std::shared_ptr<InformationSpecification>> requests;
};

} /* namespace ice */

#endif /* REQUESTMESSAGE_H_ */
