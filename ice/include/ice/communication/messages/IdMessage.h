/*
 * IdMessage.h
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#ifndef IDMESSAGE_H_
#define IDMESSAGE_H_

#include "ice/communication/messages/Message.h"

#include <vector>

namespace ice
{

class IdMessage : public Message
{
public:
  IdMessage();
  virtual ~IdMessage();

  std::vector<std::tuple<std::string, std::string>>& getIds();

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory);

private:
  std::vector<std::tuple<std::string, std::string>> ids;
};

} /* namespace ice */

#endif /* IDMESSAGE_H_ */
