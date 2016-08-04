/*
 * OntologyIdMessage.h
 *
 *  Created on: 30.07.2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_COMMUNICATION_MESSAGES_ONTOLOGYIDMESSAGE_H_
#define INCLUDE_ICE_COMMUNICATION_MESSAGES_ONTOLOGYIDMESSAGE_H_

#include "ice/communication/messages/Message.h"

#include <vector>

namespace ice
{

class OntologyIdMessage : public Message
{
public:
  OntologyIdMessage();
  virtual ~OntologyIdMessage();

  std::vector<std::pair<std::string, std::string>>& getIds();

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value, std::shared_ptr<GContainerFactory> factory);

private:
  std::vector<std::pair<std::string, std::string>> ids;
};

}
#endif /* INCLUDE_ICE_COMMUNICATION_MESSAGES_ONTOLOGYIDMESSAGE_H_ */
