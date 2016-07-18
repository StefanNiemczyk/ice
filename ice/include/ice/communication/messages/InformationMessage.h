/*
 * InformationMessage.h
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#ifndef INFORMATIONMESSAGE_H_
#define INFORMATIONMESSAGE_H_

#include "ice/communication/messages/Message.h"

#include <vector>

#include "ice/information/InformationStore.h"

namespace ice
{
class GContainer;
template<typename T>
class InformationElement;

class InformationMessage : public Message
{
public:
  InformationMessage();
  virtual ~InformationMessage();

  std::vector<std::shared_ptr<InformationElement<GContainer>>>& getInformations();

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Value& value,  std::shared_ptr<GContainerFactory> factory);

private:
  std::vector<std::shared_ptr<InformationElement<GContainer>>> informations;
};

} /* namespace ice */

#endif /* INFORMATIONMESSAGE_H_ */
