/*
 * ServalRawMessage.h
 *
 *  Created on: Dec 7, 2016
 *      Author: sni
 */

#ifndef SRC_SERVALRAWMESSAGE_H_
#define SRC_SERVALRAWMESSAGE_H_

#include <memory>

#include <ice/TypeDefs.h>
#include <ice/Time.h>
#include <ice/communication/messages/Message.h>

namespace ice
{
class GContainer;

class ServalInformationMessage : public Message
{
public:
  ServalInformationMessage(std::shared_ptr<GContainer> container,
                           ont::entity ontEntity, uint32_t collectionHash,
                           time timeValidity, time timeObservation, time timeProcessed);
  virtual ~ServalInformationMessage();

  std::shared_ptr<GContainer> getContainer();
  ont::entity getOntEntity();
  uint32_t getCollectionHash();

  time getTimeValidity();
  time getTimeObservation();
  time getTimeProcessed();

  virtual void payloadToJson(rapidjson::Document &document);
  virtual bool parsePayload(rapidjson::Document& value, std::shared_ptr<GContainerFactory> factory);

private:
  std::shared_ptr<GContainer>           container;
  ont::entity                           ontEntity;
  uint32_t                              collectionHash;
  time                                  timeValidity;           /**< validity time of the information */
  time                                  timeObservation;        /**< observation time of the information */
  time                                  timeProcessed;          /**< time of the processing of the information */
};

} /* namespace ice */

#endif /* SRC_SERVALRAWMESSAGE_H_ */
