/*
 * Message.h
 *
 *  Created on: Jul 7, 2016
 *      Author: sni
 */

#ifndef ICEMESSAGE_H_
#define ICEMESSAGE_H_

#include <memory>

#include <easylogging++.h>
#include <rapidjson/document.h>

#include "ice/representation/GContainerFactory.h"

namespace ice
{
class Entity;
}

namespace ice
{

enum IceMessageIds
{
  IMI_IDS_REQUEST                      =  1,
  IMI_IDS_RESPONSE                     =  2,
  IMI_ID_REQUEST                       =  3,
  IMI_ID_RESPONSE                      =  4,
  IMI_ONTOLOGY_IDS_REQUEST             =  5,
  IMI_ONTOLOGY_IDS_RESPONSE            =  6,
  IMI_OFFERS_REQUEST                   =  7,
  IMI_OFFERS_RESPONSE                  =  8,
  IMI_INFORMATION_REQUEST              =  9,
  IMI_INFORMATION_RESPONSE             = 10,
  IMI_INFORMATION_REQUEST_INDEX        = 11,
  IMI_SUBMODEL                         = 12,
  IMI_SUBMODEL_RESPONSE                = 13,

  IMI_ACK                              = 250,

  IMI_FINISH                           = 253,
  IMI_NO_RESULT                        = 254,
  IMI_CANCLE_JOB                       = 255
};

class Message
{
public:
  static std::shared_ptr<Message> parse(uint8_t id, std::string &jsonString,
                                        std::shared_ptr<Entity> &entity, std::shared_ptr<GContainerFactory> factory);
private:
  static el::Logger*            _logFactory;

public:
  Message(int id, bool payload);
  virtual ~Message();

  std::string toJson();

  int getId();
  uint8_t getJobId();
  void setJobId(uint8_t jobId);
  uint8_t getJobIndex();
  void setJobIndex(uint8_t jobIndex);
  std::shared_ptr<Entity> getEntity();
  void setEntity(std::shared_ptr<Entity> entity);
  bool isPayload();

protected:
  virtual void payloadToJson(rapidjson::Document &document) = 0;
  virtual bool parsePayload(rapidjson::Document& value,  std::shared_ptr<GContainerFactory> factory) = 0;

protected:
  const int                     id;
  const bool                    payload;
  uint8_t                       jobId;
  uint8_t                       jobIndex;
  std::shared_ptr<Entity>       entity;
  el::Logger*                   _log;
};

} /* namespace ice */

#endif /* ICEMESSAGE_H_ */
