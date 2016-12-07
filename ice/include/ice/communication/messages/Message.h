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

inline const char* IceMessageIdsString(IceMessageIds v)
{
    switch (v)
    {
        case IMI_IDS_REQUEST:                   return "IMI_IDS_REQUEST";
        case IMI_IDS_RESPONSE:                  return "IMI_IDS_RESPONSE";
        case IMI_ID_REQUEST:                    return "IMI_ID_REQUEST";
        case IMI_ID_RESPONSE:                   return "IMI_ID_RESPONSE";
        case IMI_ONTOLOGY_IDS_REQUEST:          return "IMI_ONTOLOGY_IDS_REQUEST";
        case IMI_ONTOLOGY_IDS_RESPONSE:         return "IMI_ONTOLOGY_IDS_RESPONSE";
        case IMI_OFFERS_REQUEST:                return "IMI_OFFERS_REQUEST";
        case IMI_OFFERS_RESPONSE:               return "IMI_OFFERS_RESPONSE";
        case IMI_INFORMATION_REQUEST:           return "IMI_INFORMATION_REQUEST";
        case IMI_INFORMATION_RESPONSE:          return "IMI_INFORMATION_RESPONSE";
        case IMI_INFORMATION_REQUEST_INDEX:     return "IMI_INFORMATION_REQUEST_INDEX";
        case IMI_SUBMODEL:                      return "IMI_SUBMODEL";
        case IMI_SUBMODEL_RESPONSE:             return "IMI_SUBMODEL_RESPONSE";

        case IMI_ACK:                           return "IMI_ACK";

        case IMI_FINISH:                        return "IMI_FINISH";
        case IMI_NO_RESULT:                     return "IMI_NO_RESULT";
        case IMI_CANCLE_JOB:                    return "IMI_CANCLE_JOB";
        default:                                return "Unknown";
    }
}

class Message
{
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
