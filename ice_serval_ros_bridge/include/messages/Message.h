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


namespace ice
{
class Entity;
}

namespace ice
{

enum IceCmd
{
  SCMD_IDS_REQUEST                      = 10,
  SCMD_IDS_RESPONSE                     = 11,
  SCMD_ID_REQUEST                       = 20,
  SCMD_ID_RESPONSE                      = 21,
  SCMD_OFFERS_REQUEST                   = 30,
  SCMD_OFFERS_RESPONSE                  = 31,
  SCMD_INFORMATION_REQUEST              = 40,
  SCMD_INFORMATION_RESPONSE             = 41
};

class Message
{
public:
  static std::shared_ptr<Message> parse(std::string &jsonString);

public:
  Message(int id, bool payload);
  virtual ~Message();

  std::string toJson();

  int getId();
  std::shared_ptr<Entity> getEntity();
  void setEntity(std::shared_ptr<Entity> entity);

protected:
  virtual rapidjson::Value payloadToJson(rapidjson::Document &document) = 0;
  virtual bool parsePayload(rapidjson::Value& value) = 0;

protected:
  const int                     id;
  const bool                    payload;
  std::shared_ptr<Entity>       entity;
  el::Logger*                   _log;
};

} /* namespace ice */

#endif /* ICEMESSAGE_H_ */
