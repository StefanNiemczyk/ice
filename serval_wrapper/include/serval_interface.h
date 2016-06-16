/*
 * servalinterface.h
 *
 *  Created on: Apr 7, 2016
 *      Author: sni
 */

#ifndef SERVALINTERFACE_H_
#define SERVALINTERFACE_H_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cpr/cpr.h>
#include <mdp_cpp.h>

#include "serval_wrapper/keyring.h"
#include "serval_wrapper/meshms.h"
#include "serval_wrapper/rhizome.h"
#include "serval_wrapper/MDPSocket.h"
#include "serval_wrapper/MSPSocket.h"

namespace ice
{

struct serval_identity
{
  serval_identity(std::string sid, std::string did, std::string name) :
      sid(sid), did(did), name(name)
  {
  }

  std::string   const   sid;
  std::string   const   did;
  std::string   const   name;

  std::string toString()
  {
    return "sid: '" + sid + "', did: '" + did + "', name: '" + name + "'";
  }
};

struct serval_conversation
{
  std::string           _id;
  std::string           my_sid;
  std::string           their_sid;
  bool                  read;
  std::string           last_message;
  int                   read_offset;

  std::string toString()
  {
    return "_id: '" + _id + "', my_sid: '" + my_sid + "', their_sid: '" + their_sid + "', read: '"
        + (read ? "true" : "false") + "', last_message: '" + last_message + "', read_offset: '"
        + std::to_string(read_offset) + "'";
  }
};

struct serval_message
{
  std::string           type;
  std::string           my_sid;
  std::string           their_sid;
  int                   offset;
  std::string           token;
  std::string           text;
  bool                  delivered;
  bool                  read;
  long                  timestamp;
  std::string           ack_offset;

  std::string toString()
  {
    return "type: '" + type + "', my_sid: '" + my_sid + "', their_sid: '" + their_sid + "', offset: '"
        + std::to_string(offset) + "', token: '" + token + "', text: '" + text + "', delivered: '"
        + (delivered ? "true" : "false") + "', read: '" + (read ? "true" : "false") + "', timestamp: '"
        + std::to_string(timestamp) + "', ack_offset: '" + ack_offset + "'";
  }
};

struct serval_message_list
{
  int                           read_offset;
  int                           latest_ack_offset;
  std::vector<serval_message>   messages;

  std::string toString()
  {
    return "read_offset: '" + std::to_string(read_offset)
              + "', latest_ack_offset: '" + std::to_string(latest_ack_offset)
              + "', message count: '" + std::to_string(messages.size()) + "'";
  }
};

struct serval_bundle
{
  std::string           token;
  int                   _id;
  std::string           service;
  std::string           id;
  long                  date;
  long                  inserttime;
  std::string           author;
  int                   fromhere;
  long                  filesize;
  std::string           filehash;
  std::string           sender;
  std::string           recipient;
  std::string           name;

  std::string toString()
  {
    return "token: '" + token
              + "', _id: '" + std::to_string(_id)
              + "', service: '" + service
              + "', _id: '" + std::to_string(_id)
              + "', date: '" + std::to_string(date)
              + "', inserttime: '" + std::to_string(inserttime)
              + "', author: '" + author
              + "', fromhere: '" + std::to_string(fromhere)
              + "', filesize: '" + std::to_string(filesize)
              + "', filehash: '" + filehash
              + "', sender: '" + sender
              + "', recipient: '" + recipient
              + "', name: '" + name + "'";
  }
};

struct serval_bundle_manifest
{
  std::string           service;
  long                  version;
  std::string           id;
  long                  date;
  std::string           name;
  long                  filesize;
  std::string           filehash;
  int                   crypt;
  std::string           tail;

  std::string toString()
  {
    return "service: '" + service
              + "', version: '" + std::to_string(version)
              + "', id: '" + id
              + "', date: '" + std::to_string(date)
              + "', name: '" + name
              + "', filesize: '" + std::to_string(filesize)
              + "', filehash: '" + filehash
              + "', crypt: '" + std::to_string(crypt)
              + "', tail: '" + tail + "'";
  }
};

class serval_interface
{
public:
  static void sidToArray(std::string const &sid, uint8_t* out);
  static std::string arrayToSid(uint8_t* sid);

public:
  serval_interface(std::string configPath, std::string const host, int const port, std::string const authName, std::string const authPass);
  virtual ~serval_interface();
  bool startDeamon();
  bool stopDeamon();

  void logError(std::string msg);
  int getTimeout();
  std::string getAddress();
  cpr::Authentication* getAuth();
  std::string getServalBin();
  int exec(std::string const &cmd, std::stringstream &output);
  std::shared_ptr<MDPSocket> createSocket(int port, std::string const &senderSid = "");
  std::shared_ptr<MSPSocket> createMSPSocket(int port, std::string const &senderSid);

public:
  serval_wrapper::keyring               keyring;
  serval_wrapper::meshms                meshms;
  serval_wrapper::rhizome               rhizome;

private:
  std::string                   const   host;
  int                           const   port;
  int                                   timeout;
  std::string                           address;
  std::string                           servalBin;
  std::string                           instancePath;
  cpr::Authentication                   *auth;
};

} /* namespace ice */

#endif /* SERVALINTERFACE_H_ */
