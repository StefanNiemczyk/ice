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

namespace ice
{

const std::string SERVAL_REST_GET_IDENTITIES = "/restful/keyring/identities.json";
const std::string SERVAL_REST_ADD_IDENTITY = "/restful/keyring/add";

const std::string SERVAL_REST_GET_CONVERSATION_LIST = "/restful/meshms/$RECIPIENTSID/conversationlist.json";
const std::string SERVAL_REST_GET_MESSAGE_LIST = "/restful/meshms/$SENDERSID/$RECIPIENTSID/messagelist.json";
const std::string SERVAL_REST_GET_MESSAGE_LIST_BY_TOKEN = "/restful/meshms/$SENDERSID/$RECIPIENTSID/newsince/$TOKEN/messagelist.json";
const std::string SERVAL_REST_POST_MESSAGE = "/restful/meshms/$SENDERSID/$RECIPIENTSID/sendmessage";

const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_LIST = "/restful/rhizome/bundlelist.json";
const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_LIST_BY_TOKEN = "/restful/rhizome/newsince/$TOKEN/bundlelist.json";
const std::string SERVAL_REST_RHIZOME_GET_BUNDLE = "/restful/rhizome/$BID.rhm";
const std::string SERVAL_REST_RHIZOME_BUNDLE_RAW = "/restful/rhizome/$BID/raw.bin";
const std::string SERVAL_REST_RHIZOME_BUNDLE_DECRIPTED = "/restful/rhizome/$BID/decrypted.bin";
const std::string SERVAL_REST_RHIZOME_POST_BUNDLE = "/restful/rhizome/insert";
const std::string SERVAL_REST_RHIZOME_POST_APPEND_BUNDLE = "/restful/rhizome/append";

struct serval_identity
{
  serval_identity(std::string sid, std::string did, std::string name) :
      sid(sid), did(did), name(name)
  {
  }

  const std::string sid;
  const std::string did;
  const std::string name;

  std::string toString()
  {
    return "sid: '" + sid + "', did: '" + did + "', name: '" + name + "'";
  }
};

struct serval_conversation
{
  std::string _id;
  std::string my_sid;
  std::string their_sid;bool read;
  std::string last_message;
  int read_offset;

  std::string toString()
  {
    return "_id: '" + _id + "', my_sid: '" + my_sid + "', their_sid: '" + their_sid + "', read: '"
        + (read ? "true" : "false") + "', last_message: '" + last_message + "', read_offset: '"
        + std::to_string(read_offset) + "'";
  }
};

struct serval_message
{
  std::string type;
  std::string my_sid;
  std::string their_sid;
  int offset;
  std::string token;
  std::string text;bool delivered;bool read;
  long timestamp;
  std::string ack_offset;

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
  int read_offset;
  int latest_ack_offset;
  std::vector<serval_message> messages;

  std::string toString()
  {
    return "read_offset: '" + std::to_string(read_offset) + "', latest_ack_offset: '"
        + std::to_string(latest_ack_offset) + "', message count: '" + std::to_string(messages.size()) + "'";
  }
};

struct serval_bundle
{
  std::string token;
  int _id;
  std::string service;
  std::string id;
  long date;
  long inserttime;
  std::string author;
  int fromhere;
  long filesize;
  std::string filehash;
  std::string sender;
  std::string recipient;
  std::string name;

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

class serval_interface
{
public:
  serval_interface(std::string const host, int const port, std::string const authName, std::string const authPass);
  virtual ~serval_interface();

  // keyring api
  std::unique_ptr<std::vector<serval_identity>> getIdentities();
  std::unique_ptr<serval_identity> addIdentity(std::string password = "");

  //meshms api
  std::unique_ptr<std::vector<serval_conversation>> getConversationList(std::string recipientSid);
  std::unique_ptr<serval_message_list> getMessageList(std::string recipientSid, std::string senderSid,
                                                      std::string token = "");
  bool postMessage(std::string recipientSid, std::string senderSid, std::string msg);

  // rhizome api
  std::unique_ptr<std::vector<serval_bundle>> getBundleList(std::string token = "");
  bool addBundle(std::string pathToFile, std::string manifest = "", std::string bundleId = "",
                 std::string author = "", std::string secret = "");

private:
  void logError(std::string msg);

private:
  std::string const host;
  int const port;
  int timeout;
  cpr::Authentication *auth;
  std::string address;
  serval_identity *self;

};

} /* namespace ice */

#endif /* SERVALINTERFACE_H_ */
