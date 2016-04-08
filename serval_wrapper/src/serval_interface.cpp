/*
 * servalinterface.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: sni
 */

#include "serval_interface.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <regex>

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

serval_interface::serval_interface(std::string const host, int const port, std::string const authName,
                                   std::string const authPass) :
    host(host), port(port), self(nullptr), timeout(5000)
{
  this->auth = new cpr::Authentication {authName, authPass};
  this->address = "http://" + this->host + ":" + std::to_string(this->port);
}

serval_interface::~serval_interface()
{
  delete this->auth;
}

std::unique_ptr<std::vector<serval_identity>> serval_interface::getIdentities()
{
  auto r = cpr::Get(cpr::Url {this->address + SERVAL_REST_GET_IDENTITIES}, *this->auth, cpr::Timeout{this->timeout});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get identities");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError("Unexpected return type '" + r.header["content-type"] + "' for get identities");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);
  std::unique_ptr<std::vector<serval_identity>> ids(new std::vector<serval_identity>);

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    std::string sid, did, name;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          sid = row.second.get_value<std::string>();
          break;
        case 2:
          did = row.second.get_value<std::string>();
          break;
        case 3:
          name = row.second.get_value<std::string>();
          break;
      }

      ++i;
    }
    serval_identity id(sid, did, name);
    ids->push_back(id);
  }

  return std::move(ids);
}

std::unique_ptr<serval_identity> serval_interface::addIdentity(std::string password)
{
  std::string param = "";
  if (password != "")
  {
    param = "?pin=" + password;
  }

  auto r = cpr::Get(cpr::Url {this->address + SERVAL_REST_ADD_IDENTITY} + param, *this->auth, cpr::Timeout{this->timeout});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for add identity");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError("Unexpected return type '" + r.header["content-type"] + "' for add identity");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);

  std::string sid = tree.get_child("identity").get<std::string>("sid");

  std::unique_ptr<serval_identity> id(new serval_identity(sid, "", ""));
  return std::move(id);
}

std::unique_ptr<std::vector<serval_conversation>> serval_interface::getConversationList(std::string recipientSid)
{
  std::string path = SERVAL_REST_GET_CONVERSATION_LIST;
  path.replace(SERVAL_REST_GET_CONVERSATION_LIST.find("$RECIPIENTSID"), 13, recipientSid);

  auto r = cpr::Get(cpr::Url {this->address + path}, *this->auth, cpr::Timeout{this->timeout});

//  std::cout << path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for get conversation list for recipient '"
            + recipientSid + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for get conversation list for recipient '"
            + recipientSid + "'");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);
  std::unique_ptr<std::vector<serval_conversation>> convs(new std::vector<serval_conversation>);

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    serval_conversation conv;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          conv._id = row.second.get_value<std::string>();
          break;
        case 2:
          conv.my_sid = row.second.get_value<std::string>();
          break;
        case 3:
          conv.their_sid = row.second.get_value<std::string>();
          break;
        case 4:
          conv.read = row.second.get_value<bool>();
          break;
        case 5:
          conv.last_message = row.second.get_value<std::string>();
          break;
        case 6:
          conv.read_offset = row.second.get_value<int>();
          break;
      }

      ++i;
    }
    convs->push_back(conv);
  }

  return std::move(convs);
}

std::unique_ptr<serval_message_list> serval_interface::getMessageList(std::string recipientSid, std::string senderSid,
                                                                      std::string token)
{
  std::string path;

  if (path == "")
  {
    path = SERVAL_REST_GET_MESSAGE_LIST;
    path.replace(path.find("$RECIPIENTSID"), 13, recipientSid);
    path.replace(path.find("$SENDERSID"), 10, senderSid);
  }
  else
  {
    path = SERVAL_REST_GET_MESSAGE_LIST_BY_TOKEN;
    path.replace(path.find("$RECIPIENTSID"), 13, recipientSid);
    path.replace(path.find("$SENDERSID"), 10, senderSid);
    path.replace(path.find("$TOKEN"), 6, token);
  }

  auto r = cpr::Get(cpr::Url {this->address + path}, *this->auth, cpr::Timeout{this->timeout});

//  std::cout << path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for get message list for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for get message list for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);
  std::unique_ptr<serval_message_list> list(new serval_message_list);
  list->read_offset = tree.get<int>("read_offset");
  list->latest_ack_offset = tree.get<int>("latest_ack_offset");

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    serval_message msg;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          msg.type = row.second.get_value<std::string>();
          break;
        case 2:
          msg.my_sid = row.second.get_value<std::string>();
          break;
        case 3:
          msg.their_sid = row.second.get_value<std::string>();
          break;
        case 4:
          msg.offset = row.second.get_value<int>();
          break;
        case 5:
          msg.token = row.second.get_value<std::string>();
          break;
        case 6:
          msg.text = row.second.get_value<std::string>();
          break;
        case 7:
          msg.delivered = row.second.get_value<bool>();
          break;
        case 8:
          msg.read = row.second.get_value<bool>();
          break;
        case 9:
          if (row.second.empty())
            msg.timestamp = -1;
          else
            msg.timestamp = row.second.get_value<long>();
          break;
        case 10:
          msg.ack_offset = row.second.get_value<std::string>();
          break;
      }

      ++i;
    }
    list->messages.push_back(msg);
  }

  return std::move(list);
}

bool serval_interface::postMessage(std::string recipientSid, std::string senderSid, std::string msg)
{
  std::string path = SERVAL_REST_POST_MESSAGE;
  path.replace(path.find("$RECIPIENTSID"), 13, recipientSid);
  path.replace(path.find("$SENDERSID"), 10, senderSid);

  auto r = cpr::Post(cpr::Url {this->address + path}, *this->auth, cpr::Multipart { {"message", msg,
                                                                                     "text/plain; charset=utf-8"}}, cpr::Timeout{this->timeout});

//  std::cout << path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 201) // HTTP_ADD
  {
    this->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for post message for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return false;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for post message for sender/recipient '" + senderSid
            + "'/'" + recipientSid + "'");
    return false;
  }

  return true;
}


std::unique_ptr<std::vector<serval_bundle>> serval_interface::getBundleList(std::string token)
{
  std::string path;

  if (token == "")
  {
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_LIST;
  }
  else
  {
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_LIST_BY_TOKEN;
    path.replace(path.find("$TOKEN"), 6, token);
  }

  auto r = cpr::Get(cpr::Url {this->address + path}, *this->auth, cpr::Timeout{this->timeout});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get bundle list");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->logError("Unexpected return type '" + r.header["content-type"] + "' for get bundle list");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);
  std::unique_ptr<std::vector<serval_bundle>> ids(new std::vector<serval_bundle>);

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    serval_bundle bundle;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          std::cout << i << std::endl;
          bundle.token = row.second.get_value<std::string>();
          break;
        case 2:
          std::cout << i << std::endl;
          bundle._id = row.second.get_value<int>();
          break;
        case 3:
          std::cout << i << std::endl;
          bundle.service = row.second.get_value<std::string>();
          break;
        case 4:
          bundle.id = row.second.get_value<std::string>();
          break;
        case 5:
          std::cout << i << std::endl;
          bundle.date = row.second.get_value<long>();
          break;
        case 6:
          std::cout << i << std::endl;
          bundle.inserttime = row.second.get_value<long>();
          break;
        case 7:
          bundle.author = row.second.get_value<std::string>();
          break;
        case 8:
          if (row.second.empty())
            bundle.fromhere = -1;
          else
            bundle.fromhere = row.second.get_value<int>();
          break;
        case 9:
          if (row.second.empty())
            bundle.filesize = -1;
          else
            bundle.filesize = row.second.get_value<long>();
          break;
        case 10:
          bundle.filehash = row.second.get_value<std::string>();
          break;
        case 11:
          bundle.sender = row.second.get_value<std::string>();
          break;
        case 12:
          bundle.recipient = row.second.get_value<std::string>();
          break;
        case 13:
          bundle.name = row.second.get_value<std::string>();
          break;
      }

      ++i;
    }
    ids->push_back(bundle);
  }

  return std::move(ids);
}

bool serval_interface::addBundle(std::string pathToFile, std::string manifest, std::string bundleId, std::string author,
                                 std::string secret)
{
  auto form = cpr::Multipart { {"manifest", "", "rhizome/manifest"}, {"payload", cpr::File {pathToFile}}};
  auto r = cpr::Post(cpr::Url {this->address + SERVAL_REST_RHIZOME_POST_BUNDLE}, *this->auth, form, cpr::Timeout{this->timeout});

  std::cout << pathToFile << std::endl;
  std::cout << r.status_code << std::endl;                  // 200
  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 201) // HTTP_ADD
  {
//      this->logError(
//          "Unexpected return code '" + std::to_string(r.status_code) + "' for post message for sender/recipient '"
//              + senderSid + "'/'" + recipientSid + "'");
    return false;
  }

  if (r.header["content-type"] != "application/json")
  {
//      this->logError(
//          "Unexpected return type '" + r.header["content-type"] + "' for post message for sender/recipient '" + senderSid
//              + "'/'" + recipientSid + "'");
    return false;
  }

  return true;
}

void serval_interface::logError(std::string msg)
{
  std::cerr << "serval_interface Error: " << msg << std::endl;
}

} /* namespace ice */
