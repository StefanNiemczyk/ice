/*
 * meshms.cpp
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#include "serval_wrapper/meshms.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <regex>
#include <cpr/cpr.h>

#include "serval_interface.h"

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

namespace serval_wrapper {

meshms::meshms(serval_interface* const interface) : interface(interface)
{

}

meshms::~meshms()
{

}

std::unique_ptr<std::vector<serval_conversation>> meshms::getConversationList(std::string const &recipientSid)
{
  std::string path = SERVAL_REST_GET_CONVERSATION_LIST;
  path.replace(path.find("$RECIPIENTSID"), 13, recipientSid);

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                    cpr::Timeout{this->interface->getTimeout()});

//  std::cout << path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for get conversation list for recipient '"
            + recipientSid + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError(
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

std::unique_ptr<serval_message_list> meshms::getMessageList(std::string const &recipientSid, std::string const &senderSid,
                                                                      std::string const token)
{
  std::string path;

  if (token == "")
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

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                    cpr::Timeout{this->interface->getTimeout()});

//  std::cout << this->interface->getAddress() + path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for get message list for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for get message list for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return nullptr;
  }

  // WORKAROUND to fix a bug in the restful api, if requesting a bundle list by a token the closing braces are missing
  if (r.text.find("\n]\n}") == std::string::npos)
  {
    r.text += "]}";
  }

  try {
    // Read json.
    pt::ptree tree;
    std::istringstream is(r.text);
    pt::read_json(is, tree);
    std::unique_ptr<serval_message_list> list(new serval_message_list);

    auto iter = tree.find("latest_ack_offset");
    if (iter == tree.not_found())
    {
      list->read_offset = -1;
    }
    else
    {
      list->read_offset = iter->second.get_value<int>();
    }

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
  catch (std::exception &e)
  {
    this->interface->logError(
        "Message list for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "' could not be parsed");
    std::cout << e.what() << std::endl;
    std::cout << r.text << std::endl;
    return nullptr;
  }
}

bool meshms::postMessage(std::string const &recipientSid, std::string const &senderSid, std::string const &msg)
{
  std::string path = SERVAL_REST_POST_MESSAGE;
  path.replace(path.find("$RECIPIENTSID"), 13, recipientSid);
  path.replace(path.find("$SENDERSID"), 10, senderSid);

  auto r = cpr::Post(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                     cpr::Multipart { {"message", msg, "text/plain; charset=utf-8"}}, cpr::Timeout{this->interface->getTimeout()});

//  std::cout << path << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 201) // HTTP_ADD
  {
    this->interface->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for post message for sender/recipient '"
            + senderSid + "'/'" + recipientSid + "'");
    return false;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for post message for sender/recipient '" + senderSid
            + "'/'" + recipientSid + "'");
    return false;
  }

  return true;
}

bool meshms::markMessagesAsRead(std::string const &dialogPartner)
{
   std::string command = "meshms read messages " + dialogPartner;
   std::stringstream ss;
   std::string line;

   this->interface->exec(command.c_str(), ss);

   // TODO check if successfull

   return true;
}

}
} /* namespace ice */
