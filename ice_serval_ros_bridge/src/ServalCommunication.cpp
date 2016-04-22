/*
 * ServalCommunication.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "ServalCommunication.h"

#include <chrono>
#include <serval_interface.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "Identity.h"

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

ServalCommunication::ServalCommunication(std::shared_ptr<IdentityDirectory> &directory, std::string configPath, std::string const host,
                                         int const port, std::string const authName, std::string const authPass) :
    CommunicationInterface(), configPath(configPath), host(host),
    port(port), authName(authName), authPass(authPass), serval(nullptr), running(false)
{
  _log = el::Loggers::getLogger("CommunicationInterface");
  this->directory = directory;
}

ServalCommunication::~ServalCommunication()
{
  if (this->running)
  {
    this->running = false;
    this->worker.join();
  }
}

void ServalCommunication::init()
{
  // create interface
  this->serval = std::make_shared<serval_interface>(this->configPath, this->host, this->port, this->authName, this->authPass);
  this->self = this->directory->self;

  // get own id
  if (this->ownSid == "")
  {
    auto id = this->serval->keyring.getSelf();
    if (id->size() == 0)
    {
      // error case, own id could not be determined
      throw (std::runtime_error("Own serval id could not be determined"));
    }

    this->ownSid = id->at(0).sid;
    this->self->addId(IdentityDirectory::ID_SERVAL, this->ownSid);
  }

  this->running = true;
  this->worker = std::thread(&ServalCommunication::checkServal, this);
}

void ServalCommunication::cleanUp()
{
  this->running = false;
  this->worker.join();

  if (this->serval != nullptr)
  {
    this->serval = nullptr;
  }
}

void ServalCommunication::requestId(std::shared_ptr<Identity> const &identity, std::string const &id)
{
  // TODO
}

void ServalCommunication::responseId(std::shared_ptr<Identity> const &identity, std::string const &id)
{
  // TODO
}

void ServalCommunication::requestIds(std::shared_ptr<Identity> const &identity)
{
  _log->info("Requesting Ids from '%s'", identity->toString());
  Message m;
  m.receiver = identity;
  m.command = IceCmd::SCMD_IDS_REQUEST;

  this->pushMessage(m);
}

void ServalCommunication::responseIds(std::shared_ptr<Identity> const &identity)
{
  _log->info("Sending Ids to '%s'", identity->toString());
  Message m;
  m.receiver = identity;
  m.command = IceCmd::SCMD_IDS_RESPONSE;

  this->self->pushIdsToMap(m.map);

  this->pushMessage(m);
}

void ServalCommunication::requestOfferedInformation(std::shared_ptr<Identity> const &identity)
{
  _log->info("Requesting offered information from '%s'", identity->toString());
  Message m;
  m.receiver = identity;
  m.command = IceCmd::SCMD_INFORMATION_REQUEST;

  this->pushMessage(m);
}

void ServalCommunication::responseOfferedInformation(std::shared_ptr<Identity> const &identity)
{

}

std::shared_ptr<serval_interface> ServalCommunication::getServalInterface()
{
  return this->serval;
}

void ServalCommunication::setOwnSid(std::string const &sid)
{
  this->ownSid = sid;
}

void ServalCommunication::pushMessage(Message &message)
{
  std::lock_guard<std::mutex> guard(this->_mtx);

  this->messages.push_back(message);
}

void ServalCommunication::checkServal()
{
  int counter = 0;
  std::vector<std::shared_ptr<Identity>> newNodes;

  while (this->running)
  {
    // check all 10 iterations for new peers
    if (counter >= 2)
    {
      for (auto &identity : newNodes)
      {
        this->requestIds(identity);
      }
      newNodes.clear();

      auto peers = this->serval->keyring.getPeerIdentities();

      for (auto &sid : *peers)
      {
        auto identity = this->directory->lookup(IdentityDirectory::ID_SERVAL, sid.sid);

        if (identity == nullptr)
        {
          // Create new instance and request ids
          identity = this->directory->create(IdentityDirectory::ID_SERVAL, sid.sid);
          // At the beginning each discovered node is expected to be an ice node
          identity->setAvailable(true);

          _log->info("New ID discovered: %s", identity->toString());
          this->updateToken(identity); //token currently to slow
          newNodes.push_back(identity);
        }

        // update timestamp
        identity->setActiveTimestamp();
      }

      this->directory->checkTimeout();

      counter = 0;
    }

    // check for new messages
    auto sids = this->directory->availableIdentities();
    std::vector<Message> msgs;
    int count;

    for (auto &sid : *sids)
    {
      if (false == this->running)
      {
        return;
      }

      msgs.clear();
      count = this->readMessages(sid, msgs);

      if (count == 0)
        continue;

      for (auto &msg : msgs)
      {
        this->handleMessage(sid, msg);
      }
    }

    // send messages
    {
      std::lock_guard<std::mutex> guard(this->_mtx);

      for (auto &msg : this->messages)
      {
        if (false == this->running)
        {
          return;
        }

        std::string sidStr;
        if (msg.receiver->getId(IdentityDirectory::ID_SERVAL, sidStr) == false)
        {
          _log->error("Message could not be send, receiver %s is missing a serval id", msg.receiver->toString());
          continue;
        }

        this->serval->meshms.postMessage(sidStr, this->ownSid, this->serializeMessage(msg));
      }

      this->messages.clear();
    }

    ++counter;
    // sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void ServalCommunication::updateToken(std::shared_ptr<Identity> &identity)
{
  std::string sidStr;

  if (identity->getId(IdentityDirectory::ID_SERVAL, sidStr) == false)
  {
    return;
  }

  auto msgs = this->serval->meshms.getMessageList(sidStr, this->ownSid);
  int count = 0;

  if (msgs == nullptr)
  {
    return;
  }

  std::string token;
  int offset = -1;

  for (auto &msg : msgs->messages)
  {
    if (offset < msg.offset && msg.token != "")
    {
      offset = msg.offset;
      token = msg.token;
    }
  }

  identity->addMetadata("SERVAL_MESSAGE_TOKEN", token);
  identity->addMetadata("SERVAL_MESSAGE_OFFSET", std::to_string(offset));
}

int ServalCommunication::readMessages(std::shared_ptr<Identity> &identity, std::vector<Message> &outMessages)
{
  // token should not be used, response is much slower then requesting whole list
  std::string token = "";
//  bool result = identity->getMetadata("SERVAL_MESSAGE_TOKEN", token);
  std::string offsetStr;
  int offset;
  bool result = identity->getMetadata("SERVAL_MESSAGE_OFFSET", offsetStr);
  if (result)
    offset = std::stoi(offsetStr);

  std::string sidStr;

  if (identity->getId(IdentityDirectory::ID_SERVAL, sidStr) == false)
  {
    return 0;
  }

  auto msgs = this->serval->meshms.getMessageList(sidStr, this->ownSid, token);
//  this->serval->meshms.markMessagesAsRead(sidStr);

  int count = 0;
  std::string lastToken = "";
  int maxOffset = -1;

  if (msgs == nullptr)
  {
    return 0;
  }

  for (auto &msg : msgs->messages)
  {
    // type identifies a received message
    if (msg.read || msg.type != "<" || msg.offset <= offset)//msgs->read_offset)
      continue;

//    std::cout << msg.toString() << std::endl;

//    if (offset < msg.offset && msg.token != "")
//    {
//      offset = msg.offset;
//      lastToken = msg.token;
//    }

    if (maxOffset < msg.offset)
    {
      maxOffset = msg.offset;
    }

    Message m;
    m.receiver = identity;

    if (false == this->deserializeMessage(msg.text, m))
      continue;

    outMessages.push_back(m);

    ++count;
  }

  if (maxOffset > 0)
  {
    identity->addMetadata("SERVAL_MESSAGE_OFFSET", std::to_string(maxOffset));
  }

  if (lastToken != "")
  {
    identity->addMetadata("SERVAL_MESSAGE_TOKEN", lastToken);
  }

  return count;
}

std::string ServalCommunication::serializeMessage(Message &message)
{
  std::stringstream ss;
  pt::ptree tree;

  tree.put("cmd", message.command);

  // serialize ids
  if (message.map.size() > 0)
  {
    pt::ptree ids;
    for (auto &element : message.map)
    {
      pt::ptree key, value, id;
      key.put("", element.first);
      value.put("", element.second);
      id.push_back(std::make_pair("", key));
      id.push_back(std::make_pair("", value));
      ids.push_back(std::make_pair("", id));
    }
    tree.add_child("ids", ids);
  }

  pt::write_json(ss, tree, false);

  return ss.str();
}

bool ServalCommunication::deserializeMessage(std::string &message, Message &outMessage)
{
  try {
    pt::ptree tree;
    std::istringstream is(message);
    pt::read_json(is, tree);

    auto iter = tree.find("cmd");

    if (iter == tree.not_found())
    {
//      std::cout << "command not found" << std::endl;
      return false;
    }

    outMessage.command = iter->second.get_value<int>();


    iter = tree.find("ids");
    if (iter != tree.not_found())
    {
      for (pt::ptree::value_type &rows : tree.get_child("ids"))
      {
        int i = 1;
        std::string key, value;
        for (pt::ptree::value_type &row : rows.second)
        {
          switch (i)
          {
            case 1:
              key = row.second.get_value<std::string>();
              break;
            case 2:
              value = row.second.get_value<std::string>();
              break;
          }

          ++i;
        }

        if (key != "" && value != "")
          outMessage.map[key] = value;
      }

      return true; // id message finished
    }

    // TODO further messages

    return true;
  }
  catch (std::exception &e)
  {
   _log->error("Message could not be deserialized: %s", std::string(e.what()));
   _log->error("Message: %s", message);
    return false;
  }
}

} /* namespace ice */
