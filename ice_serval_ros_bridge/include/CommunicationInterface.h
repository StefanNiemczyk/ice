/*
 * CommunicationInterface.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef COMMUNICATIONINTERFACE_H_
#define COMMUNICATIONINTERFACE_H_

#include <map>
#include <memory>
#include <easylogging++.h>

#include "IdentityDirectory.h"

namespace ice
{

class Identity;

enum IceCmd
{
  SCMD_IDS_REQUEST  = 10,
  SCMD_IDS_RESPONSE  = 11,
  SCMD_ID_REQUEST  = 20,
  SCMD_ID_RESPONSE  = 21,
  SCMD_INFORMATION_REQUEST  = 30,
  SCMD_INFORMATION_RESPONSE  = 31
};

struct Message
{
  std::shared_ptr<Identity> receiver;
  int command;
  std::map<std::string, std::string> map;
  std::string message;
};

class CommunicationInterface
{
public:
  CommunicationInterface();
  virtual ~CommunicationInterface();
  virtual void init() = 0;
  virtual void cleanUp() = 0;

  virtual void requestId(std::shared_ptr<Identity> const &identity, std::string const &id) = 0;
  virtual void responseId(std::shared_ptr<Identity> const &identity, std::string const &id) = 0;
  virtual void requestIds(std::shared_ptr<Identity> const &identity) = 0;
  virtual void responseIds(std::shared_ptr<Identity> const &identity) = 0;
  virtual void requestOfferedInformation(std::shared_ptr<Identity> const &identity) = 0;
  virtual void responseOfferedInformation(std::shared_ptr<Identity> const &identity) = 0;

protected:
  virtual void handleMessage(std::shared_ptr<Identity> &identity, Message &message);

protected:
  std::shared_ptr<Identity>                     self;
  std::shared_ptr<IdentityDirectory>            directory;
  el::Logger                                    *_log;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
