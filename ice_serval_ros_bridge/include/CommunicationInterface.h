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

#include <ice/information/InformationSpecification.h>
#include <easylogging++.h>

#include "EntityDirectory.h"

namespace ice
{

class Entity;
class IceServalBridge;

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
  std::shared_ptr<Entity> receiver;
  int command;
  std::map<std::string, std::string> map;
  std::vector<InformationSpecification> infos;
};

class CommunicationInterface
{
public:
  CommunicationInterface();
  virtual ~CommunicationInterface();
  virtual void init() = 0;
  virtual void cleanUp() = 0;

  virtual void requestId(std::shared_ptr<Entity> const &identity, std::string const &id) = 0;
  virtual void responseId(std::shared_ptr<Entity> const &identity, std::string const &id) = 0;
  virtual void requestIds(std::shared_ptr<Entity> const &identity) = 0;
  virtual void responseIds(std::shared_ptr<Entity> const &identity) = 0;
  virtual void requestOfferedInformation(std::shared_ptr<Entity> const &identity) = 0;
  virtual void responseOfferedInformation(std::shared_ptr<Entity> const &identity) = 0;

protected:
  virtual void handleMessage(std::shared_ptr<Entity> &identity, Message &message);

protected:
  IceServalBridge                             *bridge;
  std::shared_ptr<Entity>                     self;
  std::shared_ptr<EntityDirectory>            directory;
  el::Logger                                  *_log;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
