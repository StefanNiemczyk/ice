/*
 * ServalCommunication.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef SERVALCOMMUNICATION_H_
#define SERVALCOMMUNICATION_H_

#include <mutex>
#include <thread>
#include <vector>

#include "CommunicationInterface.h"

namespace ice
{

class serval_interface;

class ServalCommunication : public CommunicationInterface
{
public:
  ServalCommunication(std::shared_ptr<IdentityDirectory> &directory, std::string const configPath, std::string const host,
                      int const port, std::string const authName, std::string const authPass);
  virtual ~ServalCommunication();
  virtual void init();
  virtual void cleanUp();

  void checkServal();

  virtual void requestId(std::shared_ptr<Identity> const &identity, std::string const &id);
  virtual void responseId(std::shared_ptr<Identity> const &identity, std::string const &id);
  virtual void requestIds(std::shared_ptr<Identity> const &identity);
  virtual void responseIds(std::shared_ptr<Identity> const &identity);
  virtual void requestOfferedInformation(std::shared_ptr<Identity> const &identity);
  virtual void responseOfferedInformation(std::shared_ptr<Identity> const &identity);

  std::shared_ptr<serval_interface> getServalInterface();

  void setOwnSid(std::string const &sid);

private:
  void pushMessage(Message &message);
  void updateToken(std::shared_ptr<Identity> &identity);
  int readMessages(std::shared_ptr<Identity> &identity, std::vector<Message> &outMessages);
  std::string serializeMessage(Message &message);
  bool deserializeMessage(std::string &message, Message &outMessage);

private:
  std::shared_ptr<serval_interface>             serval;
  std::thread                                   worker;
  bool                                          running;
  std::string                                   ownSid;
  std::vector<Message>                          messages;

  std::string                           const   configPath;
  std::string                           const   host;
  int                                   const   port;
  std::string                           const   authName;
  std::string                           const   authPass;

  std::mutex                                    _mtx;
};

} /* namespace ice */

#endif /* SERVALCOMMUNICATION_H_ */
