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

struct Message
{
  std::shared_ptr<Identity> receiver;
  std::string message;
};

class ServalCommunication : public CommunicationInterface
{
public:
  ServalCommunication(IdentityDirectory const &directory, std::string const configPath, std::string const host,
                      int const port, std::string const authName, std::string const authPass);
  virtual ~ServalCommunication();
  virtual void init();
  virtual void cleanUp();

  virtual bool requestId(std::shared_ptr<Identity> const &identity, std::string const &id);
  virtual bool requestIds(std::shared_ptr<Identity> const &identity);

  void checkServal();

private:
  void pushMessage(Message message);

private:
  serval_interface*             serval;
  std::thread                   worker;
  bool                          running;
  std::string                   ownSid;
  std::vector<Message>          messages;

  std::string           const   configPath;
  std::string           const   host;
  int                   const   port;
  std::string           const   authName;
  std::string           const   authPass;

  std::mutex                    _mtx;
};

} /* namespace ice */

#endif /* SERVALCOMMUNICATION_H_ */
