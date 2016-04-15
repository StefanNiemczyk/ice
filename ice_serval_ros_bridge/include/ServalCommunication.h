/*
 * ServalCommunication.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef SERVALCOMMUNICATION_H_
#define SERVALCOMMUNICATION_H_

#include <thread>

#include "CommunicationInterface.h"

namespace ice
{

class serval_interface;

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
  serval_interface*             serval;
  std::thread                   worker;
  bool                          running;

  std::string           const   configPath;
  std::string           const   host;
  int                   const   port;
  std::string           const   authName;
  std::string           const   authPass;
};

} /* namespace ice */

#endif /* SERVALCOMMUNICATION_H_ */
