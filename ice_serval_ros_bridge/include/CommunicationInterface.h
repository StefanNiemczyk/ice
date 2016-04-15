/*
 * CommunicationInterface.h
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#ifndef COMMUNICATIONINTERFACE_H_
#define COMMUNICATIONINTERFACE_H_

#include <memory>

#include "IdentityDirectory.h"

namespace ice
{

class Identity;

class CommunicationInterface
{
public:
  CommunicationInterface();
  virtual ~CommunicationInterface();
  virtual void init() = 0;
  virtual void cleanUp() = 0;

  virtual bool requestId(std::shared_ptr<Identity> const &identity, std::string const &id) = 0;
  virtual bool requestIds(std::shared_ptr<Identity> const &identity) = 0;

protected:
  std::shared_ptr<Identity>                     self;
  IdentityDirectory                             directory;
};

} /* namespace ice */

#endif /* COMMUNICATIONINTERFACE_H_ */
