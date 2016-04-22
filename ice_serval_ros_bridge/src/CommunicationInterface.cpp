/*
 * CommunicationInterface.cpp
 *
 *  Created on: Apr 15, 2016
 *      Author: sni
 */

#include "CommunicationInterface.h"

#include <iostream>

#include "Entity.h"
#include "IceServalBridge.h"

namespace ice
{

CommunicationInterface::CommunicationInterface()
{
  _log = el::Loggers::getLogger("CommunicationInterface");

}

CommunicationInterface::~CommunicationInterface()
{
  //
}

void CommunicationInterface::handleMessage(std::shared_ptr<Entity> &identity, Message &message)
{
  std::cout << "Received Message with id '%s' from %s" << std::to_string(message.command) << " "<< identity->toString() << std::endl;
  _log->info("Received Message with id '%s' from %s", std::to_string(message.command), identity->toString());

  switch (message.command)
  {
    case (SCMD_IDS_REQUEST):
      this->responseIds(identity);
      break;

    case (SCMD_IDS_RESPONSE):
      identity->fuse(message.map);
      identity->checkIce();
      break;

    case (SCMD_ID_REQUEST):
      // TODO
      break;

    case (SCMD_ID_RESPONSE):
      // TODO
      break;

    case (SCMD_INFORMATION_REQUEST):
      this->responseOfferedInformation(identity);
      break;

    case (SCMD_INFORMATION_RESPONSE):
      identity->addOfferedInformation(message.infos);
      break;

    default:
      _log->error("Unknown command '%s', message will be skipped", std::to_string(message.command));
      break;
  }
}

} /* namespace ice */
