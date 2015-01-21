/*
 * Communication.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/communication/Communication.h"

#include "ice/ICEngine.h"
#include "ice/coordination/Coordinator.h"
#include "ice/processing/EventHandler.h"
#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

namespace ice
{

Communication::Communication(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
  this->_log = el::Loggers::getLogger("Communication");
}

Communication::~Communication()
{
  //
}

void Communication::init()
{
  auto e = engine.lock();
  this->coordinator = e->getCoordinator();
  this->engineId = e->getId();
  this->eventHandler = e->getEventHandler();
}

void Communication::cleanUp()
{
  auto e = engine.lock();
  this->coordinator.reset();
  this->eventHandler.reset();
}

} /* namespace ice */
