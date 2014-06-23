/*
 * Communication.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: sni
 */

#include "ice/communication/Communication.h"

#include "ice/ICEngine.h"
#include "ice/Logger.h"
#include "ice/coordination/Coordinator.h"

namespace ice
{

Communication::Communication(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
  this->_log = Logger::get("Communication");
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
}

void Communication::cleanUp()
{
  auto e = engine.lock();
  this->coordinator.reset();
}

} /* namespace ice */
