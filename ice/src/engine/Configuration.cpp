/*
 * Configuration.cpp
 *
 *  Created on: May 26, 2014
 *      Author: sni
 */

#include "ice/Configuration.h"

namespace ice
{

// implementing static values
std::string Configuration::INFORMATION_TYPE_NAME_SEPERATOR = "/";

Configuration::Configuration()
{
  //information
  this->informationStreamBufferSize = 100;

  //event
  this->eventHandlerBufferSize = 100;
  this->eventHandlerThreadCount = 2;

  // coordination
  this->heartbeatTimeout = 3000; // 3 sec
  this->coordinationMessageTimeout = 5000; // 5 sec
  this->maxRetryCount = 5;
}

Configuration::~Configuration()
{
  //
}

int Configuration::getEventHandlerBufferSize() const
{
  return eventHandlerBufferSize;
}

void Configuration::setEventHandlerBufferSize(int eventHandlerBufferSize)
{
  this->eventHandlerBufferSize = eventHandlerBufferSize;
}

int Configuration::getEventHandlerThreadCount() const
{
  return eventHandlerThreadCount;
}

void Configuration::setEventHandlerThreadCount(int eventHandlerThreadCount)
{
  this->eventHandlerThreadCount = eventHandlerThreadCount;
}

int Configuration::getInformationStreamBufferSize() const
{
  return informationStreamBufferSize;
}

void Configuration::setInformationStreamBufferSize(int informationStreamBufferSize)
{
  this->informationStreamBufferSize = informationStreamBufferSize;
}

long Configuration::getHeartbeatTimeout() const
{
  return this->heartbeatTimeout;
}

void Configuration::setHeartbeatTimeout(long heartbeatTimeout)
{
  this->heartbeatTimeout = heartbeatTimeout;
}

long Configuration::getCoordinationMessageTimeout() const
{
  return coordinationMessageTimeout;
}

void Configuration::setCoordinationMessageTimeout(long coordinationMessageTimeout)
{
  this->coordinationMessageTimeout = coordinationMessageTimeout;
}

int Configuration::getMaxRetryCount() const
{
  return maxRetryCount;
}

void Configuration::setMaxRetryCount(int maxRetryCount)
{
  this->maxRetryCount = maxRetryCount;
}

} /* namespace ice */
