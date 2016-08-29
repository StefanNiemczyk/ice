/*
 * ServalInformationReceiver.cpp
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#include <ServalInformationReceiver.h>

#include <ice/information/BaseInformationStream.h>

#include "ServalCommunication.h"

namespace ice
{

ServalInformationReceiver::ServalInformationReceiver(std::shared_ptr<BaseInformationStream> const &stream,
                                                     std::shared_ptr<ServalCommunication> const &communication)
  : InformationReceiver(stream), communication(communication)
{
  this->streamHash = this->stream->getHash();
}

ServalInformationReceiver::~ServalInformationReceiver()
{
  //
}

const uint32_t ServalInformationReceiver::getStreamHash() const
{
  return this->streamHash;
}

std::shared_ptr<BaseInformationStream> ServalInformationReceiver::getStream() const
{
  return this->stream;
}

} /* namespace ice */
