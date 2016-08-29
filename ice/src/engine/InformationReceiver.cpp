/*
 * InformationReceiver.cpp
 *
 *  Created on: 27.08.2016
 *      Author: sni
 */

#include "ice/communication/InformationReceiver.h"

#include "ice/information/BaseInformationStream.h"

namespace ice
{

InformationReceiver::InformationReceiver(std::shared_ptr<BaseInformationStream> const &stream) : stream(stream)
{
  //
}

InformationReceiver::~InformationReceiver()
{
  //
}

/*!
 * \brief Returns the type_info of the template type.
 *
 * Returns the type_info of the template type.
 */
const std::type_info* InformationReceiver::getTypeInfo()
{
  return this->stream->getTypeInfo();
}

};
