/*
 * InformationMap.cpp
 *
 *  Created on: 18.08.2016
 *      Author: sni
 */

#include "InformationMap.h"

#include <ice/ICEngine.h>
#include <ice/EntityDirectory.h>
#include <ice/communication/CommunicationInterface.h>

namespace ice
{

InformationMap::InformationMap(std::weak_ptr<ICEngine> engine) : engine(engine)
{
  //
}

InformationMap::~InformationMap()
{
  //
}

} /* namespace ice */
