/*
 * ProcessingModel.cpp
 *
 *  Created on: Apr 23, 2015
 *      Author: sni
 */

#include <ice/model/ProcessingModelGenerator.h>

#include "ice/ICEngine.h"

namespace ice
{

ProcessingModelGenerator::ProcessingModelGenerator(std::weak_ptr<ICEngine> engine)
{
  this->engine = engine;
}

ProcessingModelGenerator::~ProcessingModelGenerator()
{
  // TODO Auto-generated destructor stub
}

} /* namespace ice */
