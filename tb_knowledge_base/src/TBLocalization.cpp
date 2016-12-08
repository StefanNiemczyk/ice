/*
 * TBLocalization.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/TBLocalization.h"

#include <ice/information/InformationStream.h>

namespace ice
{

std::string TBLocalization::POS_REP = "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D";

std::shared_ptr<ice::Node> TBLocalization::createNode()
{
  auto node = std::make_shared<TBLocalization>();

  return node;
}

TBLocalization::TBLocalization()
{
  //
}

TBLocalization::~TBLocalization()
{
  //
}

std::string TBLocalization::getClassName()
{
  return "TBLocalization";
}

int TBLocalization::init()
{
  if (this->outputs.empty())
  {
    return 1;
  }

  this->output = std::static_pointer_cast<ice::InformationStream<GContainer>>(this->outputs[0]);

  return 0;
}

int TBLocalization::performNode()
{
  auto instance = this->gcontainerFactory->makeInstance(POS_REP);

//  posNew->x = 1;
//  posNew->y = 21;
//  posNew->z = 31;

  this->output->add(instance);

  return 0;
}

} /* namespace ice */
