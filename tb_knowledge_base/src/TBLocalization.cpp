/*
 * TBLocalization.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/TBLocalization.h"

#include <ice/information/InformationStream.h>

#include "container/PositionOrientation3D.h"

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

  this->out = std::static_pointer_cast<ice::InformationStream<PositionOrientation3D>>(this->outputs[0]);

  return 0;
}

const int TBLocalization::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int TBLocalization::performNode()
{
  //auto instance = this->gcontainerFactory->makeInstance(POS_REP);
  auto instance = std::make_shared<PositionOrientation3D>(this->gcontainerFactory->getRepresentation(POS_REP));

//  posNew->x = 1;
//  posNew->y = 21;
//  posNew->z = 31;

  this->out->add(instance);

  return 0;
}

} /* namespace ice */
