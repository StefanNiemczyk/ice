/*
 * TBCollectionFactory.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include <TBCollectionFactory.h>

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"
#include "container/RTLandmark.h"
#include "container/WGS84.h"

namespace ice
{

TBCollectionFactory::TBCollectionFactory(std::weak_ptr<ICEngine> engine) : CollectionFactory(engine)
{

}

TBCollectionFactory::~TBCollectionFactory()
{
  //
}

std::shared_ptr<BaseInformationStream> TBCollectionFactory::createStream(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler, int streamSize) const
{
  if (className == "")
    return nullptr;

  if ("http://vs.uni-kassel.de/Ice#CoordinatePositionRep" == className)
  {
    auto stream = std::make_shared<InformationStream<Pos3D>>(streamDescription, eventHandler, streamSize);
    stream->setGContainer(true);
    return stream;
  }
  else if ("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D" == className)
  {
    auto stream = std::make_shared<InformationStream<PositionOrientation3D>>(streamDescription, eventHandler, streamSize);
    stream->setGContainer(true);
    return stream;
  }
  else if ("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark" == className)
  {
    auto set = std::make_shared<InformationStream<RTLandmark>>(streamDescription, eventHandler, 100);
    set->setGContainer(true);
    return set;
  }
  else if ("http://vs.uni-kassel.de/Ice#WGS84Rep" == className)
  {
    auto set = std::make_shared<InformationStream<WGS84>>(streamDescription, eventHandler, 100);
    set->setGContainer(true);
    return set;
  }

  return ice::CollectionFactory::createStream(className, streamDescription, eventHandler, streamSize);
}

std::shared_ptr<BaseInformationSet> TBCollectionFactory::createSet(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler) const
{
  if (className == "")
    return nullptr;

  if ("http://vs.uni-kassel.de/Ice#CoordinatePositionRep" == className)
  {
    auto set = std::make_shared<InformationSet<Pos3D>>(streamDescription, eventHandler);
    set->setGContainer(true);
    return set;
  }
  else if ("http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D" == className)
  {
    auto set = std::make_shared<InformationSet<PositionOrientation3D>>(streamDescription, eventHandler);
    set->setGContainer(true);
    return set;
  }
  else if ("http://vs.uni-kassel.de/TurtleBot#RelativeToLandmark" == className)
  {
    auto set = std::make_shared<InformationSet<RTLandmark>>(streamDescription, eventHandler);
    set->setGContainer(true);
    return set;
  }
  else if ("http://vs.uni-kassel.de/Ice#WGS84Rep" == className)
  {
    auto set = std::make_shared<InformationSet<WGS84>>(streamDescription, eventHandler);
    set->setGContainer(true);
    return set;
  }

  return ice::CollectionFactory::createSet(className, streamDescription, eventHandler);
}

} /* namespace ice */
