/*
 * VictimDetection.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/DetectLandmarks.h"

#include <ice/Time.h>
#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

#include "TBKnowledgeBase.h"
#include "container/PositionOrientation3D.h"

namespace ice
{

std::string DetectLandmarks::POS_REP = "http://vs.uni-kassel.de/TurtleBot#PositionOrientation3D";

std::shared_ptr<ice::Node> DetectLandmarks::createNode()
{
  auto node = std::make_shared<DetectLandmarks>();

  return node;
}

DetectLandmarks::DetectLandmarks()
{
  //
}

DetectLandmarks::~DetectLandmarks()
{
  //
}

std::string DetectLandmarks::getClassName()
{
  return "DetectLandmarks";
}

int DetectLandmarks::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  auto e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());

  this->subscriber = e->nodeHandel.subscribe("logical_camera", 10, &DetectLandmarks::onLandmark, this);
  this->pos = e->positionOwn;
  this->out = std::static_pointer_cast<ice::InformationSet<PositionOrientation3D>>(this->outputSets[0]);

  return 0;
}

int DetectLandmarks::cleanUp()
{
  this->subscriber.shutdown();

  return 0;
}

void DetectLandmarks::onLandmark(const ttb_msgs::LogicalCamera& msg)
{
  _log->debug("LogicalCamera message received for '%v' of type '%v'", msg.modelName, msg.type);

  if (msg.type != "landmark")
    return;

  auto own = this->pos->getLast();

  if (own == nullptr)
    return;

  auto pos = std::make_shared<PositionOrientation3D>(this->gcontainerFactory->getRepresentation(POS_REP));

  double x = msg.pose.x * cos(own->getInformation()->alpha) - msg.pose.y * sin(own->getInformation()->alpha);
  double y = msg.pose.x * sin(own->getInformation()->alpha) + msg.pose.y * cos(own->getInformation()->alpha);

  pos->alpha = msg.pose.theta + own->getInformation()->alpha;
  pos->x = x + own->getInformation()->x;
  pos->y = y + own->getInformation()->y;
  pos->z = 0;

  auto old = this->out->get(msg.modelName);

  if (old != nullptr)
  {
    double dist = (old->getInformation()->x - pos->x) * (old->getInformation()->x - pos->x)
        + (old->getInformation()->y - pos->y) * (old->getInformation()->y - pos->y);

    if (dist < 0, 25 * 0, 25)
      return;
  }

  _log->info("Landmark '%v' added at position '%v/%v'", msg.modelName, pos->x, pos->y);

  time t = msg.timeStamp.sec * 1000000000 + msg.timeStamp.nsec;
  this->out->add(msg.modelName, pos, NO_TIME, t, t);
}

const int DetectLandmarks::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int DetectLandmarks::performNode()
{

  // source node, not necessary
  return 0;
}

} /* namespace ice */
