/*
 * DetectVictims.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/DetectVictims.h"

#include <ice/Time.h>
#include <ice/information/InformationSet.h>

#include "TBKnowledgeBase.h"
#include "container/Pos3D.h"
#include "container/PositionOrientation3D.h"

namespace ice
{

std::string DetectVictims::POS_REP = "http://vs.uni-kassel.de/Ice#CoordinatePositionRep";

std::shared_ptr<ice::Node> DetectVictims::createNode()
{
  auto node = std::make_shared<DetectVictims>();

  return node;
}

DetectVictims::DetectVictims()
{
  //
}

DetectVictims::~DetectVictims()
{
  //
}

std::string DetectVictims::getClassName()
{
  return "DetectVictims";
}

int DetectVictims::init()
{
  if (this->outputSets.empty())
  {
    return 1;
  }

  auto e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());

  this->subscriber = e->nodeHandel.subscribe("logical_camera", 10, &DetectVictims::onVictim, this);
  this->pos = e->positionOwn;
  this->out = std::static_pointer_cast<ice::InformationSet<Pos3D>>(this->outputSets[0]);

  return 0;
}

int DetectVictims::cleanUp()
{
  this->subscriber.shutdown();

  return 0;
}

void DetectVictims::onVictim(const ttb_msgs::LogicalCamera& msg)
{
  _log->debug("LogicalCamera message received for '%v' of type '%v'", msg.modelName, msg.type);

  if (msg.type != "victim")
    return;

  auto own = this->pos->getLast();

  if (own == nullptr)
    return;

  auto pos = std::make_shared<Pos3D>(this->gcontainerFactory->getRepresentation(POS_REP));

  double x = msg.pose.x * cos(own->getInformation()->alpha) - msg.pose.y * sin(own->getInformation()->alpha);
  double y = msg.pose.x * sin(own->getInformation()->alpha) + msg.pose.y * cos(own->getInformation()->alpha);

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

  _log->info("Victim '%v' added at position '%v/%v'", msg.modelName, pos->x, pos->y);

  time t = msg.timeStamp.sec * 1000000000 + msg.timeStamp.nsec;
  this->out->add(msg.modelName, pos, NO_TIME, t, t);
}

const int DetectVictims::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int DetectVictims::performNode()
{
  // source node, not necessary
  return 0;
}

} /* namespace ice */
