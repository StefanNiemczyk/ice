/*
 * TBLocalization.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include "node/TBLocalization.h"

#include <tf/transform_datatypes.h>

#include <ice/Time.h>
#include <ice/information/InformationStream.h>

#include "TBKnowledgeBase.h"
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
  puts("TBLocalization CREATED!");
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

  if (this->engine.expired())
  {
	  return -1;
  }

  auto e = std::dynamic_pointer_cast<TBKnowledgeBase>(this->engine.lock());
  std::string topic = e->getRobotName() + "/amcl_pose";

  printf("listening on topic %s\n", topic.c_str());

  this->subscriber = e->nodeHandel.subscribe(topic, 10, &TBLocalization::onPosition, this);
  this->out = std::static_pointer_cast<ice::InformationStream<PositionOrientation3D>>(this->outputs[0]);

  return 0;
}

int TBLocalization::cleanUp()
{
  this->subscriber.shutdown();

  return 0;
}

void TBLocalization::onPosition(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  auto pos = std::make_shared<PositionOrientation3D>(this->gcontainerFactory->getRepresentation(POS_REP));

  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double rx, ry, rz;
  m.getEulerZYX(rz, ry, rx);

  pos->alpha = rz;
  pos->x = msg.pose.pose.position.x;
  pos->y = msg.pose.pose.position.y;
  pos->z = msg.pose.pose.position.z;

  time t = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nsec;
  this->out->add(pos, NO_TIME, t, t);
}

const int TBLocalization::newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                           std::shared_ptr<InformationCollection> collection)
{
  // source node, not necessary
  return 0;
}

int TBLocalization::performNode()
{
  // source node, not necessary
  return 0;
}

} /* namespace ice */
