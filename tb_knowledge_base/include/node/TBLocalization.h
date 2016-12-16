/*
 * TBLocalization.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_TBLOCALIZATION_H_
#define INCLUDE_NODE_TBLOCALIZATION_H_

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ice/processing/Node.h>

namespace ice
{
template <typename T>
class InformationStream;
class PositionOrientation3D;

class TBLocalization : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  TBLocalization();
  virtual ~TBLocalization();

  int init();
  int cleanUp();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();
  void onPosition(const geometry_msgs::PoseWithCovarianceStamped& msg);

private:
  static std::string POS_REP;

private:
  ros::Subscriber                       				      subscriber;
  std::shared_ptr<InformationStream<PositionOrientation3D>>   out;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_TBLOCALIZATION_H_ */
