
#include "gtest/gtest.h"

#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
#include <memory>

#include "IceServalBridge.h"
#include "RosGContainerPublisher.h"

static geometry_msgs::Vector3::ConstPtr message;

void onMsg(const geometry_msgs::Vector3::ConstPtr& msg)
{
  message = msg;
}

TEST(GMessagePublisher, simpleTest)
{
  ice::InitParams *params1 = new ice::InitParams();

  std::string path = ros::package::getPath("ice_serval_ros_bridge");

  params1->ontologyIri = "http://vs.uni-kassel.de/IceServalBridgeTest";
  params1->ontologyIriSelf = "http://vs.uni-kassel.de/IceServalBridgeTest#Mops";
  params1->ontologyPath = path + "/tests/data/";
  params1->servalInstancePath = "/tmp/mops";
  params1->servalHost = "localhost";
  params1->servalPort = 4110;
  params1->servalUser = "peter";
  params1->servalPassword = "venkman";
  params1->xmlInfoPath = path + "/tests/data/info_bridge_req.xml";
  params1->xmlTemplateFile = path + "/tests/data/message_templates.xml";

  ros::NodeHandle nh_("");
  ros::NodeHandle pnh_("~");

  auto subscriber = nh_.subscribe("test_bridge_topic", 100, &onMsg);

  ice::IceServalBridge bridge(nh_, pnh_, params1);

  bridge.init();

  std::string repStr = bridge.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#CoordinatePositionRep");
  auto rep = bridge.gcontainerFactory->getRepresentation(repStr);
  std::shared_ptr<ice::RequiredInfo> reqInfo = bridge.getRequiredInfors()[0]; // TODO

  ASSERT_NE(nullptr, rep);

  auto x = rep->accessPath({bridge.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#XCoordinate")});
  auto y = rep->accessPath({bridge.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#YCoordinate")});
  auto z = rep->accessPath({bridge.ontologyInterface->toShortIri("http://www.semanticweb.org/sni/ontologies/2013/7/Ice#ZCoordinate")});

  ASSERT_NE(nullptr, x);
  ASSERT_NE(nullptr, y);
  ASSERT_NE(nullptr, z);

  auto instance = bridge.gcontainerFactory->makeInstance(rep);

  double xVal = 1.2;
  double yVal = 2.0;
  double zVal = 3.0;

  instance->set(x, &xVal);
  instance->set(y, &yVal);
  instance->set(z, &zVal);

  ASSERT_EQ(xVal, instance->getValue<double>(x));
  ASSERT_EQ(yVal, instance->getValue<double>(y));
  ASSERT_EQ(zVal, instance->getValue<double>(z));


  bridge.publisher->publish(reqInfo, instance);

  sleep(2);
  ros::spinOnce();

  ASSERT_NE(nullptr, message);
  ASSERT_EQ(xVal, message->x);
  ASSERT_EQ(yVal, message->y);
  ASSERT_EQ(zVal, message->z);

}
