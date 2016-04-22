#include <ros/package.h>

#include "XMLInformationReader.h"

#include "gtest/gtest.h"

TEST(XML, read)
{
  std::string path = ros::package::getPath("ice_serval_ros_bridge");
  ice::XMLInformationReader reader;

  bool result = reader.readFile(path + "/tests/data/info_sample_1.xml");

  ASSERT_TRUE(result);
  ASSERT_EQ(1, reader.getOffered().size());
  ASSERT_EQ(1, reader.getRequired().size());

  auto infoOff = reader.getOffered()[0];
  auto infoReq = reader.getRequired()[0];

  ASSERT_EQ("off_entity", infoOff->getEntity());
  ASSERT_EQ("off_entityType", infoOff->getEntityType());
  ASSERT_EQ("off_scope", infoOff->getScope());
  ASSERT_EQ("off_representation", infoOff->getRepresentation());
  ASSERT_EQ("off_relatedEntity", infoOff->getRelatedEntity());

  ASSERT_EQ("req_entity", infoReq->getEntity());
  ASSERT_EQ("req_entityType", infoReq->getEntityType());
  ASSERT_EQ("req_scope", infoReq->getScope());
  ASSERT_EQ("req_representation", infoReq->getRepresentation());
  ASSERT_EQ("req_relatedEntity", infoReq->getRelatedEntity());
}
