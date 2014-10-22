#include <iostream>
#include <jni.h>
#include <vector>

#include <ros/package.h>

#include "ice/coordination/OntologyInterface.h"

#include "gtest/gtest.h"

namespace
{


TEST(JNITest, create)
{
  std::string path = ros::package::getPath("ice");
  bool result;

  ice::OntologyInterface oi(path + "/java/lib/");

  ASSERT_FALSE(oi.errorOccurred());

  oi.addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi.errorOccurred());

//  result =  oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
  result =  oi.addOntologyIRI("http://vs.uni-kassel.de/IceTest");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.loadOntologies();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.isConsistent();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.addSystem("TestSystem");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::string infoStructure = oi.readInformationStructureAsASP();

  ASSERT_FALSE(oi.errorOccurred());
//  std::cout << infoStructure << std::endl;

  std::vector<std::string> metadatas;
  std::vector<int> metadataValues;
  std::vector<std::string> metadataGroundings;

  metadatas.push_back("Delay");
  metadataValues.push_back(5);
  metadataGroundings.push_back("NodeDelayASPGrounding");
  metadatas.push_back("Cost");
  metadataValues.push_back(5);
  metadataGroundings.push_back("NodeCostASPGrounding");

  result = oi.addNodeIndividual("TestNode", "KalmanFilter", "TestSystem", metadatas, metadataValues, metadataGroundings);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  infoStructure = oi.readNodesAndIROsAsASP();

  ASSERT_FALSE(oi.errorOccurred());
  std::cout << infoStructure << std::endl;
}

}
