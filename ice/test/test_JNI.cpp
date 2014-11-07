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
  ASSERT_FALSE(result);

  std::unique_ptr<std::vector<std::string>> systems = oi.getSystems();

  ASSERT_FALSE(oi.errorOccurred());

//  for (std::string sys : *systems)
//    std::cout << sys << std::endl;

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
  metadatas.push_back("Accuracy");
  metadataValues.push_back(-1);
  metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

  result = oi.addNodeIndividual("TestNode", "KalmanFilter", "TestSystem", metadatas, metadataValues, metadataGroundings);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  metadatas.clear();
  metadataValues.clear();
  metadataGroundings.clear();

  metadatas.push_back("Delay");
  metadataValues.push_back(3);
  metadataGroundings.push_back("IRODelayASPGrounding");
  metadatas.push_back("Cost");
  metadataValues.push_back(4);
  metadataGroundings.push_back("IROCostASPGrounding");
  metadatas.push_back("Accuracy");
  metadataValues.push_back(-5);
  metadataGroundings.push_back("IROAccuracyMaxASPGrounding");

  result = oi.addIROIndividual("TestIRO", "Ego2AlloIRO", "TestSystem", metadatas, metadataValues, metadataGroundings);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);
  for (std::string sys : *systems)
  {
    std::unique_ptr<std::vector<std::vector<std::string>>>vec= oi.readNodesAndIROsAsASP(sys);

    ASSERT_FALSE(oi.errorOccurred());

    for (std::vector<std::string> v : *vec)
    {
      for (std::string s : v)
      {
        std::cout << "#" << s << std::endl;
      }
    }
  }
}

}
