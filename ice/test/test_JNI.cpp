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
}

TEST(JNITest, loadOntology)
{
  std::string path = ros::package::getPath("ice");
  bool result;

  ice::OntologyInterface oi(path + "/java/lib/");

  oi.addIRIMapper(path + "/ontology/");

  ASSERT_FALSE(oi.errorOccurred());

  result = oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.loadOntologies();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.isConsistent();

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);
}

TEST(JNITest, addSystem)
{
  std::string path = ros::package::getPath("ice");
  bool result;

  ice::OntologyInterface oi(path + "/java/lib/");

  oi.addIRIMapper(path + "/ontology/");
  oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
  oi.loadOntologies();

  result = oi.addSystem("TestSystem");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);
}

TEST(JNITest, addInfoStructure)
{
  std::string path = ros::package::getPath("ice");
  bool result;
  std::vector<std::string> metadatas;
  std::vector<int> metadataValues;
  std::vector<int> metadataValues2;
  std::vector<std::string> metadataGroundings;

  ice::OntologyInterface oi(path + "/java/lib/");

  oi.addIRIMapper(path + "/ontology/");
  oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
  oi.loadOntologies();

  result = oi.addValueScope("TestValueScope", "TestValueScope1");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  result = oi.addValueScope("TestValueScope", "TestValueScope2");

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::vector<std::string> vec;
  vec.push_back("TestValueScope1");
  vec.push_back("TestValueScope2");
  result = oi.addRepresentation("TestRep", "TestRep1", vec);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::vector<std::string> reps;
  reps.push_back("TestRep1");
  result = oi.addEntityScope("TestScope", reps);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::vector<std::string> scopes;
  scopes.push_back("TestScope");
  result = oi.addEntityType("TestEntity", scopes);

  ASSERT_FALSE(oi.errorOccurred());
  ASSERT_TRUE(result);

  std::string str = oi.readInformationStructureAsASP();

  std::vector<std::string> toSearch;

  EXPECT_TRUE(str.find("entityType(testEntity).") >= 0);
  EXPECT_TRUE(str.find("scope(testScope).") >= 0);
  EXPECT_TRUE(str.find("valueScope(testValueScope1).") >= 0);
  EXPECT_TRUE(str.find("valueScope(testValueScope2).") >= 0);
  EXPECT_TRUE(str.find("representation(testRep1).") >= 0);

  EXPECT_TRUE(str.find("hasScope(testEntity,testScope).") >= 0);
  EXPECT_TRUE(str.find("hasRepresentation(testScope,testRep1).") >= 0);
  EXPECT_TRUE(str.find("hasDimension(testRep1,testValueScope1).") >= 0);
  EXPECT_TRUE(str.find("hasDimension(testRep1,testValueScope2).") >= 0);
}

//TEST(JNITest, addNode)
//{
//  std::string path = ros::package::getPath("ice");
//  bool result;
//  std::vector<std::string> metadatas;
//  std::vector<int> metadataValues;
//  std::vector<int> metadataValues2;
//  std::vector<std::string> metadataGroundings;
//
//  ice::OntologyInterface oi(path + "/java/lib/");
//
//  oi.addIRIMapper(path + "/ontology/");
//  oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
//  oi.loadOntologies();
//
//  result = oi.addSystem("TestSystem");
//
//  metadatas.push_back("Delay");
//  metadataValues.push_back(5);
//  metadataValues2.push_back(5);
//  metadataGroundings.push_back("NodeDelayASPGrounding");
//  metadatas.push_back("Cost");
//  metadataValues.push_back(5);
//  metadataValues2.push_back(5);
//  metadataGroundings.push_back("NodeCostASPGrounding");
//  metadatas.push_back("Accuracy");
//  metadataValues.push_back(-1);
//  metadataValues2.push_back(-1);
//  metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");
//
//  result = oi.addNodeIndividual("TestNode", "ComputationNode", "TestSystem", metadatas, metadataValues, metadataValues2,
//                                metadataGroundings);
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//
//  std::unique_ptr<std::vector<std::vector<std::string>>>vec= oi.readNodesAndIROsAsASP("TestSystem");
//
//  ASSERT_FALSE(oi.errorOccurred());
//
//      for (std::vector<std::string> v : *vec)
//      {
//        for (std::string s : v)
//        {
//          std::cout << "#" << s << std::endl;
//        }
//      }
//}

//  TEST(JNITest, old)
//  {
//  std::string path = ros::package::getPath("ice");
//  bool result;
//
//  ice::OntologyInterface oi(path + "/java/lib/");
//
//  ASSERT_FALSE(oi.errorOccurred());
//
//  oi.addIRIMapper(path + "/ontology/");
//
//  ASSERT_FALSE(oi.errorOccurred());
//
////  result =  oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
//  result =  oi.addOntologyIRI("http://vs.uni-kassel.de/IceTest");
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//
//  result = oi.loadOntologies();
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//
//  result = oi.isConsistent();
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//
//  result = oi.addSystem("TestSystem");
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_FALSE(result);
//
//  std::unique_ptr<std::vector<std::string>> systems = oi.getSystems();
//
//  ASSERT_FALSE(oi.errorOccurred());
//
////  for (std::string sys : *systems)
////    std::cout << sys << std::endl;
//
//  std::string infoStructure = oi.readInformationStructureAsASP();
//
//  ASSERT_FALSE(oi.errorOccurred());
////  std::cout << infoStructure << std::endl;
//
//  std::vector<std::string> metadatas;
//  std::vector<int> metadataValues;
//  std::vector<std::string> metadataGroundings;
//
//  metadatas.push_back("Delay");
//  metadataValues.push_back(5);
//  metadataGroundings.push_back("NodeDelayASPGrounding");
//  metadatas.push_back("Cost");
//  metadataValues.push_back(5);
//  metadataGroundings.push_back("NodeCostASPGrounding");
//  metadatas.push_back("Accuracy");
//  metadataValues.push_back(-1);
//  metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");
//
//  result = oi.addNodeIndividual("TestNode", "KalmanFilter", "TestSystem", metadatas, metadataValues, metadataGroundings);
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//
//  metadatas.clear();
//  metadataValues.clear();
//  metadataGroundings.clear();
//
//  metadatas.push_back("Delay");
//  metadataValues.push_back(3);
//  metadataGroundings.push_back("IRODelayASPGrounding");
//  metadatas.push_back("Cost");
//  metadataValues.push_back(4);
//  metadataGroundings.push_back("IROCostASPGrounding");
//  metadatas.push_back("Accuracy");
//  metadataValues.push_back(-5);
//  metadataGroundings.push_back("IROAccuracyMaxASPGrounding");
//
//  result = oi.addIROIndividual("TestIRO", "Ego2AlloIRO", "TestSystem", metadatas, metadataValues, metadataGroundings);
//
//  ASSERT_FALSE(oi.errorOccurred());
//  ASSERT_TRUE(result);
//  for (std::string sys : *systems)
//  {
//    std::unique_ptr<std::vector<std::vector<std::string>>>vec= oi.readNodesAndIROsAsASP(sys);
//
//    ASSERT_FALSE(oi.errorOccurred());
//
//    for (std::vector<std::string> v : *vec)
//    {
//      for (std::string s : v)
//      {
//        std::cout << "#" << s << std::endl;
//      }
//    }
//  }
//}

}
