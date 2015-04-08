#include <gtest/gtest.h>
#include <ClingWrapper.h>
#include "EvalScenarios.cpp"

using namespace std;
/*
 TEST(EvalModelGeneration, simpleTest)
 {
 std::string path = ros::package::getPath("ice");
 std::vector<std::string> inputs;
 std::vector<int> inputsMin;
 std::vector<int> inputsMax;
 std::vector<std::string> outputs;
 std::vector<int> outputsMin;
 std::vector<int> outputsMax;
 std::vector<std::string> metadatas;
 std::vector<int> metadataValues;
 std::vector<int> metadataValues2;
 std::vector<std::string> metadataGroundings;

 ice::OntologyInterface oi(path + "/java/lib/");

 oi.addIRIMapper(path + "/ontology/");
 oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
 oi.loadOntologies();

 // add information structure
 oi.addValueScope("TestValueScope", "TestValueScope1");

 std::vector<std::string> vec;
 vec.push_back("TestValueScope1");
 oi.addRepresentation("TestRep", "TestRep1", vec);

 std::vector<std::string> reps;
 reps.push_back("TestRep1");
 oi.addEntityScope("TestScope", reps);

 std::vector<std::string> scopes;
 scopes.push_back("TestScope");
 oi.addEntityType("TestEntity", scopes);

 // add system
 oi.addSystem("TestSystem");

 oi.addNamedStream("ReqStream", "TestScope", "TestRep1");

 oi.addIndividual("TestEntityInd", "TestEntity");
 oi.addRequiredStream("TestReqStream", "ReqStream", "TestSystem", "TestEntityInd", "");

 outputs.push_back("ReqStream");
 outputsMin.push_back(1);
 outputsMax.push_back(1);

 oi.addSourceNodeClass("TestSourceNode", outputs, outputsMin, outputsMax);

 metadatas.push_back("Delay");
 metadataValues.push_back(5);
 metadataValues2.push_back(5);
 metadataGroundings.push_back("NodeDelayFixASPGrounding");
 metadatas.push_back("Cost");
 metadataValues.push_back(5);
 metadataValues2.push_back(5);
 metadataGroundings.push_back("NodeCostASPGrounding");
 metadatas.push_back("Accuracy");
 metadataValues.push_back(90);
 metadataValues2.push_back(90);
 metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

 oi.addNodeIndividual("TestSourceNodeInd", "TestSourceNode", "TestSystem", "TestEntityInd", "", metadatas,
 metadataValues, metadataValues2, metadataGroundings);

 oi.saveOntology("/tmp/evalSimpleTest.owl");

 ModelGeneration mg(path);

 std::vector<std::string> toCheck;

 toCheck.push_back(
 "selectedStream(1,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0)");
 toCheck.push_back("sumCost(1,5)");
 toCheck.push_back("node(1,testSystem,testSourceNodeInd,testEntityInd,none)");
 toCheck.push_back(
 "metadataStream(1,delay,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0,5)");
 toCheck.push_back(
 "metadataStream(1,accuracy,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0,90)");

 auto result = mg.test("/tmp/evalSimpleTest.owl", &toCheck);

 result.print();
 }

 TEST(EvalModelGeneration, simpleTestSeries)
 {
 std::string path = ros::package::getPath("ice");
 std::vector<std::string> inputs;
 std::vector<int> inputsMin;
 std::vector<int> inputsMax;
 std::vector<std::string> outputs;
 std::vector<int> outputsMin;
 std::vector<int> outputsMax;
 std::vector<std::string> metadatas;
 std::vector<int> metadataValues;
 std::vector<int> metadataValues2;
 std::vector<std::string> metadataGroundings;

 ice::OntologyInterface oi(path + "/java/lib/");

 oi.addIRIMapper(path + "/ontology/");
 oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
 oi.loadOntologies();

 // add information structure
 oi.addValueScope("TestValueScope", "TestValueScope1");

 std::vector<std::string> vec;
 vec.push_back("TestValueScope1");
 oi.addRepresentation("TestRep", "TestRep1", vec);

 std::vector<std::string> reps;
 reps.push_back("TestRep1");
 oi.addEntityScope("TestScope", reps);

 std::vector<std::string> scopes;
 scopes.push_back("TestScope");
 oi.addEntityType("TestEntity", scopes);

 // add system
 oi.addSystem("TestSystem");

 oi.addNamedStream("ReqStream", "TestScope", "TestRep1");

 oi.addIndividual("TestEntityInd", "TestEntity");
 oi.addRequiredStream("TestReqStream", "ReqStream", "TestSystem", "TestEntityInd", "");

 outputs.push_back("ReqStream");
 outputsMin.push_back(1);
 outputsMax.push_back(1);

 oi.addSourceNodeClass("TestSourceNode", outputs, outputsMin, outputsMax);

 metadatas.push_back("Delay");
 metadataValues.push_back(5);
 metadataValues2.push_back(5);
 metadataGroundings.push_back("NodeDelayFixASPGrounding");
 metadatas.push_back("Cost");
 metadataValues.push_back(5);
 metadataValues2.push_back(5);
 metadataGroundings.push_back("NodeCostASPGrounding");
 metadatas.push_back("Accuracy");
 metadataValues.push_back(90);
 metadataValues2.push_back(90);
 metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

 oi.addNodeIndividual("TestSourceNodeInd", "TestSourceNode", "TestSystem", "TestEntityInd", "", metadatas,
 metadataValues, metadataValues2, metadataGroundings);

 oi.saveOntology("/tmp/evalSimpleTest.owl");

 ModelGeneration mg(path);

 std::vector<std::string> toCheck;

 toCheck.push_back(
 "selectedStream(1,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0)");
 toCheck.push_back("sumCost(1,5)");
 toCheck.push_back("node(1,testSystem,testSourceNodeInd,testEntityInd,none)");
 toCheck.push_back(
 "metadataStream(1,delay,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0,5)");
 toCheck.push_back(
 "metadataStream(1,accuracy,testSystem,testSourceNodeInd,testSystem,information(testEntityInd,testScope,testRep1,none),0,90)");

 auto result = mg.testSeries("/tmp/evalSimpleTest.owl", &toCheck, 50000);

 result.print();
 }*/

TEST(EvalModelGeneration, simpleEvalTests)
{
  EvalScenarios scenarios("/tmp", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
  });
  //                                global      verbose gnuplot         runs
  scenarios.chainScenario(          true,       false,  true,          5,      1, 10, 1, 10, 10, 1);
//  scenarios.representationScenario( false,       false,  true,          10,      1, 9, 1);
//  scenarios.systemsStarMashScenario(true,       false,  true,          5,     10, 300, 10);
//  scenarios.systemsFullMashScenario(false,       false,   true,          5,     10, 20, 1);
}
