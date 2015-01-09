#include <gtest/gtest.h>
#include "ModelGeneration.cpp"

using namespace std;

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

  inputs.push_back("ReqStream");
  inputsMin.push_back(1);
  inputsMax.push_back(1);

  oi.addSourceNodeClass("TestSourceNode", inputs, inputsMin, inputsMax);
  oi.addIndividual("TestEntity", "Robot");

  metadatas.push_back("Delay");
  metadataValues.push_back(5);
  metadataValues2.push_back(5);
  metadataGroundings.push_back("NodeDelayFixASPGrounding");
  metadatas.push_back("Cost");
  metadataValues.push_back(5);
  metadataValues2.push_back(5);
  metadataGroundings.push_back("NodeCostASPGrounding");
  metadatas.push_back("Accuracy");
  metadataValues.push_back(-1);
  metadataValues2.push_back(-1);
  metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

  oi.addNodeIndividual("TestSourceNodeInd", "TestSourceNode", "TestSystem", "TestEntity", "", metadatas,
                                metadataValues, metadataValues2, metadataGroundings);

  oi.saveOntology("/tmp/evalSimpleTest.owl");

  ModelGeneration mg;

  std::vector<std::string> toCheck;

  auto result = mg.readOntology("/tmp/evalSimpleTest.owl", &toCheck);

  result.print();
}
