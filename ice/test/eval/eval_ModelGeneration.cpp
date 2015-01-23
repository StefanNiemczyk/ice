#include <gtest/gtest.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "ModelGeneration.cpp"

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

TEST(EvalModelGeneration, chainTest)
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
  std::stringstream ss;

  ofstream file;

  int chainSizeMin = 10;
  int chainSizeMax = 10;
  int chainSizeStep = 1;

  int nodesMin = 10;
  int nodesMax = 10;
  int nodesStep = 1;

  int runs = 100;

  bool tree = false;

  ss.str("");
  ss << "/home/sni/Desktop/results" << chainSizeMin << "-" << chainSizeMax << "_" << nodesMin << "-" << nodesMax << ".txt";
  file.open(ss.str());

  std::string entityType = "EvalEntityType";
  std::string entity = "EvalEntity";
  std::string superValueScope = "SuperEvalValueScope";
  std::string superRepresentation = "SuperRepresentation";

  for (int chainSize = chainSizeMin; chainSize <= chainSizeMax; chainSize += chainSizeStep)
  {
    for (int nodes = nodesMin; nodes <= nodesMax; nodes += nodesStep)
    {
      std::cout << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
      std::cout << "Starting eval " << chainSize << " chain size, " << nodes << " nodes" << std::endl << std::endl;

      std::string system = "EvalSystem";

      ice::OntologyInterface oi(path + "/java/lib/");

      oi.addIRIMapper(path + "/ontology/");
      oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
      oi.loadOntologies();

      std::vector<std::string> scopes;
      oi.addEntityType(entityType, scopes);

      oi.addIndividual(entity, entityType);
      oi.addSystem(system);

      for (int i = 0; i < chainSize; ++i)
      {
        inputs.clear();
        inputsMin.clear();
        inputsMax.clear();
        outputs.clear();
        outputsMin.clear();
        outputsMax.clear();

        ss.str("");
        ss << "EvalScope" << chainSize << "_" << i;
        std::string scope = ss.str();
        ss.str("");
        ss << "EvalValueScope" << chainSize << "_" << i;
        std::string valueScope = ss.str();
        ss.str("");
        ss << "EvalRepresentation" << chainSize << "_" << i;
        std::string representation = ss.str();
        ss.str("");
        ss << "EvalStream" << chainSize << "_" << i;
        std::string stream = ss.str();
        ss.str("");
        ss << "EvalStream" << chainSize << "_" << (i - 1);
        std::string lastStream = ss.str();

        // add information structure
        oi.addValueScope(superValueScope, valueScope);

        std::vector<std::string> vec;
        vec.push_back(valueScope);
        oi.addRepresentation(superRepresentation, representation, vec);

        std::vector<std::string> reps;
        reps.push_back(representation);
        oi.addEntityScope(scope, reps);

        std::vector<std::string> scopes;
        scopes.push_back(scope);
        oi.addScopesToEntityType(entityType, scopes);

        oi.addNamedStream(stream, scope, representation);

        inputs.push_back(lastStream);
        inputsMin.push_back(1);
        inputsMax.push_back(1);
        outputs.push_back(stream);
        outputsMin.push_back(1);
        outputsMax.push_back(1);

        for (int j = 0; j < nodes; j++)
        {
          metadatas.clear();
          metadataValues.clear();
          metadataValues2.clear();
          metadataGroundings.clear();

          ss.str("");
          ss << "EvalNode" << j << "_" << i;
          std::string node = ss.str();
          if (i == 0)
          {
//            if (j < 1)
//            {
            oi.addSourceNodeClass(node, outputs, outputsMin, outputsMax);

            metadatas.push_back("Delay");
            metadataValues.push_back(5);
            metadataValues2.push_back(5);
            metadataGroundings.push_back("NodeDelayFixASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(1);
            metadataGroundings.push_back("NodeCostASPGrounding");
            metadatas.push_back("Accuracy");
            metadataValues.push_back(nodesMax - j);
            metadataValues2.push_back(nodesMax - j);
            metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

            oi.addNodeIndividual(node + "Ind", node, system, entity, "", metadatas, metadataValues, metadataValues2,
                                 metadataGroundings);
//            }
          }
          else// if (j < 1)
          {
            oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

            metadatas.push_back("Delay");
            metadataValues.push_back(0);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeDelayASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(1);
            metadataGroundings.push_back("NodeCostASPGrounding");
            metadatas.push_back("Accuracy");
            metadataValues.push_back(nodesMax - j);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

            // TODO fix
            oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2,
                                 metadataGroundings);
          }
        }
      }

      ss.str("");
      ss << "EvalStream" << chainSize << "_" << chainSize - 1;
      std::string stream = ss.str();
      std::string reqStream = "Req" + stream;
      oi.addRequiredStream(reqStream, stream, system, entity, "");

      ss.str("");
      ss << "/tmp/evalChain" << chainSize << "_" << nodes << ".owl";
      std::string fileName = ss.str();
      oi.saveOntology(fileName);

      ModelGeneration mg(path);

      std::vector<std::string> toCheck;

      ss.str("");
      ss << "metadataStream(1,accuracy,stream(1,evalSystem,evalNode0_" << chainSize - 1
          << "Ind,evalSystem,information(evalEntity,evalScope" << chainSize << "_" << chainSize - 1
          << ",evalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << "),"
          << chainSize * nodesMax << ")";
      toCheck.push_back(ss.str());

      ss.str("");
      ss << "metadataStream(1,delay,stream(1,evalSystem,evalNode0_" << chainSize - 1
          << "Ind,evalSystem,information(evalEntity,evalScope" << chainSize << "_" << chainSize - 1
          << ",evalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << "),5)";
      toCheck.push_back(ss.str());

      ss.str("");
      ss << "selectedStream(1,evalSystem,evalNode0_" << chainSize - 1
          << "Ind,evalSystem,information(evalEntity,evalScope" << chainSize << "_" << chainSize - 1
          << ",evalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << ")";
      toCheck.push_back(ss.str());

      ss.str("");
      ss << "node(1,evalSystem,evalNode0_" << chainSize - 1 << "Ind,evalEntity,none)";
      toCheck.push_back(ss.str());

      ss.str("");
      ss << "sumCost(1," << chainSize << ")";
      toCheck.push_back(ss.str());

      auto result = mg.testSeries(fileName, &toCheck, runs);

      result.print();

      // print to file
      file << chainSize << "\t";
      file << nodes << "\t";
      file << result.numberTotal << "\t";
      file << result.numberSuccessful << "\t";
      file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
          << result.worst.totalTime << "\t";
      file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime << "\t"
          << result.worst.ontologyReadTime << "\t";
      file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t" << result.best.ontologyReasonerTime << "\t"
          << result.worst.ontologyReasonerTime << "\t";
      file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t" << result.best.ontologyToASPTime << "\t"
          << result.worst.ontologyToASPTime << "\t";
      file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime << "\t"
          << result.worst.aspGroundingTime << "\t";
      file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime << "\t"
          << result.worst.aspSolvingTime << std::endl;

      // gnuplot -persist -e "set hidden3d; set dgrid3d 20,20 qnorm 2; splot './results1-10_1-10.txt' using 1:2:5 with lines"
      file.flush();
    }
  }

  file.close();
}
