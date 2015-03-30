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
//  return;

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

  int chainSizeMin = 8;
  int chainSizeMax = 20;
  int chainSizeStep = 1;

  int nodesMin = 10;
  int nodesMax = 10;
  int nodesStep = 1;

  int runs = 5;

  bool tree = false;

  ss.str("");
  ss << "/home/sni/Desktop/results" << chainSizeMin << "-" << chainSizeMax << "_" << nodesMin << "-" << nodesMax
      << ".txt";
  file.open(ss.str());

  std::string entityType = "EvalEntityType";
  std::string entity = "EvalEntity";
  std::string superValueScope = "SuperEvalValueScope";
  std::string superRepresentation = "SuperRepresentation";

  for (int chainSize = chainSizeMin; chainSize <= chainSizeMax; chainSize += chainSizeStep)
  {
    for (int nodes = nodesMin; nodes <= nodesMax; nodes += nodesStep)
    {
      inputs.clear();
      inputsMin.clear();
      inputsMax.clear();
      outputs.clear();
      outputsMin.clear();
      outputsMax.clear();
      metadatas.clear();
      metadataValues.clear();
      metadataValues2.clear();
      metadataGroundings.clear();

      std::cout << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------"
          << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------"
          << std::endl;
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
//            metadatas.push_back("Cost");
//            metadataValues.push_back(1);
//            metadataValues2.push_back(1);
//            metadataGroundings.push_back("NodeCostASPGrounding");
            metadatas.push_back("Accuracy");
            metadataValues.push_back(nodesMax - j);
            metadataValues2.push_back(nodesMax - j);
            metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

            oi.addNodeIndividual(node + "Ind", node, system, entity, "", metadatas, metadataValues, metadataValues2,
                                 metadataGroundings);
//            }
          }
          else // if (j < 1)
          {
            oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

            metadatas.push_back("Delay");
            metadataValues.push_back(0);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeDelayASPGrounding");
//            metadatas.push_back("Cost");
//            metadataValues.push_back(1);
//            metadataValues2.push_back(1);
//            metadataGroundings.push_back("NodeCostASPGrounding");
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

//      ss.str("");
//      ss << "sumCost(1," << chainSize << ")";
//      toCheck.push_back(ss.str());

      auto result = mg.testSeries(fileName, &toCheck, runs, true, 3, 21,
          [] (supplementary::ClingWrapper *asp){
//            asp->setParallelMode(4);
//        asp->setSaveProgress(0);
//            asp->setPredefConfiguration(supplementary::PredefinedConfigurations::tweety);
//            asp->setOptStrategie(4);
        //Berkmin|Vmtf|Vsids|Domain|Unit|None
//              asp->setHeuristic("Domain");
          });

      result.print();

      // print to file
      file << chainSize << "\t";
      file << nodes << "\t";
      file << result.numberTotal << "\t";
      file << result.numberSuccessful << "\t";
      file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
          << result.worst.totalTime << "\t";
      file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime
          << "\t" << result.worst.ontologyReadTime << "\t";
      file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
          << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
      file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
          << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
      file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
          << "\t" << result.worst.aspGroundingTime << "\t";
      file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
          << "\t" << result.worst.aspSolvingTime << std::endl;
      file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspSatTime << std::endl;
      file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspUnsatTime << std::endl;
      file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspModelCount << std::endl;

      // gnuplot -persist -e "set hidden3d; set dgrid3d 20,20 qnorm 2; splot './results1-10_1-10.txt' using 1:2:5 with lines"
      file.flush();
    }
  }

  file.close();
}

TEST(EvalModelGeneration, representationTest)
{
//  return;

  std::string path = ros::package::getPath("ice");
  std::vector<std::string> inputs;
  std::vector<int> inputsMin;
  std::vector<int> inputsMax;
  std::vector<std::string> inputsRelated;
  std::vector<int> inputsRelatedMin;
  std::vector<int> inputsRelatedMax;
  std::vector<std::string> outputs;
  std::vector<int> outputsMin;
  std::vector<int> outputsMax;
  std::vector<std::string> metadatas;
  std::vector<int> metadataValues;
  std::vector<int> metadataValues2;
  std::vector<std::string> metadataGroundings;
  std::stringstream ss;

  ofstream file;

  int representationSizeMin = 9;
  int representationSizeMax = 9;
  int representationSizeStep = 1;

  int runs = 10;

  bool tree = false;

  ss.str("");
  ss << "/home/sni/Desktop/results_representations" << representationSizeMin << "-" << representationSizeMax << ".txt";
  file.open(ss.str());

  std::string entityType = "EvalEntityType";
  std::string entity = "EvalEntity";
  std::string superValueScope = "SuperEvalValueScope";
  std::string superRepresentation = "SuperRepresentation";

  for (int reps = representationSizeMin; reps <= representationSizeMax; reps += representationSizeStep)
  {
    inputs.clear();
    inputsMin.clear();
    inputsMax.clear();
    outputs.clear();
    outputsMin.clear();
    outputsMax.clear();
    metadatas.clear();
    metadataValues.clear();
    metadataValues2.clear();
    metadataGroundings.clear();

    std::cout << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Starting eval " << reps << " representations" << std::endl << std::endl;

    std::string system = "EvalSystem";

    ice::OntologyInterface oi(path + "/java/lib/");

    oi.addIRIMapper(path + "/ontology/");
    oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
    oi.loadOntologies();

    std::vector<std::string> scopes;
    oi.addEntityType(entityType, scopes);

    oi.addIndividual(entity, entityType);
    oi.addSystem(system);

    ss.str("");
    ss << "EvalScope";
    std::string scope = ss.str();
    ss.str("");
    ss << "EvalValueScope";
    std::string valueScope = ss.str();

    // add information structure
    oi.addValueScope(superValueScope, valueScope);

    for (int i = 0; i < reps; ++i)
    {
      ss.str("");
      ss << "EvalRepresentation" << i;
      std::string representation = ss.str();
      ss.str("");
      ss << "EvalStream" << i;
      std::string stream = ss.str();

      std::vector<std::string> vec;
      vec.push_back(valueScope);
      oi.addRepresentation(superRepresentation, representation, vec);

      std::vector<std::string> reps;
      reps.push_back(representation);
      oi.addEntityScope(scope, reps);

      oi.addNamedStream(stream, scope, representation);
    }

    scopes.push_back(scope);
    oi.addScopesToEntityType(entityType, scopes);

    outputs.push_back("EvalStream0");
    outputsMin.push_back(1);
    outputsMax.push_back(1);

    metadatas.clear();
    metadataValues.clear();
    metadataValues2.clear();
    metadataGroundings.clear();

    std::string node = "EvalNodeSource";

    oi.addSourceNodeClass(node, outputs, outputsMin, outputsMax);

//    metadatas.push_back("Delay");
//    metadataValues.push_back(5);
//    metadataValues2.push_back(5);
//    metadataGroundings.push_back("NodeDelayFixASPGrounding");
    metadatas.push_back("Cost");
    metadataValues.push_back(1);
    metadataValues2.push_back(1);
    metadataGroundings.push_back("NodeCostASPGrounding");
//    metadatas.push_back("Accuracy");
//    metadataValues.push_back(10);
//    metadataValues2.push_back(10);
//    metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

    oi.addNodeIndividual(node + "Ind", node, system, entity, "", metadatas, metadataValues, metadataValues2,
                         metadataGroundings);

    for (int i = 0; i < reps; ++i)
    {
      ss.str("");
      ss << "EvalStream" << i;
      std::string inputStream = ss.str();

      for (int j = 0; j < reps; ++j)
      {
        if (i == j)
          continue;

        inputs.clear();
        inputsMin.clear();
        inputsMax.clear();
        outputs.clear();
        outputsMin.clear();
        outputsMax.clear();
        metadatas.clear();
        metadataValues.clear();
        metadataValues2.clear();
        metadataGroundings.clear();
        ss.str("");
        ss << "EvalStream" << j;
        std::string outputStream = ss.str();

        inputs.push_back(inputStream);
        inputsMin.push_back(1);
        inputsMax.push_back(1);
        outputs.push_back(outputStream);
        outputsMin.push_back(1);
        outputsMax.push_back(1);

        ss.str("");
        ss << "EvalNode" << i << "_" << j;
        std::string node = ss.str();

        oi.addIroNodeClass(node, inputs, inputsMin, inputsMax, inputsRelated, inputsRelatedMin, inputsRelatedMax,
                           outputs, outputsMin, outputsMax);

//        metadatas.push_back("Delay");
//        metadataValues.push_back(i);
//        metadataValues2.push_back(0);
//        metadataGroundings.push_back("NodeDelayASPGrounding");
        metadatas.push_back("Cost");
        metadataValues.push_back(i+1);
        metadataValues2.push_back(0);
        metadataGroundings.push_back("NodeCostASPGrounding");
//        metadatas.push_back("Accuracy");
//        metadataValues.push_back(0);
//        metadataValues2.push_back(0);
//        metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

        // TODO fix
        oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2,
                             metadataGroundings);

      }
    }

    ss.str("");
    ss << "EvalStream" << reps - 1;
    std::string stream = ss.str();
    std::string reqStream = "Req" + stream;
    oi.addRequiredStream(reqStream, stream, system, entity, "");

    ss.str("");
    ss << "/tmp/evalReps" << reps << ".owl";
    std::string fileName = ss.str();
    oi.saveOntology(fileName);

    ModelGeneration mg(path);

    std::vector<std::string> toCheck;

//    ss.str("");
//    ss << "metadataStream(1,accuracy,stream(1,evalSystem,evalNode0_" << reps - 1
//        << "Ind,evalSystem,information(evalEntity,evalScope,evalRepresentation" << reps-1 << ",none),2),10)";
//    toCheck.push_back(ss.str());

//    ss.str("");
//    ss << "metadataStream(1,delay,stream(1,evalSystem,evalNode0_" << reps - 1
//        << "Ind,evalSystem,information(evalEntity,evalScope,evalRepresentation" << reps-1 << ",none),2),5)";
//    toCheck.push_back(ss.str());

    ss.str("");
    ss << "selectedStream(1,evalSystem,evalNode0_" << reps - 1 <<
        "Ind,evalSystem,information(evalEntity,evalScope,evalRepresentation" << reps - 1 << ",none),2)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "node(1,evalSystem,evalNode0_" << reps - 1 << "Ind,evalEntity,none)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "sumCost(1,2)";
    toCheck.push_back(ss.str());

    auto result = mg.testSeries(fileName, &toCheck, runs, true, 3, reps+1,
        [] (supplementary::ClingWrapper *asp){
//        asp->setSaveProgress(200);
//      asp->add("base",{},"stream(1,evalSystem,evalNodeSourceInd,evalSystem,information(evalEntity,evalScope,evalRepresentation0,none)).");
//      asp->setPredefConfiguration(supplementary::PredefinedConfigurations::tweety);
    });

    result.print();

    // print to file
    file << reps << "\t";
    file << result.numberTotal << "\t";
    file << result.numberSuccessful << "\t";
    file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
        << result.worst.totalTime << "\t";
    file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime
        << "\t" << result.worst.ontologyReadTime << "\t";
    file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
        << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
    file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t" << result.best.ontologyToASPTime
        << "\t" << result.worst.ontologyToASPTime << "\t";
    file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
        << "\t" << result.worst.aspGroundingTime << "\t";
    file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime << "\t"
        << result.worst.aspSolvingTime << std::endl;
    file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspSatTime << std::endl;
    file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspUnsatTime << std::endl;
    file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspModelCount << std::endl;

    // gnuplot -persist -e "plot './results3-10.txt' using 1:4 with lines"
    file.flush();
  }

  file.close();
}

TEST(EvalModelGeneration, systemsStarMashTest)
{
  return;

  std::string path = ros::package::getPath("ice");
  std::vector<std::string> inputs;
  std::vector<int> inputsMin;
  std::vector<int> inputsMax;
  std::vector<std::string> inputsRelated;
  std::vector<int> inputsRelatedMin;
  std::vector<int> inputsRelatedMax;
  std::vector<std::string> outputs;
  std::vector<int> outputsMin;
  std::vector<int> outputsMax;
  std::vector<std::string> metadatas;
  std::vector<int> metadataValues;
  std::vector<int> metadataValues2;
  std::vector<std::string> metadataGroundings;
  std::stringstream ss;

  ofstream file;

  int systemSizeMin = 50;
  int systemSizeMax = 50;
  int systemSizeStep = 10;

  int runs = 40;

  bool tree = false;

  ss.str("");
  ss << "/home/sni/Desktop/results_systems" << systemSizeMin << "-" << systemSizeMax << ".txt";
  file.open(ss.str());

  std::string entityType = "EvalEntityType";
  std::string entity = "EvalEntity";
  std::string superValueScope = "SuperEvalValueScope";
  std::string superRepresentation = "SuperRepresentation";

  for (int systems = systemSizeMin; systems <= systemSizeMax; systems += systemSizeStep)
  {
    inputs.clear();
    inputsMin.clear();
    inputsMax.clear();
    outputs.clear();
    outputsMin.clear();
    outputsMax.clear();
    metadatas.clear();
    metadataValues.clear();
    metadataValues2.clear();
    metadataGroundings.clear();

    std::cout << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Starting eval " << systems << " systems" << std::endl << std::endl;


    ice::OntologyInterface oi(path + "/java/lib/");

    oi.addIRIMapper(path + "/ontology/");
    oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
    oi.loadOntologies();

    // add information structure
    std::vector<std::string> scopes;


    ss.str("");
    ss << "EvalScope";
    std::string scope = ss.str();
    ss.str("");
    ss << "EvalValueScope";
    std::string valueScope = ss.str();

    oi.addValueScope(superValueScope, valueScope);

    std::vector<std::string> reps;
    ss.str("");
    ss << "ReqRepresentation";
    std::string representation = ss.str();

    std::vector<std::string> vec;
    vec.push_back(valueScope);
    oi.addRepresentation(superRepresentation, representation, vec);
    reps.push_back(representation);

    ss.str("");
    ss << "EvalRepresentation";
    representation = ss.str();

    oi.addRepresentation(superRepresentation, representation, vec);
    reps.push_back(representation);

    oi.addEntityScope(scope, reps);

    scopes.push_back(scope);
    oi.addEntityType(entityType, scopes);
    oi.addIndividual(entity, entityType);

    // Add source stream
    ss.str("");
    ss << "EvalSourceStream";
    std::string stream = ss.str();

    oi.addNamedStream(stream, scope, representation);

    // Add eval node
    outputs.push_back("EvalSourceStream");
    outputsMin.push_back(1);
    outputsMax.push_back(1);

    std::string node = "EvalNodeSource";
    oi.addSourceNodeClass(node, outputs, outputsMin, outputsMax);

    // Add req stream
    stream = "EvalStream";
//    reps.clear();
//    reps.push_back(representation);
//    oi.addEntityScope(scope, reps);

    oi.addNamedStream(stream, scope, "ReqRepresentation");


    for (int i = 0; i < systems; ++i)
    {
      ss.str("");
      ss << "EvalNode" << i;
      node = ss.str();
      inputs.clear();
      inputsMin.clear();
      inputsMax.clear();
      outputs.clear();
      outputsMin.clear();
      outputsMax.clear();
      metadatas.clear();
      metadataValues.clear();
      metadataValues2.clear();
      metadataGroundings.clear();

      ss.str("");
      ss << "EvalSystem" << i;
      std::string system = ss.str();
      oi.addSystem(system);

      if (i == 0)
      {
        inputs.push_back("EvalSourceStream");
        inputsMin.push_back(2);
        inputsMax.push_back(2);
        outputs.push_back("EvalStream");
        outputsMin.push_back(1);
        outputsMax.push_back(1);

        oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

        metadatas.push_back("Delay");
        metadataValues.push_back(1);
        metadataValues2.push_back(1);
        metadataGroundings.push_back("NodeDelayASPGrounding");
        metadatas.push_back("Cost");
        metadataValues.push_back(1);
        metadataValues2.push_back(1);
        metadataGroundings.push_back("NodeCostASPGrounding");
//        metadatas.push_back("Accuracy");
//        metadataValues.push_back(0);
//        metadataValues2.push_back(0);
//        metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

        oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2,
                             metadataGroundings);

      } else {
          metadatas.push_back("Delay");
          metadataValues.push_back(i);
          metadataValues2.push_back(i);
          metadataGroundings.push_back("NodeDelayFixASPGrounding");
          metadatas.push_back("Cost");
          metadataValues.push_back(1);
          metadataValues2.push_back(1);
          metadataGroundings.push_back("NodeCostASPGrounding");
//          metadatas.push_back("Accuracy");
//          metadataValues.push_back(10);
//          metadataValues2.push_back(10);
//          metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

          oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues, metadataValues2,
                               metadataGroundings);
      }

      if (i == 0)
      {
        ss.str("");
        ss << "Req" << stream << i;
        oi.addRequiredStream(ss.str(), stream, system, entity, "");
      }
    }

    ss.str("");
    ss << "/tmp/evalSystems" << systems << ".owl";
    std::string fileName = ss.str();
    oi.saveOntology(fileName);

    ModelGeneration mg(path);

    std::vector<std::string> toCheck;

//    ss.str("");
//    ss << "metadataStream(1,accuracy,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
//        "information(evalEntity,evalScope,reqRepresentation,none),3),10)";
//    toCheck.push_back(ss.str());

    ss.str("");
    ss << "metadataStream(1,delay,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
        "information(evalEntity,evalScope,reqRepresentation,none),3),7)";
    toCheck.push_back(ss.str());


    ss.str("");
    ss << "stream(1,evalSystem0,evalNode1SourceInd,evalSystem1,information(evalEntity,evalScope,evalRepresentation,none),2)";
    toCheck.push_back(ss.str());
    ss.str("");
    ss << "stream(1,evalSystem0,evalNode2SourceInd,evalSystem2,information(evalEntity,evalScope,evalRepresentation,none),2)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "selectedStream(1,evalSystem0,evalNode0Ind,evalSystem0,information(evalEntity,evalScope,reqRepresentation,none),3)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "sumCost(1,3)";
    toCheck.push_back(ss.str());

    auto result = mg.testSeries(fileName, &toCheck, runs, true, 2, 10,
        [&] (supplementary::ClingWrapper *asp){

      for (int i=1; i < systems; ++i)
      {
        ss.str("");
        ss << "transfer(evalSystem" << i << ",evalSystem0," << 2 << "," << 2 << ").";
        asp->add("base",{},ss.str());
        ss.str("");
        ss << "transfer(evalSystem0,evalSystem" << i << "," << 2 << "," << 2 << ").";
        asp->add("base",{},ss.str());
      }

//      asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
//      asp->setOptStrategie(3);
    });

    result.print();

    // print to file
    file << systems << "\t";
    file << result.numberTotal << "\t";
    file << result.numberSuccessful << "\t";
    file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
        << result.worst.totalTime << "\t";
    file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime
        << "\t" << result.worst.ontologyReadTime << "\t";
    file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
        << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
    file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t" << result.best.ontologyToASPTime
        << "\t" << result.worst.ontologyToASPTime << "\t";
    file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
        << "\t" << result.worst.aspGroundingTime << "\t";
    file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime << "\t"
        << result.worst.aspSolvingTime << std::endl;
    file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspSatTime << std::endl;
    file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspUnsatTime << std::endl;
    file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspModelCount << std::endl;

    // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
    file.flush();
  }

  file.close();
}

TEST(EvalModelGeneration, systemsFullMashTest)
{
  return;

  std::string path = ros::package::getPath("ice");
  std::vector<std::string> inputs;
  std::vector<int> inputsMin;
  std::vector<int> inputsMax;
  std::vector<std::string> inputsRelated;
  std::vector<int> inputsRelatedMin;
  std::vector<int> inputsRelatedMax;
  std::vector<std::string> outputs;
  std::vector<int> outputsMin;
  std::vector<int> outputsMax;
  std::vector<std::string> metadatas;
  std::vector<int> metadataValues;
  std::vector<int> metadataValues2;
  std::vector<std::string> metadataGroundings;
  std::stringstream ss;

  ofstream file;

  int systemSizeMin = 3;
  int systemSizeMax = 10;
  int systemSizeStep = 1;

  int runs = 1;

  bool tree = false;

  ss.str("");
  ss << "/home/sni/Desktop/results_systemsFullMash" << systemSizeMin << "-" << systemSizeMax << ".txt";
  file.open(ss.str());

  std::string entityType = "EvalEntityType";
  std::string entity = "EvalEntity";
  std::string superValueScope = "SuperEvalValueScope";
  std::string superRepresentation = "SuperRepresentation";

  for (int systems = systemSizeMin; systems <= systemSizeMax; systems += systemSizeStep)
  {
    inputs.clear();
    inputsMin.clear();
    inputsMax.clear();
    outputs.clear();
    outputsMin.clear();
    outputsMax.clear();
    metadatas.clear();
    metadataValues.clear();
    metadataValues2.clear();
    metadataGroundings.clear();

    std::cout << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Starting eval " << systems << " systems full mash" << std::endl << std::endl;


    ice::OntologyInterface oi(path + "/java/lib/");

    oi.addIRIMapper(path + "/ontology/");
    oi.addOntologyIRI("http://www.semanticweb.org/sni/ontologies/2013/7/Ice");
    oi.loadOntologies();

    // add information structure
    std::vector<std::string> scopes;


    ss.str("");
    ss << "EvalScope";
    std::string scope = ss.str();
    ss.str("");
    ss << "EvalValueScope";
    std::string valueScope = ss.str();

    oi.addValueScope(superValueScope, valueScope);

    std::vector<std::string> reps;
    ss.str("");
    ss << "ReqRepresentation";
    std::string representation = ss.str();

    std::vector<std::string> vec;
    vec.push_back(valueScope);
    oi.addRepresentation(superRepresentation, representation, vec);
    reps.push_back(representation);

    ss.str("");
    ss << "EvalRepresentation";
    representation = ss.str();

    oi.addRepresentation(superRepresentation, representation, vec);
    reps.push_back(representation);

    oi.addEntityScope(scope, reps);

    scopes.push_back(scope);
    oi.addEntityType(entityType, scopes);
    oi.addIndividual(entity, entityType);

    // Add source stream
    ss.str("");
    ss << "EvalSourceStream";
    std::string stream = ss.str();

    oi.addNamedStream(stream, scope, representation);

    // Add eval node
    outputs.push_back("EvalSourceStream");
    outputsMin.push_back(1);
    outputsMax.push_back(1);

    std::string node = "EvalNodeSource";
    oi.addSourceNodeClass(node, outputs, outputsMin, outputsMax);

    // Add req stream
    stream = "EvalStream";
//    reps.clear();
//    reps.push_back(representation);
//    oi.addEntityScope(scope, reps);

    oi.addNamedStream(stream, scope, "ReqRepresentation");

    inputs.clear();
    inputsMin.clear();
    inputsMax.clear();
    outputs.clear();
    outputsMin.clear();
    outputsMax.clear();

    inputs.push_back("EvalSourceStream");
    inputsMin.push_back(2);
    inputsMax.push_back(2);
    outputs.push_back("EvalStream");
    outputsMin.push_back(1);
    outputsMax.push_back(1);

    oi.addComputationNodeClass("EvalNode", inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

    for (int i = 0; i < systems; ++i)
    {
      ss.str("");
      ss << "EvalNode" << i;
      node = ss.str();
      metadatas.clear();
      metadataValues.clear();
      metadataValues2.clear();
      metadataGroundings.clear();

      ss.str("");
      ss << "EvalSystem" << i;
      std::string system = ss.str();
      oi.addSystem(system);

      if (i < 1000)
      {

        metadatas.push_back("Delay");
        metadataValues.push_back(1);
        metadataValues2.push_back(1);
        metadataGroundings.push_back("NodeDelayASPGrounding");
        metadatas.push_back("Cost");
        metadataValues.push_back(1);
        metadataValues2.push_back(1);
        metadataGroundings.push_back("NodeCostASPGrounding");
//        metadatas.push_back("Accuracy");
//        metadataValues.push_back(0);
//        metadataValues2.push_back(0);
//        metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

        oi.addNodeIndividual(node + "Ind", "EvalNode", system, "", "", metadatas, metadataValues, metadataValues2,
                             metadataGroundings);
      }
//      } else {
        metadatas.clear();
        metadataValues.clear();
        metadataValues2.clear();
        metadataGroundings.clear();

          metadatas.push_back("Delay");
          metadataValues.push_back(i);
          metadataValues2.push_back(i);
          metadataGroundings.push_back("NodeDelayFixASPGrounding");
          metadatas.push_back("Cost");
          metadataValues.push_back(1);
          metadataValues2.push_back(1);
          metadataGroundings.push_back("NodeCostASPGrounding");
//          metadatas.push_back("Accuracy");
//          metadataValues.push_back(10);
//          metadataValues2.push_back(10);
//          metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

          oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues, metadataValues2,
                               metadataGroundings);
//      }

      if (i < 10)
      {
        ss.str("");
        ss << "Req" << stream << i;
        oi.addRequiredStream(ss.str(), stream, system, entity, "");
      }
    }

    ss.str("");
    ss << "/tmp/evalSystemsFullMash" << systems << ".owl";
    std::string fileName = ss.str();
    oi.saveOntology(fileName);

    ModelGeneration mg(path);

    std::vector<std::string> toCheck;

//    ss.str("");
//    ss << "metadataStream(1,accuracy,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
//        "information(evalEntity,evalScope,reqRepresentation,none),3),10)";
//    toCheck.push_back(ss.str());

    ss.str("");
    ss << "metadataStream(1,delay,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
        "information(evalEntity,evalScope,reqRepresentation,none),3),7)";
    toCheck.push_back(ss.str());


    ss.str("");
    ss << "stream(1,evalSystem0,evalNode1SourceInd,evalSystem1,information(evalEntity,evalScope,evalRepresentation,none),2)";
    toCheck.push_back(ss.str());
    ss.str("");
    ss << "stream(1,evalSystem0,evalNode2SourceInd,evalSystem2,information(evalEntity,evalScope,evalRepresentation,none),2)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "selectedStream(1,evalSystem0,evalNode0Ind,evalSystem0,information(evalEntity,evalScope,reqRepresentation,none),3)";
    toCheck.push_back(ss.str());

    ss.str("");
    ss << "sumCost(1,3)";
    toCheck.push_back(ss.str());

    auto result = mg.testSeries(fileName, &toCheck, runs, true, 2, 10,
        [&] (supplementary::ClingWrapper *asp){

      for (int i=0; i < systems; ++i)
      {
        for (int j=0; j < systems; ++j)
        {
          if (i==j)
            continue;
        ss.str("");
        ss << "transfer(evalSystem" << j << ",evalSystem" << i << "," << 2 << "," << 2 << ").";
        asp->add("base",{},ss.str());

        ss.str("");
        ss << "system(evalSystem" << i << ").";
        asp->add("base",{},ss.str());
        }
      }

//      asp->setPredefConfiguration(supplementary::PredefinedConfigurations::jumpy);
//      asp->setOptStrategie(3);
    });

    result.print();

    // print to file
    file << systems << "\t";
    file << result.numberTotal << "\t";
    file << result.numberSuccessful << "\t";
    file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
        << result.worst.totalTime << "\t";
    file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime
        << "\t" << result.worst.ontologyReadTime << "\t";
    file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
        << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
    file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t" << result.best.ontologyToASPTime
        << "\t" << result.worst.ontologyToASPTime << "\t";
    file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
        << "\t" << result.worst.aspGroundingTime << "\t";
    file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime << "\t"
        << result.worst.aspSolvingTime << std::endl;
    file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspSatTime << std::endl;
    file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspUnsatTime << std::endl;
    file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
        << "\t" << result.worst.aspModelCount << std::endl;

    // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
    file.flush();
  }

  file.close();
}

