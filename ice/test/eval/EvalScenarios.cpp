#include <gtest/gtest.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "ModelGeneration.cpp"

class EvalScenarios {
public:
  EvalScenarios(std::string logPath, std::function<void(supplementary::ClingWrapper *asp)> lambda = nullptr) {
    this->logPath = logPath;
    this->lambda = lambda;
  }

  void chainScenario(bool global, bool verbose, bool gnuplot, int runs, int chainSizeMin, int chainSizeMax,
                 int chainSizeStep, int nodesMin, int nodesMax, int nodesStep)
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

    ss.str("");
    ss << this->logPath << "/chainScenario_" << (global ? "global" : "local") << "_"
        << chainSizeMin << "-" << chainSizeMax << "_" << nodesMin << "-" << nodesMax << ".txt";
    std::string logFile = ss.str();
    file.open(logFile);

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

        if (global)
        {
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
        }

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

        auto result = mg.testSeries(fileName, &toCheck, runs, true, global, verbose, 3, chainSize,
            [this] (supplementary::ClingWrapper *asp){
          this->lambda(asp);
  //            asp->setParallelMode(4);
  //        asp->setSaveProgress(0);
//              asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
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
            << "\t" << result.worst.aspSolvingTime;
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspSatTime;
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspUnsatTime;
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspModelCount << std::endl;

        // gnuplot -persist -e "set hidden3d; set dgrid3d 20,20 qnorm 2; splot './results1-10_1-10.txt' using 1:2:5 with lines"
        file.flush();
      }
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    if (nodesMin == nodesMax)
    {
      ss <<  "gnuplot -persist -e \"set title '" << logFile << "'; plot '"
      << logFile << "' u 1:5 w l t 'sum', '"
      << logFile << "' u 1:21 w l t 'grounding', '"
      << logFile << "' u 1:(\\$21 + \\$25) w l t 'solving'\"";
    }else {
    ss <<  "gnuplot -persist -e \"set title '" << logFile << "';set hidden3d; set dgrid3d 20,20 qnorm 2; splot '"
        << logFile << "' using 1:2:5 with lines\"";
    }
    system(ss.str().c_str());
  }

  void representationScenario(bool global, bool verbose, bool gnuplot, int runs, int representationSizeMin,
                              int representationSizeMax, int representationSizeStep)
  {
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

      ss.str("");
      ss << this->logPath << "/representationScenario_" << (global ? "global" : "local") << "_"
          << representationSizeMin << "-" << representationSizeMax << ".txt";
      std::string logFile = ss.str();
      file.open(logFile);

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

//        metadatas.push_back("Delay");
//        metadataValues.push_back(5);
//        metadataValues2.push_back(5);
//        metadataGroundings.push_back("NodeDelayFixASPGrounding");
        metadatas.push_back("Cost");
        metadataValues.push_back(1);
        metadataValues2.push_back(1);
        metadataGroundings.push_back("NodeCostASPGrounding");
//        metadatas.push_back("Accuracy");
//        metadataValues.push_back(10);
//        metadataValues2.push_back(10);
//        metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

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

//            metadatas.push_back("Delay");
//            metadataValues.push_back(i);
//            metadataValues2.push_back(0);
//            metadataGroundings.push_back("NodeDelayASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
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

        if (global)
        {
      //    ss.str("");
      //    ss << "metadataStream(1,accuracy,stream(1,evalSystem,evalNode0_" << reps - 1
      //        << "Ind,evalSystem,information(evalEntity,evalScope,evalRepresentation" << reps-1 << ",none),2),10)";
      //    toCheck.push_back(ss.str());

//          ss.str("");
//          ss << "metadataStream(1,delay,stream(1,evalSystem,evalNode0_" << reps - 1
//              << "Ind,evalSystem,information(evalEntity,evalScope,evalRepresentation" << reps-1 << ",none),2),5)";
//          toCheck.push_back(ss.str());
        }

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

        auto result = mg.testSeries(fileName, &toCheck, runs, true, global, verbose, 3, reps+1,
            [this] (supplementary::ClingWrapper *asp){
          this->lambda(asp);
          asp->setModelCount(0);
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
            << result.worst.aspSolvingTime;
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspSatTime;
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspUnsatTime;
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspModelCount << std::endl;

        // gnuplot -persist -e "plot './results3-10.txt' using 1:4 with lines"
        file.flush();
      }

      file.close();

      if (!gnuplot)
        return;

      ss.str("");
      ss << "gnuplot -persist -e \"set title '" << logFile << "'; plot '"
          << logFile << "' u 1:4 w l t 'sum', '"
          << logFile << "' u 1:20 w l t 'grounding', '"
          << logFile << "' u 1:(\\$20 + \\$24) w l t 'solving'\"";
//          << logFile << "' using 1:4 with lines\"";
      system(ss.str().c_str());
  }

  void systemsStarMashScenario(bool global, bool verbose, bool gnuplot, int runs, int systemSizeMin,
                              int systemSizeMax, int systemSizeStep)
  {
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

      ss.str("");
      ss << this->logPath << "/systemsStarMashScenario_" << (global ? "global" : "local") << "_"
          << systemSizeMin << "-" << systemSizeMax << ".txt";
      std::string pathStr = ss.str();
      file.open(pathStr);

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
              metadataValues.push_back(i);//(i < 3 ? i+2 : 5));
              metadataValues2.push_back(i);//(i < 3 ? i+2 : 5));
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

        if (global)
        {
          ss.str("");
          ss << "metadataStream(1,delay,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
              "information(evalEntity,evalScope,reqRepresentation,none),3),7)";
          toCheck.push_back(ss.str());
        }

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

        auto result = mg.testSeries(fileName, &toCheck, runs, true, global, verbose, 3, 10,
            [&] (supplementary::ClingWrapper *asp){
          this->lambda(asp);

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
            << result.worst.aspSolvingTime;
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspSatTime;
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspUnsatTime;
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
            << "\t" << result.worst.aspModelCount << std::endl;

        // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
        file.flush();
      }

      file.close();

      if (!gnuplot)
        return;

      ss.str("");
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '"
          << pathStr << "' u 1:4 w l t 'sum', '"
          << pathStr << "' u 1:20 w l t 'grounding', '"
          << pathStr << "' u 1:(\\$20 + \\$24) w l t 'solving'\"";
      system(ss.str().c_str());
  }

  void systemsFullMashScenario(bool global, bool verbose, bool gnuplot, int runs, int systemSizeMin,
                              int systemSizeMax, int systemSizeStep)
  {
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

    bool tree = false;

    ss.str("");
    ss << this->logPath << "/systemsFullMashScenario_" << (global ? "global" : "local") << "_"
        << systemSizeMin << "-" << systemSizeMax << ".txt";
    std::string pathStr = ss.str();
    file.open(pathStr);

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
      inputsMin.push_back(3);
      inputsMax.push_back(3);
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

        if (i > -1)
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

        if (i < 1)
        {
          ss.str("");
          ss << "Req" << stream << i;
          oi.addRequiredStream(ss.str(), stream, system, entity, "");
        }
      }

      ss.str("");
      ss << "/tmp/evalSystemsStarMash2" << systems << ".owl";
      std::string fileName = ss.str();
      oi.saveOntology(fileName);

      ModelGeneration mg(path);

      std::vector<std::string> toCheck;

  //    ss.str("");
  //    ss << "metadataStream(1,accuracy,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
  //        "information(evalEntity,evalScope,reqRepresentation,none),3),10)";
  //    toCheck.push_back(ss.str());

      if (global)
      {
        ss.str("");
        ss << "metadataStream(1,delay,stream(1,evalSystem0,evalNode0Ind,evalSystem0," <<
            "information(evalEntity,evalScope,reqRepresentation,none),3),8)";
        toCheck.push_back(ss.str());
      }

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
      ss << "sumCost(1,4)";
      toCheck.push_back(ss.str());

      auto result = mg.testSeries(fileName, &toCheck, runs, true, global, verbose, 3, 10,
          [&] (supplementary::ClingWrapper *asp){
        this->lambda(asp);

        for (int i=0; i < systems; ++i)
        {
          for (int j=0; j < systems; ++j)
          {
            if (i==j)
              continue;
          ss.str("");
          ss << "transfer(evalSystem" << j << ",evalSystem" << i << "," << 2 << "," << 2 << ").";
          asp->add("base",{},ss.str());
          }

          ss.str("");
          ss << "system(evalSystem" << i << ").";
          asp->add("base",{},ss.str());
        }

//        asp->setPredefConfiguration(supplementary::PredefinedConfigurations::trendy);
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
          << result.worst.aspSolvingTime;
      file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspSatTime;
      file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspUnsatTime;
      file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspSatTime
          << "\t" << result.worst.aspModelCount << std::endl;

      // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
      file.flush();
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '"
        << pathStr << "' u 1:4 w l t 'sum', '"
        << pathStr << "' u 1:20 w l t 'grounding', '"
        << pathStr << "' u 1:(\\$20 + \\$24) w l t 'solving'\"";

    system(ss.str().c_str());
  }

  private:
    std::string logPath;
    std::function<void(supplementary::ClingWrapper *asp)> lambda;
};
