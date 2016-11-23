#include <gtest/gtest.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <tuple>
#include "ModelGeneration.cpp"

struct TConf {
  int parallelGroupsMin;
  int parallelGroupsMax;
  int parallelGrounsStep;
  std::vector<int> levels;
  int inputsMin;
  int inputsMax;
  bool skipLevel;

  std::string toString() {
    std::stringstream ss;

    ss.str("");
    ss << "[";
    for (int i = 0; i < levels.size(); ++i)
    {
      if (i > 0)
        ss << ",";
      ss << levels[i];
    }
    ss << "]";

    if (skipLevel)
      ss << "_skip";

    ss << "_G(" << parallelGroupsMin << "," << parallelGroupsMax << "," << parallelGrounsStep << ")";
    ss << "_IN(" << inputsMin << "," << inputsMax << ")";

    return ss.str();
  }
};

class EvalScenarios
{
public:
  EvalScenarios(std::string logPath, bool warmUp, std::function<void(supplementary::ClingWrapper *asp)> lambda = nullptr)
  {
    this->warmUp = warmUp;
    this->logPath = logPath;
    this->lambda = lambda;
  }

  void transformation(bool verbose, bool gnuplot, int runs, int models, TConf &conf)
  {
    std::string path = ros::package::getPath("ice");
    std::stringstream ss;
    std::vector<int> usedDims;

    ofstream file;

    ss.str("");
    ss << "[";

    for (int i = 0; i < conf.levels.size(); ++i)
    {
      if (i > 0)
        ss << ",";
      ss << conf.levels[i];
    }

    ss << "]";

    std::string levelStr = ss.str();

    ss.str("");
    ss << this->logPath << "/representations_" << conf.toString() << ".txt";
    std::string logFile = ss.str();
    file.open(logFile);

    std::cout << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------"
        << std::endl;
    std::cout << "---------------------------------------------------------------------------------------------------"
        << std::endl;
    std::cout << "Starting eval " << conf.toString() << std::endl;

    std::string systemStr = "EvalSystem";

    for (int pg=conf.parallelGroupsMin; pg <= conf.parallelGroupsMax; pg += conf.parallelGrounsStep)
    {
      ss.str("");
      ss << "/tmp/representations_" << conf.toString() << "_" << pg << "_";
      std::string owlPath = ss.str();
      ss.str("");

      for (int m=0; m < models; ++m)
      {
        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
        oi.loadOntologies();

        std::string entityType = "EvalEntityType";
        std::string entity = "EvalEntity";

        std::vector<std::string> scopes;
        oi.addEntityType(entityType, scopes);

        oi.addIndividual(entity, entityType);
        oi.addSystem(systemStr);

        std::vector<std::tuple<std::string,std::string>> dimensions;
        std::vector<std::tuple<std::string,std::string>> dimensionsNew;

        for (int g=0; g < pg; ++g)
        {
          std::string superValueScope = "SuperEvalValueScope_" + std::to_string(g);

          for (int i = 0; i < conf.levels.size(); ++i)
          {

            if (i == 0)
            {
              for (int j = 0; j < conf.levels.at(0); ++j)
              {
                ss.str("");
                ss << "EvalValueScope" << g << "_" << j;
                std::string valueScope = ss.str();

                // add information structure
                oi.addValueScope(superValueScope, valueScope, "DoubleRep");
                std::tuple<std::string, std::string> d(valueScope,"");
                dimensions.push_back(d);
              }

              continue;
            }

            ss.str("");
            ss << "EvalScope" << g << "_" << i;
            std::string scope = ss.str();
            std::string superRepresentation = "SuperRepresentation_" + std::to_string(g) + "_" + std::to_string(i);

            for (int j = 0; j < conf.levels.at(i); ++j)
            {
              ss.str("");
              ss << "EvalRepresentation" << g << "_" << i << "_" << j;
              std::string representation = ss.str();
              ss.str("");

              std::vector<std::string> vec;
              int count = (std::rand() % (conf.inputsMax - conf.inputsMin + 1)) + conf.inputsMin;

              oi.addRepresentation(superRepresentation, representation, vec);
              usedDims.clear();

              for (int k = 0; k < count; ++k)
              {

                int index;

                do {
                  index = std::rand() % dimensions.size();
                } while (std::find(usedDims.begin(), usedDims.end(), index) != usedDims.end());


                auto d = dimensions.at(index);
                if (std::get<1>(d) == "")
                {
                  oi.addDimensionToRep(representation, std::get<0>(d));
                }
                else
                {
                  oi.addDimensionToRep(representation, std::get<0>(d), std::get<1>(d));
                }
              }

              std::tuple<std::string, std::string> d(representation,scope);
              dimensionsNew.push_back(d);
            }

            std::vector<std::string> reps;
            reps.push_back(superRepresentation);
            oi.addEntityScope(scope, reps);

            std::vector<std::string> scopes;
            scopes.push_back(scope);
            oi.addScopesToEntityType(entityType, scopes);

            if (conf.skipLevel)
            {
              dimensions.insert(dimensions.end(), dimensionsNew.begin(), dimensionsNew.end());
              dimensionsNew.clear();
            }
            else
            {
              dimensions = dimensionsNew;
              dimensionsNew.clear();
            }
          }
          dimensions.clear();
          dimensionsNew.clear();
        }

        ss.str("");
        ss << owlPath << m << ".owl";
        std::string fileName = ss.str();
        oi.saveOntology(fileName);
        std::cout << fileName << std::endl;
      }

      std::vector<std::string> toCheck;
      ModelGeneration mg(path, true);

      auto result = mg.testSeries(owlPath, &toCheck, runs, this->warmUp, true, verbose, 3, 10,
                                  [this] (supplementary::ClingWrapper *asp)
                                  {
                                    this->lambda(asp);
                                  }, models);

      result.print();

      // print to file
      file << pg << "\t";
      file << conf.inputsMin << "\t";
      file << conf.inputsMax << "\t";
      file << result.numberTotal << "\t";
      file << result.numberSuccessful << "\t";
      file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
          << result.worst.totalTime << "\t";
      // 9
      file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t" << result.best.ontologyReadTime
          << "\t" << result.worst.ontologyReadTime << "\t";
      file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
          << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
      file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t" << result.best.ontologyToASPTime
          << "\t" << result.worst.ontologyToASPTime << "\t";
      // 22
      file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
          << "\t" << result.worst.aspGroundingTime << "\t";
      // 26
      file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime << "\t"
          << result.worst.aspSolvingTime << "\t";
      file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
          << result.worst.aspSatTime << "\t";
      file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
          << result.worst.aspUnsatTime << "\t";
      file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
          << result.worst.aspModelCount << "\t";
      file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
          << result.worst.aspAtomCount << "\t";
      file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount << "\t"
          << result.worst.aspBodiesCount << "\t";
      file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
          << "\t" << result.worst.aspAuxAtomCount << std::endl;

      // gnuplot -persist -e "set hidden3d; set dgrid3d 20,20 qnorm 2; splot './results1-10_1-10.txt' using 1:2:5 with lines"
      file.flush();
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");

    ss << "gnuplot -persist -e \"set title '" << conf.toString() << "'; plot '" << logFile << "' u 1:6 w l t 'sum', '"
        << logFile << "' u 1:22 w l t 'grounding', '" << logFile << "' u 1:(\\$22 + \\$26) w l t 'solving'\"";

    system(ss.str().c_str());
  }

  void chainScenario(bool global, bool verbose, bool gnuplot, int runs, int chainSizeMin, int chainSizeMax,
                     int chainSizeStep, int nodesMin, int nodesMax, int nodesStep, bool createOntology)
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
    ss << this->logPath << "/chainScenario_" << (global ? "global" : "local") << "_" << chainSizeMin << "-"
        << chainSizeMax << "_" << nodesMin << "-" << nodesMax << ".txt";
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
        ss.str("");
        ss << "/tmp/chainScenario_" << (global ? "global" : "local") << chainSize << "_" << nodes << ".owl";
        std::string fileName = ss.str();

        if (createOntology)
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

        if (verbose)
        {
          std::cout << std::endl;
          std::cout
              << "---------------------------------------------------------------------------------------------------"
              << std::endl;
          std::cout
              << "---------------------------------------------------------------------------------------------------"
              << std::endl;
          std::cout << "Starting eval " << chainSize << " chain size, " << nodes << " nodes" << std::endl << std::endl;
        }

        std::string system = "EvalSystem";

        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
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
          oi.addValueScope(superValueScope, valueScope, "DoubleRep");

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
              metadataValues.push_back(1);
              metadataValues2.push_back(0);
              metadataGroundings.push_back("NodeDelayFixASPGrounding");
              //            metadatas.push_back("Cost");
              //            metadataValues.push_back(1);
              //            metadataValues2.push_back(1);
              //            metadataGroundings.push_back("NodeCostASPGrounding");
              metadatas.push_back("Accuracy");
              metadataValues.push_back(nodesMax - j);
              metadataValues2.push_back(0);
              metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

              oi.addNodeIndividual(node + "Ind", node, system, entity, "", metadatas, metadataValues, metadataValues2,
                                   metadataGroundings);
              //            }
            }
            else // if (j < 1)
            {
              oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

              metadatas.push_back("Delay");
              metadataValues.push_back(1);
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
        oi.saveOntology(fileName);
        }

        ModelGeneration mg(path);

        std::vector<std::string> toCheck;

        if (global)
        {
          ss.str("");
          ss << "metadataStream(1,accuracy,stream(1,o0_EvalSystem,node(1,o0_EvalSystem,o0_EvalNode0_" << chainSize - 1
              << "Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope" << chainSize << "_" << chainSize - 1
              << ",o0_EvalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << "),"
              << chainSize * nodesMax << ")";
          toCheck.push_back(ss.str());

          ss.str("");
          ss << "metadataStream(1,delay,stream(1,o0_EvalSystem,node(1,o0_EvalSystem,o0_EvalNode0_" << chainSize - 1
              << "Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope" << chainSize << "_" << chainSize - 1
              << ",o0_EvalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << "),"
              << (chainSize) << ")";
          toCheck.push_back(ss.str());
        }

        ss.str("");
        ss << "selectedStream(1,o0_EvalSystem,node(1,o0_EvalSystem,o0_EvalNode0_" << chainSize - 1
            << "Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope" << chainSize << "_" << chainSize - 1
            << ",o0_EvalRepresentation" << chainSize << "_" << chainSize - 1 << ",none)," << chainSize << ")";
        toCheck.push_back(ss.str());

        ss.str("");
        ss << "node(1,o0_EvalSystem,o0_EvalNode0_" << chainSize - 1 << "Ind,o0_EvalEntity,none)";
        toCheck.push_back(ss.str());

        //      ss.str("");
        //      ss << "sumCost(1," << chainSize << ")";
        //      toCheck.push_back(ss.str());

        auto result = mg.testSeries(fileName, &toCheck, runs, this->warmUp, global, verbose, 3, chainSize,
                                    [this] (supplementary::ClingWrapper *asp)
                                    {
                                      this->lambda(asp);
//          asp->setModelCount(0);
                                    //            asp->setParallelMode(4);
                                    //        asp->setSaveProgress(0);
//              asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
                                    //            asp->setOptStrategie(4);
                                    //Berkmin|Vmtf|Vsids|Domain|Unit|None
                                    //              asp->setHeuristic("Domain");
                                  });


        // Ram Eval Stuff
        std::cout << result.avg.ramUsageBeforeMax << "\t" << result.avg.ramUsageMax << "\t"
            << result.avg.javaRamUsageBeforeMax << "\t" << result.avg.javaRamUsageMax << std::endl;

        if (verbose)
          result.print();

        // print to file
        file << chainSize << "\t";
        file << nodes << "\t";
        file << result.numberTotal << "\t";
        file << result.numberSuccessful << "\t";
        file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
            << result.worst.totalTime << "\t";
        file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t"
            << result.best.ontologyReadTime << "\t" << result.worst.ontologyReadTime << "\t";
        file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
            << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
        file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
            << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
        file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t"
            << result.best.aspGroundingTime << "\t" << result.worst.aspGroundingTime << "\t";
        file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
            << "\t" << result.worst.aspSolvingTime << "\t";
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspSatTime << "\t";
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspUnsatTime << "\t";
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
            << result.worst.aspModelCount << "\t";
        file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
            << result.worst.aspAtomCount << "\t";
        file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
            << "\t" << result.worst.aspBodiesCount << "\t";
        file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
            << "\t" << result.worst.aspAuxAtomCount << std::endl;

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
      ss << "gnuplot -persist -e \"set title '" << logFile << "'; plot '" << logFile << "' u 1:5 w l t 'sum', '"
          << logFile << "' u 1:21 w l t 'grounding', '" << logFile << "' u 1:(\\$21 + \\$25) w l t 'solving'\"";
    }
    else
    {
      ss << "gnuplot -persist -e \"set title '" << logFile << "';set hidden3d; set dgrid3d 20,20 qnorm 2; splot '"
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
    ss << this->logPath << "/representationScenario_" << (global ? "global" : "local") << "_" << representationSizeMin
        << "-" << representationSizeMax << ".txt";
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
      std::cout << "---------------------------------------------------------------------------------------------------"
          << std::endl;
      std::cout << "---------------------------------------------------------------------------------------------------"
          << std::endl;
      std::cout << "Starting eval " << reps << " representations" << std::endl << std::endl;

      std::string system = "EvalSystem";

      ice::OntologyInterface oi(path + "/java/lib/");

      oi.addIRIMapper(path + "/ontology/");
      oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
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
      oi.addValueScope(superValueScope, valueScope, "DoubleRep");

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

        metadatas.push_back("Delay");
        metadataValues.push_back(5);
        metadataValues2.push_back(5);
        metadataGroundings.push_back("NodeDelayFixASPGrounding");
      metadatas.push_back("Cost");
      metadataValues.push_back(1);
      metadataValues2.push_back(0);
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
      ss << "/tmp/representationScenario_" << (global ? "global" : "local") << "_" << reps << ".owl";
      std::string fileName = ss.str();
      oi.saveOntology(fileName);

      ModelGeneration mg(path);

      std::vector<std::string> toCheck;

      if (global)
      {
        //    ss.str("");
        //    ss << "metadataStream(1,accuracy,stream(1,o0_EvalSystem,o0_EvalNode0_" << reps - 1
        //        << "Ind,o0_EvalSystem,information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation" << reps-1 << ",none),2),10)";
        //    toCheck.push_back(ss.str());

//          ss.str("");
//          ss << "metadataStream(1,delay,stream(1,o0_EvalSystem,o0_EvalNode0_" << reps - 1
//              << "Ind,o0_EvalSystem,information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation" << reps-1 << ",none),2),5)";
//          toCheck.push_back(ss.str());
      }

      ss.str("");
      ss << "selectedStream(1,o0_EvalSystem,node(1,o0_EvalSystem,o0_EvalNode0_" << reps - 1
          << "Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation" << reps - 1
          << ",none),2)";
      toCheck.push_back(ss.str());

      ss.str("");
      ss << "node(1,o0_EvalSystem,o0_EvalNode0_" << reps - 1 << "Ind,o0_EvalEntity,none)";
      toCheck.push_back(ss.str());

//        ss.str("");
//        ss << "sumMetadata(1,cost,2)";
//        toCheck.push_back(ss.str());

      auto result = mg.testSeries(fileName, &toCheck, runs, this->warmUp, global, verbose, 3, reps,
                                  [this] (supplementary::ClingWrapper *asp)
                                  {
                                    this->lambda(asp);
//                                    asp->add("base", {}, "system(o0_EvalSystem,default).");
//          asp->setModelCount(0);
                                  //        asp->setSaveProgress(200);
                                  //      asp->add("base",{},"stream(1,o0_EvalSystem,o0_EvalNodeSourceInd,o0_EvalSystem,information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation0,none)).");
//          asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
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
      file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
          << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
      file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t" << result.best.aspGroundingTime
          << "\t" << result.worst.aspGroundingTime << "\t";
      file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
          << "\t" << result.worst.aspSolvingTime << "\t";
      file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
          << result.worst.aspSatTime << "\t";
      file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
          << result.worst.aspUnsatTime << "\t";
      file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
          << result.worst.aspModelCount << "\t";
      file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
          << result.worst.aspAtomCount << "\t";
      file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
          << "\t" << result.worst.aspBodiesCount << "\t";
      file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
          << "\t" << result.worst.aspAuxAtomCount << std::endl;

      // gnuplot -persist -e "plot './results3-10.txt' using 1:4 with lines"
      file.flush();
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    ss << "gnuplot -persist -e \"set title '" << logFile << "'; plot '" << logFile << "' u 1:4 w l t 'sum', '"
        << logFile << "' u 1:20 w l t 'grounding', '" << logFile << "' u 1:(\\$20 + \\$24) w l t 'solving'\"";
//          << logFile << "' using 1:4 with lines\"";
    system(ss.str().c_str());
  }

  void fuseVictimsScenario(bool global, bool verbose, bool gnuplot, int runs, bool neutrality, int sourceSizeMin, int sourceSizeMax,
                               int sourceSizeStep)
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
    ss << this->logPath << "/fuseVictimsScenario_" << (global ? "global" : "local") << "_"
        << (neutrality ? "neutrality_" : "") << sourceSizeMin << "-" << sourceSizeMax << ".txt";
    std::string pathStr = ss.str();
    file.open(pathStr);

    std::string entityType = "EvalEntityType";
    std::string entity = "EvalEntity";
    std::string superValueScope = "SuperEvalValueScope";
    std::string superRepresentation = "SuperRepresentation";

    for (int systems = sourceSizeMin + 1; systems <= sourceSizeMax+1; systems += sourceSizeStep)
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
        std::cout << "-----------------------------------------------------------------------------" << std::endl;
        std::cout << "-----------------------------------------------------------------------------" << std::endl;
        std::cout << "Starting eval " << systems << " systems and " << std::endl << std::endl;

        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
        oi.loadOntologies();
        oi.setLogLevel(ice::LogLevel::Debug);

        // add information structure
        std::vector<std::string> scopes;

        ss.str("");
        ss << "EvalScope";
        std::string scope = ss.str();
        ss.str("");
        ss << "EvalValueScope";
        std::string valueScope = ss.str();

        oi.addValueScope(superValueScope, valueScope, "DoubleRep");

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
            inputsMax.push_back(5);
            outputs.push_back("EvalStream");
            outputsMin.push_back(1);
            outputsMax.push_back(1);

            oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

//            metadatas.push_back("Delay");
//            metadataValues.push_back(10);
//            metadataValues2.push_back(0);
//            metadataGroundings.push_back("NodeDelayASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeCostASPGrounding");
            metadatas.push_back("Accuracy");
            metadataValues.push_back(1);
            metadataValues2.push_back(2);
            metadataGroundings.push_back("NodeAccuracyMinASPGrounding");

            oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2, metadataGroundings);


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

            node = "SmoothingEvalNode";

            inputs.push_back("EvalStream");
            inputsMin.push_back(1);
            inputsMax.push_back(1);
            outputs.push_back("EvalStream");
            outputsMin.push_back(1);
            outputsMax.push_back(1);

            oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

//            metadatas.push_back("Delay");
//            metadataValues.push_back(1);
//            metadataValues2.push_back(0);
//            metadataGroundings.push_back("NodeDelayASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeCostASPGrounding");
            metadatas.push_back("Accuracy");
            metadataValues.push_back(5);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeAccuracyMinASPGrounding");

            oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2, metadataGroundings);

          }
          else
          {
            metadatas.push_back("Accuracy");
            if (neutrality)
            {
              metadataValues.push_back(i < (5 + 1) ? i : (5 + 1));
              metadataValues2.push_back(0);
            }
            else
            {
              metadataValues.push_back(i);
              metadataValues2.push_back(0);
            }
            metadataGroundings.push_back("NodeAccuracyFixASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeCostASPGrounding");
//            metadatas.push_back("Delay");
//            metadataValues.push_back(1);
//            metadataValues2.push_back(0);
//            metadataGroundings.push_back("NodeDelayASPGrounding");

            oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues,
                                 metadataValues2, metadataGroundings);
          }

          if (i == 0)
          {
            ss.str("");
            ss << "Req" << stream << i;
            oi.addRequiredStream(ss.str(), stream, system, entity, "");
          }
        }

        ss.str("");
        ss << "/tmp/fuseVictimsScenario_" << (global ? "global" : "local") << "_" << (neutrality ? "neutrality_" : "") << systems << ".owl";
        std::string fileName = ss.str();
        oi.saveOntology(fileName);

        ModelGeneration mg(path);

        std::vector<std::string> toCheck;

        //    ss.str("");
        //    ss << "metadataStream(1,accuracy,stream(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalSystem0," <<
        //        "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3),10)";
        //    toCheck.push_back(ss.str());

        if (global)
        {
          ss.str("");
//          ss
//              << "metadataStream(1,accuracy,stream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_SmoothingEvalNodeInd,o0_EvalEntity,none),"
//               << "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),4),"
//               << (min(systems-1,5)*2 + 5 + 1 + systems - min(systems-1,5)) << ")";
          ss
              << "metadataStream(1,accuracy,stream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_SmoothingEvalNodeInd,o0_EvalEntity,none),"
               << "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),4),"
               << (min(systems-1,5)*2 + 5 + 1 + systems - min(systems-1,5) -1) << ")";
          toCheck.push_back(ss.str());
        }

        for (int i = systems - 1 - min(5, systems-1); i < systems-1; ++i)
        {
          ss.str("");
          ss << "stream(1,o0_EvalSystem0,node(1,o0_EvalSystem" << i << ",o0_EvalNode" << i
              << "SourceInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation,none),2)";
          toCheck.push_back(ss.str());
        }

        ss.str("");
        ss << "selectedStream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_SmoothingEvalNodeInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),4)";
        toCheck.push_back(ss.str());

        //          ss.str("");
        //          ss << "sumMetadata(1,cost," << inputsCount + 1 << ")";
        //          toCheck.push_back(ss.str());

        mg.index = systems-1;


        auto result = mg.testSeries(
            fileName, &toCheck, runs, this->warmUp, global, verbose, 3, 10, [&] (supplementary::ClingWrapper *asp)
            {
              this->lambda(asp);

              for (int i=1; i < systems; ++i)
              {
                ss.str("");
                ss << "transfer(o0_EvalSystem" << i << ",o0_EvalSystem0).";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataProcessing(cost,o0_EvalSystem" << i << ",o0_EvalSystem0," << 2 << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataOutput(delay,o0_EvalSystem" << i << ",o0_EvalSystem0," << 2 << ").";
                asp->add("base",
                    {},ss.str());

                ss.str("");
                ss << "transfer(o0_EvalSystem0,o0_EvalSystem" << i << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataProcessing(cost,o0_EvalSystem0,o0_EvalSystem" << i << "," << 2 << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataOutput(delay,o0_EvalSystem0,o0_EvalSystem" << i << "," << 2 << ").";
                asp->add("base",
                    {},ss.str());
              }

              //     asp->setModelCount(0);
            //            asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
            //      asp->setOptStrategie(3);
          });

        result.print();

        // print to file
        file << (systems-1) << "\t";
//        file << inputsCount << "\t";
        file << result.numberTotal << "\t";
        file << result.numberSuccessful << "\t";
        file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
            << result.worst.totalTime << "\t";
        file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t"
            << result.best.ontologyReadTime << "\t" << result.worst.ontologyReadTime << "\t";
        file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
            << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
        file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
            << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
        file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t"
            << result.best.aspGroundingTime << "\t" << result.worst.aspGroundingTime << "\t";
        file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
            << "\t" << result.worst.aspSolvingTime << "\t";
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspSatTime << "\t";
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspUnsatTime << "\t";
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
            << result.worst.aspModelCount << "\t";
        file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
            << result.worst.aspAtomCount << "\t";
        file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
            << "\t" << result.worst.aspBodiesCount << "\t";
        file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
            << "\t" << result.worst.aspAuxAtomCount << std::endl;

        // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
        file.flush();
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
//    if (inputSizeMin == inputSizeMax)
//    {
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '" << pathStr << "' u 1:4 w l t 'sum', '"
          << pathStr << "' u 1:20 w l t 'grounding', '" << pathStr << "' u 1:(\\$20 + \\$24) w l t 'solving'\"";
//    }
//    else
//    {
//      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; splot '" << pathStr
//          << "' using 1:2:5 with points t 'sum', '" << pathStr << "' using 1:2:21 with points t 'Grounding', '"
//          << pathStr << "' using 1:2:(\\$21+\\$25) with points t 'Grounding + Solving'\"";
//    }

    system(ss.str().c_str());
  }

  void systemsStarMashScenario(bool global, bool verbose, bool gnuplot, int runs, bool neutrality, int systemSizeMin,
                               int systemSizeMax, int systemSizeStep, int inputSizeMin, int inputSizeMax,
                               int inputSizeStep)
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
        << (neutrality ? "neutrality_" : "") << systemSizeMin << "-" << systemSizeMax << "_" << inputSizeMin << "-"
        << inputSizeMax << ".txt";
    std::string pathStr = ss.str();
    file.open(pathStr);

    std::string entityType = "EvalEntityType";
    std::string entity = "EvalEntity";
    std::string superValueScope = "SuperEvalValueScope";
    std::string superRepresentation = "SuperRepresentation";

    for (int systems = systemSizeMin + 1; systems < systemSizeMax; systems += systemSizeStep)
    {
      for (int inputsCount = inputSizeMin; inputsCount <= inputSizeMax && inputsCount < systems; inputsCount +=
          inputSizeStep)
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
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout << "Starting eval " << systems << " systems and " << inputsCount << " inputs" << std::endl
            << std::endl;

        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
        oi.loadOntologies();

        // add information structure
        std::vector<std::string> scopes;

        ss.str("");
        ss << "EvalScope";
        std::string scope = ss.str();
        ss.str("");
        ss << "EvalValueScope";
        std::string valueScope = ss.str();

        oi.addValueScope(superValueScope, valueScope, "DoubleRep");

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
            inputsMin.push_back(inputsCount);
            inputsMax.push_back(inputsCount);
            outputs.push_back("EvalStream");
            outputsMin.push_back(1);
            outputsMax.push_back(1);

            oi.addComputationNodeClass(node, inputs, inputsMin, inputsMax, outputs, outputsMin, outputsMax);

            metadatas.push_back("Delay");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeDelayASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeCostASPGrounding");
            //        metadatas.push_back("Accuracy");
            //        metadataValues.push_back(0);
            //        metadataValues2.push_back(0);
            //        metadataGroundings.push_back("NodeAccuracyMaxASPGrounding");

            oi.addNodeIndividual(node + "Ind", node, system, "", "", metadatas, metadataValues, metadataValues2,
                                 metadataGroundings);

          }
          else
          {
            metadatas.push_back("Delay");
            if (neutrality)
            {
              metadataValues.push_back(i < (inputsCount + 1) ? i : (inputsCount + 1));
              metadataValues2.push_back(0);
            }
            else
            {
              metadataValues.push_back(i);
              metadataValues2.push_back(0);
            }
            metadataGroundings.push_back("NodeDelayFixASPGrounding");
            metadatas.push_back("Cost");
            metadataValues.push_back(1);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeCostASPGrounding");
            //          metadatas.push_back("Accuracy");
            //          metadataValues.push_back(10);
            //          metadataValues2.push_back(10);
            //          metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

            oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues,
                                 metadataValues2, metadataGroundings);
          }

          if (i == 0)
          {
            ss.str("");
            ss << "Req" << stream << i;
            oi.addRequiredStream(ss.str(), stream, system, entity, "");
          }
        }

        ss.str("");
        ss << "/tmp/systemsStarMashScenario_" << (global ? "global" : "local") << "_"
            << (neutrality ? "neutrality_" : "") << systems << "_" << inputsCount << ".owl";
        std::string fileName = ss.str();
        oi.saveOntology(fileName);

        ModelGeneration mg(path);

        std::vector<std::string> toCheck;

        //    ss.str("");
        //    ss << "metadataStream(1,accuracy,stream(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalSystem0," <<
        //        "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3),10)";
        //    toCheck.push_back(ss.str());

        if (global)
        {
          ss.str("");
          ss
              << "metadataStream(1,delay,stream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalEntity,none),"
              << "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3)," << (inputsCount + 2 + 1)
              << ")";
          toCheck.push_back(ss.str());
        }

        for (int i = 1; i <= inputsCount; ++i)
        {
          ss.str("");
          ss << "stream(1,o0_EvalSystem0,node(1,o0_EvalSystem" << i << ",o0_EvalNode" << i
              << "SourceInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation,none),2)";
          toCheck.push_back(ss.str());
        }

        ss.str("");
        ss
            << "selectedStream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3)";
        toCheck.push_back(ss.str());

//          ss.str("");
//          ss << "sumMetadata(1,cost," << inputsCount + 1 << ")";
//          toCheck.push_back(ss.str());

        auto result = mg.testSeries(
            fileName, &toCheck, runs, this->warmUp, global, verbose, 3, 10, [&] (supplementary::ClingWrapper *asp)
            {
              this->lambda(asp);

              for (int i=1; i < systems; ++i)
              {
                ss.str("");
                ss << "transfer(o0_EvalSystem" << i << ",o0_EvalSystem0).";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataProcessing(cost,o0_EvalSystem" << i << ",o0_EvalSystem0," << 2 << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataOutput(delay,o0_EvalSystem" << i << ",o0_EvalSystem0," << 2 << ").";
                asp->add("base",
                    {},ss.str());

                ss.str("");
                ss << "transfer(o0_EvalSystem0,o0_EvalSystem" << i << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataProcessing(cost,o0_EvalSystem0,o0_EvalSystem" << i << "," << 2 << ").";
                asp->add("base",
                    {},ss.str());
                ss.str("");
                ss << "metadataOutput(delay,o0_EvalSystem0,o0_EvalSystem" << i << "," << 2 << ").";
                asp->add("base",
                    {},ss.str());
              }

              //     asp->setModelCount(0);
//            asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
            //      asp->setOptStrategie(3);
          });

        result.print();

        // print to file
        file << systems << "\t";
        file << inputsCount << "\t";
        file << result.numberTotal << "\t";
        file << result.numberSuccessful << "\t";
        file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
            << result.worst.totalTime << "\t";
        file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t"
            << result.best.ontologyReadTime << "\t" << result.worst.ontologyReadTime << "\t";
        file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
            << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
        file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
            << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
        file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t"
            << result.best.aspGroundingTime << "\t" << result.worst.aspGroundingTime << "\t";
        file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
            << "\t" << result.worst.aspSolvingTime << "\t";
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspSatTime << "\t";
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspUnsatTime << "\t";
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
            << result.worst.aspModelCount << "\t";
        file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
            << result.worst.aspAtomCount << "\t";
        file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
            << "\t" << result.worst.aspBodiesCount << "\t";
        file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
            << "\t" << result.worst.aspAuxAtomCount << std::endl;

        // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
        file.flush();
      }
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    if (inputSizeMin == inputSizeMax)
    {
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '" << pathStr << "' u 1:5 w l t 'sum', '"
          << pathStr << "' u 1:21 w l t 'grounding', '" << pathStr << "' u 1:(\\$21 + \\$25) w l t 'solving'\"";
    }
    else
    {
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; splot '" << pathStr
          << "' using 1:2:5 with points t 'sum', '" << pathStr << "' using 1:2:21 with points t 'Grounding', '"
          << pathStr << "' using 1:2:(\\$21+\\$25) with points t 'Grounding + Solving'\"";
    }

    system(ss.str().c_str());
  }

  void systemsFullMashScenario(bool global, bool verbose, bool gnuplot, int runs, int systemSizeMin, int systemSizeMax,
                               int systemSizeStep, int inputSizeMin, int inputSizeMax, int inputSizeStep)
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
    ss << this->logPath << "/systemsFullMashScenario_" << (global ? "global" : "local") << "_" << systemSizeMin << "-"
        << systemSizeMax << "_" << inputSizeMin << "-" << inputSizeMax << ".txt";
    std::string pathStr = ss.str();
    file.open(pathStr);

    std::string entityType = "EvalEntityType";
    std::string entity = "EvalEntity";
    std::string superValueScope = "SuperEvalValueScope";
    std::string superRepresentation = "SuperRepresentation";

    for (int systems = systemSizeMin; systems <= systemSizeMax; systems += systemSizeStep)
    {
      for (int inputsCount = inputSizeMin; inputsCount <= inputSizeMax && inputsCount < systems; inputsCount +=
          inputSizeStep)
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
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout << "Starting eval " << systems << " systems full mash and " << inputsCount << " inputs" << std::endl
            << std::endl;

        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
        oi.loadOntologies();

        // add information structure
        std::vector<std::string> scopes;

        ss.str("");
        ss << "EvalScope";
        std::string scope = ss.str();
        ss.str("");
        ss << "EvalValueScope";
        std::string valueScope = ss.str();

        oi.addValueScope(superValueScope, valueScope, "DoubleRep");

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
        inputsMin.push_back(inputsCount);
        inputsMax.push_back(inputsCount);
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

          oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues,
                               metadataValues2, metadataGroundings);
          //      }

          if (i < 1)
          {
            ss.str("");
            ss << "Req" << stream << i;
            oi.addRequiredStream(ss.str(), stream, system, entity, "");
          }
        }

        ss.str("");
        ss << "/tmp/systemsFullMashScenario_" << (global ? "global" : "local") << "_" << systems << "_" << inputsCount
            << ".owl";
        std::string fileName = ss.str();
        oi.saveOntology(fileName);

        ModelGeneration mg(path);

        std::vector<std::string> toCheck;

        //    ss.str("");
        //    ss << "metadataStream(1,accuracy,stream(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalSystem0," <<
        //        "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3),10)";
        //    toCheck.push_back(ss.str());

//        if (global)
//        {
//          ss.str("");
//          ss << "metadataStream(1,delay,stream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalEntity,none)," <<
//              "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3),8)";
//          toCheck.push_back(ss.str());
//        }
        if (global)
        {
          ss.str("");
          ss
              << "metadataStream(1,delay,stream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalEntity,none),"
              << "information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3)," << (inputsCount * 2 + 2)
              << ")";
          toCheck.push_back(ss.str());
        }

        for (int i = 1; i < inputsCount; ++i)
        {
          ss.str("");
          ss << "stream(1,o0_EvalSystem0,node(1,o0_EvalSystem" << i << ",o0_EvalNode" << i
              << "SourceInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation,none),2)";
          toCheck.push_back(ss.str());
        }

//        ss.str("");
//        ss << "stream(1,o0_EvalSystem0,node(1,o0_EvalSystem1,o0_EvalNode1SourceInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation,none),2)";
//        toCheck.push_back(ss.str());
//        ss.str("");
//        ss << "stream(1,o0_EvalSystem0,node(1,o0_EvalSystem2,o0_EvalNode2SourceInd,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_EvalRepresentation,none),2)";
//        toCheck.push_back(ss.str());

        ss.str("");
        ss
            << "selectedStream(1,o0_EvalSystem0,node(1,o0_EvalSystem0,o0_EvalNode0Ind,o0_EvalEntity,none),information(o0_EvalEntity,o0_EvalScope,o0_ReqRepresentation,none),3)";
        toCheck.push_back(ss.str());

        //      ss.str("");
        //      ss << "sumMetadata(1,cost,4)";
        //      toCheck.push_back(ss.str());

        auto result = mg.testSeries(
            fileName, &toCheck, runs, this->warmUp, global, verbose, 3, 10, [&] (supplementary::ClingWrapper *asp)
            {
              this->lambda(asp);

              for (int i=0; i < systems; ++i)
              {
                for (int j=0; j < systems; ++j)
                {
                  if (i==j)
                  continue;

                  ss.str("");
                  ss << "transfer(o0_EvalSystem" << i << ",o0_EvalSystem" << j << ").";
                  asp->add("base",
                      {},ss.str());
                  ss.str("");
                  ss << "metadataProcessing(cost,o0_EvalSystem" << i << ",o0_EvalSystem" << j << "," << 2 << ").";
                  asp->add("base",
                      {},ss.str());
                  ss.str("");
                  ss << "metadataOutput(delay,o0_EvalSystem" << i << ",o0_EvalSystem" << j << "," << 2 << ").";
                  asp->add("base",
                      {},ss.str());

                  ss.str("");
                  ss << "transfer(o0_EvalSystem" << j << ",o0_EvalSystem" << i << ").";
                  asp->add("base",
                      {},ss.str());
                  ss.str("");
                  ss << "metadataProcessing(cost,o0_EvalSystem" << j << ",o0_EvalSystem" << i << "," << 2 << ").";
                  asp->add("base",
                      {},ss.str());
                  ss.str("");
                  ss << "metadataOutput(delay,o0_EvalSystem" << j << ",o0_EvalSystem" << i << "," << 2 << ").";
                  asp->add("base",
                      {},ss.str());

                }

                //          ss.str("");
                //          ss << "system(o0_EvalSystem" << i << ").";
                //          asp->add("base",{},ss.str());
          }

          //        asp->setModelCount(0);
          });

        result.print();

        // print to file
        file << systems << "\t";
        file << inputsCount << "\t";
        file << result.numberTotal << "\t";
        file << result.numberSuccessful << "\t";
        file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
            << result.worst.totalTime << "\t";
        file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t"
            << result.best.ontologyReadTime << "\t" << result.worst.ontologyReadTime << "\t";
        file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
            << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
        file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
            << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
        file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t"
            << result.best.aspGroundingTime << "\t" << result.worst.aspGroundingTime << "\t";
        file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
            << "\t" << result.worst.aspSolvingTime << "\t";
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspSatTime << "\t";
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspUnsatTime << "\t";
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
            << result.worst.aspModelCount << "\t";
        file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
            << result.worst.aspAtomCount << "\t";
        file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
            << "\t" << result.worst.aspBodiesCount << "\t";
        file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
            << "\t" << result.worst.aspAuxAtomCount << std::endl;

        // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
        file.flush();
      }
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    if (inputSizeMin == inputSizeMax)
    {
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '" << pathStr << "' u 1:5 w l t 'sum', '"
          << pathStr << "' u 1:21 w l t 'grounding', '" << pathStr << "' u 1:(\\$21 + \\$25) w l t 'solving'\"";
    }
    else
    {
      ss << "gnuplot -persist -e \"set title '" << pathStr << "'; splot '" << pathStr
          << "' using 1:2:5 with points t 'sum', '" << pathStr << "' using 1:2:21 with points t 'Grounding', '"
          << pathStr << "' using 1:2:(\\$21+\\$25) with points t 'Grounding + Solving'\"";
    }

    system(ss.str().c_str());
  }

  void islandScenario(bool global, bool verbose, bool gnuplot, int runs, int islandsSizeMin, int islandsSizeMax,
                      int islandsSizeStep, int systemSizeMin, int systemSizeMax, int systemSizeStep)
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
    ss << this->logPath << "/islandScenario_" << (global ? "global" : "local") << "_" << islandsSizeMin << "-"
        << islandsSizeMax << "_" << systemSizeMin << "-" << systemSizeMax << ".txt";
    std::string pathStr = ss.str();
    file.open(pathStr);

    std::string entityType = "IslandEntityType";
    std::string superValueScope = "SuperEvalValueScope";
    std::string superRepresentation = "SuperRepresentation";

    for (int islands = islandsSizeMin; islands <= islandsSizeMax; islands += islandsSizeStep)
    {
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
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout
            << "---------------------------------------------------------------------------------------------------"
            << std::endl;
        std::cout << "Starting eval " << islands << " with " << systems << " systems" << std::endl << std::endl;

        ice::OntologyInterface oi(path + "/java/lib/");

        oi.addIRIMapper(path + "/ontology/");
        oi.addOntologyIRI("http://vs.uni-kassel.de/Ice");
        oi.loadOntologies();

        // add information structure
        std::vector<std::string> scopes;

        ss.str("");
        ss << "EvalScope";
        std::string scope = ss.str();
        ss.str("");
        ss << "EvalValueScope";
        std::string valueScope = ss.str();

        oi.addValueScope(superValueScope, valueScope, "DoubleRep");

        std::vector<std::string> reps;
        ss.str("");
        ss << "ReqRepresentation";
        std::string representation = ss.str();

        std::vector<std::string> vec;
        vec.push_back(valueScope);
        oi.addRepresentation(superRepresentation, representation, vec);
        reps.push_back(representation);

//          ss.str("");
//          ss << "EvalRepresentation";
//          representation = ss.str();
//
//          oi.addRepresentation(superRepresentation, representation, vec);
//          reps.push_back(representation);

        oi.addEntityScope(scope, reps);

        scopes.push_back(scope);
        oi.addEntityType(entityType, scopes);

        // Add source stream
        ss.str("");
        ss << "EvalSourceStream";
        std::string stream = ss.str();

        oi.addNamedStream(stream, scope, representation);

        // Add eval node
        outputs.push_back(stream);
        outputsMin.push_back(1);
        outputsMax.push_back(1);

        std::string node = "EvalNodeSource";
        oi.addSourceNodeClass(node, outputs, outputsMin, outputsMax);

        // Add req map
        std::string map = "EvalMap";
        oi.addNamedMap(map, entityType, scope, "ReqRepresentation");

        inputs.clear();
        inputsMin.clear();
        inputsMax.clear();
        outputs.clear();
        outputsMin.clear();
        outputsMax.clear();

        inputs.push_back(stream);
        inputsMin.push_back(0);
        inputsMax.push_back(1);
        outputs.push_back(map);
        outputsMin.push_back(1);
        outputsMax.push_back(1);

        oi.addMapNodeClass("EvalMapNode", inputs, inputsMin, inputsMax, std::vector<std::string>(), std::vector<int>(),
                           std::vector<int>(), std::vector<std::string>(), std::vector<int>(), std::vector<int>(),
                           outputs, outputsMin, outputsMax);

        for (int i = 0; i < islands; ++i)
        {
          // Entity of island
          ss.str("");
          ss << "IslandEntity" << i;
          std::string entity = ss.str();
          scopes.push_back(scope);
          oi.addEntityType(entityType, scopes);
          oi.addIndividual(entity, entityType);

          for (int j = 0; j < systems; ++j)
          {
            ss.str("");
            ss << "EvalNode" << i << "_" << j;
            node = ss.str();
            metadatas.clear();
            metadataValues.clear();
            metadataValues2.clear();
            metadataGroundings.clear();

            ss.str("");
            ss << "EvalSystem" << i << "_" << j;
            std::string system = ss.str();
            oi.addSystem(system);

            metadatas.push_back("Delay");
            metadataValues.push_back(j);
            metadataValues2.push_back(0);
            metadataGroundings.push_back("NodeDelayFixASPGrounding");
//              metadatas.push_back("Cost");
//              metadataValues.push_back(1);
//              metadataValues2.push_back(0);
//              metadataGroundings.push_back("NodeCostASPGrounding");
//              metadatas.push_back("Accuracy");
//              metadataValues.push_back(systems - j);
//              metadataValues2.push_back(0);
//              metadataGroundings.push_back("NodeAccuracyFixASPGrounding");

            oi.addNodeIndividual(node + "SourceInd", "EvalNodeSource", system, entity, "", metadatas, metadataValues,
                                 metadataValues2, metadataGroundings);

            if (i < 1 && j < 1)
            {

              metadatas.clear();
              metadataValues.clear();
              metadataValues2.clear();
              metadataGroundings.clear();

              metadatas.push_back("Delay");
              metadataValues.push_back(1);
              metadataValues2.push_back(0);
              metadataGroundings.push_back("MapDelayASPGrounding");
//                metadatas.push_back("Cost");
//                metadataValues.push_back(1);
//                metadataValues2.push_back(0);
//                metadataGroundings.push_back("MapCostASPGrounding");
              metadatas.push_back("Density");
              metadataValues.push_back(0);
              metadataValues2.push_back(1);
              metadataGroundings.push_back("MapDensitySumASPGrounding");
//                metadatas.push_back("Accuracy");
//                metadataValues.push_back(0);
//                metadataValues2.push_back(0);
//                metadataGroundings.push_back("MapAccuracyAvgASPGrounding");

              oi.addNodeIndividual(node + "Ind", "EvalMapNode", system, "", "", metadatas, metadataValues,
                                   metadataValues2, metadataGroundings);

              ss.str("");
              ss << "Req" << map << i;
              oi.addRequiredMap(ss.str(), map, system, "");
            }
          }
        }

        ss.str("");
        ss << "/tmp/islandScenario_" << (global ? "global" : "local") << "_" << islands << "_" << systems << ".owl";
        std::string fileName = ss.str();
        oi.saveOntology(fileName);

        ModelGeneration mg(path);

        std::vector<std::string> toCheck;

        if (global)
        {
          ss.str("");
          ss
              << "metadataMap(1,delay,map(1,o0_EvalSystem0_0,mapNode(1,o0_EvalSystem0_0,o0_EvalNode0_0Ind,o0_IslandEntityType,none),"
              << "informationType(o0_IslandEntityType,o0_EvalScope,o0_ReqRepresentation,none),3),7)";
          toCheck.push_back(ss.str());

//            ss.str("");
//            ss << "metadataMap(1,accuracy,map(1,o0_EvalSystem0_0,o0_EvalNode0_0Ind,o0_EvalSystem0_0," <<
//                "informationType(islandEntityType,o0_EvalScope,o0_ReqRepresentation,none),3)," << systems << ")";
//            toCheck.push_back(ss.str());
        }

        for (int i = 1; i < islands; ++i)
        {
          ss.str("");
          ss << "stream(1,o0_EvalSystem0_0,node(1,o0_EvalSystem" << i << "_0,o0_EvalNode" << i
              << "_0SourceInd,o0_IslandEntity" << i << ",none),information(o0_IslandEntity" << i
              << ",o0_EvalScope,o0_ReqRepresentation,none),2)";
          toCheck.push_back(ss.str());
        }

        ss.str("");
        ss
            << "selectedMap(1,o0_EvalSystem0_0,mapNode(1,o0_EvalSystem0_0,o0_EvalNode0_0Ind,o0_IslandEntityType,none),informationType(o0_IslandEntityType,o0_EvalScope,o0_ReqRepresentation,none),3)";
        toCheck.push_back(ss.str());

        auto result =
            mg.testSeries(
                fileName,
                &toCheck,
                runs,
                true,
                global,
                verbose,
                3,
                10,
                [&] (supplementary::ClingWrapper *asp)
                {
                  this->lambda(asp);

                  ss.str("");
                  ss << "system(o0_EvalSystem0_0, island0).";
                  asp->add("base",
                      {},ss.str());

                  for (int j=1; j < systems; ++j)
                  {
                    ss.str("");
                    ss << "transfer(o0_EvalSystem" << 0 << "_" << 0 << ",o0_EvalSystem" << 0 << "_" << j << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataProcessing(cost,o0_EvalSystem" << 0 << "_" << 0 << ",o0_EvalSystem" << 0 << "_" << j << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataOutput(delay,o0_EvalSystem" << 0 << "_" << 0 << ",o0_EvalSystem" << 0 << "_" << j << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());

                    ss.str("");
                    ss << "transfer(o0_EvalSystem" << 0 << "_" << j << ",o0_EvalSystem" << 0 << "_" << 0 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataProcessing(cost,o0_EvalSystem" << 0 << "_" << j << ",o0_EvalSystem" << 0 << "_" << 0 << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataOutput(delay,o0_EvalSystem" << 0 << "_" << j << ",o0_EvalSystem" << 0 << "_" << 0 << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());

                    ss.str("");
                    ss << "system(o0_EvalSystem" << 0 << "_" << j << ",island0).";
                    asp->add("base",
                        {},ss.str());
                  }

                  for (int i=1; i < islands; ++i)
                  {
                    ss.str("");
                    ss << "bridge(island" << i << ",island" << 0 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataProcessing(cost,island" << i << ",island" << 0 << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataOutput(delay,island" << i << ",island" << 0 << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());

                    ss.str("");
                    ss << "connectToBridge(o0_EvalSystem" << 0 << "_" << 0 << ",island" << i << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataProcessing(cost,o0_EvalSystem" << 0 << "_" << 0 << ",island" << i << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());
                    ss.str("");
                    ss << "metadataOutput(delay,o0_EvalSystem" << 0 << "_" << 0 << ",island" << i << "," << 2 << ").";
                    asp->add("base",
                        {},ss.str());

                    for (int j=0; j < systems; ++j)
                    {
                      ss.str("");
                      ss << "connectToBridge(o0_EvalSystem" << i << "_" << j << ",island" << 0 << ").";
                      asp->add("base",
                          {},ss.str());
                      ss.str("");
                      ss << "metadataProcessing(cost,o0_EvalSystem" << i << "_" << j << ",island" << 0 << "," << 2 << ").";
                      asp->add("base",
                          {},ss.str());
                      ss.str("");
                      ss << "metadataOutput(delay,o0_EvalSystem" << i << "_" << j << ",island" << 0 << "," << 2 << ").";
                      asp->add("base",
                          {},ss.str());

                      ss.str("");
                      ss << "system(o0_EvalSystem" << i << "_" << j << ",island" << i << ").";
                      asp->add("base",
                          {},ss.str());
                    }
                  }
                });

        result.print();

        // print to file
        file << islands << "\t";
        file << systems << "\t";
        file << result.numberTotal << "\t";
        file << result.numberSuccessful << "\t";
        file << result.avg.totalTime << "\t" << result.totalTimeVar << "\t" << result.best.totalTime << "\t"
            << result.worst.totalTime << "\t";
        file << result.avg.ontologyReadTime << "\t" << result.ontologyReadTimeVar << "\t"
            << result.best.ontologyReadTime << "\t" << result.worst.ontologyReadTime << "\t";
        file << result.avg.ontologyReasonerTime << "\t" << result.ontologyReasonerTimeVar << "\t"
            << result.best.ontologyReasonerTime << "\t" << result.worst.ontologyReasonerTime << "\t";
        file << result.avg.ontologyToASPTime << "\t" << result.ontologyToASPTimeVar << "\t"
            << result.best.ontologyToASPTime << "\t" << result.worst.ontologyToASPTime << "\t";
        file << result.avg.aspGroundingTime << "\t" << result.aspGroundingTimeVar << "\t"
            << result.best.aspGroundingTime << "\t" << result.worst.aspGroundingTime << "\t";
        file << result.avg.aspSolvingTime << "\t" << result.aspSolvingTimeVar << "\t" << result.best.aspSolvingTime
            << "\t" << result.worst.aspSolvingTime << "\t";
        file << result.avg.aspSatTime << "\t" << result.aspSatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspSatTime << "\t";
        file << result.avg.aspUnsatTime << "\t" << result.aspUnsatTimeVar << "\t" << result.best.aspSatTime << "\t"
            << result.worst.aspUnsatTime << "\t";
        file << result.avg.aspModelCount << "\t" << result.aspModelCountVar << "\t" << result.best.aspModelCount << "\t"
            << result.worst.aspModelCount << "\t";
        file << result.avg.aspAtomCount << "\t" << result.aspAtomCountVar << "\t" << result.best.aspAtomCount << "\t"
            << result.worst.aspAtomCount << "\t";
        file << result.avg.aspBodiesCount << "\t" << result.aspBodiesCountVar << "\t" << result.best.aspBodiesCount
            << "\t" << result.worst.aspBodiesCount << "\t";
        file << result.avg.aspAuxAtomCount << "\t" << result.aspAuxAtomCountVar << "\t" << result.best.aspAuxAtomCount
            << "\t" << result.worst.aspAuxAtomCount << std::endl;

        // gnuplot -persist -e "plot './results_systems50-500.txt' u 1:4 w l t 'sum', './results_systems50-500.txt' u 1:20 w l t 'grounding', './results_systems50-500.txt' u 1:(\$20 + \$24) w l t 'solving'"
        file.flush();
      }
    }

    file.close();

    if (!gnuplot)
      return;

    ss.str("");
    ss << "gnuplot -persist -e \"set title '" << pathStr << "'; plot '" << pathStr << "' u 1:5 w l t 'sum', '"
        << pathStr << "' u 1:21 w l t 'grounding', '" << pathStr << "' u 1:(\\$21 + \\$25) w l t 'solving'\"";

    system(ss.str().c_str());
  }

private:
  bool warmUp;
  std::string logPath;
  std::function<void(supplementary::ClingWrapper *asp)> lambda;
};
