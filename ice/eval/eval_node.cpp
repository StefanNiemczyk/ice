/*
 * eval_node.cpp
 *
 *  Created on: Nov 21, 2016
 *      Author: sni
 */

#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>

#include "MemoryMonitor.h"
#include "ClingWrapper.h"
#include "EvalScenarios.cpp"

void evalRam(int argc, char **argv)
{
  assert(argc == 2 && "Error: wrong size of arguments " + argc);

  int index = std::stoi(argv[1]);

  auto mm = MemoryManager::getInstance();
  mm->start();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  double ram = mm->getMemoryUsage().residentSetMax;
  double java = mm->getMemoryUsage().javaRamUsageMax;

  std::string path = ros::package::getPath("ice");
  bool global = true;
  std::string fileName = "chainScenario_global" + std::to_string(index) + "_10.lp";

  // Initializing ASP
  supplementary::ClingWrapper asp;

  asp.addKnowledgeFile(path + "/asp/informationProcessing/processing.lp");
  asp.addKnowledgeFile(path + "/asp/informationProcessing/searchBottomUp.lp");
  if (global)
    asp.addKnowledgeFile(path + "/asp/informationProcessing/globalOptimization.lp");
  else
    asp.addKnowledgeFile(path + "/asp/informationProcessing/localOptimization.lp");

  asp.addKnowledgeFile(path + "/eval/asp_ram_test/" + fileName);

  asp.setNoWarnings(true);
  asp.init();

  auto lastQuery = asp.getExternal("query", {1}, "query", {1, 3, 10}, true);

  asp.ground("query", {1});
  asp.solve();

  std::cout << mm->getMemoryUsage().residentSetMax << "\t" << ram << std::endl;

  mm->stop();
}

void evalGeneration(int argc, char **argv)
{
  assert(argc == 3 && "Error: wrong size of arguments " + argc);

  int index = std::stoi(argv[1]);
  bool generateOwl = std::stoi(argv[2]);

  std::string path = "/home/sni/eval";
//  std::string path = "/home/pi/eval";
  int runs = 1;

  if (false == generateOwl)
    MemoryManager::getInstance()->start();

  EvalScenarios scenarios2(path + "", false, [&] (supplementary::ClingWrapper *asp){
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::tweety);
    asp->setRandomize("20,15");
  });
  //                                 global      verbose gnuplot         runs
//  scenarios2.fuseVictimsScenario(    true,       false,  false,          runs,      false, 100, 100, 10);
//  scenarios2.fuseVictimsScenario(    true,       false,  false,          runs,      false, 6, 20, 2);
//  scenarios2.representationScenario( true,       true,  false,          runs,      2, 12, 1);
  scenarios2.chainScenario(          true,       false,  false,          runs,      index, index, 1, 10, 10, 1, generateOwl);
//  scenarios2.chainScenario(          false,      false,  false,          runs,      1, 20, 1, 10, 10, 1);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 10, 10, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 20, 20, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 30, 30, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 40, 40, 10);
//  scenarios2.islandScenario(         true,       false,  false,          runs,      2, 10, 1, 50, 50, 10);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      true, 10, 100, 5, 5, 50, 5);
//  scenarios2.systemsStarMashScenario(true,       false,  false,          runs,      false, 10, 100, 5, 5, 50, 5);
//  scenarios2.systemsFullMashScenario(true,       false,  false,          runs,      10, 100, 10, 10, 20, 10);


//  runs = 1;
//  EvalScenarios scenarios(path + "", [&] (supplementary::ClingWrapper *asp){
//    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::crafty);
//  });
  //                                 global      verbose gnuplot         runs
//  scenarios.representationScenario(  false,      false,  false,          runs,      9, 9, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      true, 10, 150, 1, 1, 149, 1);
//  scenarios.systemsStarMashScenario( false,      false,  false,          runs,      false, 150, 150, 1, 5, 10, 1);
//  scenarios.systemsFullMashScenario( false,      false,  false,          runs,      10, 20, 1);
//  scenarios.chainScenario(           false,      false,  false,          runs,      10, 10, 1, 1, 10, 1);

  MemoryManager::getInstance()->stop();
}

void evalSynthesis(int argc, char **argv)
{
  assert(argc == 3 && "Error: wrong size of arguments " + argc);

  int index = std::stoi(argv[1]);
  int groups = std::stoi(argv[2]);
//  std::string path = "/home/sni/eval";
  std::string path = std::string(getenv("HOME")) + "/eval";
  int runs = 1;
//  std::cout << "Path: " << path << std::endl;

  EvalScenarios scenariosT(path + "", false, [&] (supplementary::ClingWrapper *asp)
  {
    asp->setPredefConfiguration(supplementary::PredefinedConfigurations::trendy);
    asp->setRandomize("20,15");
  });
  TConf conf;
  
//for (int i = 50; i <= 400; i += 50)
//  {
//    conf.parallelGroupsMin = 1;
//    conf.parallelGroupsMax = 1;
//    conf.parallelGrounsStep = 1;
//    conf.levels = { 10,i};
//    conf.inputsMin = 1;
//    conf.inputsMax = 4;
//    conf.skipLevel = false;
//    scenariosT.transformation(true, false, runs, 50, conf, true, -1);
//  }

  conf.parallelGroupsMin = groups;
  conf.parallelGroupsMax = groups;
  conf.parallelGrounsStep = 50;
  conf.levels = { 10,10};
  conf.inputsMin = 1;
  conf.inputsMax = 4;
  conf.skipLevel = false;
  scenariosT.transformation(false, false, runs, 1, conf, false, index);
}

int main(int argc, char **argv)
{
//  evalRam(argc, argv);
//  evalGeneration(argc, argv);
  evalSynthesis(argc, argv);

  return 0;
}
