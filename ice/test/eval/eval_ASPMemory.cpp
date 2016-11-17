/*
 * eval_ASPMemory.cpp
 *
 *  Created on: Nov 17, 2016
 *      Author: sni
 */

#include <gtest/gtest.h>
#include "MemoryMonitor.h"
#include "ClingWrapper.h"
#include <iostream>
#include <ros/package.h>

TEST(ASPMemory, test)
{
  auto mm = MemoryManager::getInstance();
  mm->start();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  double ram = mm->getMemoryUsage().residentSetMax;
  double java = mm->getMemoryUsage().javaRamUsageMax;

  std::string path = ros::package::getPath("ice");
  bool global = true;
  std::string fileName = "chainScenario_global10_5.lp";

  // Initializing ASP
  supplementary::ClingWrapper asp;

  asp.addKnowledgeFile(path + "/asp/informationProcessing/processing.lp");
  asp.addKnowledgeFile(path + "/asp/informationProcessing/searchBottomUp.lp");
  if (global)
    asp.addKnowledgeFile(path + "/asp/informationProcessing/globalOptimization.lp");
  else
    asp.addKnowledgeFile(path + "/asp/informationProcessing/localOptimization.lp");

  asp.addKnowledgeFile(path + "/test/data/asp_ram_test/" + fileName);

  asp.setNoWarnings(true);
  asp.init();

  auto lastQuery = asp.getExternal("query", {1}, "query", {1, 3, 10}, true);

  asp.ground("query", {1});
  asp.solve();

  std::cout << "Ram:  " << mm->getMemoryUsage().residentSetMax << ", diff " << mm->getMemoryUsage().residentSetMax - ram << std::endl;
  std::cout << "Java: " << mm->getMemoryUsage().javaRamUsageMax << ", diff " << mm->getMemoryUsage().javaRamUsageMax - java << std::endl;

  asp.printLastModel();
}


