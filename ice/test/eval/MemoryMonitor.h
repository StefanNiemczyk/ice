/*
 * MemoryMonitor.cpp
 *
 *  Created on: 15.11.2016
 *      Author: sni
 */

#ifndef MEMORYMANAGER_H_
#define MEMORYMANAGER_H_

#include <chrono>
#include <ios>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <unistd.h>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "ice/ontology/OntologyInterface.h"

struct MemoryUsage
{
  double vmUsageMax;
  double residentSetMax;

  double javaRamUsageMax;
//  double javaMaxMemoryMax;
//  double javaFreeMemoryMax;
};

class MemoryManager
{
public:
  static MemoryManager* getInstance()
  {
    static MemoryManager mm;
    return &mm;
  }

public:
  ~MemoryManager() {}

  MemoryUsage& getMemoryUsage()
  {
    if (this->ontology)
    {
      double btmb = 1024.0 * 1024.0;
      double jt, jm, jf;

      this->ontology->getMemoryUsage(jt, jm, jf);


      jt /= btmb;
      jm /= btmb;
      jf /= btmb;

      if (jt > this->mu.javaRamUsageMax)
        this->mu.javaRamUsageMax = jt;
//      if (jt > this->mu.javaMaxMemoryMax)
//        this->mu.javaMaxMemoryMax = jm;
//      if (jt > this->mu.javaFreeMemoryMax)
//        this->mu.javaFreeMemoryMax = jf;
    }

    return this->mu;
  }

  bool isActive()
  {
    return this->active;
  }

  void start()
  {
    if (this->active)
      return;

    this->active = true;
    this->worker = std::thread(&MemoryManager::checkMemory, this);
  }

  void stop()
  {
    if (false == this->active)
      return;

    this->active = false;
    this->worker.join();
  }

  void reset()
  {
    this->mu.residentSetMax = 0;
    this->mu.vmUsageMax = 0;

    this->mu.javaRamUsageMax = 0;
//    this->mu.javaMaxMemoryMax = 0;
//    this->mu.javaFreeMemoryMax = 0;

    if (this->ontology)
      this->ontology->resetMemoryMonitor();
  }

  void resetOntologyInterface()
  {
    if (this->ontology)
    {
      this->ontology->stopMemoryMonitor();
    }
    this->ontology.reset();
  }

  void setOntologyInterface(std::shared_ptr<ice::OntologyInterface> &ontology)
  {
    this->ontology = ontology;
    this->ontology->startMemoryMonitor();
  }

private:
  MemoryManager()
  {
    this->active = false;
    this->sleepTime = 5;
    this->reset();
  }

  void checkMemory()
  {
    double v, r;
//    struct rusage usage;

    while (this->active)
    {
      this->process_mem_usage(v, r);
//      v = getV();
//      r = getR();

//      int ret = getrusage(RUSAGE_SELF, &usage);

      v /= 1024;
      r /= 1024;

      if (v > this->mu.vmUsageMax)
        this->mu.vmUsageMax = v;
      if (r > this->mu.residentSetMax)
        this->mu.residentSetMax = r;

//      std::cout << r << "java: " << jt << ", " << jm << ", " << jf << std::endl;

      std::this_thread::sleep_for(std::chrono::milliseconds(this->sleepTime));
    }
  }

//  int parseLine(char* line){
//      // This assumes that a digit will be found and the line ends in " Kb".
//      int i = strlen(line);
//      const char* p = line;
//      while (*p <'0' || *p > '9') p++;
//      line[i-3] = '\0';
//      i = atoi(p);
//      return i;
//  }
//
//  int getV(){ //Note: this value is in KB!
//      FILE* file = fopen("/proc/self/status", "r");
//      int result = -1;
//      char line[128];
//
//      while (fgets(line, 128, file) != NULL){
//          if (strncmp(line, "VmSize:", 7) == 0){
//              result = parseLine(line);
//              break;
//          }
//      }
//      fclose(file);
//      return result;
//  }
//
//  int getR(){ //Note: this value is in KB!
//      FILE* file = fopen("/proc/self/status", "r");
//      int result = -1;
//      char line[128];
//
//      while (fgets(line, 128, file) != NULL){
//          if (strncmp(line, "VmRSS:", 6) == 0){
//              result = parseLine(line);
//              break;
//          }
//      }
//      fclose(file);
//      return result;
//  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // process_mem_usage(double &, double &) - takes two doubles by reference,
  // attempts to read the system-dependent data for a process' virtual memory
  // size and resident set size, and return the results in KB.
  //
  // On failure, returns 0.0, 0.0

  void process_mem_usage(double& vm_usage, double& resident_set)
  {
     using std::ios_base;
     using std::ifstream;
     using std::string;

     vm_usage     = 0.0;
     resident_set = 0.0;

     // 'file' stat seems to give the most reliable results
     //
     ifstream stat_stream("/proc/self/stat",ios_base::in);

     // dummy vars for leading entries in stat that we don't care about
     //
     string pid, comm, state, ppid, pgrp, session, tty_nr;
     string tpgid, flags, minflt, cminflt, majflt, cmajflt;
     string utime, stime, cutime, cstime, priority, nice;
     string O, itrealvalue, starttime;

     // the two fields we want
     //
     unsigned long vsize;
     long rss;

     stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                 >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                 >> utime >> stime >> cutime >> cstime >> priority >> nice
                 >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

     stat_stream.close();

     long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
     vm_usage     = vsize / 1024.0;
     resident_set = rss * page_size_kb;
  }

private:
  std::thread worker;
  int sleepTime;
  bool active;
  std::shared_ptr<ice::OntologyInterface> ontology;
  MemoryUsage mu;
};


#endif

