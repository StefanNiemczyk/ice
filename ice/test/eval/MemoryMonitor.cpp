/*
 * MemoryMonitor.cpp
 *
 *  Created on: 15.11.2016
 *      Author: sni
 */

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"


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

  double getResidentSetMax()
  {
    return this->residentSetMax;
  }

  double getVmUsageMax()
  {
    return this->vmUsageMax;
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
    this->residentSetMax = 0;
    this->vmUsageMax = 0;
  }

private:
  MemoryManager()
  {
    this->active = false;
    this->sleepTime = 100;
    this->residentSetMax = 0;
    this->vmUsageMax = 0;
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

      if (v > this->vmUsageMax)
        this->vmUsageMax = v;

      if (r > this->residentSetMax)
        this->residentSetMax = r;


      std::cout << r << std::endl;

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

  double vmUsageMax;
  double residentSetMax;
};
