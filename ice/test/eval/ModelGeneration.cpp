#include <vector>
#include <ros/package.h>
#include <gringo/value.hh>
#include <chrono>

#include "ice/Entity.h"
#include "ice/EntityDirectory.h"
#include "ice/TypeDefs.h"
#include "ice/ontology/OntologyInterface.h"
#include "MemoryMonitor.h"

#include "ClingWrapper.h"
#include "External.h"

struct ModelGenerationResult
{
  unsigned long long totalTime;
  unsigned long long ontologyReadTime;
  unsigned long long ontologyReasonerTime;
  unsigned long long ontologyToASPTime;
  unsigned long long aspGroundingTime;
  unsigned long long aspSolvingTime;
  unsigned long long aspSatTime;
  unsigned long long aspUnsatTime;
  long aspModelCount;
  long aspAtomCount;
  long aspBodiesCount;
  long aspAuxAtomCount;
  bool successful;

  double vmUsageMax;
  double residentSetMax;

  double javaTotalMemoryMax;
  double javaMaxMemoryMax;
  double javaFreeMemoryMax;

  void print()
  {
    std::cout << "Result\t\t\t\t" << (successful ? "true" : "false") << std::endl;
    std::cout << "Total\t\t\t\t" << totalTime << " ms" << std::endl;
    std::cout << "Ontology read\t\t" << ontologyReadTime << " ms" << std::endl;
    std::cout << "Ontology reasoning\t" << ontologyReasonerTime << " ms" << std::endl;
    std::cout << "Ontology 2 ASP\t\t" << ontologyToASPTime << " ms" << std::endl;
    std::cout << "ASP grounding\t\t" << aspGroundingTime << " ms" << std::endl;
    std::cout << "ASP Solving\t\t\t" << aspSolvingTime << " ms" << std::endl;
    std::cout << "ASP Sat time\t\t" << aspSatTime << " ms" << std::endl;
    std::cout << "ASP unsat time\t\t" << aspUnsatTime << " ms" << std::endl;
    std::cout << "ASP model count\t\t" << aspModelCount << std::endl;
    std::cout << "ASP atom count\t\t" << aspAtomCount << std::endl;
    std::cout << "ASP bodies count\t" << aspBodiesCount << std::endl;
    std::cout << "ASP aux atom count\t" << aspAuxAtomCount << std::endl;

    std::cout << "Memory vm usage\t\t" << vmUsageMax << " mb" << std::endl;
    std::cout << "Memory ram usage\t\t" << residentSetMax << " mb" << std::endl;
    std::cout << "Memory java total\t\t" << javaTotalMemoryMax << " mb" << std::endl;
    std::cout << "Memory java max\t\t" << javaMaxMemoryMax << " mb" << std::endl;
    std::cout << "Memory java free\t\t" << javaFreeMemoryMax << " mb" << std::endl;
  }
};

struct ModelGenerationSeriesResult
{
  int numberSuccessful;
  int numberTotal;
  ModelGenerationResult best;
  ModelGenerationResult worst;
  ModelGenerationResult avg;

  double totalTimeVar;
  double ontologyReadTimeVar;
  double ontologyReasonerTimeVar;
  double ontologyToASPTimeVar;
  double aspGroundingTimeVar;
  double aspSolvingTimeVar;
  double aspSatTimeVar;
  double aspUnsatTimeVar;
  double aspModelCountVar;
  double aspAtomCountVar;
  double aspBodiesCountVar;
  double aspAuxAtomCountVar;

  double vmUsageMaxVar;
  double residentSetMaxVar;
  double javaTotalMemoryMaxVar;
  double javaMaxMemoryMaxVar;
  double javaFreeMemoryMaxVar;

  void print()
  {
    printf("Result              %5d / %5d successful runs\n", numberSuccessful, numberTotal);
    printf("Total                  %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.totalTime,
           totalTimeVar, best.totalTime, worst.totalTime);
    printf("Ontology Read          %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.ontologyReadTime,
           ontologyReadTimeVar, best.ontologyReadTime, worst.ontologyReadTime);
    printf("Ontology Reasoning     %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n",
           avg.ontologyReasonerTime, ontologyReasonerTimeVar, best.ontologyReasonerTime, worst.ontologyReasonerTime);
    printf("Ontology 2 ASP         %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.ontologyToASPTime,
           ontologyToASPTimeVar, best.ontologyToASPTime, worst.ontologyToASPTime);
    printf("ASP Grounding          %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.aspGroundingTime,
           aspGroundingTimeVar, best.aspGroundingTime, worst.aspGroundingTime);
    printf("ASP Solving            %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.aspSolvingTime,
           aspSolvingTimeVar, best.aspSolvingTime, worst.aspSolvingTime);
    printf("ASP Sat Time           %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.aspSatTime,
           aspSatTimeVar, best.aspSatTime, worst.aspSatTime);
    printf("ASP Unsat Time         %10llu ms (%10.3f var), %10llu ms (best), %10llu ms (worst)\n", avg.aspUnsatTime,
           aspUnsatTimeVar, best.aspUnsatTime, worst.aspUnsatTime);
    printf("ASP Model Count        %10lu    (%10.3f var), %10lu    (best), %10lu    (worst)\n", avg.aspModelCount,
           aspModelCountVar, best.aspModelCount, worst.aspModelCount);
    printf("ASP Atom Count         %10lu    (%10.3f var), %10lu    (best), %10lu    (worst)\n", avg.aspAtomCount,
           aspAtomCountVar, best.aspAtomCount, worst.aspAtomCount);
    printf("ASP Bodies Count       %10lu    (%10.3f var), %10lu    (best), %10lu    (worst)\n", avg.aspBodiesCount,
           aspBodiesCountVar, best.aspBodiesCount, worst.aspBodiesCount);
    printf("ASP Aux Atom Count     %10lu    (%10.3f var), %10lu    (best), %10lu    (worst)\n", avg.aspAuxAtomCount,
           aspAuxAtomCountVar, best.aspAuxAtomCount, worst.aspAuxAtomCount);

    printf("Memory vm usage        %10f    (%10.3f var), %10f    (best), %10f    (worst)\n", avg.vmUsageMax,
           vmUsageMaxVar, best.vmUsageMax, worst.vmUsageMax);
    printf("Memory ram usage       %10f    (%10.3f var), %10f    (best), %10f    (worst)\n", avg.residentSetMax,
           residentSetMaxVar, best.residentSetMax, worst.residentSetMax);
    printf("Memory java total      %10f    (%10.3f var), %10f    (best), %10f    (worst)\n", avg.javaTotalMemoryMax,
           javaTotalMemoryMaxVar, best.javaTotalMemoryMax, worst.javaTotalMemoryMax);
    printf("Memory java max        %10f    (%10.3f var), %10f    (best), %10f    (worst)\n", avg.javaMaxMemoryMax,
           javaMaxMemoryMaxVar, best.javaMaxMemoryMax, worst.javaMaxMemoryMax);
    printf("Memory java free       %10f    (%10.3f var), %10f    (best), %10f    (worst)\n", avg.javaFreeMemoryMax,
           javaFreeMemoryMaxVar, best.javaFreeMemoryMax, worst.javaFreeMemoryMax);
  }
};

class VarianceOnline
{
public:
  void add(double x)
  {
    n = n + 1;
    double delta = x - mean;
    mean = mean + delta / n;
    M2 = M2 + delta * (x - mean);
  }

  void remove(double x)
  {
    n = n - 1;
    double delta = x - mean;
    mean = mean - delta / n;
    M2 = M2 - delta * (x - mean);
  }

  void update(double oldX, double newX)
  {
    double delta = newX - oldX;
    double dold = oldX - mean;
    mean = mean + delta / n;
    double dnew = newX - mean;
    M2 = M2 + delta * (dold + dnew);
  }

  double getVariance()
  {
    if (n < 2)
      return 0;

    return M2 / (n - 1);
  }

  double getMean()
  {
    return this->mean;
  }

private:
  int n = 0;
  double mean = 0;
  double M2 = 0;
};

class ModelGeneration
{
private:
  std::string path;
  bool testRepresentations;

public:
  ModelGeneration(std::string path, bool testRepresentations = false)
  {
    this->path = path;
    this->testRepresentations = testRepresentations;
  }

  ModelGenerationSeriesResult testSeries(std::string p_ontPath, std::vector<std::string>* p_requiredModelElements,
                                         int p_count, bool warmUp, bool global, bool verbose, int maxHopCount = 3,
                                         int maxStepCount = 10,
                                         std::function<void(supplementary::ClingWrapper *asp)> lambda = nullptr, int models = 1)
  {
    ModelGenerationSeriesResult result;
    result.numberTotal = p_count;
    result.numberSuccessful = 0;
    std::vector<ModelGenerationResult> testResults(p_count + 1);
    std::chrono::time_point<std::chrono::system_clock> start, end;

    result.totalTimeVar = 0;
    result.ontologyReadTimeVar = 0;
    result.ontologyReasonerTimeVar = 0;
    result.ontologyToASPTimeVar = 0;
    result.aspGroundingTimeVar = 0;
    result.aspSolvingTimeVar = 0;

    result.vmUsageMaxVar = 0;
    result.residentSetMaxVar = 0;
    result.javaTotalMemoryMaxVar = 0;
    result.javaMaxMemoryMaxVar = 0;
    result.javaFreeMemoryMaxVar = 0;

    VarianceOnline totalTimeVar, ontologyReadTimeVar, ontologyReasonerTimeVar, ontologyToASPTimeVar,
                   aspGroundingTimeVar, aspSolvingTimeVar, aspSatTimeVar, aspUnsatTimeVar, aspModelCountVar,
                   aspAtomCountVar, aspBodiesCountVar, aspAuxAtomCountVar, vmUsageMaxVar, residentSetMaxVar,
                   javaTotalMemoryMaxVar, javaMaxMemoryMaxVar, javaFreeMemoryMaxVar;

    if (warmUp)
    {
      ice::OntologyInterface::callJniGc();

      std::cout << "Starting warm up " << std::flush;
      for (int i = 1; i < 101; ++i)
      {
        std::string ontology;

        if (this->testRepresentations)
        {
          ontology = p_ontPath + "0.owl";
        }
        else
        {
          ontology = p_ontPath;
        }

        this->test(ontology, p_requiredModelElements, true, global, false, maxHopCount, maxStepCount);

        if (i % 10 == 0)
        {
          std::cout << ".";
          std::cout << std::flush;
        }
      }
      std::cout << " done." << std::endl;
    }

    for (int m = 0; m < models; ++m)
    {
      for (int i = 0; i < p_count; ++i)
      {
        std::cout << "Starting run " << (i + 1) << " ... ";
        start = std::chrono::system_clock::now();

        std::string ontology;

        if (this->testRepresentations)
        {
          ontology = p_ontPath + std::to_string(m) + ".owl";
        }
        else
        {
          ontology = p_ontPath;
        }

        auto r = this->test(ontology, p_requiredModelElements, false, global, verbose, maxHopCount, maxStepCount, lambda);

        if (r.successful)
          ++result.numberSuccessful;

        testResults.push_back(r);

        totalTimeVar.add(r.totalTime);
        ontologyReadTimeVar.add(r.ontologyReadTime);
        ontologyReasonerTimeVar.add(r.ontologyReasonerTime);
        ontologyToASPTimeVar.add(r.ontologyToASPTime);
        aspGroundingTimeVar.add(r.aspGroundingTime);
        aspSolvingTimeVar.add(r.aspSolvingTime);
        aspSatTimeVar.add(r.aspSatTime);
        aspUnsatTimeVar.add(r.aspUnsatTime);
        aspModelCountVar.add(r.aspModelCount);
        aspAtomCountVar.add(r.aspAtomCount);
        aspBodiesCountVar.add(r.aspBodiesCount);
        aspAuxAtomCountVar.add(r.aspAuxAtomCount);

        vmUsageMaxVar.add(r.vmUsageMax);
        residentSetMaxVar.add(r.residentSetMax);
        javaTotalMemoryMaxVar.add(r.javaTotalMemoryMax);
        javaMaxMemoryMaxVar.add(r.javaMaxMemoryMax);
        javaFreeMemoryMaxVar.add(r.javaFreeMemoryMax);

        if (i == 0 && m == 0)
        {
          result.best = r;
          result.worst = r;
          result.avg = r;
        }
        else
        {
          result.avg.totalTime += r.totalTime;
          result.avg.ontologyReadTime += r.ontologyReadTime;
          result.avg.ontologyReasonerTime += r.ontologyReasonerTime;
          result.avg.ontologyToASPTime += r.ontologyToASPTime;
          result.avg.aspGroundingTime += r.aspGroundingTime;
          result.avg.aspSolvingTime += r.aspSolvingTime;
          result.avg.aspSatTime += r.aspSatTime;
          result.avg.aspUnsatTime += r.aspUnsatTime;
          result.avg.aspModelCount += r.aspModelCount;
          result.avg.aspAtomCount += r.aspAtomCount;
          result.avg.aspBodiesCount += r.aspBodiesCount;
          result.avg.aspAuxAtomCount += r.aspAuxAtomCount;

          result.avg.vmUsageMax += r.vmUsageMax;
          result.avg.residentSetMax += r.residentSetMax;
          result.avg.javaTotalMemoryMax += r.javaTotalMemoryMax;
          result.avg.javaMaxMemoryMax += r.javaMaxMemoryMax;
          result.avg.javaFreeMemoryMax += r.javaFreeMemoryMax;

          if (r.totalTime < result.best.totalTime)
            result.best.totalTime = r.totalTime;
          else if (r.totalTime > result.worst.totalTime)
            result.worst.totalTime = r.totalTime;

          if (r.ontologyReadTime < result.best.ontologyReadTime)
            result.best.ontologyReadTime = r.ontologyReadTime;
          else if (r.ontologyReadTime > result.worst.ontologyReadTime)
            result.worst.ontologyReadTime = r.ontologyReadTime;

          if (r.ontologyReasonerTime < result.best.ontologyReasonerTime)
            result.best.ontologyReasonerTime = r.ontologyReasonerTime;
          else if (r.ontologyReasonerTime > result.worst.ontologyReasonerTime)
            result.worst.ontologyReasonerTime = r.ontologyReasonerTime;

          if (r.ontologyToASPTime < result.best.ontologyToASPTime)
            result.best.ontologyToASPTime = r.ontologyToASPTime;
          else if (r.ontologyToASPTime > result.worst.ontologyToASPTime)
            result.worst.ontologyToASPTime = r.ontologyToASPTime;

          if (r.aspGroundingTime < result.best.aspGroundingTime)
            result.best.aspGroundingTime = r.aspGroundingTime;
          else if (r.aspGroundingTime > result.worst.aspGroundingTime)
            result.worst.aspGroundingTime = r.aspGroundingTime;

          if (r.aspSolvingTime < result.best.aspSolvingTime)
            result.best.aspSolvingTime = r.aspSolvingTime;
          else if (r.aspSolvingTime > result.worst.aspSolvingTime)
            result.worst.aspSolvingTime = r.aspSolvingTime;

          if (r.aspSatTime < result.best.aspSatTime)
            result.best.aspSatTime = r.aspSatTime;
          else if (r.aspSatTime > result.worst.aspSatTime)
            result.worst.aspSatTime = r.aspSatTime;

          if (r.aspUnsatTime < result.best.aspUnsatTime)
            result.best.aspUnsatTime = r.aspUnsatTime;
          else if (r.aspUnsatTime > result.worst.aspUnsatTime)
            result.worst.aspUnsatTime = r.aspUnsatTime;

          if (r.aspModelCount < result.best.aspModelCount)
            result.best.aspModelCount = r.aspModelCount;
          else if (r.aspModelCount > result.worst.aspModelCount)
            result.worst.aspModelCount = r.aspModelCount;

          if (r.aspAtomCount < result.best.aspAtomCount)
            result.best.aspAtomCount = r.aspAtomCount;
          else if (r.aspAtomCount > result.worst.aspAtomCount)
            result.worst.aspAtomCount = r.aspAtomCount;

          if (r.aspBodiesCount < result.best.aspBodiesCount)
            result.best.aspBodiesCount = r.aspBodiesCount;
          else if (r.aspBodiesCount > result.worst.aspBodiesCount)
            result.worst.aspBodiesCount = r.aspBodiesCount;

          if (r.aspAuxAtomCount < result.best.aspAuxAtomCount)
            result.best.aspAuxAtomCount = r.aspAuxAtomCount;
          else if (r.aspAuxAtomCount > result.worst.aspAuxAtomCount)
            result.worst.aspAuxAtomCount = r.aspAuxAtomCount;

          if (r.vmUsageMax < result.best.vmUsageMax)
            result.best.vmUsageMax = r.vmUsageMax;
          else if (r.vmUsageMax > result.worst.vmUsageMax)
            result.worst.vmUsageMax = r.vmUsageMax;

          if (r.residentSetMax < result.best.residentSetMax)
            result.best.residentSetMax = r.residentSetMax;
          else if (r.residentSetMax > result.worst.residentSetMax)
            result.worst.residentSetMax = r.residentSetMax;

          if (r.javaTotalMemoryMax < result.best.javaTotalMemoryMax)
            result.best.javaTotalMemoryMax = r.javaTotalMemoryMax;
          else if (r.javaTotalMemoryMax > result.worst.javaTotalMemoryMax)
            result.worst.javaTotalMemoryMax = r.javaTotalMemoryMax;

          if (r.javaMaxMemoryMax < result.best.javaMaxMemoryMax)
            result.best.javaMaxMemoryMax = r.javaMaxMemoryMax;
          else if (r.javaMaxMemoryMax > result.worst.javaMaxMemoryMax)
            result.worst.javaMaxMemoryMax = r.javaMaxMemoryMax;

          if (r.javaFreeMemoryMax < result.best.javaFreeMemoryMax)
            result.best.javaFreeMemoryMax = r.javaFreeMemoryMax;
          else if (r.javaFreeMemoryMax > result.worst.javaFreeMemoryMax)
            result.worst.javaFreeMemoryMax = r.javaFreeMemoryMax;
        }

        end = std::chrono::system_clock::now();
        std::cout << "finished " << (r.successful ? "successful" : "unsuccessful") << " after: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms, processing time: "
            << r.totalTime << " ms" << std::endl;
      }
    }

    result.totalTimeVar = totalTimeVar.getVariance();
    result.ontologyReadTimeVar = ontologyReadTimeVar.getVariance();
    result.ontologyReasonerTimeVar = ontologyReasonerTimeVar.getVariance();
    result.ontologyToASPTimeVar = ontologyToASPTimeVar.getVariance();
    result.aspGroundingTimeVar = aspGroundingTimeVar.getVariance();
    result.aspSolvingTimeVar = aspSolvingTimeVar.getVariance();
    result.aspSatTimeVar = aspSatTimeVar.getVariance();
    result.aspUnsatTimeVar = aspUnsatTimeVar.getVariance();
    result.aspModelCountVar = aspModelCountVar.getVariance();
    result.aspAtomCountVar = aspAtomCountVar.getVariance();
    result.aspBodiesCountVar = aspBodiesCountVar.getVariance();
    result.aspAuxAtomCountVar = aspAuxAtomCountVar.getVariance();

    result.vmUsageMaxVar = vmUsageMaxVar.getVariance();
    result.residentSetMaxVar = residentSetMaxVar.getVariance();
    result.javaTotalMemoryMaxVar = javaTotalMemoryMaxVar.getVariance();
    result.javaMaxMemoryMaxVar = javaMaxMemoryMaxVar.getVariance();
    result.javaFreeMemoryMaxVar = javaFreeMemoryMaxVar.getVariance();

    int runCount = models * p_count;

    result.avg.totalTime /= runCount;
    result.avg.ontologyReadTime /= runCount;
    result.avg.ontologyReasonerTime /= runCount;
    result.avg.ontologyToASPTime /= runCount;
    result.avg.aspGroundingTime /= runCount;
    result.avg.aspSolvingTime /= runCount;
    result.avg.aspSatTime /= runCount;
    result.avg.aspUnsatTime /= runCount;
    result.avg.aspModelCount /= runCount;
    result.avg.aspAtomCount /= runCount;
    result.avg.aspBodiesCount /= runCount;
    result.avg.aspAuxAtomCount /= runCount;

    result.avg.vmUsageMax /= runCount;
    result.avg.residentSetMax /= runCount;
    result.avg.javaTotalMemoryMax /= runCount;
    result.avg.javaMaxMemoryMax /= runCount;
    result.avg.javaFreeMemoryMax /= runCount;

    return result;
  }

  ModelGenerationResult test(std::string p_ontPath, std::vector<std::string>* p_requiredModelElements,
                             bool warmUp, bool global, bool verbose, int maxHopCount = 3, int maxStepCount = 10,
                             std::function<void(supplementary::ClingWrapper *asp)> lambda = nullptr)
  {
    ModelGenerationResult result;
    std::vector<std::string> entities;
    std::vector<std::shared_ptr<supplementary::External>> externals;
    std::chrono::time_point<std::chrono::system_clock> start, end, startOntologyRead, endOntologyRead,
                                                       startOntologyReasoner, endOntologyReasoner, startOntologyToASP,
                                                       endOntologyToASP, startAsp, endAsp;

    // Initializing ASP
    supplementary::ClingWrapper asp;

    if (this->testRepresentations)
    {
      asp.addKnowledgeFile(path + "/asp/transformation/computing.lp");
    }
    else
    {
      asp.addKnowledgeFile(path + "/asp/informationProcessing/processing.lp");
      asp.addKnowledgeFile(path + "/asp/informationProcessing/searchBottomUp.lp");
      if (global)
        asp.addKnowledgeFile(path + "/asp/informationProcessing/globalOptimization.lp");
      else
        asp.addKnowledgeFile(path + "/asp/informationProcessing/localOptimization.lp");
    }

    asp.setNoWarnings(true);
    asp.init();

    // Initializing OwlAPI
    auto ontology = std::make_shared<ice::OntologyInterface>(path + "/java/lib/");
    ontology->setLogLevel(ice::LogLevel::Error);
    ontology->addIRIMapper(path + "/ontology/");
    ontology->loadOntology(p_ontPath);

    // MemoryManagement
    auto mm = MemoryManager::getInstance();
    mm->setOntologyInterface(ontology);
    mm->reset();

    start = std::chrono::system_clock::now();

    // Load ontology
    startOntologyRead = std::chrono::system_clock::now();
    ontology->loadOntologies();
    endOntologyRead = std::chrono::system_clock::now();

    // Reasoning ontology
    startOntologyReasoner = std::chrono::system_clock::now();
    ontology->initReasoner(true);
    endOntologyReasoner = std::chrono::system_clock::now();

    // Ontology 2 ASP
//    if (false == warmUp)
//    std::cout << "Ontology 2 ASP" << std::endl;
    startOntologyToASP = std::chrono::system_clock::now();

    std::string programPart = "ontology" + 1;
    std::string item, noIri;
    std::stringstream ss;
    ss.str(ontology->readInformationStructureAsASP());

    while (std::getline(ss, item, '\n'))
    {
      noIri = ontology->toShortIriAll(item);
      if (std::find(entities.begin(), entities.end(), noIri) == entities.end())
      {
        entities.push_back(noIri);
        asp.add(programPart, {}, noIri);
      }
    }

    if (false == this->testRepresentations)
    {
    //
      auto ontSystems = ontology->getSystems();

//       std::string iriSelf;
//       self->getId(EntityDirectory::ID_ONTOLOGY, iriSelf);
//       iriSelf = ontology.toShortIri(iriSelf);

       std::shared_ptr<ice::Entity> entity;

       for (auto ontSystem : *ontSystems)
       {
         asp.add("base", {}, "system(" + ontology->toShortIri(ontSystem) + ",default)." );

//         entity = directory->lookup(EntityDirectory::ID_ONTOLOGY, ontSystem, true);
//
//         if (entity->getExternal() == nullptr)
//         {
//           std::string iri = ontology.toShortIri(ontSystem);
//           auto ext = asp.getExternal("system", {Gringo::Value(iri), "default"}, "system", {Gringo::Value(iri)}, true);
//           entity->setExternal(ext);
//
//           asp.add("base", {}, "transfer(" + iri + "," + iriSelf + ") :- system(" + iri + ",default).");
//         }

         auto nodes = ontology->readNodesAndIROsAsASP(ontSystem);

         auto &types = nodes->at(0);
         auto &names = nodes->at(1);
         auto &strings = nodes->at(2);
         auto &aspStrings = nodes->at(3);
         auto &cppStrings = nodes->at(4);

         for (int i = 0; i < names.size(); ++i)
         {
           std::string &name = names.at(i);
           std::string &elementStr = strings.at(i);
           std::string &aspStr = aspStrings.at(i);
           std::string &cppStr = cppStrings.at(i);
           std::string &typeStr = types.at(i);
           ice::ASPElementType type;

           if (typeStr == "" || name == "" || elementStr == "")
           {
             continue;
           }

           if (typeStr == "COMPUTATION_NODE")
           {
             type = ice::ASPElementType::ASP_COMPUTATION_NODE;
           }
           else if (typeStr == "SOURCE_NODE")
           {
             type = ice::ASPElementType::ASP_SOURCE_NODE;
           }
           else if (typeStr == "REQUIRED_STREAM")
           {
             type = ice::ASPElementType::ASP_REQUIRED_STREAM;
           }
           else if (typeStr == "MAP_NODE")
           {
             type = ice::ASPElementType::ASP_MAP_NODE;
           }
           else if (typeStr == "IRO_NODE")
           {
             type = ice::ASPElementType::ASP_IRO_NODE;
           }
           else if (typeStr == "REQUIRED_MAP")
           {
             type = ice::ASPElementType::ASP_REQUIRED_MAP;
           }
           else
           {
//             _log->error("Unknown asp element type '%v' for element '%v', element will be skipped", typeStr, name);
             continue;
           }

           auto node = std::make_shared<ice::ASPElement>();

//             _log->info("ASP element '%v' not found, creating new element", std::string(name));
             auto element = std::make_shared<ice::ASPElement>();
             element->aspString = ontology->toShortIriAll(aspStr);
             element->name = ontology->toShortIri(name);
             element->state = ice::ASPElementState::ADDED_TO_ASP;
             element->type = type;

             if (cppStr != "")
             {
               int index = cppStr.find('\n');

               if (index == std::string::npos)
                 continue;

               element->className = cppStr.substr(0, index);
               element->configAsString = cppStr.substr(index + 1);
               //element->config = "";//this->readConfiguration(element->configAsString);
             }

             std::string shortIris = ontology->toShortIriAll(elementStr);
             auto value = supplementary::ClingWrapper::stringToValue(shortIris.c_str());

//             switch (type)
//             {
//               case ice::ASPElementType::ASP_COMPUTATION_NODE:
//               case ice::ASPElementType::ASP_SOURCE_NODE:
//               case ice::ASPElementType::ASP_MAP_NODE:
//               case ice::ASPElementType::ASP_IRO_NODE:
//                 if (false == this->nodeStore->existNodeCreator(element->className))
//                 {
////                   _log->warn("Missing creator for node '%v' of type '%v', cpp grounding '%v', asp external set to false",
////                              element->name, ASPElementTypeNames[type],
////                              element->className == "" ? "NULL" : element->className);
//                   continue;
//                 }
//                 break;
//               default:
//                 break;
//             }

             element->external = asp.getExternal(*value.name(), value.args());
             element->external->assign(true);

             asp.add(element->name, {}, element->aspString);
             asp.ground(element->name, {});
         }
       }
      //
    }

    endOntologyToASP = std::chrono::system_clock::now();

    if (warmUp)
    {
      externals.clear();
      return result;
    }

    if (lambda != nullptr)
      lambda(&asp);

    // Grounding
//    std::cout << "ASP ground call" << std::endl;
    startAsp = std::chrono::system_clock::now();
    asp.ground(programPart, {});

    auto lastQuery = asp.getExternal("query", {1}, "query", {1, maxHopCount, maxStepCount}, true);
    asp.ground("query", {1});

    // Solving
//    std::cout << "ASP solving" << std::endl;
    auto solveResult = asp.solve();
//    std::cout << "ASP done" << std::endl;
    endAsp = std::chrono::system_clock::now();

    end = std::chrono::system_clock::now();

    result.successful = true;
    result.totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    result.ontologyReadTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endOntologyRead - startOntologyRead).count();
    result.ontologyReasonerTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endOntologyReasoner - startOntologyReasoner).count();
    result.ontologyToASPTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endOntologyToASP - startOntologyToASP).count();
    result.aspSolvingTime = asp.getSolvingTime();
    result.aspGroundingTime = std::chrono::duration_cast<std::chrono::milliseconds>(endAsp - startAsp).count()
        - result.aspSolvingTime;
    result.aspSatTime = asp.getSatTime();
    result.aspUnsatTime = asp.getUnsatTime();
    result.aspModelCount = asp.getModelCount();
    result.aspAtomCount = asp.getAtomCount();
    result.aspBodiesCount = asp.getBodiesCount();
    result.aspAuxAtomCount = asp.getAuxAtomsCount();

    auto &mu = mm->getMemoryUsage();
    result.vmUsageMax = mu.vmUsageMax;
    result.residentSetMax = mu.residentSetMax;
    result.javaTotalMemoryMax = mu.javaTotalMemoryMax;
    result.javaMaxMemoryMax = mu.javaMaxMemoryMax;
    result.javaFreeMemoryMax = mu.javaFreeMemoryMax;
    mm->resetOntologyInterface();

    if (solveResult == Gringo::SolveResult::SAT)
    {
      if (verbose)
      {
        asp.printLastModel(false);
        ofstream file;
        file.open("/tmp/tut.txt");
        file << asp.toStringLastModel(true);
        file.close();
      }

      bool first = true;

      for (auto toCheck : *p_requiredModelElements)
      {
        auto value = supplementary::ClingWrapper::stringToValue(toCheck.c_str());
        std::string name = *value.name();
        if (false == asp.query(name, value.args()))
        {
          if (first)
          {
            if (false == verbose)
              std::cout << std::endl << asp.toStringLastModel(false) << std::endl;
            first = false;
          }

          value.print(std::cout);
          std::cout << std::endl;
          result.successful = false;
        }
      }
    }
    else
    {
      std::cout << "UNSAT" << std::endl;
      result.successful = false;
    }

    externals.clear();

    if (verbose)
      result.print();

    return result;
  }

};
