#include <vector>
#include <ros/package.h>
#include <gringo/value.hh>
#include <chrono>

#include "ice/TypeDefs.h"
#include "ice/coordination/EngineState.h"
#include "ice/coordination/OntologyInterface.h"

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
  bool successful;

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

public:
  ModelGeneration(std::string path)
  {
    this->path = path;
  }

  ModelGenerationSeriesResult testSeries(std::string p_ontPath, std::vector<std::string>* p_requiredModelElements,
                                         int p_count, bool warmUp, bool global, bool verbose, int maxHopCount = 3,
                                         int maxStepCount = 10,
                                         std::function<void(supplementary::ClingWrapper *asp)> lambda = nullptr)
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

    VarianceOnline totalTimeVar, ontologyReadTimeVar, ontologyReasonerTimeVar, ontologyToASPTimeVar,
                   aspGroundingTimeVar, aspSolvingTimeVar, aspSatTimeVar, aspUnsatTimeVar, aspModelCountVar;

    if (warmUp)
    {
      std::cout << "Starting warm up " << std::flush;
      for (int i = 1; i < 101; ++i)
      {
        this->test(p_ontPath, p_requiredModelElements, true, global, false, maxHopCount, maxStepCount);

        if (i % 10 == 0)
        {
          std::cout << ".";
          std::cout << std::flush;
        }
      }
      std::cout << " done." << std::endl;
    }

    for (int i = 0; i < p_count; ++i)
    {
      std::cout << "Starting run " << (i + 1) << " ... ";
      start = std::chrono::system_clock::now();

      auto r = this->test(p_ontPath, p_requiredModelElements, false, global, verbose, maxHopCount, maxStepCount, lambda);

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

      if (i == 0)
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
      }

      end = std::chrono::system_clock::now();
      std::cout << "finished " << (r.successful ? "successful" : "unsuccessful") << " after: "
          << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms, processing time: "
          << r.totalTime << " ms" << std::endl;
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

    result.avg.totalTime /= p_count;
    result.avg.ontologyReadTime /= p_count;
    result.avg.ontologyReasonerTime /= p_count;
    result.avg.ontologyToASPTime /= p_count;
    result.avg.aspGroundingTime /= p_count;
    result.avg.aspSolvingTime /= p_count;
    result.avg.aspSatTime /= p_count;
    result.avg.aspUnsatTime /= p_count;
    result.avg.aspModelCount /= p_count;

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
    asp.addKnowledgeFile(path + "/asp/informationProcessing/processing.lp");
    asp.addKnowledgeFile(path + "/asp/informationProcessing/searchBottomUp.lp");
    if (global)
      asp.addKnowledgeFile(path + "/asp/informationProcessing/globalOptimization.lp");
    else
      asp.addKnowledgeFile(path + "/asp/informationProcessing/localOptimization.lp");
    asp.setNoWarnings(true);
    asp.init();

    // Initializing OwlAPI
    ice::OntologyInterface ontology(path + "/java/lib/");
    ontology.setLogging(false);
    ontology.addIRIMapper(path + "/ontology/");
    ontology.loadOntology(p_ontPath);

    start = std::chrono::system_clock::now();

    // Load ontology
    startOntologyRead = std::chrono::system_clock::now();
    ontology.loadOntologies();
    endOntologyRead = std::chrono::system_clock::now();

    // Reasoning ontology
    startOntologyReasoner = std::chrono::system_clock::now();
    ontology.initReasoner(true);
    endOntologyReasoner = std::chrono::system_clock::now();

    // Ontology 2 ASP
//    if (false == warmUp)
//    std::cout << "Ontology 2 ASP" << std::endl;
    startOntologyToASP = std::chrono::system_clock::now();
    const char* infoStructure = ontology.readInformationStructureAsASP();
    //this->entityTypeMap.clear();

    std::string programPart = "ontology" + 1;
    std::stringstream ss;
    std::string item;

    ss << infoStructure;
    delete infoStructure;

    while (std::getline(ss, item, '\n'))
    {
      if (item.find("entity(") == 0)
      {
        int index1 = item.find(",");
        int index2 = item.find(")");
        auto entity = item.substr(7, index1 - 7);
        auto entityType = item.substr(index1 + 1, index2 - index1 - 1);

//            this->entityTypeMap[entity] = entityType;
      }

      if (std::find(entities.begin(), entities.end(), item) == entities.end())
      {
        entities.push_back(item);
        asp.add(programPart, {}, item);
      }
    }

    auto ontSystems = ontology.getSystems();

    for (auto ontSystem : *ontSystems)
    {
      auto nodes = ontology.readNodesAndIROsAsASP(ontSystem);

      std::vector<const char*>* types = nodes->at(0);
      std::vector<const char*>* names = nodes->at(1);
      std::vector<const char*>* strings = nodes->at(2);
      std::vector<const char*>* aspStrings = nodes->at(3);
      std::vector<const char*>* cppStrings = nodes->at(4);

      for (int i = 0; i < names->size(); ++i)
      {
        const char* name = names->at(i);
        const char* elementStr = strings->at(i);
        const char* aspStr = aspStrings->at(i);
        const char* cppStr = cppStrings->at(i);
        const char* typeStr = types->at(i);
        ice::ASPElementType type;

        if (typeStr == nullptr)
        {
          delete name;
          delete elementStr;
          delete aspStr;
          delete cppStr;
          delete typeStr;

          continue;
        }
        else if (std::strcmp(typeStr, "COMPUTATION_NODE") == 0)
        {
          type = ice::ASPElementType::ASP_COMPUTATION_NODE;
        }
        else if (std::strcmp(typeStr, "SOURCE_NODE") == 0)
        {
          type = ice::ASPElementType::ASP_SOURCE_NODE;
        }
        else if (std::strcmp(typeStr, "REQUIRED_STREAM") == 0)
        {
          type = ice::ASPElementType::ASP_REQUIRED_STREAM;
        }
        else if (std::strcmp(typeStr, "MAP_NODE") == 0)
        {
          type = ice::ASPElementType::ASP_MAP_NODE;
        }
        else if (std::strcmp(typeStr, "IRO_NODE") == 0)
        {
          type = ice::ASPElementType::ASP_IRO_NODE;
        }
        else if (std::strcmp(typeStr, "REQUIRED_MAP") == 0)
        {
          type = ice::ASPElementType::ASP_REQUIRED_MAP;
        }
        else
        {
          delete name;
          delete elementStr;
          delete aspStr;
          delete cppStr;
          delete typeStr;

          continue;
        }

        auto external = asp.getExternal(elementStr);
        externals.push_back(external);

        switch (type)
        {
          case ice::ASPElementType::ASP_COMPUTATION_NODE:
          case ice::ASPElementType::ASP_SOURCE_NODE:
          case ice::ASPElementType::ASP_MAP_NODE:
          case ice::ASPElementType::ASP_IRO_NODE:
            external->assign(true);
            break;
          default:
            external->assign(true);
            break;
        }

        asp.add(name, {}, aspStr);

        asp.ground(name, {});

        delete name;
        delete elementStr;
        delete aspStr;
        delete cppStr;
        delete typeStr;
      }
      delete types;
      delete names;
      delete strings;
      delete aspStrings;
      delete cppStrings;

      delete ontSystem;
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
            std::cout << std::endl << asp.toStringLastModel(false);
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
      result.successful = false;
    }

    externals.clear();

    if (verbose)
      result.print();

    return result;
  }

};
