#include <vector>
#include <ros/package.h>
#include <gringo/value.hh>
#include <chrono>

#include "ice/Logger.h"
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
  }
};

struct ModelGenerationSeriesResult
{
  int numberSuccessful;
  int numberTotal;
  ModelGenerationResult best;
  ModelGenerationResult worst;
  ModelGenerationResult avg;

  void print()
  {
    std::cout << "Result\t\t\t\t" << numberSuccessful << "/" << numberTotal << std::endl;
    std::cout << "Total\t\t\t\t" << avg.totalTime << " ms (avg)\t" << best.totalTime << " ms (best)\t"
        << worst.totalTime << " ms (worst)" << std::endl;
    std::cout << "Ontology read\t\t" << avg.ontologyReadTime << " ms (avg)\t" << best.ontologyReadTime << " ms (best)\t"
        << worst.ontologyReadTime << " ms (worst)" << std::endl;
    std::cout << "Ontology reasoning\t" << avg.ontologyReasonerTime << " ms (avg)\t" << best.ontologyReasonerTime
        << " ms (best)\t" << worst.ontologyReasonerTime << " ms (worst)" << std::endl;
    std::cout << "Ontology 2 ASP\t\t" << avg.ontologyToASPTime << " ms (avg)\t" << best.ontologyToASPTime
        << " ms (best)\t" << worst.ontologyToASPTime << " ms (worst)" << std::endl;
    std::cout << "ASP grounding\t\t" << avg.aspGroundingTime << " ms (avg)\t" << best.aspGroundingTime << " ms (best)\t"
        << worst.aspGroundingTime << " ms (worst)" << std::endl;
    std::cout << "ASP Solving\t\t\t" << avg.aspSolvingTime << " ms (avg)\t" << best.aspSolvingTime << " ms (best)\t"
        << worst.aspSolvingTime << " ms (worst)" << std::endl;
  }
};

class ModelGeneration
{
public:
  ModelGenerationSeriesResult testSeries(std::string p_ontPath, std::vector<std::string>* p_requiredModelElements,
                                         int p_count)
  {
    ModelGenerationSeriesResult result;
    result.numberTotal = p_count;
    result.numberSuccessful = 0;
    std::vector<ModelGenerationResult> testResults(p_count + 1);
    std::chrono::time_point<std::chrono::system_clock> start, end;

    for (int i = 0; i < p_count; ++i)
    {
      std::cout << "Starting run " << (i+1) << " ... ";
      start = std::chrono::system_clock::now();

      auto r = this->test(p_ontPath, p_requiredModelElements);

      if (r.successful)
        ++result.numberSuccessful;

      testResults.push_back(r);

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
      }

      end = std::chrono::system_clock::now();
      std::cout << "finished " << (r.successful ? "successful" : "unsuccessful") << " after: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms, processing time: " << r.totalTime << " ms" << std::endl;;
    }

    result.avg.totalTime /= p_count;
    result.avg.ontologyReadTime /= p_count;
    result.avg.ontologyReasonerTime /= p_count;
    result.avg.ontologyToASPTime /= p_count;
    result.avg.aspGroundingTime /= p_count;
    result.avg.aspSolvingTime /= p_count;

    return result;
  }

  ModelGenerationResult test(std::string p_ontPath, std::vector<std::string>* p_requiredModelElements)
  {
    ModelGenerationResult result;
    std::string path = ros::package::getPath("ice");
    std::vector<std::string> entities;
    std::vector<std::shared_ptr<supplementary::External>> externals;
    std::chrono::time_point<std::chrono::system_clock> start, end, startOntologyRead, endOntologyRead,
                                                       startOntologyReasoner, endOntologyReasoner, startOntologyToASP,
                                                       endOntologyToASP, startAspGrounding, endAspGrounding,
                                                       startAspSolving, endAspSolving;

    // Initializing ASP
    supplementary::ClingWrapper asp;
    asp.addKnowledgeFile(path + "/asp/nodeComposition.lp");
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
//        std::cout << elementStr << std::endl;
//        std::cout << aspStr << std::endl;

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

    // Grounding
    startAspGrounding = std::chrono::system_clock::now();
    asp.ground(programPart, {});

    auto lastQuery = asp.getExternal("query", {1}, true);
    asp.ground("query", {1});
    endAspGrounding = std::chrono::system_clock::now();

    // Solving
    startAspSolving = std::chrono::system_clock::now();
    auto solveResult = asp.solve();
    endAspSolving = std::chrono::system_clock::now();

    end = std::chrono::system_clock::now();

    result.successful = true;
    result.totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    result.ontologyReadTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endOntologyRead - startOntologyRead).count();
    result.ontologyReasonerTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endOntologyReasoner - startOntologyReasoner).count();
    result.ontologyToASPTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endOntologyToASP - startOntologyToASP).count();
    result.aspGroundingTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endAspGrounding - startAspGrounding).count();
    result.aspSolvingTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endAspSolving - startAspSolving).count();

    if (solveResult == Gringo::SolveResult::SAT)
    {
//      asp.printLastModel(false);

      for (auto toCheck : *p_requiredModelElements)
      {
        auto value = supplementary::ClingWrapper::splitASPExternalString(toCheck.c_str());
        std::string name = *value.name();
        if (false == asp.query(name, value.args()))
        {
          value.print(std::cout);
          std::cout << std::endl;
          result.successful = false;
//          break;
        }
      }
    }

    externals.clear();

    return result;
  }

//private:
//  Gringo::Value splitASPExternalString(std::string p_aspString)
//  {
//    if (p_aspString == "")
//      return Gringo::Value();
//
//    std::vector<Gringo::Value> vec;
//
//    int start = p_aspString.find("(");
//    int end = p_aspString.find_last_of(")");
//
//    if (start == std::string::npos || end == std::string::npos || start > end)
//    {
//      return Gringo::Value(p_aspString);
//    }
//
//    std::string name = p_aspString.substr(0, start);
//    std::string values = p_aspString.substr(start + 1, end - start - 1);
//
//    istringstream f(values);
//    std::string s;
//    int intNumber;
//
//    while (values != "")
//    {
//      //    std::cout << values << std::endl;
//      int index1 = values.find("(");
//      int index2 = values.find(",");
//
//      if (index2 == 0)
//      {
//        values = values.substr(1, values.size());
//        continue;
//      }
//
//      if (index2 != std::string::npos && index1 == std::string::npos)
//      {
//        s = values.substr(0, index2);
//        if (isNumber(&s, &intNumber))
//        {
//          vec.push_back(Gringo::Value(intNumber));
//        }
//        else
//        {
//          vec.push_back(Gringo::Value(s));
//        }
//        values = values.substr(index2 + 1, values.size());
//      }
//      else if (index2 == std::string::npos && index1 != std::string::npos)
//      {
//        index1 = values.find_last_of(")") + 1;
//        vec.push_back(this->splitASPExternalString(values.substr(0, index1)));
//        values = values.substr(index1, values.size());
//      }
//      else if (index2 == std::string::npos && index1 == std::string::npos)
//      {
//        s = values;
//        if (isNumber(&s, &intNumber))
//        {
//          vec.push_back(Gringo::Value(intNumber));
//        }
//        else
//        {
//          vec.push_back(Gringo::Value(s));
//        }
//        values = "";
//      }
//      else if (index2 < index1)
//      {
//        s = values.substr(0, index2);
//        if (isNumber(&s, &intNumber))
//        {
//          vec.push_back(Gringo::Value(intNumber));
//        }
//        else
//        {
//          vec.push_back(Gringo::Value(s));
//        }
//        values = values.substr(index2 + 1, values.size());
//      }
//      else
//      {
//        index1 = values.find_last_of(")") + 1;
//        vec.push_back(this->splitASPExternalString(values.substr(0, index1)));
//        values = values.substr(index1, values.size());
//      }
//    }
//
//    return Gringo::Value(name, vec);
//  }
//
//  bool isNumber(std::string* p_string, int* p_number)
//  {
//    std::string::size_type end;
//
//    try
//    {
//      *p_number = std::stoi(*p_string, &end, 10);
//    }
//    catch (std::invalid_argument &e)
//    {
//      return false;
//    }
//    return end == p_string->size();
//  }

};
