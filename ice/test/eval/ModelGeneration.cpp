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
  long long totalTime;
  long long ontologyReadTime;
  long long ontologyReasonerTime;
  long long ontologyToASPTime;
  long long aspGroundingTime;
  long long aspSolvingTime;
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

class ModelGeneration
{
public:
  ModelGenerationResult readOntology(std::string p_ontPath, std::vector<std::string>* requiredModelElements)
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
    std::string infoStructure = ontology.readInformationStructureAsASP();
    //this->entityTypeMap.clear();

    std::string programPart = "ontology" + 1;
    std::stringstream ss;
    ss << infoStructure;
    std::string item;

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

      std::vector<std::string> types = nodes->at(0);
      std::vector<std::string> names = nodes->at(1);
      std::vector<std::string> strings = nodes->at(2);
      std::vector<std::string> aspStrings = nodes->at(3);
      std::vector<std::string> cppStrings = nodes->at(4);

      for (int i = 0; i < names.size(); ++i)
      {
        std::string name = names.at(i);
        std::string elementStr = strings.at(i);
        std::string aspStr = aspStrings.at(i);
        std::string cppStr = cppStrings.at(i);
        ice::ASPElementType type;

        if (types.at(i) == "COMPUTATION_NODE")
        {
          type = ice::ASPElementType::ASP_COMPUTATION_NODE;
        }
        else if (types.at(i) == "SOURCE_NODE")
        {
          type = ice::ASPElementType::ASP_SOURCE_NODE;
        }
        else if (types.at(i) == "REQUIRED_STREAM")
        {
          type = ice::ASPElementType::ASP_REQUIRED_STREAM;
        }
        else if (types.at(i) == "MAP_NODE")
        {
          type = ice::ASPElementType::ASP_MAP_NODE;
        }
        else if (types.at(i) == "IRO_NODE")
        {
          type = ice::ASPElementType::ASP_IRO_NODE;
        }
        else if (types.at(i) == "REQUIRED_MAP")
        {
          type = ice::ASPElementType::ASP_REQUIRED_MAP;
        }
        else
        {
          continue;
        }

        auto value = this->splitASPExternalString(elementStr);
        //              std::cout << value << std::endl;
        auto external = asp.getExternal(*value.name(), value.args());
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
      }
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
    result.ontologyReasonerTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endOntologyReasoner - startOntologyReasoner).count();
    result.ontologyToASPTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endOntologyToASP - startOntologyToASP).count();
    result.aspGroundingTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endAspGrounding - startAspGrounding).count();
    result.aspSolvingTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(endAspSolving - startAspSolving).count();

    if (solveResult == Gringo::SolveResult::SAT)
    {
      asp.printLastModel(false);

      for (auto toCheck : *requiredModelElements)
      {
        auto value = this->splitASPExternalString(toCheck);
        std::string name = *value.name();
        if (false == asp.query(name, value.args()))
        {
          result.successful = false;
          break;
        }
      }
    }

    externals.clear();

    return result;
  }

private:
  Gringo::Value splitASPExternalString(std::string p_aspString)
  {
    if (p_aspString == "")
      return Gringo::Value();

    std::vector<Gringo::Value> vec;

    int start = p_aspString.find("(");
    int end = p_aspString.find_last_of(")");

    if (start == std::string::npos || end == std::string::npos || start > end)
    {
      return Gringo::Value(p_aspString);
    }

    std::string name = p_aspString.substr(0, start);
    std::string values = p_aspString.substr(start + 1, end - start - 1);

    istringstream f(values);
    std::string s;

    while (values != "")
    {
      //    std::cout << values << std::endl;
      int index1 = values.find("(");
      int index2 = values.find(",");

      if (index2 != std::string::npos && index1 == std::string::npos)
      {
        vec.push_back(Gringo::Value(values.substr(0, index2)));
        values = values.substr(index2 + 1, values.size());
      }
      else if (index2 == std::string::npos && index1 != std::string::npos)
      {
        index1 = values.find_last_of(")") + 1;
        vec.push_back(this->splitASPExternalString(values.substr(0, index1)));
        values = values.substr(index1, values.size());
      }
      else if (index2 == std::string::npos && index1 == std::string::npos)
      {
        vec.push_back(Gringo::Value(values));
        values = "";
      }
      else if (index2 < index1)
      {
        vec.push_back(Gringo::Value(values.substr(0, index2)));
        values = values.substr(index2 + 1, values.size());
      }
      else
      {
        index1 = values.find_last_of(")") + 1;
        vec.push_back(this->splitASPExternalString(values.substr(0, index1)));
        values = values.substr(index1, values.size());
      }
    }

    return Gringo::Value(name, vec);
  }

};
