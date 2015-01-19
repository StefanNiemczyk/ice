/*
 * ASPCoordinator.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: sni
 */

#include "ice/coordination/ASPCoordinator.h"

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/InformationStore.h"
#include "ice/processing/NodeStore.h"
#include "easylogging++.h"

namespace ice
{

ASPCoordinator::ASPCoordinator(std::weak_ptr<ICEngine> engine, std::string const ownName)
{
  this->_log = el::Loggers::getLogger("ASPCoordinator");
  _log->verbose("Constructor", "Constructor called");

  auto en = engine.lock();
  this->nodeStore = en->getNodeStore();
  this->informationStore = en->getInformationStore();

  this->engine = engine;
  this->queryIndex = 0;
  this->groundingDirty = true;
  this->self = this->getEngineStateByIRI(ownName);

  std::string path = ros::package::getPath("ice");
  _log->debug("Constructor", "Default ontology path %s", path.c_str());

  // Initializing ASP
  this->asp = std::make_shared<supplementary::ClingWrapper>();
  this->asp->addKnowledgeFile(path + "/asp/nodeComposition.lp");
  this->asp->init();

  // Initializing OwlAPI
  this->ontology = std::make_shared<OntologyInterface>(path + "/java/lib/");
  this->ontology->addIRIMapper(path + "/ontology/");
}

ASPCoordinator::~ASPCoordinator()
{
  // nothing to do here
}

void ASPCoordinator::optimizeInformationFlow()
{
  _log->verbose("optimizeInformationFlow", "Start optimizing");

  if (this->ontology->isLoadDirty())
  {
    _log->debug("optimizeInformationFlow", "Load flag dirty, reload ontologies");
    this->ontology->loadOntologies();
  }

  if (this->ontology->isInformationDirty())
  {
    _log->debug("optimizeInformationFlow", "Information model flag dirty, reload information structure");
    this->readInfoStructureFromOntology();
  }

  if (this->ontology->isSystemDirty())
  {
    _log->debug("optimizeInformationFlow", "System flag dirty, reload systems");
    this->readSystemsFromOntology();
  }

  if (groundingDirty)
  {
    this->groundingDirty = false;
    _log->debug("optimizeInformationFlow", "Grounding flag dirty, grounding asp program");

    if (this->lastQuery)
    {
      this->lastQuery->assign(false);
      this->lastQuery->release();
    }

    this->lastQuery = this->asp->getExternal("query", {++this->queryIndex}, true);
    this->asp->ground("query", {this->queryIndex});
  }

  // Solving
  auto solveResult = this->asp->solve();

  _log->info("optimizeInformationFlow", "Solving finished: %d", solveResult);

  if (solveResult == Gringo::SolveResult::SAT)
  {
    //_log->verbose("optimizeInformationFlow","Resulting Model \n%s", this->asp->printLastModel());
    this->asp->printLastModel();
    bool valid = true;
    std::vector<std::shared_ptr<Node>> nodes;

    // node(1,testSystem,testSourceNodeInd,testEntity1)
    std::vector<Gringo::Value> values;
    values.push_back(this->queryIndex);
    values.push_back(std::string(this->self->getSystemIriShort()));
    values.push_back("?");
    values.push_back("?");
    values.push_back("?");
    auto nodeQuery = std::make_shared<Gringo::Value>("node", values);

    auto queryResult = this->asp->queryAllTrue(nodeQuery);

    for (auto nodeValue : *queryResult)
    {
      auto nodeName = *nodeValue.args()[2].name();
      auto nodeEntity = *nodeValue.args()[3].name();
      auto nodeEntity2 = *nodeValue.args()[4].name();

      _log->debug("optimizeInformationFlow", "Look up node '%s' to process entity '%s'", nodeName.c_str(),
                  nodeEntity.c_str());

      auto aspNode = this->self->getASPElementByName(nodeName);

      if (aspNode == nullptr)
      {
        _log->error("optimizeInformationFlow", "No node '%s' found, asp system description is invalid!",
                    nodeName.c_str());
        valid = false;
        break;
      }

      NodeType type;
      switch (aspNode->type)
      {
        case ASPElementType::ASP_SOURCE_NODE:
          type = NodeType::SOURCE;
          break;
        case ASPElementType::ASP_COMPUTATION_NODE:
          type = NodeType::PROCESSING;
          break;
      }

      auto node = this->nodeStore->registerNode(type, aspNode->className, aspNode->name, nodeEntity, aspNode->config);

      if (node == nullptr)
      {
        _log->error("optimizeInformationFlow",
                    "Node '%s' (%s) could not be created, asp system description is invalid!", nodeName.c_str(),
                    aspNode->className.c_str());
        valid = false;
        break;
      }
//      connect(1,testSystem,testComputationalNodeInd,testSourceNodeInd,testSystem,
//        information(testEntity1,testScope1,testRepresentation1,none))
      std::vector<Gringo::Value> values;
      values.push_back(this->queryIndex);
      values.push_back(std::string(this->self->getSystemIriShort()));
      values.push_back(Gringo::Value(aspNode->name));
      values.push_back(Gringo::Value(nodeEntity));
      values.push_back(Gringo::Value(nodeEntity2));
      values.push_back("?");
      values.push_back("?");
      values.push_back("?");
      values.push_back("?");
      auto connectQuery = std::make_shared<Gringo::Value>("connectToNode", values);
      auto connectResult = this->asp->queryAllTrue(connectQuery);

      if (false == connectResult)
      {
        _log->error("optimizeInformationFlow", "No asp model by look up of connected streams to node '%s'",
                    aspNode->name.c_str());
        valid = false;
        break;
      }
      else
      {
        for (auto connect : *connectResult)
        {
          _log->debug("optimizeInformationFlow", "Look up connected stream for node '%s'", nodeName.c_str());

          auto lastProcessing = *connect.args()[5].name();
          auto sourceSystem = *connect.args()[6].name();
          auto info = connect.args()[7];
          auto step = connect.args()[8];
          auto stream = this->getStream(info, lastProcessing, sourceSystem, step);

          if (false == stream)
          {
            std::stringstream o;
            o << connect;
            _log->error("optimizeInformationFlow",
                        "Stream '%s' could not be created, asp system description is invalid!", o.str().c_str());
            valid = false;
            break;
          }

          node->addInput(stream, true); // TODO stream is trigger?
        }
      }

      // stream(1,testSystem,testSourceNodeInd,testSystem,information(testEntity1,testScope1,testRepresentation1,none),step)
      values.clear();
      values.push_back(this->queryIndex);
      values.push_back(std::string(this->self->getSystemIriShort()));
      values.push_back(Gringo::Value(aspNode->name));
      values.push_back(std::string(this->self->getSystemIriShort()));
      values.push_back("?");
      values.push_back("?");
      auto streamQuery = std::make_shared<Gringo::Value>("stream", values);
      auto streamResult = this->asp->queryAllTrue(streamQuery);

      for (auto output : *streamResult)
      {
        _log->debug("optimizeInformationFlow", "Look up output stream for node '%s'", nodeName.c_str());

        auto lastProcessing = *output.args()[2].name();
        auto sourceSystem = *output.args()[3].name();
        auto info = output.args()[4];
        auto step = output.args()[5];
        auto stream = this->getStream(info, lastProcessing, sourceSystem, step);

        if (false == stream)
        {
          std::stringstream o;
          o << output;
          _log->error("optimizeInformationFlow", "Stream '%s' could not be created, asp system description is invalid!",
                      o.str().c_str());
          valid = false;
          break;
        }

        node->addOutput(stream);
      }

      if (false == valid)
        break;

      nodes.push_back(node);
    }

    // TODO interpret IRO
    // TODO interpret information extraction
    // TODO interpret map
    // TODO selected stream

    if (valid)
    {
      _log->info("optimizeInformationFlow", "Optimizing successfully completed");
      this->nodeStore->cleanUpUnusedNodes(nodes);
      for (auto node : nodes)
      {
        node->activate();
      }
    }
    else
    {
      _log->error("optimizeInformationFlow", "Optimizing failed, no processing was established");
      for (auto node : nodes)
      {
        this->nodeStore->cleanUpNodes(nodes);
      }
    }

    this->informationStore->cleanUpStreams();
  }

  _log->verbose("optimizeInformationFlow", "End optimizing");
}

void ASPCoordinator::readInfoStructureFromOntology()
{
  _log->verbose("readInfoStructureFromOntology", "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  if (false == this->ontology->isInformationDirty())
    return;

  const char* infoStructure = this->ontology->readInformationStructureAsASP();

  _log->debug("readInfoStructureFromOntology", "Extracted structure from ontology");
  _log->verbose("readInfoStructureFromOntology", infoStructure);

  this->entityTypeMap.clear();

  std::string programPart = "ontology" + this->queryIndex;
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

      this->entityTypeMap[entity] = entityType;
    }

    if (std::find(this->entities.begin(), this->entities.end(), item) == this->entities.end())
    {
//      std::cout << item << std::endl;
      this->entities.push_back(item);
      this->asp->add(programPart, {}, item);
    }
  }

  this->asp->ground(programPart, {});

  this->groundingDirty = true;
}

void ASPCoordinator::readSystemsFromOntology()
{
  _log->verbose("readSystemsFromOntology", "Read systems from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  if (false == this->ontology->isSystemDirty())
    return;

  auto ontSystems = this->ontology->getSystems();

  if (ontSystems == nullptr)
  {
    _log->error("readSystemsFromOntology", "Error occurred while reading systems");
    return;
  }

  std::shared_ptr<EngineState> system;

  for (auto ontSystem : *ontSystems)
  {
    _log->debug("readSystemsFromOntology", "Checking system " + std::string(ontSystem));
    system = this->getEngineStateByIRI(ontSystem);

    auto nodes = this->ontology->readNodesAndIROsAsASP(ontSystem);

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
      ASPElementType type;

      if (typeStr == "COMPUTATION_NODE")
      {
        type = ASPElementType::ASP_COMPUTATION_NODE;
      }
      else if (typeStr == "SOURCE_NODE")
      {
        type = ASPElementType::ASP_SOURCE_NODE;
      }
      else if (typeStr == "REQUIRED_STREAM")
      {
        type = ASPElementType::ASP_REQUIRED_STREAM;
      }
      else if (typeStr == "MAP_NODE")
      {
        type = ASPElementType::ASP_MAP_NODE;
      }
      else if (typeStr == "IRO_NODE")
      {
        type = ASPElementType::ASP_IRO_NODE;
      }
      else if (typeStr == "REQUIRED_MAP")
      {
        type = ASPElementType::ASP_REQUIRED_MAP;
      }
      else
      {
        _log->error("readSystemsFromOntology", "Unknown asp element type '%s', element will be skipped",
                    types->at(i));

        delete name;
        delete elementStr;
        delete aspStr;
        delete cppStr;
        delete typeStr;

        continue;
      }

      auto node = system->getASPElementByName(type, name);

      if (!node)
      {
        _log->debug("readSystemsFromOntology", "ASP element '%s' not found, creating new element",
                    std::string(name).c_str());
        auto element = std::make_shared<ASPElement>();
        element->aspString = aspStr;
        element->name = name;
        element->state = ASPElementState::ADDED_TO_ASP;
        element->type = type;

        if (std::strlen(cppStr) != 0)
        {
          const char* index = std::strchr(cppStr, '\n');
          element->className = std::string(cppStr, index);
          element->config = this->readConfiguration(std::string(index + 1, std::strlen(cppStr)));
        }

        auto value = this->splitASPExternalString(elementStr);
//              std::cout << value << std::endl;
        element->external = this->asp->getExternal(*value.name(), value.args());

        switch (type)
        {
          case ASPElementType::ASP_COMPUTATION_NODE:
          case ASPElementType::ASP_SOURCE_NODE:
          case ASPElementType::ASP_MAP_NODE:
          case ASPElementType::ASP_IRO_NODE:
            if (false == this->nodeStore->existNodeCreator(element->className))
            {
              _log->warning("checkASPFromOntology",
                            "Missing creator for node '%s' of type '%s', cpp grounding '%s', asp external set to false",
                            element->name.c_str(), ASPElementTypeNames[type].c_str(), element->className.c_str());
              element->external->assign(false);
            }
            else
            {
              element->external->assign(true);
            }
            break;
          default:
            element->external->assign(true);
            break;
        }

        this->asp->add(name, {}, aspStr);
        std::cout << aspStr << std::endl;
        this->asp->ground(name, {});

        system->addASPElement(element);
        this->groundingDirty = true;
      }

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
}

std::shared_ptr<OntologyInterface> ASPCoordinator::getOntologyInterface()
{
  return this->ontology;
}

std::shared_ptr<supplementary::ClingWrapper> ASPCoordinator::getClingWrapper()
{
  return this->asp;
}

Gringo::Value ASPCoordinator::splitASPExternalString(std::string p_aspString)
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

    if (index2 == 0) {
      values = values.substr(1, values.size());
      continue;
    }

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

std::shared_ptr<EngineState> ASPCoordinator::getEngineStateByIRI(std::string p_iri)
{
  for (auto system : this->systems)
  {
    if (system->getSystemIri() == p_iri)
      return system;
  }

  _log->info("getEngineStateByIRI", "New system found %s", p_iri.c_str());
  std::shared_ptr<EngineState> system = std::make_shared<EngineState>(p_iri, this->engine);
  this->systems.push_back(system);

  return system;
}

std::map<std::string, std::string> ASPCoordinator::readConfiguration(std::string const config)
{
  std::map<std::string, std::string> configuration;
  std::stringstream ss(config);
  std::string item;
  while (std::getline(ss, item, ';'))
  {
    int index = item.find("=");

    if (index == std::string::npos)
    {
      _log->warning("readConfiguration", "Broken configuration '%s', skipped", item.c_str());
    }

    configuration[item.substr(0, index)] = item.substr(index + 1, item.size());
  }

  return configuration;
}

void ASPCoordinator::readMetadata(std::map<std::string, int>* metadata, const std::string provider,
                                  const std::string sourceSystem, Gringo::Value information, Gringo::Value step)
{
  this->readMetadata("delay", metadata, provider, sourceSystem, information, step);
  this->readMetadata("accuracy", metadata, provider, sourceSystem, information, step);
}

void ASPCoordinator::readMetadata(std::string name, std::map<std::string, int> *metadata, std::string const provider,
                                  std::string const sourceSystem, Gringo::Value information, Gringo::Value step)
{
  // metadataStream(1,metadata,testSystem,testComputationalNodeInd,testSystem,information(testEntity1,testScope1,testRepresentation2,none),2)
  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(name));
  values.push_back(std::string(this->self->getSystemIriShort()));
  values.push_back(Gringo::Value(provider));
  values.push_back(Gringo::Value(sourceSystem));
  values.push_back(information);
  values.push_back(step);
  values.push_back("?");

  auto delayQuery = std::make_shared<Gringo::Value>("metadataStream", values);
  auto delayResult = this->asp->queryAllTrue(delayQuery);

  if (delayResult->size() != 1)
  {
    std::stringstream o;
    o << information;
    _log->warning("readMetadata", "Wrong size '%d' for metadata '%s' of stream '%s', '%s', '%s'", delayResult->size(),
                  name.c_str(), o.str().c_str(), provider.c_str(), sourceSystem.c_str());
    return;
  }

  auto delayValue = delayResult->at(0);

  Gringo::Value value = delayValue.args()[7];

  if (value.type() != Gringo::Value::Type::NUM)
  {
    std::stringstream o, o2;
    o << information;
    o2 << value;
    _log->warning("readMetadata", "Wrong type '%d' of '%s' for metadata '%s' of stream '%s', '%s', '%s'",
                  value.type(), o2.str().c_str(), name.c_str(), o.str().c_str(), provider.c_str(),
                  sourceSystem.c_str());
    return;
  }

  std::stringstream o;
     o << information;
  _log->warning("readMetadata", "Metadata '%s' of stream '%s', '%s', '%s' has value '%d'",
                    name.c_str(), o.str().c_str(), provider.c_str(),
                    sourceSystem.c_str(), value.num());

  (*metadata)[name] = value.num();
}

std::string ASPCoordinator::dataTypeForRepresentation(std::string representation)
{
  // TODO
  return representation;
}

std::shared_ptr<BaseInformationStream> ASPCoordinator::getStream(Gringo::Value info, std::string lastProcessing,
                                                                 std::string sourceSystem, Gringo::Value step)
{
  auto entity = *info.args()[0].name();
  auto relatedEntity = *info.args()[3].name();

  if (relatedEntity == "none")
    relatedEntity = "";

  auto infoSpec = std::make_shared<InformationSpecification>(entity, this->entityTypeMap[entity],
                                                             *info.args()[1].name(), *info.args()[2].name(),
                                                             relatedEntity);

  auto stream = this->informationStore->getBaseStream(infoSpec.get(), lastProcessing, sourceSystem);

  if (false == stream)
  {
    std::map<std::string, int> metadata;
    this->readMetadata(&metadata, lastProcessing, sourceSystem, info, step);

    //        std::shared_ptr<InformationSpecification> specification,
    //        const std::string name,
    //        const int streamSize,
    //        std::map<std::string, int> metadatas,
    //        std::string provider,
    //        std::string sourceSystem

    std::string dataType = this->dataTypeForRepresentation(*info.args()[2].name());
    std::string name = *info.args()[0].name() + "-" + lastProcessing + "-" + sourceSystem;
    stream = this->informationStore->registerBaseStream(dataType, infoSpec, name, 100, metadata, lastProcessing,
                                                        sourceSystem);
  }

  return stream;
}

} /* namespace ice */
