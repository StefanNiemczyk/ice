/*
 * ASPCoordinator.cpp
 *
 *  Created on: Oct 24, 2014
 *      Author: sni
 */

#include "ice/model/aspModel/ASPModelGenerator.h"

#include "ice/ICEngine.h"
#include "ice/information/InformationSpecification.h"
#include "ice/information/InformationStore.h"
#include "ice/coordination/Coordinator.h"
#include "ice/coordination/EngineState.h"
#include "ice/model/aspModel/ASPSystem.h"
#include "ice/processing/NodeStore.h"
#include "ice/ontology/OntologyInterface.h"

namespace ice
{

ASPModelGenerator::ASPModelGenerator(std::weak_ptr<ICEngine> engine) : ProcessingModelGenerator(engine)
{
  this->_log = el::Loggers::getLogger("ASPModelGenerator");
  _log->verbose(1, "Constructor called");

  this->maxChainLength = 10;
  this->engine = engine;
  this->queryIndex = 0;
  this->groundingDirty = true;
  this->globalOptimization = true;
}

void ASPModelGenerator::init()
{
  std::string path = ros::package::getPath("ice");
  _log->debug("Default ontology path %v", path.c_str());

  auto en = engine.lock();
  this->nodeStore = en->getNodeStore();
  this->informationStore = en->getInformationStore();
  this->ontology = en->getOntologyInterface();
  this->coordinator = en->getCoordinator();
  this->self = this->getASPSystemByIRI(en->getIri());

  // Initializing ASP
  this->asp = std::make_shared<supplementary::ClingWrapper>();
  this->asp->addKnowledgeFile(path + "/asp/informationProcessing/processing.lp");
  this->asp->addKnowledgeFile(path + "/asp/informationProcessing/searchBottomUp.lp");

  if (this->globalOptimization)
    this->asp->addKnowledgeFile(path + "/asp/informationProcessing/globalOptimization.lp");
  else
    this->asp->addKnowledgeFile(path + "/asp/informationProcessing/localOptimization.lp");

  this->asp->setNoWarnings(true);
  this->asp->init();
}

void ASPModelGenerator::cleanUp()
{
  this->nodeStore.reset();
  this->informationStore.reset();
  this->ontology.reset();
}

ASPModelGenerator::~ASPModelGenerator()
{
  // nothing to do here
}

void ASPModelGenerator::createProcessingModel()
{
  _log->verbose(1, "Start optimizing");

  if (this->ontology->isLoadDirty())
  {
    _log->debug("Load flag dirty, reload ontologies");
    this->ontology->loadOntologies();
  }

  if (this->ontology->isInformationDirty())
  {
    _log->debug("Information model flag dirty, reload information structure");
    this->readInfoStructureFromOntology();
  }

  if (this->ontology->isSystemDirty())
  {
    _log->debug("System flag dirty, reload systems");
    this->readSystemsFromOntology();
  }

  if (groundingDirty)
  {
    this->groundingDirty = false;
    _log->debug("Grounding flag dirty, grounding asp program");

    if (this->lastQuery)
    {
      this->lastQuery->assign(false);
      this->lastQuery->release();
    }
    ++this->queryIndex;
    this->lastQuery = this->asp->getExternal("query", {this->queryIndex}, "query", {this->queryIndex,3,this->maxChainLength}, true);
  }

  // Solving
  auto solveResult = this->asp->solve();
  _log->info("Solving finished: %v", (Gringo::SolveResult::SAT == solveResult) ? "SAT" : "UNSAT");

  if (solveResult == Gringo::SolveResult::SAT)
  {
    _log->info("Resulting Model %v", this->asp->toStringLastModel());
    bool valid = true;
    std::vector<std::shared_ptr<Node>> nodes;

    // node(1,testSystem,testSourceNodeInd,testEntity1,none)
    std::vector<Gringo::Value> values;
    values.push_back(this->queryIndex);
    values.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
    values.push_back("?");
    values.push_back("?");
    values.push_back("?");

    Gringo::Value nodeQuery("node", values);
    auto queryResult = this->asp->queryAllTrue(&nodeQuery);

    for (auto nodeValue : *queryResult)
    {
      auto nodeName = *nodeValue.args()[2].name();
      auto nodeEntity = *nodeValue.args()[3].name();
      auto nodeEntity2 = *nodeValue.args()[4].name();

      _log->debug("Look up node '%v' to process entity '%v'", nodeName.c_str(),
                  nodeEntity.c_str());

      auto aspNode = this->self->getASPElementByName(nodeName);

      if (aspNode == nullptr)
      {
        _log->error("No node '%v' found, asp system description is invalid!",
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
        _log->error("Node '%v' (%v) could not be created, asp system description is invalid!", nodeName.c_str(),
                    aspNode->className.c_str());
        valid = false;
        break;
      }

      // connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,PROVIDER,SOURCE,INFO,STEP))
      std::vector<Gringo::Value> nodeValues;
      nodeValues.push_back(this->queryIndex);
      nodeValues.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
      nodeValues.push_back(Gringo::Value(aspNode->name));
      nodeValues.push_back(Gringo::Value(nodeEntity));
      nodeValues.push_back(Gringo::Value(nodeEntity2));

      std::vector<Gringo::Value> streamValues;
      streamValues.push_back(this->queryIndex);
      streamValues.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
      streamValues.push_back("?");
      streamValues.push_back("?");
      streamValues.push_back("?");
      streamValues.push_back("?");


      std::vector<Gringo::Value> values;
      values.push_back(Gringo::Value("node", nodeValues));
      values.push_back(Gringo::Value("stream", streamValues));

      Gringo::Value connectQuery("connectToNode", values);
      auto connectResult = this->asp->queryAllTrue(&connectQuery);

      if (false == connectResult)
      {
        _log->error("No asp model by look up of connected streams to node '%v'",
                    aspNode->name.c_str());
        valid = false;
        break;
      }
      else
      {
        for (auto connect : *connectResult)
        {
          _log->debug("Look up connected stream for node '%v'", nodeName.c_str());

          auto streamValue = connect.args()[1];

          auto lastProcessing = *streamValue.args()[2].name();
          auto sourceSystem = *streamValue.args()[3].name();
          auto info = streamValue.args()[4];
          auto step = streamValue.args()[5];
          auto stream = this->getStream(info, lastProcessing, sourceSystem, step);

          if (false == stream)
          {
            std::stringstream o;
            o << connect;
            _log->error("Stream '%v' could not be created, asp system description is invalid!", o.str().c_str());
            valid = false;
            break;
          }

          node->addInput(stream, true); // TODO stream is trigger?
        }
      }

      // stream(1,testSystem,testSourceNodeInd,testSystem,information(testEntity1,testScope1,testRepresentation1,none),step)
      values.clear();
      values.push_back(this->queryIndex);
      values.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
      values.push_back(Gringo::Value(aspNode->name));
      values.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
      values.push_back("?");
      values.push_back("?");

      Gringo::Value streamQuery("stream", values);
      auto streamResult = this->asp->queryAllTrue(&streamQuery);

      for (auto output : *streamResult)
      {
        _log->debug("Look up output stream for node '%v'", nodeName.c_str());

        auto lastProcessing = *output.args()[2].name();
        auto sourceSystem = *output.args()[3].name();
        auto info = output.args()[4];
        auto step = output.args()[5];
        auto stream = this->getStream(info, lastProcessing, sourceSystem, step);

        if (false == stream)
        {
          std::stringstream o;
          o << output;
          _log->error("Stream '%v' could not be created, asp system description is invalid!",
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

    // TODO interpret map
    // TODO selected stream

    if (valid)
    {
      _log->info("Optimizing successfully completed");
      this->nodeStore->cleanUpUnusedNodes(nodes);
      for (auto node : nodes)
      {
        node->activate();
      }
    }
    else
    {
      _log->error("Optimizing failed, no processing was established");
      for (auto node : nodes)
      {
        this->nodeStore->cleanUpNodes(nodes);
      }
    }

    this->informationStore->cleanUpStreams();
  }

  _log->verbose(1, "End optimizing");
}

void ASPModelGenerator::readInfoStructureFromOntology()
{
  _log->verbose(1, "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  if (false == this->ontology->isInformationDirty())
    return;

  const char* infoStructure = this->ontology->readInformationStructureAsASP();

  _log->debug("Extracted structure from ontology");
  _log->verbose(1, infoStructure);

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
      this->entities.push_back(item);
      this->asp->add(programPart, {}, item);
    }
  }

  this->asp->ground(programPart, {});

  this->groundingDirty = true;
}

void ASPModelGenerator::readSystemsFromOntology()
{
  _log->verbose(1, "Read systems from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

  if (false == this->ontology->isSystemDirty())
    return;

  auto ontSystems = this->ontology->getSystems();

  if (ontSystems == nullptr)
  {
    _log->error("Error occurred while reading systems");
    return;
  }

  std::shared_ptr<ASPSystem> system;

  for (auto ontSystem : *ontSystems)
  {
    _log->debug("Checking system " + std::string(ontSystem));
    system = this->getASPSystemByIRI(ontSystem);

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

      if (typeStr == nullptr || name == nullptr || elementStr == nullptr)
      {
        _log->error("Empty string for element '%v': '%v' (elementStr), '%v' (typeStr), element will be skipped",
                    name == nullptr ? "null" : name, elementStr == nullptr ? "null" : elementStr, typeStr == nullptr ? "null" : typeStr);

        delete name;
        delete elementStr;
        delete aspStr;
        delete cppStr;
        delete typeStr;

        continue;
      }

      if (std::strcmp(typeStr, "COMPUTATION_NODE") == 0)
      {
        type = ASPElementType::ASP_COMPUTATION_NODE;
      }
      else if (std::strcmp(typeStr, "SOURCE_NODE") == 0)
      {
        type = ASPElementType::ASP_SOURCE_NODE;
      }
      else if (std::strcmp(typeStr, "REQUIRED_STREAM") == 0)
      {
        type = ASPElementType::ASP_REQUIRED_STREAM;
      }
      else if (std::strcmp(typeStr, "MAP_NODE") == 0)
      {
        type = ASPElementType::ASP_MAP_NODE;
      }
      else if (std::strcmp(typeStr, "IRO_NODE") == 0)
      {
        type = ASPElementType::ASP_IRO_NODE;
      }
      else if (std::strcmp(typeStr, "REQUIRED_MAP") == 0)
      {
        type = ASPElementType::ASP_REQUIRED_MAP;
      }
      else
      {
        _log->error("Unknown asp element type '%v' for element '%v', element will be skipped",
                    typeStr, name);

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
        _log->info("ASP element '%v' not found, creating new element",
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

        auto value = supplementary::ClingWrapper::stringToValue(elementStr);
        element->external = this->asp->getExternal(*value.name(), value.args());

        switch (type)
        {
          case ASPElementType::ASP_COMPUTATION_NODE:
          case ASPElementType::ASP_SOURCE_NODE:
          case ASPElementType::ASP_MAP_NODE:
          case ASPElementType::ASP_IRO_NODE:
            if (false == this->nodeStore->existNodeCreator(element->className))
            {
              _log->warn("Missing creator for node '%v' of type '%v', cpp grounding '%v', asp external set to false",
                         element->name.c_str(), ASPElementTypeNames[type].c_str(),
                         element->className == "" ? "NULL" : element->className.c_str());
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

std::shared_ptr<OntologyInterface> ASPModelGenerator::getOntologyInterface()
{
  return this->ontology;
}

std::shared_ptr<supplementary::ClingWrapper> ASPModelGenerator::getClingWrapper()
{
  return this->asp;
}

std::shared_ptr<ASPSystem> ASPModelGenerator::getASPSystemByIRI(std::string p_iri)
{
  for (auto system : this->systems)
  {
    if (system->getEngineState()->getSystemIri() == p_iri)
      return system;
  }

  _log->info("New asp system found %v", p_iri.c_str());

  std::shared_ptr<ASPSystem> system = std::make_shared<ASPSystem>(this->engine, this->coordinator->getEngineState(p_iri));
  this->systems.push_back(system);

  return system;
}

std::map<std::string, std::string> ASPModelGenerator::readConfiguration(std::string const config)
{
  std::map<std::string, std::string> configuration;
  std::stringstream ss(config);
  std::string item;

  while (std::getline(ss, item, ';'))
  {
    int index = item.find("=");

    if (index == std::string::npos)
    {
      _log->warn("Broken configuration '%v', skipped", item.c_str());
    }

    configuration[item.substr(0, index)] = item.substr(index + 1, item.size());
  }

  return configuration;
}

void ASPModelGenerator::readMetadata(std::map<std::string, int>* metadata, const std::string provider,
                                  const std::string sourceSystem, Gringo::Value information, Gringo::Value step)
{
  // TODO read metadata from ontology
  this->readMetadata("delay", metadata, provider, sourceSystem, information, step);
  this->readMetadata("accuracy", metadata, provider, sourceSystem, information, step);
}

void ASPModelGenerator::readMetadata(std::string name, std::map<std::string, int> *metadata, std::string const provider,
                                  std::string const sourceSystem, Gringo::Value information, Gringo::Value step)
{
  // metadataStream(1,accuracy,stream(1,testSystem,testComputationalNodeInd,testSystem,information(testEntity1,testScope1,testRepresentation2,none),2),5)

  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(name));

  std::vector<Gringo::Value> valuesStream;
  valuesStream.push_back(this->queryIndex);
  valuesStream.push_back(std::string(this->self->getEngineState()->getSystemIriShort()));
  valuesStream.push_back(Gringo::Value(provider));
  valuesStream.push_back(Gringo::Value(sourceSystem));
  valuesStream.push_back(information);
  valuesStream.push_back(step);

  values.push_back(Gringo::Value("stream",valuesStream));
  values.push_back("?");

  Gringo::Value query("metadataStream", values);
  auto result = this->asp->queryAllTrue(&query);

  if (result->size() != 1)
  {
    std::stringstream o;
    o << information;
    _log->warn("Wrong size '%v' for metadata '%v' of stream '%v', '%v', '%v'", result->size(), name.c_str(),
               o.str().c_str(), provider.c_str(), sourceSystem.c_str());
    return;
  }

  Gringo::Value value = result->at(0).args()[3];

  if (value.type() != Gringo::Value::Type::NUM)
  {
    std::stringstream o, o2;
    o << information;
    o2 << value;
    _log->warn("Wrong type '%v' of '%v' for metadata '%v' of stream '%v', '%v', '%v'", value.type(), o2.str().c_str(),
               name.c_str(), o.str().c_str(), provider.c_str(), sourceSystem.c_str());
    return;
  }

  std::stringstream o;
     o << information;
  _log->debug("Metadata '%v' of stream '%v', '%v', '%v' has value '%v'", name.c_str(), o.str().c_str(), provider.c_str(),
             sourceSystem.c_str(), value.num());

  (*metadata)[name] = value.num();
}

std::string ASPModelGenerator::dataTypeForRepresentation(std::string representation)
{
  // TODO
  return representation;
}

std::shared_ptr<BaseInformationStream> ASPModelGenerator::getStream(Gringo::Value info, std::string lastProcessing,
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
    stream = this->informationStore->registerBaseStream(dataType, infoSpec, name, 10, metadata, lastProcessing,
                                                        sourceSystem);
  }

  return stream;
}

} /* namespace ice */
