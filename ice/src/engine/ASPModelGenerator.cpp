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
#include "ice/communication/Communication.h"
#include "ice/coordination/Coordinator.h"
#include "ice/coordination/EngineState.h"
#include "ice/model/aspModel/ASPSystem.h"
#include "ice/processing/NodeStore.h"
#include "ice/ontology/OntologyInterface.h"

using namespace std;

namespace ice
{
std::mutex ASPModelGenerator::mtxModelGen_;

ASPModelGenerator::ASPModelGenerator(std::weak_ptr<ICEngine> engine) :
    ProcessingModelGenerator(engine)
{
  this->_log = el::Loggers::getLogger("ASPModelGenerator");
  _log->verbose(1, "Constructor called");

  this->maxChainLength = 10;
//  this->engine = engine;
  this->queryIndex = 0;
  this->groundingDirty = true;
  this->globalOptimization = true;
  this->subModelIndex = 0;
}

void ASPModelGenerator::initInternal()
{
  std::lock_guard<std::mutex> guard(mtxModelGen_);

  std::string path = ros::package::getPath("ice") + "/asp/informationProcessing/";
  _log->debug("Default ASP path %v", path);

  auto en = this->engine.lock();
//  this->nodeStore = en->getNodeStore();
//  this->informationStore = en->getInformationStore();
//  this->coordinator = en->getCoordinator();
//  this->communication = en->getCommunication();

  // Initializing ASP
  this->asp = std::make_shared<supplementary::ClingWrapper>();
  this->asp->addKnowledgeFile(path + "processing.lp");
  this->asp->addKnowledgeFile(path + "searchBottomUp.lp");

  if (this->globalOptimization)
    this->asp->addKnowledgeFile(path + "globalOptimization.lp");
  else
    this->asp->addKnowledgeFile(path + "localOptimization.lp"); //DO NOT USE

  this->asp->setNoWarnings(true);
  this->asp->init();
}

void ASPModelGenerator::cleanUpInternal()
{
//  this->nodeStore.reset();
//  this->informationStore.reset();
}

ASPModelGenerator::~ASPModelGenerator()
{
  // nothing to do here
}

void ASPModelGenerator::readOntology()
{
  if (this->ontology->isLoadDirty())
  {
    _log->debug("Load flagged dirty, reload ontologies");
    this->ontology->loadOntologies();
  }

//  if (this->ontology->isInformationDirty())
//  {
//    _log->debug("Information model flagged dirty, reload information structure");
//  }
  this->readInfoStructureFromOntology();

  if (this->ontology->isSystemDirty())
  {
    _log->debug("System flagged dirty, reload systems");
    this->readSystemsFromOntology();
  }
}

std::shared_ptr<ProcessingModel> ASPModelGenerator::createProcessingModel()
{
  _log->verbose(1, "Start model creation");

  std::lock_guard<std::mutex> guard(mtxModelGen_);

  this->ontology->attachCurrentThread();

  this->readOntology();

  if (groundingDirty)
  {
    this->groundingDirty = false;
    _log->debug("Grounding flagged dirty, grounding asp program");

    if (this->lastQuery)
    {
      this->lastQuery->assign(false);
      this->lastQuery->release();
    }

    this->lastQuery = this->asp->getExternal("query", {this->queryIndex}, "query",
                                             {this->queryIndex, 3, this->maxChainLength}, true);
  }

  if (false == this->self)
  {
    auto en = this->engine.lock();
    this->self = this->getASPSystemByIRI(en->getIri());

    for (auto system : this->systems)
    {
      if (this->self && system != this->self)
      {
        // TODO add metadata
        this->asp->add(
            "base",
            {},
            "transfer(" + system->getShortIri() + "," + this->self->getShortIri() + ") :- system("
                + system->getShortIri() + ",default).");
      }
    }
  }

  // activate and deactivate systems
  for (auto system : this->systems)
  {
    if (this->self != system)
      system->updateExternals(false);
  }

  // Solving
  auto solveResult = this->asp->solve();
  _log->info("Solving finished: %v", (Gringo::SolveResult::SAT == solveResult) ? "SAT" : "UNSAT");

  if (solveResult != Gringo::SolveResult::SAT)
  {
    _log->error("Optimizing failed, no processing was created");
    return nullptr;
  }

  _log->debug("Resulting ASP Model %v", this->asp->toStringLastModel());
  std::shared_ptr<ProcessingModel> model = std::make_shared<ProcessingModel>();

  // Extract nodes that needs to be activated within own system
  if (false == this->extractNodes(model->getNodes().get(), this->self))
  {
    _log->error("Optimizing failed, error by extracting own processing model");
    return nullptr;
  }

  // Request information from other systems
  for (auto system : this->systems)
  {
    if (this->self == system || system->getEngineState() == nullptr
        || false == system->getEngineState()->isCooperationPossible())
      continue;

    _log->debug("Check if sub model exists for engine '%v'", system->getIri());

    std::shared_ptr<SubModel> subModel = std::make_shared<SubModel>();
    subModel->engine = system->getEngineState();

    if (false == this->extractedSubModel(system, subModel))
    {
      _log->error("Optimizing failed, error by extracting sub model for system '%v'", system->getIri());
      return nullptr;
    }

    if (subModel->model != nullptr)
    {
      model->getSubModels()->push_back(subModel);
    }

    std::shared_ptr<StreamTransfer> send = std::make_shared<StreamTransfer>();
    send->engine = system->getEngineState();

    if (false == this->extractStreamTransfers(this->self, system, &send->transfer))
    {
      _log->error("Optimizing failed, error by extracting streams transfers from '%v' to '%v'", this->self->getIri(),
                  system->getIri());
      return nullptr;
    }

    if (send->transfer.size() > 0)
      model->getSend()->push_back(send);

    std::shared_ptr<StreamTransfer> receive = std::make_shared<StreamTransfer>();
    receive->engine = system->getEngineState();

    if (false == this->extractStreamTransfers(system, this->self, &receive->transfer))
    {
      _log->error("Optimizing failed, error by extracting streams transfers from '%v' to '%v'", system->getIri(),
                  this->self->getIri());
      return nullptr;
    }

    if (receive->transfer.size() > 0)
      model->getReceive()->push_back(receive);
  }

  _log->info("Model successfully created");

  return model;
}

bool ASPModelGenerator::extractedSubModel(std::shared_ptr<ASPSystem> system, std::shared_ptr<SubModel> subModel)
{
  _log->debug("Look up ASP elements for system '%v'", system->getIri());

  bool valid = true;
  vector<NodeDesc> nodes;

  // extract nodes
  valid = this->extractNodes(&nodes, system);

  // identify streams send from self -> system
  std::vector<TransferDesc> send;
  valid = this->extractStreamTransfers(system, this->self, &send);

  // identify streams send from system -> self
  std::vector<TransferDesc> receive;
  valid = this->extractStreamTransfers(this->self, system, &receive);

  if (false == valid)
  {
    _log->error("Sub model extraction for system '%v' failed", system->getShortIri());
    subModel->model = nullptr;

    return false;
  }

  if (nodes.size() == 0 && send.size() == 0 && receive.size() == 0)
  {
    _log->info("No Sub model extraction for system '%v'", system->getShortIri());
    subModel->model = nullptr;
  }
  else
  {
    _log->info("Sub model extraction for system '%v' successfully completed", system->getShortIri());
    // create sub model description
    subModel->model = std::shared_ptr<SubModelDesc>(
        new SubModelDesc(this->subModelIndex, nodes, send, receive));
  }

  return true;
}

bool ASPModelGenerator::extractNodes(vector<NodeDesc> *nodes, std::shared_ptr<ASPSystem> system)
{
  bool valid = true;

  // node(QUERY_INDEX, SYSTEM, NODE, ENTITY, ENTITY2)
  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(std::string(system->getShortIri()));
  values.push_back("?");
  values.push_back("?");
  values.push_back("?");

  Gringo::Value nodeQuery("node", values);
  auto queryResult = this->asp->queryAllTrue(&nodeQuery);

  for (auto nodeValue : *queryResult)
  {
    vector<InputStreamDesc> inputs;
    vector<OutputStreamDesc> outputs;

    auto nodeName = *nodeValue.args()[2].name();
    auto nodeEntity = *nodeValue.args()[3].name();
    auto nodeEntity2 = *nodeValue.args()[4].name();

    _log->debug("Look up node '%v' to process entity '%v'", nodeName, nodeEntity);

    auto aspNode = system->getASPElementByName(nodeName);

    if (aspNode == nullptr)
    {
      _log->error("No node '%v' found, asp system description is invalid!", nodeName);
      valid = false;
      break;
    }

    // connectToNode(node(k,SYSTEM,NODE,ENTITY,ENTITY2), stream(k,SYSTEM,node(k,SOURCE,PROVIDER,ENTITY3,ENTITY4),INFO,STEP))
    std::vector<Gringo::Value> nodeValues;
    nodeValues.push_back(this->queryIndex);
    nodeValues.push_back(std::string(system->getShortIri()));
    nodeValues.push_back(Gringo::Value(aspNode->name));
    nodeValues.push_back(Gringo::Value(nodeEntity));
    nodeValues.push_back(Gringo::Value(nodeEntity2));

    std::vector<Gringo::Value> streamValues;
    streamValues.push_back(this->queryIndex);
    streamValues.push_back(std::string(system->getShortIri()));
    streamValues.push_back("?");
    streamValues.push_back("?");
    streamValues.push_back("?");

    std::vector<Gringo::Value> values;
    values.push_back(Gringo::Value("node", nodeValues));
    values.push_back(Gringo::Value("stream", streamValues));

    Gringo::Value connectQuery("connectToNode", values);
    auto connectResult = this->asp->queryAllTrue(&connectQuery);

    for (auto connect : *connectResult)
    {
      _log->debug("Look up input stream for node '%v'", nodeName);

      // get the stream connected to the node
      auto streamValue = connect.args()[1];

      auto node = streamValue.args()[2];
      auto info = streamValue.args()[3];
      auto step = streamValue.args()[4];

      auto entity = *info.args()[0].name();
      auto scope = *info.args()[1].name();
      auto rep = *info.args()[2].name();
      auto relatedEntity = *info.args()[3].name();

      std::string source = *node.args()[1].name();
      std::string nodeName = *node.args()[2].name();
      std::string nodeEntity = *node.args()[3].name();
      std::string nodeEntity2 = *node.args()[4].name();

      if (relatedEntity == "none")
        relatedEntity = "";

      if (nodeEntity2 == "none")
        nodeEntity2 = "";

      InputStreamDesc input(source, nodeName, nodeEntity, nodeEntity2, entity, scope, rep, relatedEntity);
      inputs.push_back(input);
    }

    // stream(k,SYSTEM,node(k,SOURCE,NODE,ENTITY,ENTITY2),INFO,STEP)
    values.clear();
    values.push_back(this->queryIndex);
    values.push_back(std::string(system->getShortIri()));
    values.push_back(Gringo::Value("node", nodeValues));
    values.push_back("?");
    values.push_back("?");

    Gringo::Value streamQuery("stream", values);
    auto streamResult = this->asp->queryAllTrue(&streamQuery);

    for (auto output : *streamResult)
    {
      _log->debug("Look up output stream for node '%v'", nodeName);
      auto info = output.args()[3];
      auto step = output.args()[4];

      auto entity = *info.args()[0].name();
      auto scope = *info.args()[1].name();
      auto rep = *info.args()[2].name();
      auto relatedEntity = *info.args()[3].name();

      if (relatedEntity == "none")
        relatedEntity = "";

      OutputStreamDesc outputStream(entity, scope, rep, relatedEntity);
      outputs.push_back(outputStream);
    }

    nodes->push_back(
        std::make_tuple(aspNode->getNodeType(), aspNode->className, aspNode->name, nodeEntity, nodeEntity2,
                        aspNode->configAsString, inputs, outputs));

    if (false == valid)
      break;
  }

  // TODO interpret map
  // TODO selected stream

  return valid;
}

bool ASPModelGenerator::extractStreamTransfers(std::shared_ptr<ASPSystem> from, std::shared_ptr<ASPSystem> to,
                                           std::vector<TransferDesc> *transfers)
{
  bool valid = true;

  _log->debug("Look up streams transfered from '%v' to '%v'", from->getIri(), to->getIri());

  // stream(k,SYSTEM_SOURCE,node(k,SYSTEM_SOURCE,NODE2,ENTITY3,ENTITY4),INFO,STEP)
  std::vector<Gringo::Value> values;
  std::vector<Gringo::Value> nodeValues;
  nodeValues.push_back(this->queryIndex);
  nodeValues.push_back(std::string(from->getShortIri()));
  nodeValues.push_back("?");
  nodeValues.push_back("?");
  nodeValues.push_back("?");

  values.push_back(this->queryIndex);
  values.push_back(std::string(to->getShortIri()));
  values.push_back(Gringo::Value("node", nodeValues));
  values.push_back("?");
  values.push_back("?");

  Gringo::Value sendQuery("stream", values);
  auto results = this->asp->queryAllTrue(&sendQuery);

  // get streams connected to the node
  for (auto streamValue : *results)
  {
    auto node = streamValue.args()[2];
    auto info = streamValue.args()[3];
    auto step = streamValue.args()[4];

    auto entity = *info.args()[0].name();
    auto scope = *info.args()[1].name();
    auto rep = *info.args()[2].name();
    auto relatedEntity = *info.args()[3].name();

    std::string source = *node.args()[1].name();
    std::string nodeName = *node.args()[2].name();
    std::string nodeEntity = *node.args()[3].name();
    std::string nodeEntity2 = *node.args()[4].name();

    if (relatedEntity == "none")
      relatedEntity = "";

    if (nodeEntity2 == "none")
      nodeEntity2 = "";

    TransferDesc transferTo(source, nodeName, nodeEntity, nodeEntity2, entity, scope, rep, relatedEntity);
    transfers->push_back(transferTo);
  }

  return valid;
}

void ASPModelGenerator::readInfoStructureFromOntology()
{
  _log->verbose(1, "Read information structure from ontology");

  if (this->ontology->isLoadDirty())
    this->ontology->loadOntologies();

//  if (false == this->ontology->isInformationDirty())
//    return;

  const char* infoStructure = this->ontology->readInformationStructureAsASP();

  _log->debug("Extracted structure from ontology");
  _log->verbose(1, infoStructure);

  std::string programPart = "ontology" + ++this->queryIndex;
  std::stringstream ss;
  std::string item;

  ss << infoStructure;
//  delete infoStructure;

  while (std::getline(ss, item, '\n'))
  {
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
                    name == nullptr ? "null" : name, elementStr == nullptr ? "null" : elementStr,
                    typeStr == nullptr ? "null" : typeStr);

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
        _log->error("Unknown asp element type '%v' for element '%v', element will be skipped", typeStr, name);

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
        _log->info("ASP element '%v' not found, creating new element", std::string(name));
        auto element = std::make_shared<ASPElement>();
        element->aspString = aspStr;
        element->name = name;
        element->state = ASPElementState::ADDED_TO_ASP;
        element->type = type;

        if (std::strlen(cppStr) != 0)
        {
          const char* index = std::strchr(cppStr, '\n');
          element->className = std::string(cppStr, index);
          element->configAsString = std::string(index + 1, std::strlen(cppStr));
          element->config = this->readConfiguration(element->configAsString);
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
                         element->name, ASPElementTypeNames[type],
                         element->className == "" ? "NULL" : element->className);
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

//    delete ontSystem;
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
    if (system->getIri() == p_iri)
      return system;
  }

  std::string asp = this->ontology->toShortIri(p_iri);

  _log->info("New asp system found %v, short iri '%v'", p_iri, asp);


  // TODO add island
  auto external = this->asp->getExternal("system", {Gringo::Value(asp), "default"}, "system", {Gringo::Value(asp)},
                                         true);

  std::shared_ptr<ASPSystem> system = std::make_shared<ASPSystem>(p_iri, asp, this->engine,
                                                                  this->coordinator->getEngineStateNoMutex(p_iri), external);
  this->systems.push_back(system);

  // adding transfer to other systems
  if (this->self && p_iri != this->self->getIri())
  {
    // TODO add metadata
    this->asp->add("base", {},
                   "transfer(" + asp + "," + this->self->getShortIri() + ") :- system(" + asp + ",default).");
  }

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
      _log->warn("Broken configuration '%v', skipped", item);
    }

    configuration[item.substr(0, index)] = item.substr(index + 1, item.size());
  }

  return configuration;
}

void ASPModelGenerator::readMetadata(std::map<std::string, int>* metadata, const Gringo::Value element)
{
  // TODO read metadata from ontology
  this->readMetadata("delay", metadata, element);
  this->readMetadata("accuracy", metadata, element);
}

void ASPModelGenerator::readMetadata(std::string name, std::map<std::string, int> *metadata,
                                     const Gringo::Value element)
{
  // metadataStream(k,METADATA,stream(k,SYSTEM,node(k,SOURCE,NODE,ENTITY,ENTITY2),INFO,STEP),VALUE)
  std::vector<Gringo::Value> values;
  values.push_back(this->queryIndex);
  values.push_back(Gringo::Value(name));
  values.push_back(element);
  values.push_back("?");

  Gringo::Value query("metadataStream", values);
  auto result = this->asp->queryAllTrue(&query);

  if (result->size() != 1)
  {
    std::stringstream o;
    o << element;
    _log->warn("Wrong size '%v' for metadata '%v' of stream '%v'", result->size(), name, o.str());
    return;
  }

  Gringo::Value value = result->at(0).args()[3];

  if (value.type() != Gringo::Value::Type::NUM)
  {
    std::stringstream o, o2;
    o << element;
    o2 << value;
    _log->warn("Wrong type '%v' of '%v' for metadata '%v' of stream '%v'", value.type(), o2.str(), name,
               o.str());
    return;
  }

  std::stringstream o;
  o << element;
  _log->debug("Metadata '%v' of stream '%v' has value '%v'", name, o.str(), value.num());

  (*metadata)[name] = value.num();
}

std::string ASPModelGenerator::dataTypeForRepresentation(std::string representation)
{
  // TODO
  return representation;
}

} /* namespace ice */
