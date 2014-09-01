/*
 * icengine.cpp
 *
 *  Created on: May 13, 2014
 *      Author: sni
 */

#include "ice/ICEngine.h"

namespace ice
{

ICEngine::ICEngine(std::shared_ptr<TimeFactory> timeFactory, std::shared_ptr<StreamFactory> streamFactory,
                   std::string id, std::shared_ptr<Configuration> config)
{
  this->initialized = false;
  this->config = config;
  this->timeFactory = timeFactory;
  this->streamFactory = streamFactory;

  if (id == "")
    this->id = IDGenerator::getInstance()->getIdentifier();
  else
    this->id = IDGenerator::getInstance()->getIdentifier(id);
}

ICEngine::~ICEngine()
{
  this->coordinator->cleanUp();
  this->communication->cleanUp();
//  this->eventHandler;
//  this->informationStore;
//  this->nodeStore ;
//  this->modelComperator;
}

void ICEngine::init()
{
  if (this->initialized)
    return;

  std::lock_guard<std::mutex> guard();

  if (this->initialized)
    return;

  this->communication = std::make_shared<RosCommunication>(this->shared_from_this());
  this->eventHandler = std::make_shared<EventHandler>(this->shared_from_this());
  this->informationStore = std::make_shared<InformationStore>(this->shared_from_this());
  this->nodeStore = std::make_shared<NodeStore>(this->shared_from_this());
  this->modelComperator = std::make_shared<ModelComperator>();
  this->coordinator = std::make_shared<Coordinator>(this->shared_from_this());

  // Initialize components
  this->coordinator->init();
  this->communication->init();

  this->initialized = true;
}

std::shared_ptr<TimeFactory> ICEngine::getTimeFactory()
{
  return this->timeFactory;
}

std::shared_ptr<Configuration> ICEngine::getConfig()
{
  return this->config;
}

std::shared_ptr<EventHandler> ICEngine::getEventHandler()
{
  return this->eventHandler;
}

std::shared_ptr<InformationStore> ICEngine::getInformationStore()
{
  return this->informationStore;
}

std::shared_ptr<NodeStore> ICEngine::getNodeStore()
{
  return this->nodeStore;
}

std::shared_ptr<Communication> ICEngine::getCommunication()
{
  return this->communication;
}

std::shared_ptr<Coordinator> ICEngine::getCoordinator()
{
  return this->coordinator;
}

bool ICEngine::readFromFile(const std::string fileName)
{
  return this->readFromFiles( {fileName});
}

bool ICEngine::readFromFiles(std::initializer_list<std::string> fileNameList)
{
  this->init();

  XMLReader reader;
  bool result = true;
  boost::uuids::uuid uuid;
  std::shared_ptr<InformationType> type;
  std::shared_ptr<BaseInformationStream> baseStream, resultBaseStream;

  // Reading files into xml reader data structure
  bool readerResult = reader.readFiles(fileNameList);

  if (false == readerResult)
  {
    std::cerr << "ICEngine: Error while reading XML file, reading aborted";

    return false;
  }

  // Reading xml reader data structure
  // Reading information types
  for (auto information : *reader.getInformations())
  {
    this->readXMLInformation(information, Configuration::INFORMATION_TYPE_NAME_SEPERATOR);
  }

  // Reading named streams
  for (auto stream : *reader.getStreams())
  {
    uuid = boost::lexical_cast<boost::uuids::uuid>(stream->informationUuid);
    type = this->informationStore->getInformationType(uuid);

    if (false == type)
    {
      std::cerr << "ICEngine: No Type found with uuid '" << stream->informationUuid << "' for stream '" << stream->name
          << "'" << std::endl;
      result = false;
      continue;
    }

    baseStream = type->getBaseStream(stream->name);

    if (baseStream)
    {
      std::cerr << "ICEngine: Stream '" << stream->name << "'for Type with uuid '" << stream->informationUuid
          << "' already exists" << std::endl;
      result = false;
      continue;
    }

    //const std::string& className,
    //const std::string name,
    //std::weak_ptr<InformationType> informationType,
    //std::shared_ptr<EventHandler> eventHandler,
    //std::shared_ptr<InformationSpecification> specification,
    //int streamSize,
    //std::string provider,
    //std::string description
    //bool shared
    //int sharingMaxCount
    baseStream = this->streamFactory->createStream(type->getSpecification()->getTypeString(), stream->name, type,
                                                   this->eventHandler, type->getSpecification(), stream->size,
                                                   stream->provider, stream->_desc,
                                                   (stream->sharingState == "active" ? true : false), stream->sharingMaxCount);

    if (false == baseStream)
    {
      result = false;
      std::cerr << "ICEngine: Stream '" << stream->name << "' for Type with uuid '" << stream->informationUuid
          << "' could not be created, unknown class name '" << type->getSpecification()->getTypeString() << "'"
          << std::endl;
      continue;
    }

    bool notAdded;
    resultBaseStream = type->registerStream(baseStream, &notAdded);

    // stream was not added to the information type container
    if (notAdded)
    {
      // a stream with this name already exists
      if (resultBaseStream)
      {
        std::cerr << "ICEngine: Stream '" << stream->name << "'for Type with uuid '" << stream->informationUuid
            << "' already exists" << std::endl;
      }
      // Information specifications does not match
      else
      {
        std::cerr << "ICEngine: Stream '" << stream->name << "' with uuid '" << stream->informationUuid
            << "' does not match information type with uuid '" << type->getSpecification()->getUUID() << "'"
            << std::endl;
        result = false;
      }
    }
  }

  // Reading stream templates
  for (auto streamTemplate : *reader.getStreamTemplates())
  {
    uuid = boost::lexical_cast<boost::uuids::uuid>(streamTemplate->informationUuid);
    type = this->informationStore->getInformationType(uuid);

    if (false == type)
    {
      std::cerr << "ICEngine: No Type found with uuid '" << streamTemplate->informationUuid << "' for streamTemplate '"
          << streamTemplate->name << "'" << std::endl;
      result = false;
      continue;
    }
    // std::shared_ptr<StreamFactory> streamFactory,
    // std::string className,
    // const std::string name,
    // std::weak_ptr<InformationType> informationType,
    // std::shared_ptr<EventHandler> eventHandler,
    // std::shared_ptr<InformationSpecification> specification,
    // int streamSize
    // std::string provider = "",
    // std::string description = "",
    auto st = std::make_shared<InformationStreamTemplate>(this->streamFactory,
                                                          type->getSpecification()->getTypeString(),
                                                          streamTemplate->name, type, this->eventHandler,
                                                          type->getSpecification(), streamTemplate->maxStreamCount,
                                                          streamTemplate->size, streamTemplate->provider,
                                                          streamTemplate->_desc);

    type->registerStreamTemplate(st);
  }

  // Reading Nodes
  for (auto xmlNode : *reader.getNodes())
  {
    std::shared_ptr<Node> node = this->nodeStore->getNode(xmlNode->name);

    if (node)
    {
      std::cerr << "ICEngine: Node '" << xmlNode->name << "' already exists" << std::endl;
      result = false;
      continue;
    }

    node = Node::createNode(xmlNode->className);

    if (false == node)
    {
      std::cerr << "ICEngine: No Node with class name '" << xmlNode->className << "' registered" << std::endl;
      result = false;
      continue;
    }

    node->setName(xmlNode->name);
    node->setEventHandler(this->eventHandler);
    node->setType(xmlNode->type);
    node->setCyclicTriggerTime(xmlNode->trigger);
    node->setStringDescription(xmlNode->_desc);
    node->setSource(xmlNode->source);

    for (auto xmlInput : xmlNode->inputs)
    {
      auto stream = this->informationStore->getBaseStream(xmlInput->name);

      if (false == stream)
      {
        std::cerr << "ICEngine: No input stream with name '" << xmlInput->name << "' for node '" << xmlNode->name << "'"
            << std::endl;
        result = false;
        continue;
      }

      node->addInput(stream, xmlInput->trigger, true);
    }

    for (auto xmlInputTemplate : xmlNode->inputTemplates)
    {
      auto streamTemplate = this->informationStore->getStreamTemplate(xmlInputTemplate->name);

      if (false == streamTemplate)
      {
        std::cerr << "ICEngine: No StreamTemplate with name '" << xmlInputTemplate->name << "' for node '"
            << xmlNode->name << "'" << std::endl;
        result = false;
        continue;
      }

      node->addInputTemplate(streamTemplate, xmlInputTemplate->trigger);
    }

    for (auto xmlOutput : xmlNode->outputs)
    {
      auto stream = this->informationStore->getBaseStream(xmlOutput->name);

      if (false == stream)
      {
        std::cerr << "ICEngine: No output stream with name '" << xmlOutput->name << "' for node '" << xmlNode->name
            << "'" << std::endl;
        result = false;
        continue;
      }

      node->addOutput(stream);
    }

    node->setConfiguration(xmlNode->configs);

    node->init();
    node->activate();

    this->nodeStore->addNode(node);
  }

  return result;
}

identifier ICEngine::getId() const
{
  return this->id;
}

int ICEngine::readXMLInformation(XMLInformation* information, const std::string namePrefix)
{
  identifier uuid = IDGenerator::getInstance()->getIdentifier(information->uuid);
  std::shared_ptr<InformationSpecification> spec;
  std::shared_ptr<InformationType> type = this->informationStore->getInformationType(uuid);

  if (type)
  {
    std::cerr << "ICEngine: Type already registered '" << information->topic << "', '" << uuid << "'" << std::endl;
    return 1;
  }

  std::string newNamePrefix = namePrefix + information->topic + Configuration::INFORMATION_TYPE_NAME_SEPERATOR;

  spec = std::make_shared<InformationSpecification>(uuid, namePrefix + information->topic);
  type = this->informationStore->registerInformationType(spec);

  if (information->type.size() != 0)
    spec->setTypeString(information->type);

  if (information->_desc.size() != 0)
    spec->setDescription(information->_desc);

  for (auto info : information->nested)
  {
    this->readXMLInformation(info, newNamePrefix);
  }

  return 0;
}

std::shared_ptr<InformationModel> ICEngine::getInformationModel()
{
  std::lock_guard<std::mutex> guard(this->mtx_);

  auto model = std::make_shared<InformationModel>();

  // adding stream descriptions and stream template descriptions
  this->informationStore->addDescriptionsToInformationModel(model);

  // adding node descriptions
  this->nodeStore->addDescriptionsToInformationModel(model);

  return model;
}

} /* namespace ice */
