/*
 * StreamStore.cpp
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#include "ice/information/StreamStore.h"

namespace ice
{

StreamStore::StreamStore(std::weak_ptr<ICEngine> engine)
{
  this->_log = el::Loggers::getLogger("StreamStore");
  this->engine = engine;
}

StreamStore::StreamStore(std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<CollectionFactory> streamFactory)
{
  this->_log = el::Loggers::getLogger("StreamStore");
  this->eventHandler = eventHandler;
  this->factory = streamFactory;
}

StreamStore::~StreamStore()
{
  //
}

std::shared_ptr<BaseInformationStream> StreamStore::registerBaseStream(
    std::string dataType, std::shared_ptr<InformationSpecification> specification, const std::string name,
    const int streamSize, std::map<std::string, int> &metadata, std::string provider, std::string sourceSystem)
{
  auto ptr = this->getBaseCollection(specification.get(), provider, sourceSystem);

  //stream already registered
  if (ptr != nullptr)
  {
    _log->warn("Duplicated Stream with '%v', '%v', '%v'",
                  specification->toString(), provider, sourceSystem);
    return ptr;
  }

  auto desc = std::make_shared<CollectionDescription>(specification, name, provider, sourceSystem, metadata);
  std::string type = dataType;
  auto stream = this->factory->createStream(type, desc, this->eventHandler, streamSize);

  if (stream)
  {
    _log->debug("Created stream with '%v', '%v', '%v', '%v'", specification->toString(),
                provider, sourceSystem, type);
    this->collections.push_back(stream);
  }
  else
  {
    _log->error("Stream with '%v', '%v', '%v', '%v' could not be created", specification->toString(),
                provider, sourceSystem, type);
  }
  return stream;
}

} /* namespace ice */

