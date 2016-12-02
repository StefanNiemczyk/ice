/*
 * StreamStore.cpp
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#include "ice/information/StreamStore.h"

#include "ice/ICEngine.h"
#include "ice/information/CollectionFactory.h"
#include "ice/ontology/OntologyInterface.h"

#include "easylogging++.h"

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

void StreamStore::init()
{
  if (false == this->engine.expired())
  {
    auto engineObject = engine.lock();

    this->eventHandler = engineObject->getEventHandler();
    this->factory = engineObject->getCollectionFactory();
  }
}

void StreamStore::cleanUp()
{
  this->eventHandler.reset();
  this->factory.reset();

  for(auto stream : this->streams)
  {
    stream->destroy();
  }

  this->streams.clear();
}

StreamStore::~StreamStore()
{
  //
}

std::shared_ptr<BaseInformationStream> StreamStore::registerBaseStream(
    std::string dataType, std::shared_ptr<InformationSpecification> specification, const std::string name,
    const int streamSize, std::map<std::string, int> &metadata, std::string provider, std::string sourceSystem)
{
  auto ptr = this->getBaseStream(specification.get(), provider, sourceSystem);

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
    this->streams.push_back(stream);
  }
  else
  {
    _log->error("Stream with '%v', '%v', '%v', '%v' could not be created", specification->toString(),
                provider, sourceSystem, type);
  }
  return stream;
}

std::shared_ptr<BaseInformationStream> StreamStore::getBaseStream(InformationSpecification *specification,
                                                                       std::string provider, std::string sourceSystem)
{
  _log->debug("Get stream by '%v', '%v', '%v'", specification->toString(), provider,
              sourceSystem);

  std::vector<std::shared_ptr<BaseInformationStream>> selected;

  for (auto stream : this->streams)
  {
    auto spec = stream->getStreamDescription();
    if (*spec->getInformationSpecification() == *specification)
    {
      if (provider != "")
      {
        if (provider == spec->getProvider() && sourceSystem == spec->getSourceSystem())
        {
          selected.push_back(stream);
        }
      }
      else
      {
        selected.push_back(stream);
      }
    }
  }

  if (selected.size() == 0)
  {
    return nullptr;
  }

  return selectBestStream(&selected);
}

std::shared_ptr<BaseInformationStream> StreamStore::selectBestStream(
    std::vector<std::shared_ptr<BaseInformationStream>> *streams)
{
  if (streams->size() == 0)
  {
    return nullptr;
  }

  auto best = streams->at(0);

  for (int i = 0; i < streams->size(); ++i)
  {
    // TODO
  }

  return best;
}

void StreamStore::cleanUpUnused()
{
  std::lock_guard<std::mutex> guard(this->_mtx);
  _log->verbose(1, "Start removing unused streams");
  int counter = 0;

  for (int i=0; i < this->streams.size(); ++i)
  {
    auto stream = this->streams.at(i);


    _log->info("Checking stream '%v', reference count %v", stream->toString(), stream.use_count());

    if (stream.use_count() == 2)
    {
      _log->info("Remove unused stream '%v'", stream->toString());

      ++counter;
      this->streams.erase(this->streams.begin() + i);
      --i;
    }
  }

  _log->info("Clean up information store: '%v' streams are removed", counter);
}

} /* namespace ice */

