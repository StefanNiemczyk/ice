/*
 * StreamStore.cpp
 *
 *  Created on: May 19, 2014
 *      Author: sni
 */

#include "ice/information/StreamStore.h"

#include "ice/representation/Transformation.h"

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

int StreamStore::getInformation(std::shared_ptr<InformationSpecification> request,
                                std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo,
                                bool useTransfromation)
{
  int count = 0;

  for (auto &stream : this->collections)
  {
    auto gstream = std::static_pointer_cast<InformationStream<GContainer>>(stream);

    auto infoSpec = stream->getDescription()->getInformationSpecification();

    if (request->getEntity() != "*" && request->getEntity() != infoSpec->getEntity())
    {
      continue;
    }
    if (request->getEntityType() != infoSpec->getEntityType())
    {
      continue;
    }
    if (request->getScope() != infoSpec->getScope())
    {
      continue;
    }
    if (request->getRepresentation() != infoSpec->getRepresentation())
    {
      if (useTransfromation && request->getRelatedEntity() == ""
          && infoSpec->getRelatedEntity() == "")
      {
        // check if transformation exists
        auto rep = this->gcontainerFactory->getTransformation(infoSpec->getRepresentation(),
                                                              request->getRepresentation());

        if (rep == nullptr)
          continue;

        std::shared_ptr<GContainer> input[1];
        input[0] = gstream->getLast()->getInformation();
        auto transInfo = rep->transform(input);

        if (transInfo == nullptr)
          continue;

        auto spec = std::make_shared<InformationSpecification>(infoSpec->getEntity(),
                                                               infoSpec->getEntityType(),
                                                               infoSpec->getScope(),
                                                               request->getRepresentation(),
                                                               infoSpec->getRelatedEntity());
        auto element = std::make_shared<InformationElement<GContainer>>(spec, transInfo);
        outInfo.push_back(element);
        ++count;
      }

      continue;
    }
    if (request->getRelatedEntity() != infoSpec->getRelatedEntity())
    {
      continue;
    }

    outInfo.push_back(gstream->getLast());
    ++count;
  }

  return count;
}

} /* namespace ice */

