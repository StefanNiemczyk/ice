/*
 * StreamFactory.cpp
 *
 *  Created on: May 30, 2014
 *      Author: sni
 */

#include <ice/information/CollectionFactory.h>
#include "ice/ICEngine.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"

namespace ice
{

CollectionFactory::CollectionFactory(std::weak_ptr<ICEngine> engine) :
    engine(engine)
{
  //
}

CollectionFactory::~CollectionFactory()
{
  //
}

void CollectionFactory::init()
{
  auto e = this->engine.lock();
  this->gcontainerFactory = e->getGContainerFactory();
}

void CollectionFactory::cleanUp()
{
  this->gcontainerFactory.reset();
}

std::shared_ptr<BaseInformationStream> CollectionFactory::createStream(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler, int streamSize) const
{
  std::shared_ptr<BaseInformationStream> stream;

  if (className == "")
    return stream;

  auto rep = this->gcontainerFactory->getRepresentation(className);

  if (rep != nullptr)
  {
    stream = std::make_shared<InformationStream<GContainer>>(streamDescription, eventHandler, streamSize);
    stream->setGContainer(true);
  }

  return stream;
}

std::shared_ptr<BaseInformationSet> CollectionFactory::createSet(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler) const
{
  std::shared_ptr<BaseInformationSet> set;

  if (className == "")
    return set;

  auto rep = this->gcontainerFactory->getRepresentation(className);

  if (rep != nullptr)
  {
    set = std::make_shared<InformationSet<GContainer>>(streamDescription, eventHandler);
    set->setGContainer(true);
  }

  return set;
}

} /* namespace ice */
