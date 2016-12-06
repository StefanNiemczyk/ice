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

//  std::shared_ptr<StreamDescription> streamDescription,
//  std::shared_ptr<EventHandler> eventHandler,
//  int streamSize,
//  int sharingMaxCount = 0

  if ("int" == className)
  {
    stream = std::make_shared<InformationStream<int>>(streamDescription, eventHandler, streamSize);
  }
  else if ("long" == className)
  {
    stream = std::make_shared<InformationStream<long>>(streamDescription, eventHandler, streamSize);
  }
  else if ("double" == className)
  {
    stream = std::make_shared<InformationStream<double>>(streamDescription, eventHandler, streamSize);
  }
  else if ("float" == className)
  {
    stream = std::make_shared<InformationStream<float>>(streamDescription, eventHandler, streamSize);
  }
  else if ("bool" == className)
  {
    stream = std::make_shared<InformationStream<float>>(streamDescription, eventHandler, streamSize);
  }
  else if ("List[int]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<int>>>(streamDescription, eventHandler, streamSize);
  }
  else if ("List[long]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<long>>>(streamDescription, eventHandler, streamSize);
  }
  else if ("List[double]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<double>>>(streamDescription, eventHandler, streamSize);
  }
  else if ("List[float]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<float>>>(streamDescription, eventHandler, streamSize);
  }
  else if ("List[bool]" == className)
  {
    stream = std::make_shared<InformationStream<std::vector<bool>>>(streamDescription, eventHandler, streamSize);
  }
  else
  {
    auto rep = this->gcontainerFactory->getRepresentation(className);

    if (rep != nullptr)
    {
      stream = std::make_shared<InformationStream<GContainer>>(streamDescription, eventHandler, streamSize);
      stream->setGContainer(true);
    }
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

  //  std::shared_ptr<StreamDescription> streamDescription,
  //  std::shared_ptr<EventHandler> eventHandler,
  //  int streamSize,
  //  int sharingMaxCount = 0

  if ("int" == className)
  {
    set = std::make_shared<InformationSet<int>>(streamDescription, eventHandler);
  }
  else if ("long" == className)
  {
    set = std::make_shared<InformationSet<long>>(streamDescription, eventHandler);
  }
  else if ("double" == className)
  {
    set = std::make_shared<InformationSet<double>>(streamDescription, eventHandler);
  }
  else if ("float" == className)
  {
    set = std::make_shared<InformationSet<float>>(streamDescription, eventHandler);
  }
  else if ("bool" == className)
  {
    set = std::make_shared<InformationSet<float>>(streamDescription, eventHandler);
  }
  else if ("List[int]" == className)
  {
    set = std::make_shared<InformationSet<std::vector<int>>>(streamDescription, eventHandler);
  }
  else if ("List[long]" == className)
  {
    set = std::make_shared<InformationSet<std::vector<long>>>(streamDescription, eventHandler);
  }
  else if ("List[double]" == className)
  {
    set = std::make_shared<InformationSet<std::vector<double>>>(streamDescription, eventHandler);
  }
  else if ("List[float]" == className)
  {
    set = std::make_shared<InformationSet<std::vector<float>>>(streamDescription, eventHandler);
  }
  else if ("List[bool]" == className)
  {
    set = std::make_shared<InformationSet<std::vector<bool>>>(streamDescription, eventHandler);
  }
  else
  {
    auto rep = this->gcontainerFactory->getRepresentation(className);

    if (rep != nullptr)
    {
      set = std::make_shared<InformationSet<GContainer>>(streamDescription, eventHandler);
      set->setGContainer(true);
    }
  }

  return set;
}

} /* namespace ice */
