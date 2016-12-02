/*
 * StreamFactory.cpp
 *
 *  Created on: May 30, 2014
 *      Author: sni
 */

#include "ice/information/StreamFactory.h"

#include "ice/ICEngine.h"
#include "ice/representation/GContainer.h"
#include "ice/representation/GContainerFactory.h"

namespace ice
{

StreamFactory::StreamFactory(std::weak_ptr<ICEngine> engine) : engine(engine)
{
  //
}

StreamFactory::~StreamFactory()
{
  //
}

void StreamFactory::init()
{
  auto e = this->engine.lock();
  this->gcontainerFactory = e->getGContainerFactory();
}

void StreamFactory::cleanUp()
{
  this->gcontainerFactory.reset();
}

std::shared_ptr<BaseInformationStream> StreamFactory::createStream(const std::string& className,
                                                                   std::shared_ptr<CollectionDescription> streamDescription,
                                                                   std::shared_ptr<EventHandler> eventHandler,
                                                                   int streamSize) const
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
    }
  }

  return stream;
}

} /* namespace ice */
