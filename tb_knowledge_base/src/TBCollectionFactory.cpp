/*
 * TBCollectionFactory.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#include <TBCollectionFactory.h>

#include <ice/information/InformationSet.h>
#include <ice/information/InformationStream.h>

namespace ice
{

TBCollectionFactory::TBCollectionFactory(std::weak_ptr<ICEngine> engine) : CollectionFactory(engine)
{

}

TBCollectionFactory::~TBCollectionFactory()
{
  //
}

std::shared_ptr<BaseInformationStream> TBCollectionFactory::createStream(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler, int streamSize) const
{
  std::shared_ptr<BaseInformationStream> stream;

  if (className == "")
    return nullptr;

  if ("int" == className)
  {
    stream = std::make_shared<InformationStream<int>>(streamDescription, eventHandler, streamSize);
  }

  if (stream == nullptr)
        stream = ice::CollectionFactory::createStream(className, streamDescription, eventHandler, streamSize);

  return stream;
}

std::shared_ptr<BaseInformationSet> TBCollectionFactory::createSet(
    const std::string& className, std::shared_ptr<CollectionDescription> streamDescription,
    std::shared_ptr<EventHandler> eventHandler) const
{
  std::shared_ptr<BaseInformationSet> set;

  if (className == "")
    return nullptr;

  if ("int" == className)
  {
    set = std::make_shared<InformationSet<int>>(streamDescription, eventHandler);
  }

  if (set == nullptr)
    set = ice::CollectionFactory::createSet(className, streamDescription, eventHandler);

  return set;
}

} /* namespace ice */
