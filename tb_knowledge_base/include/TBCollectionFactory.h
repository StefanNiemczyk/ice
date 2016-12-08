/*
 * TBCollectionFactory.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_TBCOLLECTIONFACTORY_H_
#define INCLUDE_TBCOLLECTIONFACTORY_H_

#include <ice/information/CollectionFactory.h>

namespace ice
{

class TBCollectionFactory : public CollectionFactory
{
public:
  TBCollectionFactory(std::weak_ptr<ICEngine> engine);
  virtual ~TBCollectionFactory();

  virtual std::shared_ptr<BaseInformationStream> createStream(const std::string& className,
                                                                std::shared_ptr<CollectionDescription> streamDescription,
                                                                std::shared_ptr<EventHandler> eventHandler,
                                                                int streamSize) const;

  virtual std::shared_ptr<BaseInformationSet> createSet(const std::string& className,
                                                              std::shared_ptr<CollectionDescription> streamDescription,
                                                              std::shared_ptr<EventHandler> eventHandler) const;
};

} /* namespace ice */

#endif /* INCLUDE_TBCOLLECTIONFACTORY_H_ */
