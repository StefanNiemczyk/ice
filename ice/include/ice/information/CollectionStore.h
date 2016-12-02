/*
 * CollectionStore.h
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#ifndef INCLUDE_ICE_INFORMATION_COLLECTIONSTORE_H_
#define INCLUDE_ICE_INFORMATION_COLLECTIONSTORE_H_

#include <vector>

#include "ice/ICEngine.h"
#include "ice/Configuration.h"
#include "ice/information/CollectionDescription.h"
#include "ice/information/CollectionFactory.h"
#include "ice/information/InformationSpecification.h"
#include "ice/processing/EventHandler.h"
#include "easylogging++.h"

namespace ice
{
//Forward declarations
//class ICEngine;
class CollectionFactory;


template <typename T>
class CollectionStore
{
public:
  CollectionStore()
  {
    _log = nullptr;
  };
  virtual ~CollectionStore() {};


  virtual void init()
  {
    if (false == this->engine.expired())
    {
      auto engineObject = engine.lock();

      this->eventHandler = engineObject->getEventHandler();
      this->factory = engineObject->getCollectionFactory();
    }
  }

  virtual void cleanUp()
  {
    this->eventHandler.reset();
    this->factory.reset();

    for(auto collection : this->collections)
    {
      collection->destroy();
    }

    this->collections.clear();
  }

  /*!
   * \brief Returns a T for the given specification.
   *
   * Returns a T for the given specification. NULL is returned if no stream exists.
   */
  std::shared_ptr<T> getBaseCollection(InformationSpecification *specification,
                                       std::string provider, std::string sourceSystem)
  {
    _log->debug("Get collection by '%v', '%v', '%v'", specification->toString(), provider, sourceSystem);

    std::vector<std::shared_ptr<T>> selected;

    for (auto collection : this->collections)
    {
      auto spec = collection->getDescription();
      if (false == (*spec->getInformationSpecification() == *specification))
        continue;

      if (provider != "" && provider != spec->getProvider())
        continue;

      if (sourceSystem != "" && sourceSystem != spec->getSourceSystem())
        continue;

      selected.push_back(collection);
    }

    if (selected.size() == 0)
    {
      return nullptr;
    }

    return selectBest(selected);
  }

  void cleanUpUnused()
  {
    std::lock_guard<std::mutex> guard(this->_mtx);
    _log->verbose(1, "Start removing unused collections");
    int counter = 0;

    for (int i=0; i < this->collections.size(); ++i)
    {
      auto collection = this->collections.at(i);


      _log->info("Checking collection '%v', reference count %v", collection->toString(), collection.use_count());

      if (collection.use_count() == 2)
      {
        _log->info("Remove unused collection '%v'", collection->toString());

        ++counter;
        this->collections.erase(this->collections.begin() + i);
        --i;
      }
    }

    _log->info("Clean up collection store: '%v' collections are removed", counter);
  }

protected:
  std::shared_ptr<T> selectBest(std::vector<std::shared_ptr<T>> &collection)
  {
    if (collection.size() == 0)
    {
      return nullptr;
    }

    auto best = collection.at(0);

    for (int i = 0; i < collection.size(); ++i)
    {
      // TODO
    }

    return best;
  }

protected:
  std::weak_ptr<ICEngine>               engine;         /**< Weak pointer to the engine */
  std::vector<std::shared_ptr<T>>       collections;    /**< The information steams */
  std::shared_ptr<EventHandler>         eventHandler;   /**< Handler to execute events asynchronously */
  std::shared_ptr<CollectionFactory>    factory;        /**< Factory to create collections */
  std::mutex                            _mtx;           /**< Mutex */
  el::Logger*                           _log;           /**< Logger */
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_COLLECTIONSTORE_H_ */
