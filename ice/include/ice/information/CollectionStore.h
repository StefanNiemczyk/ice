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
#include "ice/information/KnowledgeBase.h"
#include "ice/processing/EventHandler.h"
#include "ice/representation/GContainerFactory.h"
#include "easylogging++.h"

namespace ice
{
template <typename T>
struct SelectedCollection
{
  std::shared_ptr<T> selected;
  std::shared_ptr<T> current;
};

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
      this->gcontainerFactory = engineObject->getGContainerFactory();
      this->knowledgeBase = engineObject->getKnowlegeBase();
    }
  }

  virtual void cleanUp()
  {
    this->eventHandler.reset();
    this->factory.reset();
    this->gcontainerFactory.reset();
    this->knowledgeBase.reset();

    for(auto collection : this->collections)
    {
      collection->destroy();
    }

    this->collections.clear();
  }

  /*!
   * \brief Returns a collection for the given specification.
   *
   * Returns a collection for the given specification. NULL is returned if no stream exists.
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

  /*!
   * \brief Returns a selected collection for the given specification.
   *
   * Returns a selected collection for the given specification. NULL is returned if no stream exists.
   */
  std::shared_ptr<T> getSelectedBaseCollection(InformationSpecification *specification)
  {
    _log->debug("Get selected collection by '%v'", specification->toString());

    for (auto collection : this->selected)
    {
      auto spec = collection.selected->getDescription();
      if (false == (*spec->getInformationSpecification() == *specification))
        continue;

      return collection.selected;
    }

    _log->warn("No selected collection found for '%v'", specification->toString());

    return nullptr;
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

  void print()
  {
    std::lock_guard<std::mutex> guard(this->_mtx);
    std::cout << "------------------------------------------------------------" << std::endl;
    for (auto &collection : this->collections)
    {
      std::cout << collection->toString() << std::endl;
    }
    std::cout << "------------------------------------------------------------" << std::endl;
  }

  int registerSelected(std::shared_ptr<T> collection)
  {
    std::lock_guard<std::mutex> guard(this->_mtx);
    _log->info("Register selected collection for '%v'", collection->getSpecification()->toString());

    for (auto &selected : this->selected)
    {
      if (*selected.selected->getDescription()->getInformationSpecification()  == *collection->getSpecification())
      {
        if (selected.current != nullptr)
        {
          selected.current->unregisterBaseListenerSync(selected.selected);
        }

        collection->registerBaseListenerSync(selected.selected);
        selected.current = collection;

        return 0;
      }
    }

    std::map<std::string, int> metadatas;
    auto desc = std::make_shared<CollectionDescription>(collection->getSpecification(),
                                                        "selecte_collection", "selected", "selected", metadatas);

    SelectedCollection<T> selected;
    auto dataType = this->knowledgeBase->dataTypeForRepresentation(collection->getSpecification()->getRepresentation());
    std::shared_ptr<InformationCollection> ic;

    if (collection->getCollectionType() == CollectionType::CT_SET)
      ic = this->factory->createSet(dataType, desc, this->eventHandler);
    else
      ic = this->factory->createStream(dataType, desc, this->eventHandler, 100);

    selected.selected = std::static_pointer_cast<T>(ic);
    selected.current = collection;
    collection->registerBaseListenerSync(selected.selected);

    this->selected.push_back(selected);

    return 1;
  }


  template<typename G>
  std::shared_ptr<G> generateSelected(std::shared_ptr<InformationSpecification> infoSpec, CollectionType type)
  {
    std::lock_guard<std::mutex> guard(this->_mtx);
    _log->info("Generate selected collection for '%v'", infoSpec->toString());

    for (auto &selected : this->selected)
    {
      if (*selected.selected->getDescription()->getInformationSpecification()  == *infoSpec)
      {
        return std::dynamic_pointer_cast<G>(selected.selected);
      }
    }

    std::map<std::string, int> metadatas;
    auto desc = std::make_shared<CollectionDescription>(infoSpec,
                                                        "selecte_collection", "selected", "selected", metadatas);

    SelectedCollection<T> selected;
    auto dataType = this->knowledgeBase->dataTypeForRepresentation(infoSpec->getRepresentation());
    std::shared_ptr<InformationCollection> ic;

    if (type == CollectionType::CT_SET)
      ic = this->factory->createSet(dataType, desc, this->eventHandler);
    else
      ic = this->factory->createStream(dataType, desc, this->eventHandler, 100);

    selected.selected = std::static_pointer_cast<T>(ic);

    this->selected.push_back(selected);

    return std::dynamic_pointer_cast<G>(selected.selected);
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
  std::weak_ptr<ICEngine>               engine;                 /**< Weak pointer to the engine */
  std::shared_ptr<GContainerFactory>    gcontainerFactory;      /**< The gcontainer factory */
  std::vector<std::shared_ptr<T>>       collections;            /**< The information steams */
  std::vector<SelectedCollection<T>>    selected;               /**< The selected information collections */
  std::shared_ptr<EventHandler>         eventHandler;           /**< Handler to execute events asynchronously */
  std::shared_ptr<CollectionFactory>    factory;                /**< Factory to create collections */
  std::shared_ptr<KnowledgeBase>        knowledgeBase;          /**< The knowledge base */
  std::mutex                            _mtx;                   /**< Mutex */
  el::Logger*                           _log;                   /**< Logger */
};

} /* namespace ice */

#endif /* INCLUDE_ICE_INFORMATION_COLLECTIONSTORE_H_ */
