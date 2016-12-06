/*
 * SetStore.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#include "ice/information/SetStore.h"

#include "ice/representation/Transformation.h"

namespace ice
{

SetStore::SetStore(std::weak_ptr<ICEngine> engine)
{
  this->_log = el::Loggers::getLogger("SetStore");
  this->engine = engine;
}

SetStore::SetStore(std::shared_ptr<EventHandler> eventHandler, std::shared_ptr<CollectionFactory> factory)
{
  this->_log = el::Loggers::getLogger("SetStore");
  this->eventHandler = eventHandler;
  this->factory = factory;
}

SetStore::~SetStore()
{
  //
}

std::shared_ptr<BaseInformationSet> SetStore::registerBaseSet(
    std::string dataType, std::shared_ptr<InformationSpecification> specification, const std::string name,
    std::map<std::string, int> &metadata, std::string provider, std::string sourceSystem)
{
  auto ptr = this->getBaseCollection(specification.get(), provider, sourceSystem);

  //set already registered
  if (ptr != nullptr)
  {
    _log->warn("Duplicated Set with '%v', '%v', '%v'",
                  specification->toString(), provider, sourceSystem);
    return ptr;
  }

  auto desc = std::make_shared<CollectionDescription>(specification, name, provider, sourceSystem, metadata);
  std::string type = dataType;
  auto set = this->factory->createSet(type, desc, this->eventHandler);

  if (set)
  {
    _log->debug("Created set with '%v', '%v', '%v', '%v'", specification->toString(),
                provider, sourceSystem, type);
    this->collections.push_back(set);
  }
  else
  {
    _log->error("Set with '%v', '%v', '%v', '%v' could not be created", specification->toString(),
                provider, sourceSystem, type);
  }
  return set;
}


int SetStore::getInformation(std::shared_ptr<InformationSpecification> request,
                                std::vector<std::shared_ptr<InformationElement<GContainer>>> &outInfo,
                                bool useTransfromation)
{
  int count = 0;

  for (auto &set : this->collections)
  {
    std::shared_ptr<Transformation> trans;
    auto gset = std::static_pointer_cast<InformationSet<GContainer>>(set);
    auto infoSpec = set->getDescription()->getInformationSpecification();


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
        trans = this->gcontainerFactory->getTransformation(infoSpec->getRepresentation(),
                                                              request->getRepresentation());

        if (trans == nullptr)
          continue;
      }

      continue;
    }
    if (request->getRelatedEntity() != infoSpec->getRelatedEntity())
    {
      continue;
    }

    for (auto &entity : set->getAllEntities())
    {
      if (request->getEntity() != "*" && request->getEntity() != entity)
      {
        continue;
      }

      if (trans)
      {
        std::shared_ptr<GContainer> input[1];
        input[0] = gset->get(entity)->getInformation();
        auto transInfo = trans->transform(input);

        if (transInfo == nullptr)
        continue;

        auto spec = std::make_shared<InformationSpecification>(entity,
        infoSpec->getEntityType(),
        infoSpec->getScope(),
        request->getRepresentation(),
        infoSpec->getRelatedEntity());
        auto element = std::make_shared<InformationElement<GContainer>>(spec, transInfo);
        outInfo.push_back(element);
        ++count;
      }
      else
      {
        outInfo.push_back(gset->get(entity));
        ++count;
      }
    }
  }

  return count;
}
} /* namespace ice */
