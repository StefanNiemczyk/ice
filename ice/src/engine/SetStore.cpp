/*
 * SetStore.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: sni
 */

#include "ice/information/SetStore.h"

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

} /* namespace ice */
