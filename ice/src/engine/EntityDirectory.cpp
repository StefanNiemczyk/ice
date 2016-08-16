/*
 * identities.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "ice/EntityDirectory.h"

#include <iostream>
#include <vector>

#include "ice/ontology/OntologyInterface.h"
#include "ice/ICEngine.h"
#include "ice/Time.h"

namespace ice
{

const std::string EntityDirectory::ID_ICE               = "id_ice";
const std::string EntityDirectory::ID_SERVAL            = "id_serval";
const std::string EntityDirectory::ID_ONTOLOGY          = "id_ontology";
const std::string EntityDirectory::ID_ONTOLOGY_SHORT    = "id_ontology_short";

EntityDirectory::EntityDirectory(std::weak_ptr<ICEngine> const &engine) : engine(engine)
{
  //
}

EntityDirectory::~EntityDirectory()
{
  //
}

void EntityDirectory::init()
{
  auto e = this->engine.lock();
  this->timeFactory = e->getTimeFactory();

  std::initializer_list<Id> ids = {{ID_SERVAL, ""}, {ID_ONTOLOGY, ""}};
  this->self = std::make_shared<Entity>(this->shared_from_this(), this->engine, this->timeFactory, ids);
  this->self->setIceIdentity(true);
}

void EntityDirectory::cleanUp()
{
  //
}

int EntityDirectory::initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface)
{
  if (ontologyInterface->isLoadDirty())
    ontologyInterface->loadOntologies();

  if (false == ontologyInterface->isSystemDirty())
    return 0;

  auto ontSystems = ontologyInterface->getSystems();

  if (ontSystems == nullptr)
  {
//    _log->error("Error occurred while reading systems");
    return 0;
  }

  for (auto ontSystem : *ontSystems)
  {
    std::cout << "Found identity with IRI: '" << std::string(ontSystem) << "'" << std::endl;
    this->lookup({{EntityDirectory::ID_ONTOLOGY_SHORT, ontSystem},
                  {EntityDirectory::ID_ONTOLOGY, ontologyInterface->toShortIri(ontSystem)}}, true);
  }

  return ontSystems->size();
}

std::shared_ptr<Entity> EntityDirectory::lookup(const std::initializer_list<Id>& ids, bool create)
{
  std::vector<std::shared_ptr<Entity>> matches;
  std::vector<std::shared_ptr<Entity>> including;

  // Check if an identity with this
  for (auto &id : this->entities)
  {
    switch (id->checkMatching(ids))
    {
      case (entity_match::FULL_MATCH):
      case (entity_match::INCLUDING):
        matches.push_back(id);
        break;
      case (entity_match::INCLUDED):
        including.push_back(id);
        break;
    }
  }

  if (matches.size() > 1)
  {
    // multiple identities matches
    return nullptr; // TODO
  }
  else if (matches.size() == 1)
  {
    // return matching identity
    return matches[0];
  }

  if (including.size() > 1)
  {
    // multiple identities are included in
    return nullptr; // TODO
  }
  else if (including.size() == 1)
  {
    // include new ids in single id which is included in new ide
    auto id = including[0];
    id->fuse(ids);

    return id;
  }

  if (false == create)
    return nullptr;

  return this->create(ids);
}

std::shared_ptr<Entity> EntityDirectory::lookup(std::string const &key, std::string const &value, bool create)
{
  return this->lookup({{key, value}}, create);
}

std::shared_ptr<Entity> EntityDirectory::create(std::string const &key, std::string const &value)
{
  return this->create({{key, value}});
}

std::shared_ptr<Entity> EntityDirectory::create(const std::initializer_list<Id>& ids)
{
  // create new entity
  auto entity = std::make_shared<Entity>(this->shared_from_this(), this->engine, this->timeFactory, ids);
  this->entities.push_back(entity);

  return entity;
}

std::unique_ptr<std::vector<std::shared_ptr<Entity>>> EntityDirectory::allEntities()
{
  std::unique_ptr<std::vector<std::shared_ptr<Entity>>> vec (new std::vector<std::shared_ptr<Entity>>(this->entities));

  return vec;
}

std::unique_ptr<std::vector<std::shared_ptr<Entity>>> EntityDirectory::availableEntities()
{
  std::unique_ptr<std::vector<std::shared_ptr<Entity>>> vec (new std::vector<std::shared_ptr<Entity>>);

  for (auto &id : this->entities)
  {
    if (id == this->self)
    {
      continue;
    }

    if (id->isTimeout() == false && id->isAvailable())
    {
      vec->push_back(id);
    }
  }

  return vec;
}

std::unique_ptr<std::vector<std::shared_ptr<Entity>>> EntityDirectory::activeCooperationEntities()
{
  std::unique_ptr<std::vector<std::shared_ptr<Entity>>> vec (new std::vector<std::shared_ptr<Entity>>);

  for (auto &id : this->entities)
  {
    if (id == this->self)
    {
      continue;
    }

    if (id->isActiveCooperation())
    {
      vec->push_back(id);
    }
  }

  return vec;
}

void EntityDirectory::checkTimeout()
{
  for (auto &entity : this->entities)
  {
    if(entity->isAvailable() && entity->isTimeout())
        entity->setAvailable(false);
  }
}

int EntityDirectory::count()
{
  return this->entities.size();
}

void EntityDirectory::print()
{
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "EntityDirectory: " << this->entities.size() << " entries" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  for (auto &id : this->entities)
  {
    std::cout << id->toString() << std::endl;
  }
  std::cout << "---------------------------------------------------------" << std::endl;
}

} /* namespace ice */
