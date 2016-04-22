/*
 * identities.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "IdentityDirectory.h"

#include <iostream>

#include <ice/ontology/OntologyInterface.h>

namespace ice
{

const std::string IdentityDirectory::ID_SERVAL = "id_serval";
const std::string IdentityDirectory::ID_ONTOLOGY = "id_ontology";
const std::string IdentityDirectory::ID_ONTOLOGY_SHORT = "id_ontology_short";

IdentityDirectory::IdentityDirectory()
{
  this->self = std::shared_ptr<Identity>(new Identity(this, {{"id_serval", ""}, {"id_ontology", ""}}));
  this->self->setIceIdentity(true);
}

IdentityDirectory::~IdentityDirectory()
{
  //
}

int IdentityDirectory::initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface)
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
    this->lookup({{IdentityDirectory::ID_ONTOLOGY_SHORT, ontSystem},
                  {IdentityDirectory::ID_ONTOLOGY, ontologyInterface->toShortIri(ontSystem)}}, true);
  }

  return ontSystems->size();
}

std::shared_ptr<Identity> IdentityDirectory::lookup(const std::initializer_list<Id>& ids, bool create)
{
  std::vector<std::shared_ptr<Identity>> matches;
  std::vector<std::shared_ptr<Identity>> including;

  // Check if an identity with this
  for (auto &id : this->identities)
  {
    switch (id->checkMatching(ids))
    {
      case (identity_match::FULL_MATCH):
      case (identity_match::INCLUDING):
        matches.push_back(id);
        break;
      case (identity_match::INCLUDED):
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

std::shared_ptr<Identity> IdentityDirectory::lookup(std::string const &key, std::string const &value, bool create)
{
  return this->lookup({{key, value}}, create);
}

std::shared_ptr<Identity> IdentityDirectory::create(std::string const &key, std::string const &value)
{
  return this->create({{key, value}});
}

std::shared_ptr<Identity> IdentityDirectory::create(const std::initializer_list<Id>& ids)
{
  // create new identity
  auto newId = std::make_shared<Identity>(this, ids);
  this->identities.push_back(newId);

  return newId;
}

std::unique_ptr<std::vector<std::shared_ptr<Identity>>> IdentityDirectory::availableIdentities()
{
  std::unique_ptr<std::vector<std::shared_ptr<Identity>>> vec (new std::vector<std::shared_ptr<Identity>>);

  for (auto &id : this->identities)
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

void IdentityDirectory::checkTimeout()
{
  for (auto &identity : this->identities)
  {
    if(identity->isAvailable() && identity->isTimeout())
        identity->setAvailable(false);
  }
}

void IdentityDirectory::registerDiscoveredIceIdentityHook(discoveredIceIdentityHook hook)
{
  this->discoveredIceIdentityHooks.push_back(hook);
}

void IdentityDirectory::unregisterDiscoveredIceIdentityHook(discoveredIceIdentityHook hook)
{
//  this->discoveredIceIdentityHooks.erase(hook);
}


void IdentityDirectory::callDiscoveredIceIdentityHooks(std::shared_ptr<Identity> const identity)
{
  for (auto &hook : this->discoveredIceIdentityHooks)
  {
    hook(identity);
  }
}

void IdentityDirectory::registerVanishedIceIdentityHooks(vanishedIceIdentityHook hook)
{
  this->vanishedIceIdentityHooks.push_back(hook);
}

void IdentityDirectory::unregisterVanishedIceIdentityHooks(vanishedIceIdentityHook hook)
{
//  this->vanishedIceIdentityHooks.erase(hook);
}


void IdentityDirectory::callVanishedIceIdentityHooks(std::shared_ptr<Identity> const identity)
{
  for (auto &hook : this->vanishedIceIdentityHooks)
  {
    hook(identity);
  }
}

int IdentityDirectory::count()
{
  return this->identities.size();
}

void IdentityDirectory::print()
{
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "IdentityDirectory: " << this->identities.size() << " entries" << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
  for (auto &id : this->identities)
  {
    std::cout << id->toString() << std::endl;
  }
  std::cout << "---------------------------------------------------------" << std::endl;
}

} /* namespace ice */
