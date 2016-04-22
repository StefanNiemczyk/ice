/*
 * identities.h
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#ifndef IDENTITIES_H_
#define IDENTITIES_H_

#include <memory>
#include <string>
#include <vector>

#include "Entity.h"

namespace ice
{

class OntologyInterface;
struct Id;

typedef std::string serval_id;
typedef std::string ontology_iri;

// hooks to register for discovered or vanished entities
using discoveredEntityHook = std::function<void(std::shared_ptr<Entity> const &)>;
using vanishedEntityHook = std::function<void(std::shared_ptr<Entity> const &)>;

// hooks to register for discovered or vanished communication entities
using discoveredIceEntityHook = std::function<void(std::shared_ptr<Entity> const &)>;
using vanishedIceEntityHook = std::function<void(std::shared_ptr<Entity> const &)>;

// hooks to register for new offered information
using offeredInformationHook = std::function<void(std::shared_ptr<Entity> const &)>;

class EntityDirectory
{
public:
  static const std::string ID_SERVAL;
  static const std::string ID_ONTOLOGY;
  static const std::string ID_ONTOLOGY_SHORT;

public:
  EntityDirectory();
  virtual ~EntityDirectory();

  int initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface);

  std::shared_ptr<Entity> lookup(std::string const &key, std::string const &value, bool create = false);
  std::shared_ptr<Entity> lookup(const std::initializer_list<Id>& ids, bool create = false);

  std::shared_ptr<Entity> create(std::string const &key, std::string const &value);
  std::shared_ptr<Entity> create(const std::initializer_list<Id>& ids);

  std::unique_ptr<std::vector<std::shared_ptr<Entity>>> availableIdentities();
  void checkTimeout();

  void registerDiscoveredIceIdentityHook(discoveredIceEntityHook hook);
  void unregisterDiscoveredIceIdentityHook(discoveredIceEntityHook hook);
  void callDiscoveredIceIdentityHooks(std::shared_ptr<Entity> const identity);

  void registerVanishedIceIdentityHooks(vanishedIceEntityHook hook);
  void unregisterVanishedIceIdentityHooks(vanishedIceEntityHook hook);
  void callVanishedIceIdentityHooks(std::shared_ptr<Entity> const identity);

  void registerOfferedInformationHooks(offeredInformationHook hook);
  void unregisterOfferedInformationHooks(offeredInformationHook hook);
  void callOfferedInformationHooks(std::shared_ptr<Entity> const identity);

  int count();
  void print();

public:
  std::shared_ptr<Entity> self;

private:
  std::vector<std::shared_ptr<Entity>> entities;
  std::vector<discoveredIceEntityHook> discoveredIceIdentityHooks;
  std::vector<vanishedIceEntityHook> vanishedIceIdentityHooks;
  std::vector<offeredInformationHook> offeredInformationHooks;
};

} /* namespace ice */

#endif /* IDENTITIES_H_ */
