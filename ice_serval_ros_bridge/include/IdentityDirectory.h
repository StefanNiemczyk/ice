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

#include "Identity.h"

namespace ice
{

class OntologyInterface;
struct Id;

typedef std::string serval_id;
typedef std::string ontology_iri;
using discoveredIceIdentityHook = std::function<void(std::shared_ptr<Identity> const &)>;
using vanishedIceIdentityHook = std::function<void(std::shared_ptr<Identity> const &)>;

class IdentityDirectory
{
public:
  static const std::string ID_SERVAL;
  static const std::string ID_ONTOLOGY;
  static const std::string ID_ONTOLOGY_SHORT;

public:
  IdentityDirectory();
  virtual ~IdentityDirectory();

  int initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface);

  std::shared_ptr<Identity> lookup(std::string const &key, std::string const &value, bool create = false);
  std::shared_ptr<Identity> lookup(const std::initializer_list<Id>& ids, bool create = false);

  std::shared_ptr<Identity> create(std::string const &key, std::string const &value);
  std::shared_ptr<Identity> create(const std::initializer_list<Id>& ids);

  std::unique_ptr<std::vector<std::shared_ptr<Identity>>> availableIdentities();
  void checkTimeout();

  void registerDiscoveredIceIdentityHook(discoveredIceIdentityHook hook);
  void unregisterDiscoveredIceIdentityHook(discoveredIceIdentityHook hook);
  void callDiscoveredIceIdentityHooks(std::shared_ptr<Identity> const identity);

  void registerVanishedIceIdentityHooks(vanishedIceIdentityHook hook);
  void unregisterVanishedIceIdentityHooks(vanishedIceIdentityHook hook);
  void callVanishedIceIdentityHooks(std::shared_ptr<Identity> const identity);

  int count();
  void print();

public:
  std::shared_ptr<Identity> self;

private:
  std::vector<std::shared_ptr<Identity>> identities;
  std::vector<discoveredIceIdentityHook> discoveredIceIdentityHooks;
  std::vector<vanishedIceIdentityHook> vanishedIceIdentityHooks;
};

} /* namespace ice */

#endif /* IDENTITIES_H_ */
