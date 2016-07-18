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

#include "ice/processing/CallbackList.h"
#include "ice/Entity.h"

namespace ice
{

class OntologyInterface;
struct Id;

typedef std::string serval_id;
typedef std::string ontology_iri;

class EntityDirectory
{
public:
  static const std::string ID_ICE;
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

  int count();
  void print();

public:
  std::shared_ptr<Entity> self;
  CallbackList<std::shared_ptr<Entity>> disvoeredIceIdentity;
  CallbackList<std::shared_ptr<Entity>> vanishedIceIdentity;
  CallbackList<std::shared_ptr<Entity>> offeredInformation;

private:
  std::vector<std::shared_ptr<Entity>> entities;
};

} /* namespace ice */

#endif /* IDENTITIES_H_ */
