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

namespace ice
{

class Identity;
struct Id;

typedef std::string serval_id;
typedef std::string ontology_iri;

class IdentityDirectory
{
public:
  static const std::string ID_SERVAL;
  static const std::string ID_ONTOLOGY;

public:
  IdentityDirectory();
  virtual ~IdentityDirectory();

  std::shared_ptr<Identity> lookup(std::string const &key, std::string const &value, bool create = false);
  std::shared_ptr<Identity> lookup(const std::initializer_list<Id>& ids, bool create = false);

  std::shared_ptr<Identity> create(std::string const &key, std::string const &value);
  std::shared_ptr<Identity> create(const std::initializer_list<Id>& ids);

private:
  std::vector<std::shared_ptr<Identity>> identities;
};

} /* namespace ice */

#endif /* IDENTITIES_H_ */
