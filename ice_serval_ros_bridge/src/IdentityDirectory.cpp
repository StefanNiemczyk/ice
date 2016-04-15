/*
 * identities.cpp
 *
 *  Created on: Apr 11, 2016
 *      Author: sni
 */

#include "IdentityDirectory.h"

#include "Identity.h"

namespace ice
{

const std::string IdentityDirectory::ID_SERVAL = "id_serval";
const std::string IdentityDirectory::ID_ONTOLOGY = "id_ontology";

IdentityDirectory::IdentityDirectory()
{
  // TODO Auto-generated constructor stub

}

IdentityDirectory::~IdentityDirectory()
{
  //
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
      case (identity_match::INCLUDED):
        matches.push_back(id);
        break;
      case (identity_match::INCLUDING):
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
    auto id = matches[0];
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
  auto newId = std::make_shared<Identity>(ids);
  this->identities.push_back(newId);

  return newId;
}

std::unique_ptr<std::vector<std::shared_ptr<Identity>>> IdentityDirectory::activeIdentities()
{
  std::unique_ptr<std::vector<std::shared_ptr<Identity>>> vec (new std::vector<std::shared_ptr<Identity>>);

  for (auto &id : this->identities)
  {
    if (id->isTimeout() == false && id->isIceIdentity())
    {
      vec->push_back(id);
    }
  }

  return vec;
}

} /* namespace ice */
