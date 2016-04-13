/*
 * Identity.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: sni
 */

#include <iostream>

#include "Identity.h"

namespace ice
{

Identity::Identity(const std::initializer_list<Id>& ids)
{
  if (ids.size() == 0)
  {
    throw  std::runtime_error(std::string("Identity ids can not be empty"));
  }

  for (const auto& id : ids) {
          this->ids[id.key] = id.value;
  }
}

Identity::~Identity()
{
  // TODO Auto-generated destructor stub
}

identity_match Identity::checkMatching(std::shared_ptr<Identity> &identity)
{
  int matchCount = 0;
  bool conflicting = false;

  for (auto &id : this->ids)
  {
    auto rid = identity->ids.find(id.first);

    if (rid == identity->ids.end())
    {
      continue;
    }

    if (rid->second == id.second)
    {
      ++matchCount;
    }
    else
    {
      conflicting = true;
    }
  }

  if (matchCount == 0)
  {
    return identity_match::NO_MATCH;
  }

  if (conflicting)
  {
    return identity_match::CONFLICTING;
  }

  if (matchCount == this->ids.size() && matchCount == identity->ids.size())
  {
    return identity_match::FULL_MATCH;
  }

  if (matchCount == this->ids.size())
  {
    return identity_match::INCLUDED;
  }

  if (matchCount == identity->ids.size())
  {
    return identity_match::INCLUDING;
  }

  return identity_match::PARTIAL_MATCH;
}

void Identity::fuse(std::shared_ptr<Identity> &identity)
{
  for (auto &id : identity->ids)
  {
    this->ids[id.first] = id.second;
  }
}

} /* namespace ice */
