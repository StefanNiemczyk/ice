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

Identity::Identity(const std::initializer_list<Id>& ids) : iceIdentity(false)
{
  if (ids.size() == 0)
  {
    throw  std::runtime_error(std::string("Identity ids can not be empty"));
  }

  for (const auto& id : ids) {
          this->ids[id.key] = id.value;
  }

  timeoutDuration = std::chrono::milliseconds(2000);
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

identity_match Identity::checkMatching(const std::initializer_list<Id>& ids)
{
  int matchCount = 0;
  bool conflicting = false;

  for (auto &id : ids)
  {
    auto rid = this->ids.find(id.key);

    if (rid == this->ids.end())
    {
      continue;
    }

    if (rid->second == id.value)
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

  if (matchCount == this->ids.size() && matchCount == ids.size())
  {
    return identity_match::FULL_MATCH;
  }

  if (matchCount == this->ids.size())
  {
    return identity_match::INCLUDED;
  }

  if (matchCount == ids.size())
  {
    return identity_match::INCLUDING;
  }

  return identity_match::PARTIAL_MATCH;
}

identity_match Identity::checkMatching(std::string &key, std::string &value)
{
  auto id = this->ids.find(key);

  if (id == this->ids.end())
  {
    return identity_match::NO_MATCH;
  }

  if (id->second == value)
    return identity_match::FULL_MATCH;

  return identity_match::CONFLICTING;
}

void Identity::fuse(std::shared_ptr<Identity> &identity)
{
  for (auto &id : identity->ids)
  {
    this->ids[id.first] = id.second;
  }
}


void Identity::fuse(const std::initializer_list<Id>& ids)
{
  for (auto &id : ids)
  {
    this->ids[id.key] = id.value;
  }
}

bool Identity::isIceIdentity()
{
  return this->iceIdentity;
}

void Identity::setIceIdentity(bool value)
{
  this->iceIdentity = value;
}

std::chrono::steady_clock::time_point Identity::getActiveTimestamp()
{
  return this->timestamp;
}

void Identity::setActiveTimestamp(std::chrono::steady_clock::time_point value)
{
  this->timestamp = value;
}

bool Identity::isTimeout()
{
  auto now = std::chrono::steady_clock::now();

  return ((now - this->timestamp) > this->timeoutDuration);
}

bool Identity::getId(std::string const &key, std::string &outValue)
{
  auto cq = this->ids.find(key);

  if (cq == this->metadata.end())
    return false;

  outValue = cq->second;

  return true;
}

void Identity::addMetadata(std::string const &key, std::string &value)
{
  this->metadata[key] = value;
}

bool Identity::getMetadata(std::string const &key, std::string &outValue)
{
  auto cq = this->metadata.find(key);

  if (cq == this->metadata.end())
    return false;

  outValue = cq->second;

  return true;
}

void Identity::addConnectionQuality(std::string const &key, double &value)
{
  this->connectionQuality[key] = value;
}

bool Identity::getConnectionQuality(std::string const &key, double &outValue)
{
  auto cq = this->connectionQuality.find(key);

  if (cq == this->connectionQuality.end())
    return false;

  outValue = cq->second;

  return true;
}

} /* namespace ice */
