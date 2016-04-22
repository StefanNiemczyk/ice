/*
 * Identity.cpp
 *
 *  Created on: Apr 12, 2016
 *      Author: sni
 */

#include "Identity.h"

#include <iostream>
#include <sstream>

#include <ice/ontology/OntologyInterface.h>

#include "IdentityDirectory.h"

namespace ice
{

Identity::Identity(IdentityDirectory *directory, const std::initializer_list<Id>& ids)
      : iceIdentity(false), directory(directory), available(false), _log(el::Loggers::getLogger("Identity"))
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
  //
}

int Identity::initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface)
{
  _log->debug("Extract asp information from ontology for identity '%s'", this->toString());
  std::string ownIri;

  if (false == this->getId(IdentityDirectory::ID_ONTOLOGY, ownIri))
    return -1;

  auto nodes = ontologyInterface->readNodesAndIROsAsASP(ownIri);

  std::vector<const char*>* types = nodes->at(0);
  std::vector<const char*>* names = nodes->at(1);
  std::vector<const char*>* strings = nodes->at(2);
  std::vector<const char*>* aspStrings = nodes->at(3);
  std::vector<const char*>* cppStrings = nodes->at(4);

  int count = 0;

  for (int i = 0; i < names->size(); ++i)
  {
    const char* name = names->at(i);
    const char* elementStr = strings->at(i);
    const char* aspStr = aspStrings->at(i);
    const char* cppStr = cppStrings->at(i);
    const char* typeStr = types->at(i);
    ASPElementType type;

    if (typeStr == nullptr || name == nullptr || elementStr == nullptr)
    {
      _log->error("Empty string for element '%v': '%v' (elementStr), '%v' (typeStr), element will be skipped",
                  name == nullptr ? "null" : name, elementStr == nullptr ? "null" : elementStr,
                  typeStr == nullptr ? "null" : typeStr);

      delete name;
      delete elementStr;
      delete aspStr;
      delete cppStr;
      delete typeStr;

      continue;
    }

    if (std::strcmp(typeStr, "COMPUTATION_NODE") == 0)
    {
      type = ASPElementType::ASP_COMPUTATION_NODE;
    }
    else if (std::strcmp(typeStr, "SOURCE_NODE") == 0)
    {
      type = ASPElementType::ASP_SOURCE_NODE;
    }
    else if (std::strcmp(typeStr, "REQUIRED_STREAM") == 0)
    {
      type = ASPElementType::ASP_REQUIRED_STREAM;
    }
    else if (std::strcmp(typeStr, "MAP_NODE") == 0)
    {
      type = ASPElementType::ASP_MAP_NODE;
    }
    else if (std::strcmp(typeStr, "IRO_NODE") == 0)
    {
      type = ASPElementType::ASP_IRO_NODE;
    }
    else if (std::strcmp(typeStr, "REQUIRED_MAP") == 0)
    {
      type = ASPElementType::ASP_REQUIRED_MAP;
    }
    else
    {
      _log->error("Unknown asp element type '%v' for element '%v', element will be skipped", typeStr, name);

      delete name;
      delete elementStr;
      delete aspStr;
      delete cppStr;
      delete typeStr;

      continue;
    }

    auto node = this->getASPElementByName(type, name);

    if (!node)
    {
      _log->info("ASP element '%v' not found, creating new element", std::string(name));
      auto element = std::make_shared<ASPElement>();
      element->aspString = aspStr;
      element->name = name;
      element->state = ASPElementState::ADDED_TO_ASP;
      element->type = type;

      if (std::strlen(cppStr) != 0)
      {
        const char* index = std::strchr(cppStr, '\n');
        element->className = std::string(cppStr, index);
        element->configAsString = std::string(index + 1, std::strlen(cppStr));
        element->config = this->readConfiguration(element->configAsString);
      }

      element->raw = elementStr;

      // ASP external stuff removed
//      auto value = supplementary::ClingWrapper::stringToValue();
//      element->external = this->asp->getExternal(*value.name(), value.args());
//
//      switch (type)
//      {
//        case ASPElementType::ASP_COMPUTATION_NODE:
//        case ASPElementType::ASP_SOURCE_NODE:
//        case ASPElementType::ASP_MAP_NODE:
//        case ASPElementType::ASP_IRO_NODE:
//          if (false == this->nodeStore->existNodeCreator(element->className))
//          {
//            _log->warn("Missing creator for node '%v' of type '%v', cpp grounding '%v', asp external set to false",
//                       element->name, ASPElementTypeNames[type],
//                       element->className == "" ? "NULL" : element->className);
//            element->external->assign(false);
//          }
//          else
//          {
//            element->external->assign(true);
//          }
//          break;
//        default:
//          element->external->assign(true);
//          break;
//      }
//
//      this->asp->add(name, {}, aspStr);
//      this->asp->ground(name, {});
//
//      this->addASPElement(element);
//      this->groundingDirty = true;
      ++count;
    }

    delete name;
    delete elementStr;
    delete aspStr;
    delete cppStr;
    delete typeStr;
  }

  delete types;
  delete names;
  delete strings;
  delete aspStrings;
  delete cppStrings;

  return count;
}

std::map<std::string, std::string> Identity::readConfiguration(std::string const config)
{
  std::map<std::string, std::string> configuration;
  std::stringstream ss(config);
  std::string item;

  while (std::getline(ss, item, ';'))
  {
    int index = item.find("=");

    if (index == std::string::npos)
    {
      _log->warn("Broken configuration '%v', skipped", item);
    }

    configuration[item.substr(0, index)] = item.substr(index + 1, item.size());
  }

  return configuration;
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

void Identity::fuse(std::map<std::string,std::string>& ids)
{
  for (auto &id : ids)
  {
    this->ids[id.first] = id.second;
  }
}

void Identity::pushIdsToMap(std::map<std::string,std::string> &map)
{
  for (auto &id : this->ids)
  {
    map[id.first] = id.second;
  }
}

void Identity::checkIce()
{
  if (this->iceIdentity)
    return;

  this->iceIdentity = (this->ids.find(IdentityDirectory::ID_ONTOLOGY) != this->metadata.end());

  if (this->iceIdentity && this->available)
    this->directory->callDiscoveredIceIdentityHooks(this->shared_from_this());
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

bool Identity::isAvailable()
{
  return this->available;
}

void Identity::setAvailable(bool const &value)
{
  if (value)
  {
    if (this->isIceIdentity() && false == this->available)
    {
      this->directory->callDiscoveredIceIdentityHooks(this->shared_from_this());
    }
  }
  else
  {
    if (this->isIceIdentity())
    {
      this->directory->callVanishedIceIdentityHooks(this->shared_from_this());
    }
  }

  this->available = value;
}

bool Identity::isTimeout()
{
  auto now = std::chrono::steady_clock::now();

  return ((now - this->timestamp) > this->timeoutDuration);
}

void Identity::addId(std::string const &key, std::string const &value)
{
  this->ids[key] = value;
}

bool Identity::getId(std::string const &key, std::string &outValue)
{
  auto cq = this->ids.find(key);

  if (cq == this->metadata.end())
    return false;

  outValue = cq->second;

  return true;
}

void Identity::addMetadata(std::string const &key, std::string const &value)
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

void Identity::addConnectionQuality(std::string const &key, double const &value)
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

std::string Identity::toString()
{
  std::stringstream ss;
  ss << "Identity(";

  for (auto &id : this->ids)
  {
    ss << id.first << "='" << id.second << "',";
  }

  ss << ")";
  return ss.str();
}


// -----------------------------------------------------------------------------------
// ----------------------------------- ASP Stuff -------------------------------------
// -----------------------------------------------------------------------------------

std::shared_ptr<ASPElement> Identity::getASPElementByName(ASPElementType type, std::string const name)
{
  switch (type)
  {
    case ASP_COMPUTATION_NODE:
      for (auto node : this->aspNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_SOURCE_NODE:
      for (auto node : this->aspSourceNodes)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_IRO_NODE:
      for (auto node : this->aspIro)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_REQUIRED_STREAM:
      for (auto node : this->aspRequiredStreams)
      {
        if (node->name == name)
          return node;
      }
      break;
    case ASP_REQUIRED_MAP:
      for (auto node : this->aspRequiredMaps)
      {
        if (node->name == name)
          return node;
      }
      break;
  }

  return nullptr;
}

std::shared_ptr<ASPElement> Identity::getASPElementByName(std::string const name)
{
  for (auto node : this->aspNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspSourceNodes)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspIro)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspRequiredStreams)
  {
    if (node->name == name)
      return node;
  }

  for (auto node : this->aspRequiredMaps)
  {
    if (node->name == name)
      return node;
  }

  return nullptr;
}

void Identity::addASPElement(std::shared_ptr<ASPElement> node)
{
  switch (node->type)
  {
    case ASP_COMPUTATION_NODE:
      this->aspNodes.push_back(node);
      break;
    case ASP_SOURCE_NODE:
      this->aspSourceNodes.push_back(node);
      break;
    case ASP_IRO_NODE:
      this->aspIro.push_back(node);
      break;
    case ASP_REQUIRED_STREAM:
      this->aspRequiredStreams.push_back(node);
      break;
    case ASP_REQUIRED_MAP:
      this->aspRequiredMaps.push_back(node);
      break;
  }
}


} /* namespace ice */
