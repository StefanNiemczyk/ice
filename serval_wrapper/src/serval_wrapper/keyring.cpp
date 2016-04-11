/*
 * keyring.cpp
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#include "serval_wrapper/keyring.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <regex>
#include <cpr/cpr.h>
#include <sstream>

#include "serval_interface.h"

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{
namespace serval_wrapper {

keyring::keyring(serval_interface* const interface) : interface(interface)
{
}

keyring::~keyring()
{
  // TODO Auto-generated destructor stub
}


std::unique_ptr<std::vector<serval_identity>> keyring::getIdentities()
{
  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + SERVAL_REST_GET_IDENTITIES},
                    *this->interface->getAuth(), cpr::Timeout{this->interface->getTimeout()});

  std::cout<< r.status_code << std::endl;                  // 200
  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get identities");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for get identities");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);
  std::unique_ptr<std::vector<serval_identity>> ids(new std::vector<serval_identity>);

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    std::string sid, did, name;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          sid = row.second.get_value<std::string>();
          break;
        case 2:
          did = row.second.get_value<std::string>();
          break;
        case 3:
          name = row.second.get_value<std::string>();
          break;
      }

      ++i;
    }
    serval_identity id(sid, did, name);
    ids->push_back(id);
  }

  return std::move(ids);
}

std::unique_ptr<serval_identity> keyring::addIdentity(std::string password)
{
  std::string param = "";
  if (password != "")
  {
    param = "?pin=" + password;
  }

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + SERVAL_REST_ADD_IDENTITY} + param,
                    *this->interface->getAuth(), cpr::Timeout{this->interface->getTimeout()});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for add identity");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for add identity");
    return nullptr;
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);

  std::string sid = tree.get_child("identity").get<std::string>("sid");

  std::unique_ptr<serval_identity> id(new serval_identity(sid, "", ""));
  return std::move(id);
}

std::unique_ptr<serval_identity> keyring::setIdentity(std::string sid, std::string name, std::string did)
{
  cpr::Parameters params;

  if (did != "")
  {
    params = cpr::Parameters{{"did", did}, {"name", name}};
  }
  else
  {
    params = cpr::Parameters{{"name", name}};
  }

  std::string path = SERVAL_REST_GET_IDENTITY_SET;
  path.replace(path.find("$SID"), 4, sid);

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, params,
                    *this->interface->getAuth(), cpr::Timeout {this->interface->getTimeout()});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for set identity '" + sid + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for set identity '" + sid + "'");
    return nullptr;
  }

// Read json.
  pt::ptree tree;
  std::istringstream is(r.text);
  pt::read_json(is, tree);

  std::string _sid = tree.get_child("identity").get<std::string>("sid");
  std::string _did = tree.get_child("identity").get<std::string>("did");
  std::string _name = tree.get_child("identity").get<std::string>("name");

  std::unique_ptr<serval_identity> id(new serval_identity(_sid, _did, _name));
  return std::move(id);
}

std::unique_ptr<std::vector<serval_identity>> keyring::getPeerIdentities()
{
  return std::move(this->getIdentitiesCmdLine("peers"));
}

std::unique_ptr<std::vector<serval_identity>> keyring::getAllPeerIdentities()
{
  return std::move(this->getIdentitiesCmdLine("allpeers"));
}

std::unique_ptr<std::vector<serval_identity>> keyring::getSelf()
{
  return std::move(this->getIdentitiesCmdLine("self"));
}

std::unique_ptr<std::vector<serval_identity>> keyring::getIdentitiesCmdLine(std::string param)
{
  std::string command = this->interface->getServalBin() + " id " + param;
  std::stringstream ss;
  std::string line;

  this->interface->exec(command.c_str(), ss);

  // the first line should be a 1
  std::getline(ss, line);
  if (line != "1")
  {
    this->interface->logError("Could not load ids '" + param + "' via command line " + line + ss.str());
    return nullptr;
  }

  // the secound line should be 'sid'
  std::getline(ss, line);
  if (line != "sid")
  {
    this->interface->logError("Could not load ids '" + param + "' via command line " + line + ss.str());
    return nullptr;
  }

  std::unique_ptr<std::vector<serval_identity>> ids(new std::vector<serval_identity>);

  for (std::string line; std::getline(ss, line); )
  {
    // read sids
    serval_identity id(line, "", "");
    ids->push_back(id);
  }

  return ids;
}

} /* namespace serval_wrapper */
} /* namespace ice */
