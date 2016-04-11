/*
 * rhizome.cpp
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#include "serval_wrapper/rhizome.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <regex>
#include <cpr/cpr.h>

#include "serval_interface.h"

// Short alias for this namespace
namespace pt = boost::property_tree;

namespace ice
{

namespace serval_wrapper {

rhizome::rhizome(serval_interface *interface) : interface(interface)
{

}

rhizome::~rhizome()
{

}

std::unique_ptr<std::vector<serval_bundle>> rhizome::getBundleList(std::string token)
{
  std::string path;

  if (token == "")
  {
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_LIST;
  }
  else
  {
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_LIST_BY_TOKEN;
    path.replace(path.find("$TOKEN"), 6, token);
  }

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                    cpr::Timeout{this->interface->getTimeout()});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get bundle list");
    return nullptr;
  }

  if (r.header["content-type"] != "application/json")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for get bundle list");
    return nullptr;
  }

  // WORKAROUND to fix a bug in the restful api, if requesting a bundle list by a token the closing braces are missing
  if (r.text.find("\n]\n}") == std::string::npos)
  {
    r.text += "\n]\n}\n";
  }

  // Read json.
  pt::ptree tree;
  std::istringstream is(r.text);

  try
  {
    pt::read_json(is, tree);
  }
  catch (std::exception &e)
  {
    std::cout<< r.text << std::endl;                         // JSON text string
    this->interface->logError("JSON could not be parsed for get bundle list");
    return nullptr;
  }
  std::unique_ptr<std::vector<serval_bundle>> ids(new std::vector<serval_bundle>);

  for (pt::ptree::value_type &rows : tree.get_child("rows"))
  {
    int i = 1;
    serval_bundle bundle;
    for (pt::ptree::value_type &row : rows.second)
    {
      switch (i)
      {
        case 1:
          bundle.token = row.second.get_value<std::string>();
          break;
        case 2:
          bundle._id = row.second.get_value<int>();
          break;
        case 3:
          bundle.service = row.second.get_value<std::string>();
          break;
        case 4:
          bundle.id = row.second.get_value<std::string>();
          break;
        case 5:
          bundle.date = row.second.get_value<long>();
          break;
        case 6:
          bundle.inserttime = row.second.get_value<long>();
          break;
        case 7:
          bundle.author = row.second.get_value<std::string>();
          break;
        case 8:
          if (row.second.empty())
            bundle.fromhere = -1;
          else
            bundle.fromhere = row.second.get_value<int>();
          break;
        case 9:
          if (row.second.empty())
            bundle.filesize = -1;
          else
            bundle.filesize = row.second.get_value<long>();
          break;
        case 10:
          bundle.filehash = row.second.get_value<std::string>();
          break;
        case 11:
          bundle.sender = row.second.get_value<std::string>();
          break;
        case 12:
          bundle.recipient = row.second.get_value<std::string>();
          break;
        case 13:
          bundle.name = row.second.get_value<std::string>();
          break;
      }

      ++i;
    }
    ids->push_back(bundle);
  }

  return std::move(ids);
}


std::unique_ptr<serval_bundle_manifest> rhizome::getBundleManifest(std::string bundleId)
{
  std::string path = SERVAL_REST_RHIZOME_GET_BUNDLE;
  path.replace(path.find("$BID"), 4, bundleId);

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                    cpr::Timeout{this->interface->getTimeout()});

//  std::cout << this->address + path << std::endl;
//  std::cout << r.error.message << std::endl;
//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get bundle manifest for bid '" + bundleId + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "rhizome-manifest/text")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for get bundle manifest for bid '" + bundleId + "'");
    return nullptr;
  }

  return std::move(this->parseManifest(r.text));
}


std::string rhizome::getBundlePayload(std::string bundleId, bool decrypted)
{
  std::string path;

  if (decrypted)
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_DECRIPTED;
  else
    path = SERVAL_REST_RHIZOME_GET_BUNDLE_RAW;
  path.replace(path.find("$BID"), 4, bundleId);

  auto r = cpr::Get(cpr::Url {this->interface->getAddress() + path}, *this->interface->getAuth(),
                    cpr::Timeout{this->interface->getTimeout()});

//  std::cout<< r.status_code << std::endl;                  // 200
//  std::cout<< r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout<< r.text << std::endl;                         // JSON text string

  if (r.status_code != 200) // HTTP_OK
  {
    this->interface->logError("Unexpected return code '" + std::to_string(r.status_code) + "' for get bundle manifest for bid '" + bundleId + "'");
    return "";
  }

  if (r.header["content-type"] != "application/octet-stream")
  {
    this->interface->logError("Unexpected return type '" + r.header["content-type"] + "' for get bundle manifest for bid '" + bundleId + "'");
    return "";
  }

  return r.text;
}

std::unique_ptr<serval_bundle_manifest> rhizome::addBundle(std::string pathToFile, std::string manifest, std::string bundleId, std::string author,
                                 std::string secret)
{
  auto form = cpr::Multipart { {"manifest", "", "rhizome/manifest"}, {"payload", cpr::File {pathToFile}}};
  auto r = cpr::Post(cpr::Url {this->interface->getAddress() + SERVAL_REST_RHIZOME_POST_BUNDLE},
                     *this->interface->getAuth(), form, cpr::Timeout{this->interface->getTimeout()});

//  std::cout << pathToFile << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 201) // HTTP_ADD
  {
    this->interface->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for add rhizome file '" + pathToFile + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "rhizome-manifest/text")
  {
    this->interface->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for add rhizome file '" + pathToFile + "'");
    return nullptr;
  }

  return std::move(this->parseManifest(r.text));
}

std::unique_ptr<serval_bundle_manifest> rhizome::appendBundle(std::string pathToFile, std::string manifest, std::string bundleId, std::string author,
                                 std::string secret)
{
  auto form = cpr::Multipart { {"manifest", "", "rhizome/manifest"}, {"payload", cpr::File {pathToFile}}};
  auto r = cpr::Post(cpr::Url {this->interface->getAddress() + SERVAL_REST_RHIZOME_POST_APPEND_BUNDLE},
                     *this->interface->getAuth(), form, cpr::Timeout{this->interface->getTimeout()});

//  std::cout << pathToFile << std::endl;
//  std::cout << r.status_code << std::endl;                  // 200
//  std::cout << r.header["content-type"] << std::endl;       // application/json; charset=utf-8
//  std::cout << r.text << std::endl;                         // JSON text string

  if (r.status_code != 201) // HTTP_ADD
  {
    this->interface->logError(
        "Unexpected return code '" + std::to_string(r.status_code) + "' for add rhizome file '" + pathToFile + "'");
    return nullptr;
  }

  if (r.header["content-type"] != "rhizome-manifest/text")
  {
    this->interface->logError(
        "Unexpected return type '" + r.header["content-type"] + "' for add rhizome file '" + pathToFile + "'");
    return nullptr;
  }

  return std::move(this->parseManifest(r.text));
}

std::unique_ptr<serval_bundle_manifest> rhizome::parseManifest(std::string &source)
{
  std::unique_ptr<serval_bundle_manifest> manifest(new serval_bundle_manifest());

  manifest->service = this->parse(source, "service");
  manifest->version = std::stol(this->parse(source, "version"));
  manifest->id = this->parse(source, "id");
  manifest->date = std::stol(this->parse(source, "date"));
  manifest->name = this->parse(source, "name");
  manifest->filesize = std::stol(this->parse(source, "filesize"));
  manifest->filehash = this->parse(source, "filehash");

  std::string crypt = this->parse(source, "crypt");
  if (crypt != "")
    manifest->crypt =  std::stoi(crypt);
  else
    manifest->crypt = 0;
  manifest->tail = this->parse(source, "tail");

  return std::move(manifest);
}

std::string rhizome::parse(std::string &source, std::string key)
{
  int index1, index2;
  index1 =  source.find(key + "=");
  if (index1 == std::string::npos)
    return "";

  index1 +=  + key.size() + 1;
  index2 = source.find("\n", index1);

  if (index2 == std::string::npos)
    return "";

  return source.substr(index1, index2-index1);
}

}
} /* namespace ice */
