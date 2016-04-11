/*
 * rhizome.h
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#ifndef RHIZOME_H_
#define RHIZOME_H_

#include <memory>
#include <vector>

namespace ice
{

class serval_interface;
struct serval_bundle;
struct serval_bundle_manifest;

namespace serval_wrapper {

class rhizome
{
private:
  const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_LIST = "/restful/rhizome/bundlelist.json";
  const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_LIST_BY_TOKEN = "/restful/rhizome/newsince/$TOKEN/bundlelist.json";
  const std::string SERVAL_REST_RHIZOME_GET_BUNDLE = "/restful/rhizome/$BID.rhm";
  const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_RAW = "/restful/rhizome/$BID/raw.bin";
  const std::string SERVAL_REST_RHIZOME_GET_BUNDLE_DECRIPTED = "/restful/rhizome/$BID/decrypted.bin";
  const std::string SERVAL_REST_RHIZOME_POST_BUNDLE = "/restful/rhizome/insert";
  const std::string SERVAL_REST_RHIZOME_POST_APPEND_BUNDLE = "/restful/rhizome/append";

public:
  rhizome(serval_interface *interface);
  virtual ~rhizome();

  std::unique_ptr<std::vector<serval_bundle>> getBundleList(std::string token = "");
  std::unique_ptr<serval_bundle_manifest> getBundleManifest(std::string bundleId);
  std::string getBundlePayload(std::string bundleId, bool decrypted = false);
  std::unique_ptr<serval_bundle_manifest> addBundle(std::string pathToFile, std::string manifest = "", std::string bundleId = "",
                 std::string author = "", std::string secret = "");
  std::unique_ptr<serval_bundle_manifest> appendBundle(std::string pathToFile, std::string manifest = "", std::string bundleId = "",
                 std::string author = "", std::string secret = "");

private:
  std::unique_ptr<serval_bundle_manifest> parseManifest(std::string &source);
  std::string parse(std::string &source, std::string key);

private:
  serval_interface* interface;
};

}
} /* namespace ice */

#endif /* RHIZOME_H_ */
