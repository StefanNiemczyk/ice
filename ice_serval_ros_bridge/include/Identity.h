/*
 * Identity.h
 *
 *  Created on: Apr 12, 2016
 *      Author: sni
 */

#ifndef IDENTITY_H_
#define IDENTITY_H_

#include <chrono>
#include <map>
#include <string>
#include <memory>
#include <easylogging++.h>

#include <ice/model/aspModel/ASPSystem.h>

namespace ice
{

class IdentityDirectory;
class OntologyInterface;

enum identity_match {
  FULL_MATCH,
  PARTIAL_MATCH,
  INCLUDED,
  INCLUDING,
  NO_MATCH,
  CONFLICTING
};

struct Id {
    template <typename KeyType, typename ValueType>
    Id(KeyType&& key, ValueType&& value)
            : key{key}, value{value} {}

    std::string key;
    std::string value;
};

class Identity : public std::enable_shared_from_this<Identity>
{
public:
  Identity(IdentityDirectory *directory, const std::initializer_list<Id>& ids);
  virtual ~Identity();

  int initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface);
  std::map<std::string, std::string> readConfiguration(std::string const config);

  identity_match checkMatching(std::shared_ptr<Identity> &identity);
  identity_match checkMatching(const std::initializer_list<Id>& ids);
  identity_match checkMatching(std::string &key, std::string &value);
  void fuse(std::shared_ptr<Identity> &identity);
  void fuse(const std::initializer_list<Id>& ids);
  void fuse(std::map<std::string,std::string>& ids);
  void pushIdsToMap(std::map<std::string,std::string> &map);

  void checkIce();
  bool isIceIdentity();
  void setIceIdentity(bool value);
  bool isTimeout();

  std::chrono::steady_clock::time_point getActiveTimestamp();
  void setActiveTimestamp(std::chrono::steady_clock::time_point = std::chrono::steady_clock::now());

  bool isAvailable();
  void setAvailable(bool const &value);

  void addId(std::string const &key, std::string const &value);
  bool getId(std::string const &key, std::string &outValue);

  void addMetadata(std::string const &key, std::string const &value);
  bool getMetadata(std::string const &key, std::string &outValue);

  void addConnectionQuality(std::string const &key, double const &value);
  bool getConnectionQuality(std::string const &key, double &outValue);

  std::string toString();

  // ASP Stuff
  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);
  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);
  void addASPElement(std::shared_ptr<ASPElement> node);

private:
  IdentityDirectory                                     *directory;
  bool                                                  iceIdentity;
  bool                                                  available;
  std::chrono::steady_clock::time_point                 timestamp;
  std::chrono::milliseconds                             timeoutDuration;
  std::map<std::string, std::string>                    ids;
  std::map<std::string, std::string>                    metadata;
  std::map<std::string, double>                         connectionQuality;
  el::Logger*                                           _log;                   /**< Logger */


  std::shared_ptr<supplementary::External>              systemExternal;         /**< The external for the system */
  std::vector<std::shared_ptr<ASPElement>>              aspNodes;               /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspSourceNodes;         /**< Vector of asp source nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspIro;                 /**< Vector of iro nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredStreams;     /**< Vector of required streams */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredMaps;        /**< Vector of required maps */
};

} /* namespace ice */

#endif /* IDENTITY_H_ */
