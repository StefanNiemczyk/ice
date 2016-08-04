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

#include "ice/information/InformationSpecification.h"
#include "ice/model/aspModel/ASPSystem.h"
#include "ice/Time.h"

namespace ice
{

class EntityDirectory;
class OntologyInterface;
class TimeFactory;

enum entity_match {
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

struct SharedSubModel
{
  std::shared_ptr<SubModelDesc> subModel; /**< Sub model description */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsSend; /**< List of streams send to this engine */
  std::vector<std::shared_ptr<BaseInformationStream>> streamsReceived; /**< List of streams received from this engine */
};

class Entity : public std::enable_shared_from_this<Entity>
{
public:
  Entity(const std::shared_ptr<EntityDirectory> const &directory,
		  std::shared_ptr<TimeFactory> const &factory, const std::initializer_list<Id>& ids);
  virtual ~Entity();

  uint8_t getNextIndex();

  int initializeFromOntology(std::shared_ptr<OntologyInterface> const &ontologyInterface);
  std::map<std::string, std::string> readConfiguration(std::string const config);

  entity_match checkMatching(std::shared_ptr<Entity> &identity);
  entity_match checkMatching(const std::initializer_list<Id>& ids);
  entity_match checkMatching(std::string &key, std::string &value);
  void fuse(std::shared_ptr<Entity> &identity);
  void fuse(const std::initializer_list<Id>& ids);
  void fuse(std::vector<std::tuple<std::string, std::string>>& ids);
  void pushIds(std::vector<std::tuple<std::string, std::string>>& vector);

  void checkIce();
  bool isIceIdentity();
  void setIceIdentity(bool value);
  bool isTimeout();

  time getActiveTimestamp();
  void setActiveTimestamp();
  void setActiveTimestamp(time timestamp);

  bool isAvailable();
  void setAvailable(bool const &value);

  void addId(std::string const &key, std::string const &value);
  bool getId(std::string const &key, std::string &outValue);

  void addMetadata(std::string const &key, std::string const &value);
  bool getMetadata(std::string const &key, std::string &outValue);

  void addConnectionQuality(std::string const &key, double const &value);
  bool getConnectionQuality(std::string const &key, double &outValue);

  std::string toString();

  std::vector<InformationSpecification>& getOfferedInformation();
  void addOfferedInformation(std::vector<InformationSpecification> const &offeres);

  std::vector<std::pair<std::string, std::string>>& getOntologyIds();

  // Sharing Stuff
  std::shared_ptr<SharedSubModel> const& getSendSubModel();
  std::shared_ptr<SharedSubModel> const& getReceivedSubModel();

  // ASP Stuff
  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);
  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);
  void addASPElement(std::shared_ptr<ASPElement> node);

private:
  std::shared_ptr<EntityDirectory>                      directory;
  std::shared_ptr<TimeFactory>				timeFactory;
  bool                                                  iceIdentity;
  bool                                                  available;
  uint8_t                                               index;
  time							timestamp;
  unsigned long long 		                        timeoutDuration;
  std::map<std::string, std::string>                    ids;
  std::map<std::string, std::string>                    metadata;
  std::map<std::string, double>                         connectionQuality;
  std::vector<InformationSpecification>                 offeredInformation;
  std::vector<std::pair<std::string, std::string>>      ontologyIds;
  el::Logger*                                           _log;                   /**< Logger */

  // Sharing Stuff
  std::shared_ptr<SharedSubModel>                       sendSubModel;
  std::shared_ptr<SharedSubModel>                       receivedSubModel;

  // ASP Stuff
  std::shared_ptr<supplementary::External>              systemExternal;         /**< The external for the system */
  std::vector<std::shared_ptr<ASPElement>>              aspNodes;               /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspSourceNodes;         /**< Vector of asp source nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspIro;                 /**< Vector of iro nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredStreams;     /**< Vector of required streams */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredMaps;        /**< Vector of required maps */
};

} /* namespace ice */

#endif /* IDENTITY_H_ */
