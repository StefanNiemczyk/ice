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
#include <memory>
#include <set>
#include <string>

#include "ice/information/InformationSpecification.h"
#include "ice/processing/NodeDescription.h"
#include "ice/Time.h"
#include "easylogging++.h"

namespace supplementary
{
class External;
} /* namespace supplementary */

namespace ice
{

class BaseInformationStream;
class BaseInformationSet;
class EntityDirectory;
class ICEngine;
class Node;
class OntologyInterface;
class TimeFactory;
struct SubModelDesc;

//* ASPNodeState
/**
 * Enum of states an ASP node.
 *
 */
enum ASPElementState
{
  NEW_ELEMENT, ADDED_TO_ASP
};

//* ASPNodeType
/**
 * Enum of ASP node types
 *
 */
enum ASPElementType
{
  ASP_COMPUTATION_NODE, ASP_SOURCE_NODE, ASP_TRANSFORMATION_NODE, ASP_SET_NODE, ASP_REQUIRED_STREAM, ASP_REQUIRED_SET
};
const std::string ASPElementTypeNames[] {"ASP_COMPUTATION_NODE", "ASP_SOURCE_NODE", "ASP_TRANSFORMATION_NODE", "ASP_SET_NODE",
                                         "ASP_REQUIRED_STREAM", "ASP_REQUIRED_SET"};

//* ASPNode
/**
 * This struct contains the asp informations of a node.
 *
 */
struct ASPElement
{
  std::shared_ptr<supplementary::External> external;
  std::string aspString;
  std::string name;
  std::string className;
  std::string configAsString;
  std::string raw;
  bool defect;
  std::map<std::string, std::string> config;
  ASPElementState state;
  ASPElementType type;

  NodeType getNodeType() {
    switch (this->type)
    {
      case ASPElementType::ASP_SOURCE_NODE:
        return NodeType::SOURCE;
      case ASPElementType::ASP_COMPUTATION_NODE:
        return NodeType::PROCESSING;
      case ASPElementType::ASP_TRANSFORMATION_NODE:
        return NodeType::TRANSFORMATION;
      case ASPElementType::ASP_SET_NODE:
        return NodeType::SET;
    }

    return NodeType::PROCESSING;
  }
};

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
  SharedSubModel() : accepted(false) {}

  bool                                                  accepted;        /**< True if the submodel can be accepted */
  std::shared_ptr<SubModelDesc>                         subModel;        /**< Sub model description */
  std::vector<std::shared_ptr<BaseInformationStream>>   streamsSend;     /**< List of streams send to this engine */
  std::vector<std::shared_ptr<BaseInformationStream>>   streamsReceived; /**< List of streams received from this engine */
  std::vector<std::shared_ptr<BaseInformationSet>>      setsSend;        /**< List of streams send to this engine */
  std::vector<std::shared_ptr<BaseInformationSet>>      setsReceived;    /**< List of streams received from this engine */
};

class Entity : public std::enable_shared_from_this<Entity>
{
public:
  Entity(const std::initializer_list<Id> ids);
  Entity(std::shared_ptr<EntityDirectory> const &directory, std::weak_ptr<ICEngine> engine,
		  std::shared_ptr<TimeFactory> const &factory, const std::initializer_list<Id> ids);
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
  void checkDirectory();

  void checkIce();
  bool isIceIdentity();
  void setIceIdentity(bool value);
  bool isTimeout();

  time getActiveTimestamp();
  void setActiveTimestamp();
  void setActiveTimestamp(time timestamp);

  bool isAvailable();
  void setAvailable(bool const &value);

  bool isCooperationPossible();
  bool isActiveCooperation();

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
  void setSuperEngine(bool value);

  // Sharing Stuff
  SharedSubModel& getSendSubModel();
  SharedSubModel& getReceivedSubModel();
  std::set<std::shared_ptr<Node>>& getNodes();
  std::vector<std::pair<std::string,std::string>>& getOntologyIriDiff();
  void updateReceived(std::vector<std::shared_ptr<Node>> &nodes,
                        std::vector<std::shared_ptr<BaseInformationStream>> &streamsSend,
                        std::vector<std::shared_ptr<BaseInformationStream>> &streamsReceived,
                        std::vector<std::shared_ptr<BaseInformationSet>> &setsSend,
                        std::vector<std::shared_ptr<BaseInformationSet>> &setsReceived);
  void updateSend(std::vector<std::shared_ptr<Node>> &nodes,
                       std::vector<std::shared_ptr<BaseInformationStream>> &streamsSend,
                       std::vector<std::shared_ptr<BaseInformationStream>> &streamsReceived,
                       std::vector<std::shared_ptr<BaseInformationSet>> &setsSend,
                       std::vector<std::shared_ptr<BaseInformationSet>> &setsReceived);
  void clearReceived();
  void clearSend();

  void toShortIris(std::string &str);
  void toLongIris(std::string &str);


  // ASP Stuff
  bool isNodesExtracted();
  void setNodesExtracted(bool value);
  std::shared_ptr<ASPElement> getASPElementByName(ASPElementType type, std::string const name);
  std::shared_ptr<ASPElement> getASPElementByName(std::string const name);
  void addASPElement(std::shared_ptr<ASPElement> node);
  void setExternal(std::shared_ptr<supplementary::External> &external);
  std::shared_ptr<supplementary::External>& getExternal();
  bool updateExternals(bool activateRequired);
  int getNodeForClass(std::string className, std::vector<std::shared_ptr<ASPElement>> &result);

private:
  void updateContainer(SharedSubModel &container, std::vector<std::shared_ptr<Node>> &nodes,
                       std::vector<std::shared_ptr<BaseInformationStream>> &streamsSend,
                       std::vector<std::shared_ptr<BaseInformationStream>> &streamsReceived,
                       std::vector<std::shared_ptr<BaseInformationSet>> &setsSend,
                       std::vector<std::shared_ptr<BaseInformationSet>> &setsReceived);
  void clearContainer(SharedSubModel &container);

private:
  bool                                                  superEngine;
  std::weak_ptr<ICEngine>                               engine;
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
  SharedSubModel                                        sendSubModel;
  SharedSubModel                                        receivedSubModel;
  std::set<std::shared_ptr<Node>>                       nodes;                  /**< Nodes activated by this engine */
  std::vector<std::pair<std::string,std::string>>       ontologyIriDiff;

  // ASP Stuff
  bool                                                  nodesExtracted;         /**< True if the nodes are extrected */
  std::shared_ptr<supplementary::External>              external;               /**< The external for the system */
  std::vector<std::shared_ptr<ASPElement>>              aspNodes;               /**< Vector of asp nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspSourceNodes;         /**< Vector of asp source nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspSetNodes;            /**< Vector of asp set nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspTransformation;      /**< Vector of transformation nodes */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredStreams;     /**< Vector of required streams */
  std::vector<std::shared_ptr<ASPElement>>              aspRequiredSets;        /**< Vector of required sets */
};

} /* namespace ice */

#endif /* IDENTITY_H_ */
