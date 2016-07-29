/*
 * ProcessingModel.h
 *
 *  Created on: Sep 17, 2015
 *      Author: sni
 */

#ifndef PROCESSINGMODEL_H_
#define PROCESSINGMODEL_H_

#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

using namespace std;

//Forward declaration
namespace ice
{
  class EngineState;
} /* namespace ice */

namespace ice
{
// Type defs for model description transfer
struct InputStreamDesc
{
  string                        sourceSystem;
  string                        nodeName;
  string                        nodeEntity;
  string                        nodeEntityRelated;
  string                        entity;
  string                        scope;
  string                        representation;
  string                        relatedEntity;
  std::map<std::string, int>    metadata;

  friend class boost::serialization::access;
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      // serialize base class information
      ar & sourceSystem;
      ar & nodeName;
      ar & nodeEntity;
      ar & nodeEntityRelated;
      ar & entity;
      ar & scope;
      ar & representation;
      ar & relatedEntity;
      ar & metadata;
    }
};

//typedef tuple<string,                   // source system
//              string,                   // node name
//              string,                   // node entity
//              string,                   // node entity 2
//              string,                   // entity
//              string,                   // scope
//              string,                   // representation
//              string,                   // related entity
//              std::map<std::string, int>// metadata
//              > InputStreamDesc;

struct OutputStreamDesc
{
  string                        entity;
  string                        scope;
  string                        representation;
  string                        relatedEntity;
  std::map<std::string, int>    metadata;

  friend class boost::serialization::access;
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      // serialize base class information
      ar & entity;
      ar & scope;
      ar & representation;
      ar & relatedEntity;
      ar & metadata;
    }
};
//typedef tuple<string,                   // entity
//              string,                   // scope
//              string,                   // representation
//              string,                   // related entity
//              std::map<std::string, int>// metadata
//              > OutputStreamDesc;

struct NodeDesc
{
  uint8_t                       type;
  string                        className;
  string                        aspName;
  string                        entity;
  string                        relatedEntity;
  string                        config;
  vector<InputStreamDesc>       inputs;
  vector<OutputStreamDesc>      outputs;

  friend class boost::serialization::access;
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      // serialize base class information
      ar & type;
      ar & className;
      ar & aspName;
      ar & entity;
      ar & relatedEntity;
      ar & config;
      ar & inputs;
      ar & outputs;
    }
};
//typedef tuple<uint8_t,                  // Type
//              string,                   // class name
//              string,                   // asp name
//              string,                   // entity
//              string,                   // entity 2
//              string,                   // config as string
//              vector<InputStreamDesc>,  // inputs
//              vector<OutputStreamDesc>  // outputs
//              > NodeDesc;

struct TransferDesc
{
  string                        sourceSystem;
  string                        nodeName;
  string                        nodeEntity;
  string                        nodeEntityRelated;
  string                        entity;
  string                        scope;
  string                        representation;
  string                        relatedEntity;

  friend class boost::serialization::access;
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      // serialize base class information
      ar & sourceSystem;
      ar & nodeName;
      ar & nodeEntity;
      ar & nodeEntityRelated;
      ar & entity;
      ar & scope;
      ar & representation;
      ar & relatedEntity;
    }
};
//typedef tuple<string,                   // source
//              string,                   // node name
//              string,                   // node entity
//              string,                   // node entity 2
//              string,                   // entity
//              string,                   // scope
//              string,                   // representation
//              string                    // related entity
//              > TransferDesc;

struct SubModelDesc
{
  int                           index;
  vector<NodeDesc>              nodes;
  vector<TransferDesc>          send;
  vector<TransferDesc>          receive;

  friend class boost::serialization::access;
  template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      // serialize base class information
      ar & index;
      ar & nodes;
      ar & send;
      ar & receive;
    }
};
//typedef tuple<int,                      // The index of the sub model
//              vector<NodeDesc>,         // Nodes which needs to be activated
//              vector<TransferDesc>,     // Streams transfered from self to the external system
//              vector<TransferDesc>      // Streams transfered from the external system to self
//            > SubModelDesc;

struct SubModel
{
  std::shared_ptr<EngineState> engine;
  bool accepted = false;
  std::shared_ptr<SubModelDesc> model;
};

struct StreamTransfer
{
  std::shared_ptr<EngineState> engine;
  std::vector<TransferDesc> transfer;
};

class ProcessingModel
{
private:
  static int counter;

public:
  ProcessingModel();
  virtual ~ProcessingModel();

  int getIndex() const;

  std::vector<NodeDesc>& getNodes();

  std::vector<std::shared_ptr<StreamTransfer>>& getReceive();

  std::vector<std::shared_ptr<StreamTransfer>>& getSend();

  std::vector<std::shared_ptr<SubModel>>& getSubModels();

private:
  const int index; /**< Index of the processing model */
  std::vector<std::shared_ptr<SubModel>> subModels; /**< Sub models that needs to be estableshed by other engines */
  std::vector<NodeDesc> nodes; /**< Nodes required by this model */
  std::vector<std::shared_ptr<StreamTransfer>> send; /**< Streams that needs to be send to other engines */
  std::vector<std::shared_ptr<StreamTransfer>> receive; /**< Streams that needs to be received from other engines */
};

} /* namespace ice */

#endif /* PROCESSINGMODEL_H_ */
