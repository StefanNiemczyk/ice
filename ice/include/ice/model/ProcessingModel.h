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

using namespace std;

//Forward declaration
namespace ice
{
  class EngineState;
} /* namespace ice */

namespace ice
{
// Type defs for model description transfer
typedef tuple<string,                   // source system
              string,                   // node name
              string,                   // node entity
              string,                   // node entity 2
              string,                   // entity
              string,                   // scope
              string,                   // representation
              string,                   // related entity
              std::map<std::string, int>// metadata
              > InputStreamDesc;

typedef tuple<string,                   // entity
              string,                   // scope
              string,                   // representation
              string,                   // related entity
              std::map<std::string, int>// metadata
              > OutputStreamDesc;

typedef tuple<uint8_t,                  // Type
              string,                   // class name
              string,                   // asp name
              string,                   // entity
              string,                   // entity 2
              string,                   // config as string
              vector<InputStreamDesc>,  // inputs
              vector<OutputStreamDesc>  // outputs
              > NodeDesc;

typedef tuple<string,                   // source
              string,                   // node name
              string,                   // node entity
              string,                   // node entity 2
              string,                   // entity
              string,                   // scope
              string,                   // representation
              string                    // related entity
              > TransferDesc;

typedef tuple<int,                      // The index of the sub model
              vector<NodeDesc>,         // Nodes which needs to be activated
              vector<TransferDesc>,     // Streams transfered from self to the external system
              vector<TransferDesc>      // Streams transfered from the external system to self
            > SubModelDesc;

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

  const std::shared_ptr<std::vector<NodeDesc>>& getNodes() const;

  const std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>>& getReceive() const;

  const std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>>& getSend() const;

  const std::shared_ptr<std::vector<std::shared_ptr<SubModel>>>& getSubModels() const;

private:
  const int index; /**< Index of the processing modcel */
  std::shared_ptr<std::vector<std::shared_ptr<SubModel>>> subModels; /**< Sub models that needs to be estableshed by other engines */
  std::shared_ptr<std::vector<NodeDesc>> nodes; /**< Nodes required by this model */
  std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>> send; /**< Streams that needs to be send to other engines */
  std::shared_ptr<std::vector<std::shared_ptr<StreamTransfer>>> receive; /**< Streams that needs to be received from other engines */
};

} /* namespace ice */

#endif /* PROCESSINGMODEL_H_ */
