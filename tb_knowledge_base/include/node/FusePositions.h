/*
 * FuseVictims.h
 *
 *  Created on: Dec 9, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_FUSEPOSITIONS_H_
#define INCLUDE_NODE_FUSEPOSITIONS_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationSet;
template <typename T>
class InformationStream;

class FusePositions : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  FusePositions();
  virtual ~FusePositions();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string POS_REP;

private:
  std::vector<std::shared_ptr<InformationSet<GContainer>>>      inSets;
  std::vector<std::shared_ptr<InformationStream<GContainer>>>   ins;
  std::shared_ptr<InformationSet<GContainer>>                   out;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_FUSEPOSITIONS_H_ */
