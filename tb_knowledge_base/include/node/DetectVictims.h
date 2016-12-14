/*
 * DetectVictims.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_DETECTVICTIMS_H_
#define INCLUDE_NODE_DETECTVICTIMS_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
template <typename T>
class InformationSet;
class Pos3D;

class DetectVictims : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  DetectVictims();
  virtual ~DetectVictims();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string POS_REP;

private:
  std::shared_ptr<InformationSet<Pos3D>>   out;
};
} /* namespace ice */

#endif /* INCLUDE_NODE_DETECTVICTIMS_H_ */
