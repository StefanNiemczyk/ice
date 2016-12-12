/*
 * VictimDetection.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_VICTIMDETECTION_H_
#define INCLUDE_NODE_VICTIMDETECTION_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationSet;

class VictimDetection : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  VictimDetection();
  virtual ~VictimDetection();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string POS_REP;

private:
  std::shared_ptr<InformationSet<GContainer>>   out;
};
} /* namespace ice */

#endif /* INCLUDE_NODE_VICTIMDETECTION_H_ */
