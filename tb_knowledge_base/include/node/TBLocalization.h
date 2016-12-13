/*
 * TBLocalization.h
 *
 *  Created on: Dec 8, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_TBLOCALIZATION_H_
#define INCLUDE_NODE_TBLOCALIZATION_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
template <typename T>
class InformationStream;
class PositionOrientation3D;

class TBLocalization : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  TBLocalization();
  virtual ~TBLocalization();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string POS_REP;

private:
  std::shared_ptr<InformationStream<PositionOrientation3D>>   out;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_TBLOCALIZATION_H_ */
