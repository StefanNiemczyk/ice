/*
 * RelativeToLandmark2Pos3D.h
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_RELATIVETOLANDMARK2POS3D_H_
#define INCLUDE_NODE_RELATIVETOLANDMARK2POS3D_H_

#include <memory>

#include <ice/processing/Node.h>

namespace ice
{
class GContainer;
template <typename T>
class InformationSet;
template <typename T>
class InformationStream;
class PositionOrientation3D;

class RelativeToLandmark2Pos3D : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  RelativeToLandmark2Pos3D();
  virtual ~RelativeToLandmark2Pos3D();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string REP_IN;
  static std::string REP_OUT;

private:
  bool                                                          isSet;
  std::shared_ptr<InformationStream<GContainer>>                inStream;
  std::shared_ptr<InformationStream<GContainer>>                outStream;
  std::shared_ptr<InformationSet<GContainer>>                   inSet;
  std::shared_ptr<InformationSet<GContainer>>                   outSet;
  std::shared_ptr<InformationSet<PositionOrientation3D>>        positionLandmarks;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_RELATIVETOLANDMARK2POS3D_H_ */
