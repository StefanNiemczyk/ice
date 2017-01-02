/*
 * Pos3D2RelativeToLandmark.h
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_POSITIONORIENTATION3D2POS3D_H_
#define INCLUDE_NODE_POSITIONORIENTATION3D2POS3D_H_

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
class TBKnowledgeBase;

class PositionOrientation3D2Pos3D : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  PositionOrientation3D2Pos3D();
  virtual ~PositionOrientation3D2Pos3D();

  int init();
  int cleanUp();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  bool                                                          isSet;
  std::shared_ptr<InformationStream<GContainer>>                inStream;
  std::shared_ptr<InformationStream<GContainer>>                outStream;
  std::shared_ptr<InformationSet<GContainer>>                   inSet;
  std::shared_ptr<InformationSet<GContainer>>                   outSet;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_POS3D2RELATIVETOLANDMARK_H_ */
