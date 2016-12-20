/*
 * Pos3D2RelativeToLandmark.h
 *
 *  Created on: Dec 12, 2016
 *      Author: sni
 */

#ifndef INCLUDE_NODE_POS3D2RELATIVETOLANDMARK_H_
#define INCLUDE_NODE_POS3D2RELATIVETOLANDMARK_H_

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

class Pos3D2RelativeToLandmark : public Node
{
public:
  static std::shared_ptr<Node> createNode();

public:
  Pos3D2RelativeToLandmark();
  virtual ~Pos3D2RelativeToLandmark();

  int init();

  virtual std::string getClassName();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual int performNode();

private:
  static std::string REP_IN;
  static std::string REP_OUT;

private:
  std::shared_ptr<TBKnowledgeBase>                              tbKnowledgeBase;
  bool                                                          isSet;
  std::shared_ptr<InformationStream<GContainer>>                inStream;
  std::shared_ptr<InformationStream<GContainer>>                outStream;
  std::shared_ptr<InformationSet<GContainer>>                   inSet;
  std::shared_ptr<InformationSet<GContainer>>                   outSet;
  std::shared_ptr<InformationSet<PositionOrientation3D>>        positionLandmarks;
};

} /* namespace ice */

#endif /* INCLUDE_NODE_POS3D2RELATIVETOLANDMARK_H_ */
