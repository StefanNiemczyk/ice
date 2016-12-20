/*
 * AutoIRONode.h
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#ifndef AUTOIRONODE_H_
#define AUTOIRONODE_H_

#include "ice/processing/Node.h"

#include <memory>

namespace ice
{
class GContainer;
template<typename T>
class InformationStream;
template<typename T>
class InformationSet;
class Transformation;

class TransformationNode : public Node
{
public:
  TransformationNode(std::shared_ptr<Transformation> const &transformation);
  virtual ~TransformationNode();

  virtual int init();
  virtual int cleanUp();

  virtual int performNode();
  virtual const int newEvent(std::shared_ptr<InformationElement<GContainer>> element,
                             std::shared_ptr<InformationCollection> collection);
  virtual std::string getClassName();

private:
  std::vector<std::shared_ptr<InformationStream<GContainer>>>   inStreams;
  std::shared_ptr<InformationStream<GContainer>>                outStream;
  std::vector<std::shared_ptr<InformationSet<GContainer>>>      inSets;
  std::shared_ptr<InformationSet<GContainer>>                   outSet;
  std::vector<std::shared_ptr<GContainer>>                      inputElements;
  int                                                           inputSize;
  std::shared_ptr<Transformation>                               transformation;
  bool                                                          streamTransform;
};

} /* namespace ice */

#endif /* AUTOIRONODE_H_ */
