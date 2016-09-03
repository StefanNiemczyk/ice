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
class Transformation;

class TransformationNode : public Node
{
public:
  TransformationNode(std::shared_ptr<Transformation> const &transformation);
  virtual ~TransformationNode();

  virtual int init();
  virtual int cleanUp();

  virtual int performNode();
  virtual std::string getClassName();

private:
  std::vector<std::shared_ptr<InformationStream<GContainer>>>   inputStreams;
  std::shared_ptr<InformationStream<GContainer>>                outputStream;
  std::vector<std::shared_ptr<GContainer>>                      inputElements;
  int                                                           inputSize;
  std::shared_ptr<Transformation>                               transformation;
};

} /* namespace ice */

#endif /* AUTOIRONODE_H_ */
