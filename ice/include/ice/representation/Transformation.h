/*
 * Transformation.h
 *
 *  Created on: Nov 20, 2015
 *      Author: sni
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

//Forward declaration
namespace ice
{
class Representation;
} /* namespace ice */

#include <vector>

namespace ice
{

enum TransformationOperation {
  DEFAULT,
  USE
};

struct Operation {
  int* field;
  TransformationOperation operation;
  void* value;
  int* target;
  int idInput;
};

class Transformation
{
public:
  Transformation(std::shared_ptr<ice::RepresentationFactory> factory, std::string targetRepresentation, int inputCount);
  virtual ~Transformation();

  RepresentationInstance* transform(RepresentationInstance** inputs);

  std::vector<Operation>& getOperations();

private:
  Representation* targetRepresentation;
  int inputCount;
  std::vector<Operation> operations;
  std::shared_ptr<ice::RepresentationFactory> factory;
};

} /* namespace ice */

#endif /* TRANSFORMATION_H_ */
