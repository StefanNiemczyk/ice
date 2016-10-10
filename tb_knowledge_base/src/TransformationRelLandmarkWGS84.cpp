/*
 * TransformationRelLandmarkWGS84.cpp
 *
 *  Created on: 19.09.2016
 *      Author: sni
 */

#include <TransformationRelLandmarkWGS84.h>

#include <ice/ICEngine.h>

namespace ice
{

TransformationRelLandmarkWGS84::TransformationRelLandmarkWGS84(std::weak_ptr<ICEngine> engine)
    : Transformation(factory, "TestTransformation", "http://vs.uni-kassel.de/Ice#Position")
{
  auto f = factory.lock();
  this->factory = f;
  targetRepresentation = f->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos3D");
  inputs.push_back(f->getRepresentation("http://vs.uni-kassel.de/IceTest#Pos2D"));
}

TransformationRelLandmarkWGS84::~TransformationRelLandmarkWGS84()
{
  // TODO Auto-generated destructor stub
}

} /* namespace ice */
