/*
 * TransformationRelLandmarkWGS84.h
 *
 *  Created on: 19.09.2016
 *      Author: sni
 */

#ifndef INCLUDE_TRANSFORMATIONRELLANDMARKWGS84_H_
#define INCLUDE_TRANSFORMATIONRELLANDMARKWGS84_H_

#include <memory>

#include <ice/representation/Transformation.h>

namespace ice
{
class ICEngine;

// see http://www.movable-type.co.uk/scripts/latlong.html for more information

class TransformationRelLandmarkWGS84 : public ice::Transformation
{
public:
  TransformationRelLandmarkWGS84(std::weak_ptr<ICEngine> engine);
  virtual ~TransformationRelLandmarkWGS84();
};

} /* namespace ice */

#endif /* INCLUDE_TRANSFORMATIONRELLANDMARKWGS84_H_ */
