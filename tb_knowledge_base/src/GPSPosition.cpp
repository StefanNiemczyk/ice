/*
 * GPSPosition.cpp
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#include <GPSPosition.h>

namespace ice
{

int Pos3D::Z_COORDINATE_PATH = 0;
int Pos3D::X_COORDINATE_PATH = 1;
int Pos3D::Y_COORDINATE_PATH = 2;

Pos3D::Pos3D(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), x(0), y(0), z(0)
{
  //
}

Pos3D::~Pos3D()
{
  //
}

GContainer* Pos3D::clone()
{
  auto pos = new Pos3D(this->representation);

  pos->x = this->x;
  pos->y = this->y;
  pos->z = this->z;

  return pos;
}

void Pos3D::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> Pos3D::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == Z_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->z);
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->x);
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->y);
  }

  return std::make_pair(BasicRepresentationType::UNSET, nullptr);
}

void* Pos3D::get(std::vector<int> *indices, int index)
{
  if(indices->at(index) == Z_COORDINATE_PATH)
  {
    return &this->z;
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    return &this->x;
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    return &this->y;
  }

  return nullptr;
}

bool Pos3D::set(std::vector<int> *indices, int index, const void* value)
{
  if(indices->at(index) == Z_COORDINATE_PATH)
  {
    this->z = *((double*) value);
    return true;
  }
  else if (indices->at(index) == X_COORDINATE_PATH)
  {
    this->x = *((double*) value);
    return true;
  }
  else if (indices->at(index) == Y_COORDINATE_PATH)
  {
    this->y = *((double*) value);
    return true;
  }

  return false;
}

std::string Pos3D::toJSON()
{
  // TODO
  // {"http://vs.uni-kassel.de/IceTest#Pos3D":{"http://vs.uni-kassel.de/Ice#XCoordinate":{"http://vs.uni-kassel.de/Ice#DoubleRep":5.0},"http://vs.uni-kassel.de/Ice#YCoordinate":{"http://vs.uni-kassel.de/Ice#DoubleRep":6.0},"http://vs.uni-kassel.de/Ice#ZCoordinate":{"http://vs.uni-kassel.de/Ice#DoubleRep":7.0}}}

  return "";
}

Value Pos3D::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

} /* namespace ice */
