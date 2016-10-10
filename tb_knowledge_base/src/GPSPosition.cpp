/*
 * GPSPosition.cpp
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#include <GPSPosition.h>

namespace ice
{

int GPSPosition::ALTITUDE_PATH = 0;
int GPSPosition::LATITUDE_PATH = 1;
int GPSPosition::LONGITUDE_PATH = 2;

GPSPosition::GPSPosition(std::shared_ptr<Representation> const &representation) :
    GContainer(representation), latitude(0), longitude(0), altitude(0)
{
  //
}

GPSPosition::~GPSPosition()
{
  //
}

GContainer* GPSPosition::clone()
{
  auto pos = new GPSPosition(this->representation);

  pos->latitude = this->latitude;
  pos->longitude = this->longitude;
  pos->altitude = this->altitude;

  return pos;
}

void GPSPosition::print(int level, std::string dimension)
{
  // TODO
}

std::pair<BasicRepresentationType, void*> GPSPosition::getPair(std::vector<int> *indices, int index)
{
  if (indices->at(index) == ALTITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->altitude);
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->latitude);
  }
  else if (indices->at(index) == LONGITUDE_PATH)
  {
    return std::make_pair(BasicRepresentationType::DOUBLE, &this->longitude);
  }

  return std::make_pair(BasicRepresentationType::UNSET, nullptr);
}

void* GPSPosition::get(std::vector<int> *indices, int index)
{
  if(indices->at(index) == ALTITUDE_PATH)
  {
    return &this->altitude;
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    return &this->latitude;
  }
  else if (indices->at(index) == LONGITUDE_PATH)
  {
    return &this->longitude;
  }

  return nullptr;
}

bool GPSPosition::set(std::vector<int> *indices, int index, const void* value)
{
  if(indices->at(index) == ALTITUDE_PATH)
  {
    this->altitude = *((double*) value);
    return true;
  }
  else if (indices->at(index) == LATITUDE_PATH)
  {
    this->latitude = *((double*) value);
    return true;
  }
  else if (indices->at(index) == LONGITUDE_PATH)
  {
    this->longitude = *((double*) value);
    return true;
  }

  return false;
}

std::string GPSPosition::toJSON()
{
  // TODO
  return "";
}

Value GPSPosition::toJSONValue(Document &d)
{
  // TODO
  Value value;
  return value;
}

} /* namespace ice */
