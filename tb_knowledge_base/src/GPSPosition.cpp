/*
 * GPSPosition.cpp
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#include <GPSPosition.h>

namespace ice
{

GPSPosition::GPSPosition() : latitude(0), longtitude(0), altitude(0)
{
  //
}

GPSPosition::~GPSPosition()
{
  //
}

void GPSPosition::print(int level, std::string dimension)
{
  // TODO Auto-generated destructor stub
}

std::pair<BasicRepresentationType, void*> GPSPosition::getPair(std::vector<int> *indices, int index)
{
  // TODO Auto-generated destructor stub
}

void* GPSPosition::get(std::vector<int> *indices, int index)
{
  // TODO Auto-generated destructor stub
}

bool GPSPosition::set(std::vector<int> *indices, int index, const void* value)
{
  // TODO Auto-generated destructor stub
}

std::string GPSPosition::toJSON()
{
  // TODO Auto-generated destructor stub
}

Value GPSPosition::toJSONValue(Document &d)
{
  // TODO Auto-generated destructor stub
}

} /* namespace ice */
