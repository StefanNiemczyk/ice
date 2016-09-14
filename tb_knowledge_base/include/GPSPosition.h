/*
 * GPSPosition.h
 *
 *  Created on: 14.09.2016
 *      Author: sni
 */

#ifndef INCLUDE_GPSPOSITION_H_
#define INCLUDE_GPSPOSITION_H_

#include <ice/representation/GContainer.h>

namespace ice
{

class GPSPosition : public GContainer
{
public:
  GPSPosition();
  virtual ~GPSPosition();

  virtual void print(int level, std::string dimension = "");
  virtual std::pair<BasicRepresentationType, void*> getPair(std::vector<int> *indices, int index);
  virtual void* get(std::vector<int> *indices, int index);
  virtual bool set(std::vector<int> *indices, int index, const void* value);
  virtual std::string toJSON();
  virtual Value toJSONValue(Document &d);

private:
  double latitude;
  double longtitude;
  double altitude;
};

} /* namespace ice */

#endif /* INCLUDE_GPSPOSITION_H_ */
