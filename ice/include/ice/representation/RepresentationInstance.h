#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>

namespace ice {

class BaseRepresentationInstance {};

template<class T>
class RepresentationInstance : public BaseRepresentationInstance {
public:

  RepresentationInstance(Representation *rep, T val)
  {
    representation = rep;
    data = new T;
    *data = val;
  }

  Representation *representation;
  T *data;
  
};

}

#endif // REPRESENTATION_INSTANCE_H
