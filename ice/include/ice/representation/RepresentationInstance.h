#ifndef REPRESENTATION_INSTANCE_H
#define REPRESENTATION_INSTANCE_H

#include <iostream>

namespace ice {


class RepresentationInstance {
public:

  template<typename T>
  RepresentationInstance(Representation *rep, T val)
  {
    representation = rep;

    T *p = new T;
    *p = val;
    data = (void*)p;
    
  }

  Representation *representation;
  void *data;
  
  template<typename T>
  T* get() {
    T* p = reinterpret_cast<T*>(data);
    return p;
  }
};

}

#endif // REPRESENTATION_INSTANCE_H
