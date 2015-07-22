/*
 * Representation.cpp
 *
 *  Created on: 15.07.2015
 *      Author: paspartout
 */

#include <ice/representation/Representation.h>
#include <stddef.h>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace ice {

Representation::Representation() {
  type = UnknownType;
  data = NULL;
}

Representation::~Representation() {
  if (data != NULL) {
    switch (type) {
    case StringRep:
      delete (char*) data;
      break;
    case BooleanRep:
      delete (bool*) data;
      break;
    default:
      break;
    }
  }
}

int Representation::fromCSV(std::string reprStr, const char delim) {
  const size_t length = reprStr.length() + 1;
  char *repCstr = new char[length];


  if (reprStr.empty()) {
    return -1;
  }
  if (!isprint(delim)) {
    return -1;
  }

  strncpy(repCstr, reprStr.c_str(), length);

  if (repCstr == NULL) {
    return -1;
  }

  const char *typeStr = strtok(repCstr, &delim);
  if (typeStr == NULL) {
    std::cerr << "Error while tokenizing representation string" << std::endl;
    return -1;
  }
  const char *dataStr = strtok(NULL, &delim);
  if (dataStr == NULL) {
    std::cerr << "Error while tokenizing representation string" << std::endl;
    return -1;
  }

  errno = 0;
  char *ep;
  const int typeNum = strtol(typeStr, &ep, 10);
  if (typeStr == ep || *ep != '\0') {
    std::cerr << "Error while converting " << typeStr << " to int." << std::endl;
    return -1;
  }

  if (typeNum < 0 || typeNum > UnknownType) {
    std::cerr << "Error: invalid representation type" << std::endl;
    return -1;
  }

  type = static_cast<RepresentationType>(typeNum);
  data = convertDataStr(dataStr, type);

  delete repCstr;
  return 0;
}

void *Representation::convertDataStr(const char *dataStr,
    RepresentationType type) {
  size_t len;
  void *res;

  switch (type) {

  case BooleanRep:
    res = new bool;
    if (strcmp(dataStr, "true") == 0) {
      *(bool*)res = true;
    } else if(strcmp(dataStr, "false") == 0) {
      *(bool*)res = false;
    } else {
      // error
    }
    break;

  case StringRep:
    len = strlen(dataStr) + 1;
    res = new char[len];
    strncpy((char*) res, dataStr, len);
    break;

  default:
    break;
  }

  return res;
}

}
