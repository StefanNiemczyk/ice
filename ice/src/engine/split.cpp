#include "ice/representation/split.h"

std::unique_ptr<std::vector<std::string>> split(const char *str, const char c)
{
  std::unique_ptr<std::vector<std::string>> result(new std::vector<std::string>);

  do {
    const char *begin = str;

    while (*str != c && *str)
      str++;

    result->push_back(std::string(begin, str));
  } while (0 != *str++);

  return std::move(result);
}
