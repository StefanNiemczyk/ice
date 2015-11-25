#ifndef SPLIT_H
#define SPLIT_H

#include <memory>
#include <string>
#include <vector>

std::unique_ptr<std::vector<std::string>> split(const char *str, const char c);

#endif // SPLIT_H
