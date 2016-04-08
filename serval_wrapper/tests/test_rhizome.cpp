/*
 * test_rhizome.cpp
 *
 *  Created on: Apr 8, 2016
 *      Author: sni
 */

#include <string>
#include <limits.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "serval_interface.h"


TEST(JNITest, get_bundle_list)
{
  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto bundles = si.getBundleList();
  ASSERT_TRUE(bundles != nullptr);

  for (auto &bundle : * bundles)
  {
    std::cout << bundle.toString() << std::endl;
  }
}

TEST(JNITest, add_bundle)
{
  char buff[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", buff, PATH_MAX );
  std::string path( buff, (count > 0) ? count : 0 );

  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto manifest1 = si.addBundle("/home/sni/pjx/catkin_ws/src/ice/serval_wrapper/tests/data/test.txt");
  ASSERT_TRUE(manifest1 != nullptr);
  std::cout << manifest1->toString() << std::endl;

  auto manifest2 = si.getBundleManifest(manifest1->id);
  ASSERT_TRUE(manifest2 != nullptr);

  std::cout << manifest2->toString() << std::endl;

  ASSERT_EQ(manifest1->service, manifest2->service);
  ASSERT_EQ(manifest1->version, manifest2->version);
  ASSERT_EQ(manifest1->id, manifest2->id);
  ASSERT_EQ(manifest1->date, manifest2->date);
  ASSERT_EQ(manifest1->name, manifest2->name);
  ASSERT_EQ(manifest1->filesize, manifest2->filesize);
  ASSERT_EQ(manifest1->filehash, manifest2->filehash);

  auto raw = si.getBundlePayload(manifest1->id, false);
  ASSERT_NE(raw, "");

  auto decripted = si.getBundlePayload(manifest1->id, true);
  ASSERT_NE(decripted, "");

  ASSERT_EQ(raw, decripted);

  auto bundles = si.getBundleList();
  ASSERT_TRUE(bundles != nullptr);
  ASSERT_NE(bundles->size(), 0);

  auto bundles2 = si.getBundleList(bundles->at(0).token);
  ASSERT_NE(bundles2, nullptr);
  ASSERT_EQ(bundles2->size(), 0);
}

TEST(JNITest, append_bundle)
{
  char buff[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", buff, PATH_MAX );
  std::string path( buff, (count > 0) ? count : 0 );

  ice::serval_interface si("localhost", 4110, "peter", "venkman");

  auto manifest1 = si.appendBundle("/home/sni/pjx/catkin_ws/src/ice/serval_wrapper/tests/data/test.txt");
  ASSERT_TRUE(manifest1 != nullptr);
  std::cout << manifest1->toString() << std::endl;

  auto manifest2 = si.getBundleManifest(manifest1->id);
  ASSERT_TRUE(manifest2 != nullptr);

  std::cout << manifest2->toString() << std::endl;

  ASSERT_EQ(manifest1->service, manifest2->service);
  ASSERT_EQ(manifest1->version, manifest2->version);
  ASSERT_EQ(manifest1->id, manifest2->id);
  ASSERT_EQ(manifest1->date, manifest2->date);
  ASSERT_EQ(manifest1->name, manifest2->name);
  ASSERT_EQ(manifest1->filesize, manifest2->filesize);
  ASSERT_EQ(manifest1->filehash, manifest2->filehash);

  auto raw = si.getBundlePayload(manifest1->id, false);
  ASSERT_NE(raw, "");

  auto decripted = si.getBundlePayload(manifest1->id, true);
  ASSERT_NE(decripted, "");

  ASSERT_EQ(raw, decripted);
}
