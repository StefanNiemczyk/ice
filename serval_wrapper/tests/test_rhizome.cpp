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

#ifndef TEST_FILE_ROOT_PATH
#error TEST_FILE_ROOT_PATH is not defined
#endif

TEST(rhizome, get_bundle_list)
{
  ice::serval_interface si("/tmp/instance1", "localhost", 4110, "peter", "venkman");

  auto bundles = si.rhizome.getBundleList();
  ASSERT_TRUE(bundles != nullptr);
}

TEST(rhizome, add_bundle)
{
  ice::serval_interface si("/tmp/instance1", "localhost", 4110, "peter", "venkman");

  std::string path = std::string(TEST_FILE_ROOT_PATH) + std::string("/tests/data/test.txt");
  auto manifest1 = si.rhizome.addBundle(path);
  ASSERT_TRUE(manifest1 != nullptr);

  auto manifest2 = si.rhizome.getBundleManifest(manifest1->id);
  ASSERT_TRUE(manifest2 != nullptr);

  ASSERT_EQ(manifest1->service, manifest2->service);
  ASSERT_EQ(manifest1->version, manifest2->version);
  ASSERT_EQ(manifest1->id, manifest2->id);
  ASSERT_EQ(manifest1->date, manifest2->date);
  ASSERT_EQ(manifest1->name, manifest2->name);
  ASSERT_EQ(manifest1->filesize, manifest2->filesize);
  ASSERT_EQ(manifest1->filehash, manifest2->filehash);

  auto raw = si.rhizome.getBundlePayload(manifest1->id, false);
  ASSERT_NE(raw, "");

  auto decripted = si.rhizome.getBundlePayload(manifest1->id, true);
  ASSERT_NE(decripted, "");

  ASSERT_EQ(raw, decripted);

  auto bundles = si.rhizome.getBundleList();
  ASSERT_TRUE(bundles != nullptr);
  ASSERT_NE(bundles->size(), 0);

  auto bundles2 = si.rhizome.getBundleList(bundles->at(0).token);
  ASSERT_NE(bundles2, nullptr);
  ASSERT_EQ(bundles2->size(), 0);
}

TEST(JNITest, append_bundle)
{
  ice::serval_interface si("/tmp/instance1", "localhost", 4110, "peter", "venkman");

  std::string path = std::string(TEST_FILE_ROOT_PATH) + std::string("/tests/data/test.txt");
  auto manifest1 = si.rhizome.appendBundle("/home/sni/pjx/catkin_ws/src/ice/serval_wrapper/tests/data/test.txt"); // TODO
  ASSERT_TRUE(manifest1 != nullptr);

  auto manifest2 = si.rhizome.getBundleManifest(manifest1->id);
  ASSERT_TRUE(manifest2 != nullptr);

  ASSERT_EQ(manifest1->service, manifest2->service);
  ASSERT_EQ(manifest1->version, manifest2->version);
  ASSERT_EQ(manifest1->id, manifest2->id);
  ASSERT_EQ(manifest1->date, manifest2->date);
  ASSERT_EQ(manifest1->name, manifest2->name);
  ASSERT_EQ(manifest1->filesize, manifest2->filesize);
  ASSERT_EQ(manifest1->filehash, manifest2->filehash);

  auto raw = si.rhizome.getBundlePayload(manifest1->id, false);
  ASSERT_NE(raw, "");

  auto decripted = si.rhizome.getBundlePayload(manifest1->id, true);
  ASSERT_NE(decripted, "");

  ASSERT_EQ(raw, decripted);
}
