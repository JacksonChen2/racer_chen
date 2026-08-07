#include "racer_3d_cpp/serialization.hpp"
#include "racer_3d_cpp/types.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <gtest/gtest.h>

namespace racer_3d_cpp {

TEST(CommunicationSerialization, SparseChunkRoundTripAndMerge) {
  SparseMapChunk input;
  input.originId = 2;
  input.sequence = 17U;
  input.indices = {0U, 3U, 7U};
  input.states = {FREE, OCCUPIED, FREE};

  SparseMapChunk decoded;
  std::string error;
  ASSERT_TRUE(decodeSparseMapChunk(encodeSparseMapChunk(input), decoded, &error))
      << error;
  EXPECT_EQ(decoded.originId, input.originId);
  EXPECT_EQ(decoded.sequence, input.sequence);
  EXPECT_EQ(decoded.indices, input.indices);
  EXPECT_EQ(decoded.states, input.states);

  VoxelMap map(1.0, Point3(0.0, 0.0, 0.0), Point3(2.0, 2.0, 2.0));
  map.mergeSparse(decoded.indices, decoded.states);
  EXPECT_EQ(map.states()[0], FREE);
  EXPECT_EQ(map.states()[3], OCCUPIED);
  EXPECT_EQ(map.states()[7], FREE);
  EXPECT_EQ(map.states()[1], UNKNOWN);
}

TEST(CommunicationSerialization, ManifestUsesCompactRanges) {
  MapManifest input;
  input.senderId = 1;
  input.ranges[0] = {{1U, 4U}, {7U, 9U}};
  input.ranges[2] = {{3U, 3U}};

  MapManifest decoded;
  std::string error;
  ASSERT_TRUE(decodeMapManifest(encodeMapManifest(input), decoded, &error))
      << error;
  EXPECT_EQ(decoded.senderId, 1);
  EXPECT_TRUE(manifestContains(decoded, 0, 1U));
  EXPECT_TRUE(manifestContains(decoded, 0, 8U));
  EXPECT_FALSE(manifestContains(decoded, 0, 5U));
  EXPECT_TRUE(manifestContains(decoded, 2, 3U));
  EXPECT_FALSE(manifestContains(decoded, 3, 1U));
}

TEST(CommunicationSerialization, RejectsOutOfOrderManifestRange) {
  MapManifest manifest;
  std::string error;
  EXPECT_FALSE(decodeMapManifest(
      R"({"sender":0,"origins":[{"origin":0,"ranges":[[5,2]]}]})",
      manifest, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace racer_3d_cpp
