#pragma once

#include "racer_3d_cpp/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace racer_3d_cpp {

class VoxelMap;

// The shape is in NumPy C-order (z, y, x), exactly as published by the
// Python implementation's states.shape and states.tobytes().
struct MapShare {
  int droneId{-1};
  std::uint64_t sequence{};
  std::array<std::size_t, 3> shape{};
  double resolution{};
  std::vector<std::int8_t> states;
};

struct PairwiseAllocationMessage {
  int fromId{-1};
  int toId{-1};
  std::vector<std::string> cells;
  std::vector<std::string> senderCells;
  std::vector<std::string> unionCells;
  std::vector<std::string> route;
  double stamp{};
};

std::string encodeMapShare(const MapShare &share, int compression_level = 3);
bool decodeMapShare(const std::string &json, MapShare &share,
                    std::string *error = nullptr);

// Convenience wrappers used by the ROS agent.
std::string encodeMap(const VoxelMap &map, int drone_id,
                      std::uint64_t sequence);
bool decodeAndMergeMap(const std::string &json, VoxelMap &map,
                       int self_id = -1, std::string *error = nullptr);

std::string encodePeerState(const PeerState &state);
bool decodePeerState(const std::string &json, PeerState &state,
                     std::string *error = nullptr);

std::string encodePairwiseAllocation(
    const PairwiseAllocationMessage &message);
bool decodePairwiseAllocation(const std::string &json,
                              PairwiseAllocationMessage &message,
                              std::string *error = nullptr);

}  // namespace racer_3d_cpp
