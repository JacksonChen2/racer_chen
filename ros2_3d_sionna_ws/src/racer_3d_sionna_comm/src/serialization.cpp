#include "racer_3d_cpp/serialization.hpp"

#include "racer_3d_cpp/voxel_map.hpp"

#include <jsoncpp/json/json.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace racer_3d_cpp {
namespace {

void setError(std::string *error, const std::string &message) {
  if (error != nullptr) {
    *error = message;
  }
}

std::string writeJson(const Json::Value &root) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, root);
}

bool parseJson(const std::string &text, Json::Value &root,
               std::string *error) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::string errors;
  std::istringstream input(text);
  if (!Json::parseFromStream(builder, input, &root, &errors)) {
    setError(error, errors.empty() ? "invalid JSON" : errors);
    return false;
  }
  if (!root.isObject()) {
    setError(error, "JSON root must be an object");
    return false;
  }
  return true;
}

std::string base64Encode(const std::vector<unsigned char> &input) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((input.size() + 2U) / 3U) * 4U);
  for (std::size_t offset = 0; offset < input.size(); offset += 3U) {
    const std::uint32_t first = input[offset];
    const bool have_second = offset + 1U < input.size();
    const bool have_third = offset + 2U < input.size();
    const std::uint32_t second = have_second ? input[offset + 1U] : 0U;
    const std::uint32_t third = have_third ? input[offset + 2U] : 0U;
    const std::uint32_t value = (first << 16U) | (second << 8U) | third;
    output.push_back(kAlphabet[(value >> 18U) & 0x3fU]);
    output.push_back(kAlphabet[(value >> 12U) & 0x3fU]);
    output.push_back(have_second ? kAlphabet[(value >> 6U) & 0x3fU] : '=');
    output.push_back(have_third ? kAlphabet[value & 0x3fU] : '=');
  }
  return output;
}

int base64Digit(char value) {
  if (value >= 'A' && value <= 'Z') {
    return value - 'A';
  }
  if (value >= 'a' && value <= 'z') {
    return value - 'a' + 26;
  }
  if (value >= '0' && value <= '9') {
    return value - '0' + 52;
  }
  if (value == '+') {
    return 62;
  }
  if (value == '/') {
    return 63;
  }
  return -1;
}

bool base64Decode(const std::string &encoded,
                  std::vector<unsigned char> &output,
                  std::string *error) {
  std::string input;
  input.reserve(encoded.size());
  for (const unsigned char value : encoded) {
    if (!std::isspace(value)) {
      input.push_back(static_cast<char>(value));
    }
  }
  if (input.empty()) {
    output.clear();
    return true;
  }
  if (input.size() % 4U != 0U) {
    setError(error, "base64 length is not divisible by four");
    return false;
  }

  output.clear();
  output.reserve((input.size() / 4U) * 3U);
  for (std::size_t offset = 0; offset < input.size(); offset += 4U) {
    const bool final_group = offset + 4U == input.size();
    const int first = base64Digit(input[offset]);
    const int second = base64Digit(input[offset + 1U]);
    const bool third_padding = input[offset + 2U] == '=';
    const bool fourth_padding = input[offset + 3U] == '=';
    const int third = third_padding ? 0 : base64Digit(input[offset + 2U]);
    const int fourth =
        fourth_padding ? 0 : base64Digit(input[offset + 3U]);
    if (first < 0 || second < 0 || third < 0 || fourth < 0 ||
        (third_padding && !fourth_padding) ||
        (!final_group && (third_padding || fourth_padding))) {
      setError(error, "invalid base64 data");
      return false;
    }
    const std::uint32_t value =
        (static_cast<std::uint32_t>(first) << 18U) |
        (static_cast<std::uint32_t>(second) << 12U) |
        (static_cast<std::uint32_t>(third) << 6U) |
        static_cast<std::uint32_t>(fourth);
    output.push_back(static_cast<unsigned char>((value >> 16U) & 0xffU));
    if (!third_padding) {
      output.push_back(
          static_cast<unsigned char>((value >> 8U) & 0xffU));
    }
    if (!fourth_padding) {
      output.push_back(static_cast<unsigned char>(value & 0xffU));
    }
  }
  return true;
}

std::size_t checkedShapeSize(const std::array<std::size_t, 3> &shape) {
  std::size_t result = 1U;
  for (const std::size_t dimension : shape) {
    if (dimension != 0U &&
        result > std::numeric_limits<std::size_t>::max() / dimension) {
      throw std::overflow_error("map shape product overflows size_t");
    }
    result *= dimension;
  }
  return result;
}

Json::Value stringArray(const std::vector<std::string> &values) {
  Json::Value result(Json::arrayValue);
  for (const auto &value : values) {
    result.append(value);
  }
  return result;
}

bool readStringArray(const Json::Value &root, const char *name,
                     std::vector<std::string> &values,
                     std::string *error, bool required = false) {
  if (!root.isMember(name)) {
    if (required) {
      setError(error, std::string("missing field: ") + name);
      return false;
    }
    values.clear();
    return true;
  }
  const Json::Value &array = root[name];
  if (!array.isArray()) {
    setError(error, std::string(name) + " must be an array");
    return false;
  }
  values.clear();
  values.reserve(array.size());
  for (const auto &value : array) {
    if (!value.isString()) {
      setError(error, std::string(name) + " must contain strings");
      return false;
    }
    values.push_back(value.asString());
  }
  return true;
}

Json::Value pointToJson(const Point3 &point) {
  Json::Value result(Json::arrayValue);
  result.append(point.x());
  result.append(point.y());
  result.append(point.z());
  return result;
}

bool pointFromJson(const Json::Value &value, Point3 &point,
                   const char *name, std::string *error) {
  if (!value.isArray() || value.size() < 3U || !value[0].isNumeric() ||
      !value[1].isNumeric() || !value[2].isNumeric()) {
    setError(error, std::string(name) + " must be a numeric xyz array");
    return false;
  }
  point = Point3(value[0].asDouble(), value[1].asDouble(),
                 value[2].asDouble());
  if (!point.array().isFinite().all()) {
    setError(error, std::string(name) + " contains non-finite values");
    return false;
  }
  return true;
}

}  // namespace

std::string encodeMapShare(const MapShare &share, int compression_level) {
  const std::size_t expected = checkedShapeSize(share.shape);
  if (share.states.size() != expected) {
    throw std::invalid_argument("map state size does not match shape");
  }
  if (share.states.size() >
      static_cast<std::size_t>(std::numeric_limits<uLong>::max())) {
    throw std::overflow_error("map is too large for zlib");
  }
  compression_level =
      std::max(Z_NO_COMPRESSION, std::min(Z_BEST_COMPRESSION,
                                          compression_level));
  const uLong source_size = static_cast<uLong>(share.states.size());
  uLongf compressed_size = compressBound(source_size);
  std::vector<unsigned char> compressed(
      static_cast<std::size_t>(compressed_size));
  static const unsigned char kEmpty = 0U;
  const auto *source =
      share.states.empty()
          ? &kEmpty
          : reinterpret_cast<const unsigned char *>(share.states.data());
  const int result =
      compress2(compressed.data(), &compressed_size, source, source_size,
                compression_level);
  if (result != Z_OK) {
    throw std::runtime_error("zlib compression failed: " +
                             std::to_string(result));
  }
  compressed.resize(static_cast<std::size_t>(compressed_size));

  Json::Value root(Json::objectValue);
  root["drone_id"] = share.droneId;
  root["sequence"] = Json::UInt64(share.sequence);
  Json::Value shape(Json::arrayValue);
  for (const std::size_t dimension : share.shape) {
    shape.append(Json::UInt64(dimension));
  }
  root["shape"] = std::move(shape);
  root["resolution"] = share.resolution;
  root["data"] = base64Encode(compressed);
  return writeJson(root);
}

bool decodeMapShare(const std::string &json, MapShare &share,
                    std::string *error) {
  Json::Value root;
  if (!parseJson(json, root, error)) {
    return false;
  }
  try {
    if (!root.isMember("drone_id") || !root["drone_id"].isIntegral() ||
        !root.isMember("sequence") || !root["sequence"].isIntegral() ||
        !root.isMember("shape") || !root["shape"].isArray() ||
        root["shape"].size() != 3U || !root.isMember("resolution") ||
        !root["resolution"].isNumeric() || !root.isMember("data") ||
        !root["data"].isString()) {
      setError(error, "map share fields have invalid types");
      return false;
    }

    MapShare decoded;
    decoded.droneId = root["drone_id"].asInt();
    decoded.sequence = root["sequence"].asUInt64();
    decoded.resolution = root["resolution"].asDouble();
    for (Json::ArrayIndex axis = 0; axis < 3U; ++axis) {
      const Json::Value &dimension = root["shape"][axis];
      if (!dimension.isIntegral() || dimension.asInt64() < 0) {
        setError(error, "map shape contains an invalid dimension");
        return false;
      }
      decoded.shape[axis] =
          static_cast<std::size_t>(dimension.asUInt64());
    }
    const std::size_t expected = checkedShapeSize(decoded.shape);
    if (expected >
        static_cast<std::size_t>(std::numeric_limits<uLongf>::max())) {
      setError(error, "map is too large for zlib");
      return false;
    }

    std::vector<unsigned char> compressed;
    if (!base64Decode(root["data"].asString(), compressed, error)) {
      return false;
    }
    if (compressed.size() >
        static_cast<std::size_t>(std::numeric_limits<uLong>::max())) {
      setError(error, "compressed map is too large for zlib");
      return false;
    }
    std::vector<unsigned char> raw(std::max<std::size_t>(1U, expected));
    uLongf raw_size = static_cast<uLongf>(raw.size());
    static const unsigned char kEmpty = 0U;
    const auto *source = compressed.empty() ? &kEmpty : compressed.data();
    const int result =
        uncompress(raw.data(), &raw_size, source,
                   static_cast<uLong>(compressed.size()));
    if (result != Z_OK) {
      setError(error,
               "zlib decompression failed: " + std::to_string(result));
      return false;
    }
    if (raw_size != expected) {
      setError(error, "decompressed map size does not match shape");
      return false;
    }
    decoded.states.resize(expected);
    std::transform(raw.begin(), raw.begin() +
                                    static_cast<std::ptrdiff_t>(expected),
                   decoded.states.begin(),
                   [](unsigned char value) {
                     return static_cast<std::int8_t>(value);
                   });
    share = std::move(decoded);
    return true;
  } catch (const Json::Exception &exception) {
    setError(error, exception.what());
    return false;
  } catch (const std::exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

std::string encodeMap(const VoxelMap &map, int drone_id,
                      std::uint64_t sequence) {
  MapShare share;
  share.droneId = drone_id;
  share.sequence = sequence;
  share.shape = {
      static_cast<std::size_t>(map.nz()),
      static_cast<std::size_t>(map.ny()),
      static_cast<std::size_t>(map.nx()),
  };
  share.resolution = map.resolution();
  share.states = map.states();
  return encodeMapShare(share, 3);
}

bool decodeAndMergeMap(const std::string &json, VoxelMap &map, int self_id,
                       std::string *error) {
  MapShare share;
  if (!decodeMapShare(json, share, error)) {
    return false;
  }
  if (self_id >= 0 && share.droneId == self_id) {
    setError(error, "ignored local map share");
    return false;
  }
  const std::array<std::size_t, 3> expected_shape = {
      static_cast<std::size_t>(map.nz()),
      static_cast<std::size_t>(map.ny()),
      static_cast<std::size_t>(map.nx()),
  };
  if (share.shape != expected_shape) {
    setError(error, "received map shape does not match local map");
    return false;
  }
  map.merge(share.states);
  return true;
}

std::string encodePeerState(const PeerState &state) {
  Json::Value root(Json::objectValue);
  root["drone_id"] = state.drone_id;
  root["stamp"] = state.stamp;
  root["position"] = pointToJson(state.position);
  root["velocity"] = pointToJson(state.velocity);
  root["owned_cells"] = stringArray(state.owned_cells);
  Json::Value trajectory(Json::arrayValue);
  for (const auto &sample : state.trajectory) {
    Json::Value item(Json::arrayValue);
    item.append(sample.time);
    item.append(sample.position.x());
    item.append(sample.position.y());
    item.append(sample.position.z());
    trajectory.append(std::move(item));
  }
  root["trajectory"] = std::move(trajectory);
  return writeJson(root);
}

bool decodePeerState(const std::string &json, PeerState &state,
                     std::string *error) {
  Json::Value root;
  if (!parseJson(json, root, error)) {
    return false;
  }
  try {
    if (!root.isMember("drone_id") || !root["drone_id"].isIntegral()) {
      setError(error, "peer state is missing drone_id");
      return false;
    }
    PeerState decoded;
    decoded.drone_id = root["drone_id"].asInt();
    if (root.isMember("stamp")) {
      if (!root["stamp"].isNumeric()) {
        setError(error, "stamp must be numeric");
        return false;
      }
      decoded.stamp = root["stamp"].asDouble();
    }
    if (root.isMember("received")) {
      if (!root["received"].isNumeric()) {
        setError(error, "received must be numeric");
        return false;
      }
      decoded.received = root["received"].asDouble();
    }
    if (root.isMember("position") &&
        !pointFromJson(root["position"], decoded.position, "position",
                       error)) {
      return false;
    }
    if (root.isMember("velocity") &&
        !pointFromJson(root["velocity"], decoded.velocity, "velocity",
                       error)) {
      return false;
    }
    if (!readStringArray(root, "owned_cells", decoded.owned_cells, error)) {
      return false;
    }
    if (root.isMember("trajectory")) {
      const Json::Value &trajectory = root["trajectory"];
      if (!trajectory.isArray()) {
        setError(error, "trajectory must be an array");
        return false;
      }
      decoded.trajectory.reserve(trajectory.size());
      for (const auto &item : trajectory) {
        if (!item.isArray() || item.size() < 4U ||
            !item[0].isNumeric() || !item[1].isNumeric() ||
            !item[2].isNumeric() || !item[3].isNumeric()) {
          setError(error, "trajectory samples must be [time,x,y,z]");
          return false;
        }
        TrajectorySample sample;
        sample.time = item[0].asDouble();
        sample.position =
            Point3(item[1].asDouble(), item[2].asDouble(),
                   item[3].asDouble());
        if (!std::isfinite(sample.time) ||
            !sample.position.array().isFinite().all()) {
          setError(error, "trajectory contains non-finite values");
          return false;
        }
        decoded.trajectory.push_back(sample);
      }
    }
    state = std::move(decoded);
    return true;
  } catch (const Json::Exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

std::string encodePairwiseAllocation(
    const PairwiseAllocationMessage &message) {
  Json::Value root(Json::objectValue);
  root["from"] = message.fromId;
  root["to"] = message.toId;
  root["cells"] = stringArray(message.cells);
  root["sender_cells"] = stringArray(message.senderCells);
  root["union"] = stringArray(message.unionCells);
  root["route"] = stringArray(message.route);
  root["stamp"] = message.stamp;
  return writeJson(root);
}

bool decodePairwiseAllocation(const std::string &json,
                              PairwiseAllocationMessage &message,
                              std::string *error) {
  Json::Value root;
  if (!parseJson(json, root, error)) {
    return false;
  }
  try {
    PairwiseAllocationMessage decoded;
    if (root.isMember("from")) {
      if (!root["from"].isIntegral()) {
        setError(error, "from must be integral");
        return false;
      }
      decoded.fromId = root["from"].asInt();
    }
    if (root.isMember("to")) {
      if (!root["to"].isIntegral()) {
        setError(error, "to must be integral");
        return false;
      }
      decoded.toId = root["to"].asInt();
    }
    if (!readStringArray(root, "cells", decoded.cells, error) ||
        !readStringArray(root, "sender_cells", decoded.senderCells, error) ||
        !readStringArray(root, "union", decoded.unionCells, error) ||
        !readStringArray(root, "route", decoded.route, error)) {
      return false;
    }
    if (!root.isMember("union")) {
      decoded.unionCells = decoded.cells;
      decoded.unionCells.insert(decoded.unionCells.end(),
                                decoded.senderCells.begin(),
                                decoded.senderCells.end());
      std::sort(decoded.unionCells.begin(), decoded.unionCells.end());
      decoded.unionCells.erase(
          std::unique(decoded.unionCells.begin(), decoded.unionCells.end()),
          decoded.unionCells.end());
    }
    if (root.isMember("stamp")) {
      if (!root["stamp"].isNumeric()) {
        setError(error, "stamp must be numeric");
        return false;
      }
      decoded.stamp = root["stamp"].asDouble();
    }
    message = std::move(decoded);
    return true;
  } catch (const Json::Exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

std::string encodeSparseMapChunk(const SparseMapChunk &chunk) {
  if (chunk.originId < 0 || chunk.indices.size() != chunk.states.size()) {
    throw std::invalid_argument("invalid sparse map chunk");
  }
  Json::Value root(Json::objectValue);
  root["origin"] = chunk.originId;
  root["sequence"] = Json::UInt64(chunk.sequence);
  Json::Value indices(Json::arrayValue);
  Json::Value states(Json::arrayValue);
  for (std::size_t offset = 0; offset < chunk.indices.size(); ++offset) {
    indices.append(Json::UInt(chunk.indices[offset]));
    states.append(static_cast<int>(chunk.states[offset]));
  }
  root["indices"] = std::move(indices);
  root["states"] = std::move(states);
  return writeJson(root);
}

bool decodeSparseMapChunk(const std::string &json, SparseMapChunk &chunk,
                          std::string *error) {
  Json::Value root;
  if (!parseJson(json, root, error)) return false;
  try {
    if (!root.isMember("origin") || !root["origin"].isIntegral() ||
        !root.isMember("sequence") || !root["sequence"].isIntegral() ||
        !root.isMember("indices") || !root["indices"].isArray() ||
        !root.isMember("states") || !root["states"].isArray() ||
        root["indices"].size() != root["states"].size()) {
      setError(error, "sparse map chunk fields have invalid types");
      return false;
    }
    SparseMapChunk decoded;
    decoded.originId = root["origin"].asInt();
    decoded.sequence = root["sequence"].asUInt64();
    if (decoded.originId < 0) {
      setError(error, "sparse map chunk origin must be non-negative");
      return false;
    }
    decoded.indices.reserve(root["indices"].size());
    decoded.states.reserve(root["states"].size());
    for (Json::ArrayIndex offset = 0; offset < root["indices"].size(); ++offset) {
      const auto &index = root["indices"][offset];
      const auto &state = root["states"][offset];
      if (!index.isIntegral() || index.asInt64() < 0 ||
          index.asUInt64() > std::numeric_limits<std::uint32_t>::max() ||
          !state.isIntegral() || state.asInt() < -1 || state.asInt() > 100) {
        setError(error, "sparse map chunk contains an invalid voxel");
        return false;
      }
      decoded.indices.push_back(index.asUInt());
      decoded.states.push_back(static_cast<std::int8_t>(state.asInt()));
    }
    chunk = std::move(decoded);
    return true;
  } catch (const Json::Exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

std::string encodeMapManifest(const MapManifest &manifest) {
  Json::Value root(Json::objectValue);
  root["sender"] = manifest.senderId;
  Json::Value origins(Json::arrayValue);
  std::vector<int> ids;
  ids.reserve(manifest.ranges.size());
  for (const auto &[origin, ranges] : manifest.ranges) {
    (void)ranges;
    ids.push_back(origin);
  }
  std::sort(ids.begin(), ids.end());
  for (const int origin : ids) {
    Json::Value entry(Json::objectValue);
    entry["origin"] = origin;
    Json::Value ranges(Json::arrayValue);
    for (const auto &[first, last] : manifest.ranges.at(origin)) {
      Json::Value range(Json::arrayValue);
      range.append(Json::UInt64(first));
      range.append(Json::UInt64(last));
      ranges.append(std::move(range));
    }
    entry["ranges"] = std::move(ranges);
    origins.append(std::move(entry));
  }
  root["origins"] = std::move(origins);
  return writeJson(root);
}

bool decodeMapManifest(const std::string &json, MapManifest &manifest,
                       std::string *error) {
  Json::Value root;
  if (!parseJson(json, root, error)) return false;
  try {
    if (!root.isMember("sender") || !root["sender"].isIntegral() ||
        !root.isMember("origins") || !root["origins"].isArray()) {
      setError(error, "map manifest fields have invalid types");
      return false;
    }
    MapManifest decoded;
    decoded.senderId = root["sender"].asInt();
    for (const auto &entry : root["origins"]) {
      if (!entry.isObject() || !entry.isMember("origin") ||
          !entry["origin"].isIntegral() || !entry.isMember("ranges") ||
          !entry["ranges"].isArray()) {
        setError(error, "map manifest origin entry is invalid");
        return false;
      }
      const int origin = entry["origin"].asInt();
      auto &ranges = decoded.ranges[origin];
      for (const auto &range : entry["ranges"]) {
        if (!range.isArray() || range.size() != 2U ||
            !range[0].isIntegral() || !range[1].isIntegral()) {
          setError(error, "map manifest range is invalid");
          return false;
        }
        const auto first = range[0].asUInt64();
        const auto last = range[1].asUInt64();
        if (first == 0U || last < first) {
          setError(error, "map manifest range is not ordered");
          return false;
        }
        ranges.emplace_back(first, last);
      }
      std::sort(ranges.begin(), ranges.end());
    }
    manifest = std::move(decoded);
    return true;
  } catch (const Json::Exception &exception) {
    setError(error, exception.what());
    return false;
  }
}

bool manifestContains(const MapManifest &manifest, int origin_id,
                      std::uint64_t sequence) {
  const auto found = manifest.ranges.find(origin_id);
  if (found == manifest.ranges.end()) return false;
  for (const auto &[first, last] : found->second) {
    if (sequence < first) return false;
    if (sequence <= last) return true;
  }
  return false;
}

}  // namespace racer_3d_cpp
