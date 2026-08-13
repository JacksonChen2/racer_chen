#pragma once

#include <cstddef>

namespace racer_sionna_comm {

struct LinkModelConfig {
  double bandwidth_hz{20.0e6};
  double mac_efficiency{0.55};
  double snr_midpoint_db{7.0};
  double snr_slope_db{1.8};
  std::size_t mtu_bytes{1200};
};

class LinkModel {
 public:
  explicit LinkModel(LinkModelConfig config);

  double bitRate(double snr_db) const;
  double serializationDelay(double snr_db, std::size_t bytes) const;
  double packetErrorRate(double snr_db, std::size_t bytes) const;

 private:
  LinkModelConfig config_;
};

}  // namespace racer_sionna_comm
