#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace racer_sionna_comm {

struct LinkModelConfig {
  double bandwidth_hz{100.0e6};
  double subcarrier_spacing_hz{120.0e3};
  int resource_blocks{66};
  double data_re_efficiency{0.82};
  double target_initial_tbler{0.10};
  double tbler_slope_db{1.35};
  std::size_t transport_block_bytes{1200};
};

struct McsEntry {
  std::string_view name;
  int modulation_order{};
  double code_rate{};
  double target_snr_db{};
};

class LinkModel {
 public:
  explicit LinkModel(LinkModelConfig config);

  const McsEntry &selectMcs(double snr_db) const;
  double transportBlockErrorRate(double snr_db) const;
  double bitRate(double snr_db) const;
  double serializationDelay(double snr_db, std::size_t bytes) const;
  double packetErrorRate(double snr_db, std::size_t bytes) const;
  double slotDuration() const;
  const LinkModelConfig &config() const noexcept { return config_; }

  static constexpr std::array<McsEntry, 4> kMcsTable{{
      {"QPSK", 2, 0.4902, 1.0},
      {"16QAM", 4, 0.4785, 7.0},
      {"64QAM", 6, 0.6504, 14.0},
      {"256QAM", 8, 0.7363, 21.0},
  }};

 private:
  LinkModelConfig config_;
};

}  // namespace racer_sionna_comm
