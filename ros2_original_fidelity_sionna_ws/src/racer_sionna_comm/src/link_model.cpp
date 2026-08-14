#include <racer_sionna_comm/link_model.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace racer_sionna_comm {

LinkModel::LinkModel(LinkModelConfig config) : config_(config) {
  const double occupied_bandwidth =
      12.0 * static_cast<double>(config_.resource_blocks) *
      config_.subcarrier_spacing_hz;
  if (config_.bandwidth_hz <= 0.0 ||
      config_.subcarrier_spacing_hz <= 0.0 || config_.resource_blocks <= 0 ||
      occupied_bandwidth > config_.bandwidth_hz ||
      config_.data_re_efficiency <= 0.0 ||
      config_.data_re_efficiency > 1.0 ||
      config_.target_initial_tbler <= 0.0 ||
      config_.target_initial_tbler >= 1.0 || config_.tbler_slope_db <= 0.0 ||
      config_.transport_block_bytes == 0U) {
    throw std::invalid_argument("invalid link model configuration");
  }
}

const McsEntry &LinkModel::selectMcs(double snr_db) const {
  const McsEntry *selected = &kMcsTable.front();
  for (const auto &candidate : kMcsTable) {
    if (snr_db + 1.0e-12 < candidate.target_snr_db) break;
    selected = &candidate;
  }
  return *selected;
}

double LinkModel::transportBlockErrorRate(double snr_db) const {
  const auto &mcs = selectMcs(snr_db);
  // Each MCS threshold is the calibrated SNR at which the initial LDPC
  // transport-block error rate is the configured target.  The logistic curve
  // represents the 5G NR LDPC waterfall without adding a fading process.
  const double odds_at_target =
      (1.0 - config_.target_initial_tbler) /
      config_.target_initial_tbler;
  const double exponent = std::clamp(
      (snr_db - mcs.target_snr_db) / config_.tbler_slope_db, -60.0, 60.0);
  return std::clamp(1.0 / (1.0 + odds_at_target * std::exp(exponent)),
                    0.0, 1.0);
}

double LinkModel::bitRate(double snr_db) const {
  const auto &mcs = selectMcs(snr_db);
  const double slots_per_second =
      1000.0 * config_.subcarrier_spacing_hz / 15.0e3;
  constexpr double kOfdmSymbolsPerNormalCpSlot = 14.0;
  const double data_resource_elements_per_second =
      12.0 * static_cast<double>(config_.resource_blocks) *
      kOfdmSymbolsPerNormalCpSlot * slots_per_second *
      config_.data_re_efficiency;
  return std::max(1.0e3,
                  data_resource_elements_per_second *
                      static_cast<double>(mcs.modulation_order) *
                      mcs.code_rate);
}

double LinkModel::slotDuration() const {
  return 1.0e-3 * 15.0e3 / config_.subcarrier_spacing_hz;
}

double LinkModel::serializationDelay(double snr_db, std::size_t bytes) const {
  const double unquantized =
      8.0 * static_cast<double>(bytes) / bitRate(snr_db);
  const double slot = slotDuration();
  return std::max(slot, std::ceil(unquantized / slot) * slot);
}

double LinkModel::packetErrorRate(double snr_db, std::size_t bytes) const {
  const double tbler = transportBlockErrorRate(snr_db);
  const std::size_t transport_blocks = std::max<std::size_t>(
      1U, (bytes + config_.transport_block_bytes - 1U) /
              config_.transport_block_bytes);
  return std::clamp(
      1.0 - std::pow(1.0 - tbler, static_cast<double>(transport_blocks)),
      0.0, 1.0);
}

}  // namespace racer_sionna_comm
