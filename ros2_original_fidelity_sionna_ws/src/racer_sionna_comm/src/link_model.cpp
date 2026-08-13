#include <racer_sionna_comm/link_model.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace racer_sionna_comm {

LinkModel::LinkModel(LinkModelConfig config) : config_(config) {
  if (config_.bandwidth_hz <= 0.0 || config_.mac_efficiency <= 0.0 ||
      config_.snr_slope_db <= 0.0 || config_.mtu_bytes == 0U) {
    throw std::invalid_argument("invalid link model configuration");
  }
}

double LinkModel::bitRate(double snr_db) const {
  double spectral_efficiency = 0.25;
  if (snr_db >= 4.0) spectral_efficiency = 0.5;
  if (snr_db >= 7.0) spectral_efficiency = 1.0;
  if (snr_db >= 10.0) spectral_efficiency = 2.0;
  if (snr_db >= 16.0) spectral_efficiency = 4.0;
  if (snr_db >= 23.0) spectral_efficiency = 6.0;
  return std::max(
      1.0e3, config_.bandwidth_hz * spectral_efficiency * config_.mac_efficiency);
}

double LinkModel::serializationDelay(double snr_db, std::size_t bytes) const {
  return 8.0 * static_cast<double>(bytes) / bitRate(snr_db);
}

double LinkModel::packetErrorRate(double snr_db, std::size_t bytes) const {
  const double exponent = std::clamp(
      (snr_db - config_.snr_midpoint_db) / config_.snr_slope_db, -60.0, 60.0);
  const double frame_per = 1.0 / (1.0 + std::exp(exponent));
  const std::size_t frames =
      std::max<std::size_t>(1U, (bytes + config_.mtu_bytes - 1U) / config_.mtu_bytes);
  return std::clamp(1.0 - std::pow(1.0 - frame_per, frames), 0.0, 1.0);
}

}  // namespace racer_sionna_comm
