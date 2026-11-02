#pragma once

namespace esphome {
namespace toshiba_ab {
namespace detail {

// ESPHome moved custom preset registration from ClimateTraits to Climate.
// Detect the available API instead of assuming a particular release version.
template<typename Climate, typename Traits>
auto configure_night_setback(Climate &entity, Traits &, int)
    -> decltype(entity.set_supported_custom_presets({"Night setback"}), void()) {
  entity.set_supported_custom_presets({"Night setback"});
}

template<typename Climate, typename Traits>
void configure_night_setback(Climate &, Traits &traits, long) {
  traits.set_supported_custom_presets({"Night setback"});
}

// Custom fan registration moved alongside custom presets in ESPHome.
template<typename Climate, typename Traits>
auto configure_fan_levels(Climate &entity, Traits &, bool five, int)
    -> decltype(entity.set_supported_custom_fan_modes({"Low-Medium", "Medium-High"}), void()) {
  if (five) entity.set_supported_custom_fan_modes({"Low-Medium", "Medium-High"});
  else entity.set_supported_custom_fan_modes({});
}

template<typename Climate, typename Traits>
void configure_fan_levels(Climate &, Traits &traits, bool five, long) {
  if (five) traits.set_supported_custom_fan_modes({"Low-Medium", "Medium-High"});
  else traits.set_supported_custom_fan_modes({});
}

}  // namespace detail
}  // namespace toshiba_ab
}  // namespace esphome
