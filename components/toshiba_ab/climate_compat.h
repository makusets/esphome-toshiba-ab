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

}  // namespace detail
}  // namespace toshiba_ab
}  // namespace esphome
