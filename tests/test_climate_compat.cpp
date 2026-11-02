#include <cassert>
#include <cstring>
#include <initializer_list>
#include "components/toshiba_ab/climate_compat.h"

struct OldClimate {};
struct OldTraits {
  bool configured{false};
  void set_supported_custom_presets(std::initializer_list<const char *> presets) {
    assert(presets.size() == 1);
    assert(std::strcmp(*presets.begin(), "Night setback") == 0);
    configured = true;
  }
};
struct NewClimate : OldTraits {};
struct NewTraits {};  // Also compile when the old traits API is absent entirely.

struct FanTraits {
  unsigned count{0};
  void set_supported_custom_fan_modes(std::initializer_list<const char *> modes) {
    count = modes.size();
    if (count) {
      assert(count == 2);
      assert(std::strcmp(*modes.begin(), "Low-Medium") == 0);
      assert(std::strcmp(*(modes.begin() + 1), "Medium-High") == 0);
    }
  }
};
struct FanClimate : FanTraits {};

int main() {
  OldClimate legacy_fan;
  FanTraits fan_traits;
  esphome::toshiba_ab::detail::configure_fan_levels(legacy_fan, fan_traits, true, 0);
  assert(fan_traits.count == 2);
  esphome::toshiba_ab::detail::configure_fan_levels(legacy_fan, fan_traits, false, 0);
  assert(fan_traits.count == 0);
  FanClimate modern_fan;
  NewTraits modern_traits;
  esphome::toshiba_ab::detail::configure_fan_levels(modern_fan, modern_traits, true, 0);
  assert(modern_fan.count == 2);
  esphome::toshiba_ab::detail::configure_fan_levels(modern_fan, modern_traits, false, 0);
  assert(modern_fan.count == 0);

  OldClimate old_entity;
  OldTraits old_traits;
  esphome::toshiba_ab::detail::configure_night_setback(old_entity, old_traits, 0);
  assert(old_traits.configured);
  NewClimate new_entity;
  NewTraits new_traits;
  esphome::toshiba_ab::detail::configure_night_setback(new_entity, new_traits, 0);
  assert(new_entity.configured);
  OldTraits transitional_traits;
  new_entity.configured = false;
  esphome::toshiba_ab::detail::configure_night_setback(new_entity, transitional_traits, 0);
  assert(new_entity.configured && !transitional_traits.configured);
}
