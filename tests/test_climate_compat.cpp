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

int main() {
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
