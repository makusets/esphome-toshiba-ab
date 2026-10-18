#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "toshiba_ab.h"

namespace esphome
{
  namespace toshiba_ab
  {

    class ToshibaAbOnDataReceivedTrigger : public Trigger<std::vector<uint8_t>>
    {
    public:
      ToshibaAbOnDataReceivedTrigger(ToshibaAbClimate *climate)
      {
        climate->add_on_data_received_callback(
            [this](const struct DataFrame *frame)
            {
              this->trigger(frame->get_data());
            });
      }
    };

    template<typename... Ts> class ToshibaAbSendRawFrameAction : public Action<Ts...> {
     public:
      ToshibaAbSendRawFrameAction(ToshibaAbClimate *climate) : climate_(climate) {}

      TEMPLATABLE_VALUE(std::string, frame)

      void play(const Ts &... x) override {
        this->climate_->send_raw_frame_from_text(this->frame_.value(x...));
      }

     protected:
      ToshibaAbClimate *climate_;
    };

  } // namespace toshiba_ab
} // namespace esphome
