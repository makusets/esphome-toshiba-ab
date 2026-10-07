#define STATUS_DECODING_FIXTURES_ONLY
#include "master_status_test.cpp"

using namespace esphome::climate;

void checksum(Protocol p, std::vector<uint8_t> &data) {
  if (p == Protocol::TCC) {
    data.back() = 0;
    for (size_t i=0; i+1<data.size(); i++) data.back() ^= data[i];
  } else if (p == Protocol::TU2C) {
    data[data.size()-2]=0;
    for (size_t i=2; i+2<data.size(); i++) data[data.size()-2] += data[i];
  } else {
    uint16_t crc=0xFFFF;
    for (size_t i=0; i+2<data.size(); i++) {
      crc ^= data[i];
      for (unsigned bit=0; bit<8; bit++) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    data[data.size()-2]=crc>>8; data.back()=crc;
  }
}
void deliver(Bus &bus, Protocol p, std::vector<uint8_t> data) {
  checksum(p,data); bus.feed(data);
  unsigned decoded=0;
  for (const auto &line: debug_logs) if (line.find("Decoded ") == 0) decoded++;
  assert(decoded == 1);
}
int main() {
  for (Protocol p: {Protocol::TCC, Protocol::A0, Protocol::TU2C}) {
    Bus bus(p,SystemType::AIR); bus.confirm(p);
    auto data=Bus::frame(p,p==Protocol::TU2C?0xA0:0x58,p==Protocol::TU2C?0x38:0x81,
                         p==Protocol::TCC?10:p==Protocol::A0?15:22);
    const unsigned mode=p==Protocol::TCC?6:p==Protocol::A0?11:8;
    data[mode]=0x41; // Cool, powered.
    data[mode+1]=0x64; // High fan and ventilation.
    data[mode+2]=0x82; // Filter and preheating.
    data[mode+4]=114; // 22 C setpoint.
    data[mode+5]=120; // 25 C room.
    if (p==Protocol::TU2C) data[17]=3;
    deliver(bus,p,data);
    assert(bus.mode==CLIMATE_MODE_COOL && bus.action==CLIMATE_ACTION_COOLING);
    assert(bus.target_temperature==22 && bus.current_temperature==25);
    assert(*bus.fan_mode==CLIMATE_FAN_HIGH);
    if(p==Protocol::TU2C) assert(*bus.preset==CLIMATE_PRESET_ECO);
    unsigned count=bus.publish_count; deliver(bus,p,data); assert(bus.publish_count==count);
    data[mode+4]=0; data[mode+5]=1;
    deliver(bus,p,data); assert(bus.target_temperature==22 && bus.current_temperature==25);
  }
  for (Protocol p: {Protocol::A0,Protocol::TU2C}) {
    Bus bus(p,SystemType::WATER);
    ToshibaAbThermostat dhw(&bus,WaterCircuit::DHW), zone1(&bus,WaterCircuit::ZONE_1), zone2(&bus,WaterCircuit::ZONE_2);
    bus.confirm(p);
    auto data=Bus::frame(p,p==Protocol::A0?0x58:0xC0,p==Protocol::A0?0x03C6:0x31,
                         p==Protocol::A0?18:21,0,0xFF,0xE0);
    unsigned flags=p==Protocol::A0?11:8, target=p==Protocol::A0?14:11;
    data[flags]=0x43; // Both enabled, heating flag coexists with Auto.
    data[flags+1]=0x44; // Auto and DHW boost.
    data[target]=132; data[target+1]=76; // DHW50, Z1 22.
    if(p==Protocol::A0) {data[16]=80;data[17]=132;data[18]=76;data[19]=80;}
    else {data[10]=0x0C;data[14]=130;data[15]=100;data[16]=74;}
    deliver(bus,p,data);
    assert(dhw.mode==CLIMATE_MODE_HEAT && dhw.target_temperature==50);
    assert(zone1.mode==CLIMATE_MODE_AUTO && std::isnan(zone1.target_temperature));
    assert(*dhw.preset==CLIMATE_PRESET_BOOST);
    assert(zone2.mode==CLIMATE_MODE_OFF);
    if(p==Protocol::A0) assert(zone2.target_temperature==24);
    else {assert(dhw.current_temperature==49);assert(zone1.current_temperature==21);assert(dhw.action==CLIMATE_ACTION_HEATING);}
    unsigned count=zone1.publish_count;deliver(bus,p,data);assert(zone1.publish_count==count);
    data[flags]=0x21;data[flags+1]=0;
    deliver(bus,p,data);
    assert(dhw.mode==CLIMATE_MODE_OFF && zone1.mode==CLIMATE_MODE_COOL && zone1.target_temperature==22);
    data[flags]=0x41;deliver(bus,p,data);assert(zone1.mode==CLIMATE_MODE_HEAT);
    data[flags]=0;data[flags+1]=4;deliver(bus,p,data);
    assert(zone1.mode==CLIMATE_MODE_OFF && dhw.mode==CLIMATE_MODE_OFF);
    if(p==Protocol::TU2C) {
      // A short status can set Auto without containing either setpoint.
      auto short_data=Bus::frame(p,0,0x31,12,0,0xFF,0xE0);
      short_data[8]=1;short_data[9]=4;
      deliver(bus,p,short_data);
      assert(zone1.mode==CLIMATE_MODE_AUTO && std::isnan(zone1.target_temperature));
      assert(dhw.mode==CLIMATE_MODE_OFF && dhw.target_temperature==50);
    }
  }
  std::cout << "Status decoding tests passed (air, water, Auto setpoint suppression, DHW independence, tank mapping, short frames, duplicate updates).\n";
}
