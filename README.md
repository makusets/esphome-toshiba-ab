# ESPHome Toshiba_AB AC Component

<img src="hardware/v1/Final.jpg" width="170">    <img src="hardware/v1/Final2.jpg" width="170">


ESPHome component to integrate with Toshiba Air Conditioners via AB line protocol


This project implements functions to decode Toshiba AB protocol from indoor units to wired controllers and provides a hardware design to communicate.

In particular, this project has been tested with remote control unit RBC-AMT32E and RBC-AMT54E and central unit RAV-SM1103DT-A but should work with other models using the AB protocol.


Requires reader & writer circuit to interface with the AB line, connected to the remote AB ports. 
The circuit board was designed in easyEDA and all necessary files are included here.

Most of the work is based on previous work from @muxa: https://github.com/muxa/esphome-tcc-link
and the hard bits of decoding and initial board design by @issalig https://github.com/issalig/toshiba_air_cond


## To install, add or modify these sections in your esphome device yaml file

```yaml

logger:
  baud_rate: 0  #disable hardware UART log to use pins for UART communication with the AC unit 
  level: DEBUG

external_components:
  - source:
      type: git
      url: https://github.com/makusets/esphome-toshiba-ab
    refresh: 0s  #optional, how often to download fresh files from source, defaults to 1 day, use 0 to force updates 

uart:
  tx_pin: GPIO15
  rx_pin: GPIO13
  baud_rate: 2400
  parity: EVEN
  rx_buffer_size: 2048    # increase buffer to avoid dropped bytes

climate:
  - platform: toshiba_ab
    name: "Toshiba AC"
    id: toshiba_ac
    connected:
      name: "Toshiba AC Connected"  # optional, binary sensor that monitors link with AC
    master: 0x00 # Master ID in Toshiba protocol, optional, default is 0x00, for some units needs to be set to 0x01, 0x04 and possibly others
```

## Optional section if you install a BME280 sensor

This option is not really needed.

The option of a BME280 sensor is added to give the option of reading the temperature, pressure and humidity from the thermostat. It is done as any other sensor in the Home Assistant environment. If configured, it will be exposed to the frontend.

```yaml
# If installed, it will report the BME280 temp, humidity and pressure values

i2c:
  sda: GPIO2
  scl: GPIO14
  scan: True


sensor:
  - platform: bme280_i2c
    temperature:
      name: "Indoor Temperature"
      id: bme_temp
      oversampling: 1x
    pressure:
      name: "Indoor Pressure"
      id: bme_pressure
      accuracy_decimals: 0
      oversampling: 1x
    humidity:
      name: "Indoor Humidity"
      id: bme_humidity
      accuracy_decimals: 0
      oversampling: 2x
    address: 0x76
    update_interval: 30s
    
```

# Hardware installation

You will need to build the esphome compatible hardware. Instructions below.

- Most likely, the first time, you will have to flash the board with the firmware via USB, typical ESPHome process. Once working, OTA updates will work.

- Isolate the AC unit completely off (at the electrical distribution board ideally)
- Take out the cover of your remote controller
- Loose the screws of AB terminals. **WARNING**: The PCB assumes A is positive and B is negative. If this is not your case you can damage the PCB.
- Wire the remote A,B terminals to the pcb A,B ports

![image](https://github.com/issalig/toshiba_air_cond/blob/master/pcb/remote_back_pcb.jpg)

# Hardware design

This is the schematic of the board, it is powered by the AB line

![image](hardware/Schematic.JPG)


It should look something like that

![image](hardware/Board.JPG)


I2C headers have been added for the BME280 I2C sensor option, also for future inclusion of a screen or other I2C device
If a BME280 sensor is installed and setup in yaml it will report the readings to HA

All files necessary can be found in the hardware folder, including the EasyEDA Project:

https://github.com/makusets/esphome-toshiba-ab/tree/main/hardware
