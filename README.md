# Elero Remote Control Component for ESPHome

This project is heavily based on the work of two other people:

  * All encryption/decryption structures copied from https://github.com/QuadCorei8085/elero_protocol (MIT)
  * All remote handling based on code from https://github.com/stanleypa/eleropy (GPLv3)

## Goal and Status

Ultimately, this component should allow you to control Elero blinds with the
bidirectional protocol directly from Home Assistant using an ESP32 with a CC1101
module attached. Apart from SPI (MISO, MOSI, SCK, CS) only GDO0 is required (in contrast to the other projects, GDO1 is not needed - it's not available on my module so I configured the CC1101 a bit differently.

The current code can transmit and simulate the TempoTel 2 that I have. Since some values are different to the two projects mentioned above, I'm not sure which/if this has an impact. I might make the remaining differences configurable configurable so that any type of remote can be simulated.

At the moment, there is no feedback and status of the blinds to Home Assistant, only controls are working.

Please be advised that this is very early development make, features might not work as intended!