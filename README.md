# Elero Remote Control Component for ESPHome

This project is heavily based on the work of two other people:

  * All encryption/decryption structures copied from https://github.com/QuadCorei8085/elero_protocol (MIT)
  * All remote handling based on code from https://github.com/stanleypa/eleropy (GPLv3)

## Goal and Status

Ultimately, this component should allow you to control Elero blinds with the
bidirectional protocol directly from Home Assistant using an ESP32 with a CC1101
module attached. 

Currently, the code can successfully listen to, decode and dump remote messages
from an Elero remote / Elero blind. Transmitting and mapping to blinds is not
yet supported.
