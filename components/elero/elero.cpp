#include "elero.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace elero {

static const char *TAG = "elero";
static const uint8_t flash_table_encode[] = {0x08, 0x02, 0x0d, 0x01, 0x0f, 0x0e, 0x07, 0x05, 0x09, 0x0c, 0x00, 0x0a, 0x03, 0x04, 0x0b, 0x06};
static const uint8_t flash_table_decode[] = {0x0a, 0x03, 0x01, 0x0c, 0x0d, 0x07, 0x0f, 0x06, 0x00, 0x08, 0x0b, 0x0e, 0x09, 0x02, 0x05, 0x04};

void Elero::loop() {
  if(this->received_) {
    //ESP_LOGD(TAG, "loop says \"received\"");
    this->received_ = false;
    uint8_t len = this->read_status(CC1101_RXBYTES);
    if(len & 0x7F && !(len & 0x80)) { // No overflow and bytes available
      if(len > 64) {
        ESP_LOGD(TAG, "Received more bytes than FIFO length - wtf?");
      } else {
        this->read_buf(CC1101_RXFIFO, this->msg_rx_, len);
        //std::string data = format_hex(this->msg_rx_, len);
        //ESP_LOGD(TAG, "Received: 0x%s", data.c_str());
        // Sanity check
        if(this->msg_rx_[0] + 3 == len) {
          this->interprete_msg();
        }
      }
    }
    this->flush_and_rx();
  }
}

void IRAM_ATTR Elero::interrupt(Elero *arg) {
  arg->set_received();
}

void IRAM_ATTR Elero::set_received() {
  this->received_ = true;
}

void Elero::dump_config() {
  ESP_LOGCONFIG(TAG, "Elero Config: ");
}

void Elero::setup() {
  ESP_LOGI(TAG, "Setting up Elero Component...");
  this->spi_setup();
  this->gdo0_pin_->setup();
  this->gdo0_irq_pin_ = this->gdo0_pin_->to_isr();
  this->gdo0_pin_->attach_interrupt(Elero::interrupt, this, gpio::INTERRUPT_FALLING_EDGE);
  this->reset();
  this->init();
}

void Elero::flush_and_rx() {
  this->write_cmd(CC1101_SIDLE);
  this->write_cmd(CC1101_SFRX);
  this->write_cmd(CC1101_SFTX);
  this->write_cmd(CC1101_SRX);
  this->received_ = false;
}

void Elero::reset() {
  // We don't do a hardware reset as we can't read
  // the MISO pin directly. Rely on software-reset only.
  
  this->enable();
  this->write_byte(CC1101_SRES);
  delayMicroseconds(50);
  this->write_byte(CC1101_SIDLE);
  delayMicroseconds(50);
  this->disable();
}

void Elero::init() {
  uint8_t patable_data[] = {0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};

  this->write_reg(CC1101_FSCTRL1, 0x08);
  this->write_reg(CC1101_FSCTRL0, 0x00);
  this->write_reg(CC1101_FREQ2, 0x21);
  this->write_reg(CC1101_FREQ1, 0x71); 
  this->write_reg(CC1101_FREQ0, 0xC0); 
  this->write_reg(CC1101_MDMCFG4, 0x7B);
  this->write_reg(CC1101_MDMCFG3, 0x83);
  this->write_reg(CC1101_MDMCFG2, 0x13); 
  this->write_reg(CC1101_MDMCFG1, 0x52);
  this->write_reg(CC1101_MDMCFG0, 0xF8);
  this->write_reg(CC1101_CHANNR, 0x00);
  this->write_reg(CC1101_DEVIATN, 0x43);
  this->write_reg(CC1101_FREND1, 0xB6);
  this->write_reg(CC1101_FREND0, 0x10);
  this->write_reg(CC1101_MCSM0, 0x18);
  this->write_reg(CC1101_MCSM1, 0x3F);
  this->write_reg(CC1101_FOCCFG, 0x1D);
  this->write_reg(CC1101_BSCFG, 0x1F);
  this->write_reg(CC1101_AGCCTRL2, 0xC7);
  this->write_reg(CC1101_AGCCTRL1, 0x00);
  this->write_reg(CC1101_AGCCTRL0, 0xB2);
  this->write_reg(CC1101_FSCAL3, 0xEA);
  this->write_reg(CC1101_FSCAL2, 0x2A);
  this->write_reg(CC1101_FSCAL1, 0x00);
  this->write_reg(CC1101_FSCAL0, 0x1F);
  this->write_reg(CC1101_FSTEST, 0x59);
  this->write_reg(CC1101_TEST2, 0x81);
  this->write_reg(CC1101_TEST1, 0x35);
  this->write_reg(CC1101_TEST0, 0x09);
  this->write_reg(CC1101_IOCFG0, 0x06);
  this->write_reg(CC1101_PKTCTRL1, 0x8C);  
  this->write_reg(CC1101_PKTCTRL0, 0x45);
  this->write_reg(CC1101_ADDR, 0x00);
  this->write_reg(CC1101_PKTLEN, 0x3C);
  this->write_reg(CC1101_SYNC1, 0xD3);
  this->write_reg(CC1101_SYNC0, 0x91);
  this->write_burst(CC1101_PATABLE, patable_data, 8);

  this->write_cmd(CC1101_SRX);
  this->wait_rx();
}

void Elero::write_reg(uint8_t addr, uint8_t data) {
  this->enable();
  this->write_byte(addr);
  this->write_byte(data);
  this->disable();
  delayMicroseconds(15);
}

void Elero::write_burst(uint8_t addr, uint8_t *data, uint8_t len) {
  this->enable();
  this->write_byte(addr | CC1101_WRITE_BURST);
  for(int i=0; i<len; i++)
    this->write_byte(data[i]);
  this->disable();
  delayMicroseconds(15);
}

void Elero::write_cmd(uint8_t cmd) {
  this->enable();
  this->write_byte(cmd);
  this->disable();
  delayMicroseconds(15);
}

bool Elero::wait_rx() {
  uint8_t timeout = 200;
  while ((this->read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_RX) || (--timeout == 0)) {
    delayMicroseconds(200);
  }
  
  if(timeout > 0)
    return true;
  ESP_LOGD(TAG, "Timed out waiting for RX");
  return false;
}

bool Elero::wait_tx_done() {
  uint8_t timeout = 200;
  while ((!this->received_) || (--timeout == 0)) {
    delayMicroseconds(200);
  }

  if(timeout > 0)
    return true;
  ESP_LOGD(TAG, "Timed out waiting for TX");
  return false;
}

void Elero::transmit() {
  this->flush_and_rx();
  if(!this->wait_rx())
    return;
  
  this->write_burst(CC1101_TXFIFO, this->msg_tx_, this->msg_tx_[0]);
  this->write_cmd(CC1101_STX);
  this->wait_tx_done();
  if((this->read_status(CC1101_TXBYTES) & 0x7f) != 0) {
    ESP_LOGD(TAG, "Error transferring, bytes left in buffer");
  } else {
    ESP_LOGD(TAG, "Transmission successful");
  }
  this->flush_and_rx();
}

uint8_t Elero::read_reg(uint8_t addr) {
  uint8_t data;

  this->enable();
  this->write_byte(addr);
  data = this->read_byte();
  return data;
}

uint8_t Elero::read_status(uint8_t addr) {
  uint8_t data;

  this->enable();
  this->write_byte(addr | CC1101_READ_BURST);
  data = this->read_byte();
  this->disable();
  delayMicroseconds(15);
  return data;
}

void Elero::read_buf(uint8_t addr, uint8_t *buf, uint8_t len) {
  this->enable();
  this->write_byte(addr | CC1101_READ_BURST);
  for(uint8_t i=0; i<len; i++)
    buf[i] = this->read_byte();
  this->disable();
  delayMicroseconds(15);
}

uint8_t Elero::count_bits(uint8_t byte)
{
  uint8_t i;
  uint8_t ones = 0;
  uint8_t mask = 1;

  for( i = 0; i < 8; i++ )
  {
    if( mask & byte )
    {
      ones += 1;
    }

    mask <<= 1;
  }

  return ones & 0x01;
}


void Elero::calc_parity(uint8_t* msg)
{
  uint8_t i;
  uint8_t p = 0;

  for( i = 0; i < 4; i++ )
  {
    uint8_t a = count_bits( msg[0 + i*2] );
    uint8_t b = count_bits( msg[1 + i*2] );

    p |= a ^ b;
    p <<= 1;
  }

  msg[7] = (p << 3);
}

void Elero::add_r20_to_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length)
{
  uint8_t i;

  for( i = 0; i < 8; i++ )
  {
    uint8_t d = msg[i];

    uint8_t ln = (d + r20) & 0x0F;
    uint8_t hn = ((d & 0xF0) + (r20 & 0xF0)) & 0xFF;

    msg[i] = hn | ln;

    r20 = (r20 - 0x22) & 0xFF;
  }
}

void Elero::sub_r20_from_nibbles(uint8_t* msg, uint8_t r20, uint8_t start, uint8_t length)
{
  uint8_t i;

  for(i = start; i < length; i++)
  {
    uint8_t d = msg[i];

    uint8_t ln = (d - r20) & 0x0F;
    uint8_t hn = ((d & 0xF0) - (r20 & 0xF0)) & 0xFF;

    msg[i] = hn | ln;

    r20 = (r20 - 0x22) & 0xFF;
  }
}

void Elero::xor_2byte_in_array_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1)
{
  uint8_t i;

  for( i = 1; i < 4; i++ )
  {
    msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
    msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
  }
}

void Elero::xor_2byte_in_array_decode(uint8_t* msg, uint8_t xor0, uint8_t xor1)
{
  uint8_t i;

  for( i = 0; i < 4; i++ )
  {
    msg[i*2 + 0] = msg[i*2 + 0] ^ xor0;
    msg[i*2 + 1] = msg[i*2 + 1] ^ xor1;
  }
}

void Elero::encode_nibbles(uint8_t* msg)
{
  uint8_t i;

  for( i = 0; i < 8; i++ )
  {
    uint8_t nh = (msg[i] >> 4) & 0x0F;
    uint8_t nl = msg[i] & 0x0F;

    uint8_t dh = flash_table_encode[nh];
    uint8_t dl = flash_table_encode[nl];

    msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
  }
}

void Elero::decode_nibbles(uint8_t* msg, uint8_t len)
{
  uint8_t i;

  for( i = 0; i < len; i++ )
  {
    uint8_t nh = (msg[i] >> 4) & 0x0F;
    uint8_t nl = msg[i] & 0x0F;

    uint8_t dh = flash_table_decode[nh];
    uint8_t dl = flash_table_decode[nl];

    msg[i] = ((dh << 4) & 0xFF) | ((dl) & 0xFF);
  }
}

void Elero::msg_decode(uint8_t *msg) {
  decode_nibbles(msg, 8);
  sub_r20_from_nibbles(msg, 0xFE, 0, 2);
  xor_2byte_in_array_decode(msg, msg[0], msg[1]);
  sub_r20_from_nibbles(msg, 0xBA, 2, 8);
}

void Elero::msg_encode(uint8_t* msg, uint8_t xor0, uint8_t xor1) {
  calc_parity(msg);
  add_r20_to_nibbles(msg, 0xFE, 0, 8);
  xor_2byte_in_array_encode(msg, xor0, xor1);
  encode_nibbles(msg);
}

void Elero::interprete_msg() {
  uint8_t length = this->msg_rx_[0];
  uint8_t cnt = this->msg_rx_[1];
  uint8_t typ = this->msg_rx_[2];
  uint8_t chl = this->msg_rx_[6];
  uint32_t src = ((uint32_t)this->msg_rx_[7] << 16) | ((uint32_t)this->msg_rx_[8] << 8) | (this->msg_rx_[9]);
  uint32_t bwd = ((uint32_t)this->msg_rx_[10] << 16) | ((uint32_t)this->msg_rx_[11] << 8) | (this->msg_rx_[12]);
  uint32_t fwd = ((uint32_t)this->msg_rx_[13] << 16) | ((uint32_t)this->msg_rx_[14] << 8) | (this->msg_rx_[15]);
  uint8_t num_dests = this->msg_rx_[16];
  uint32_t dst;
  uint8_t dests_len;
  if(typ > 0x60) {
    dests_len = this->msg_rx_[16] * 3;
    dst = ((uint32_t)this->msg_rx_[17] << 16) | ((uint32_t)this->msg_rx_[18] << 8) | (this->msg_rx_[19]);
  } else {
    dests_len = this->msg_rx_[16];
    dst = this->msg_rx_[17];
  }
  uint8_t crc = this->msg_rx_[length + 2] >> 7;
  uint8_t lqi = this->msg_rx_[length + 2] & 0x7f;
  float rssi;
  if(this->msg_rx_[length+1] > 127)
    rssi = (float)((this->msg_rx_[length+1]-256)/2-74);
  else
    rssi = (float)((this->msg_rx_[length+1])/2-74);
  uint8_t *payload = &msg_rx_[19 + dests_len];
  msg_decode(payload);
  ESP_LOGD(TAG, "len=%02d, cnt=%02d, typ=0x%02x, chl=%02d, src=0x%06x, bwd=0x%06x, fwd=0x%06x, #dst=%02d, dst=%06x, rssi=%2.1f, lqi=%2d, crc=%2d, payload=[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]", length, cnt, typ, chl, src, bwd, fwd, num_dests, dst, rssi, lqi, crc, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]);

  if(typ == 0xca) { // Status message from a blind
    // Check if we know the blind
    // status = payload[6]
  }
}

}  // namespace elero
}  // namespace esphome
