#include "SPIRIT1.h"

#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)

static void read_block(SPIRIT1_CONFIG * sconf, uint8_t addr, uint8_t * buf, uint8_t len){

  sconf->SPI_start();

  sconf->stat1 = sconf->SPI_transfer(SPIRIT1_SPI_READ);
  sconf->stat2 = sconf->SPI_transfer(addr);

  int i;
  for(i = 0; i < len; i++){
    buf[i] = sconf->SPI_transfer(0x00);
  }

  sconf->SPI_stop();

}

static void write_block(SPIRIT1_CONFIG * sconf, uint8_t addr, uint8_t * buf, uint8_t len){

  sconf->SPI_start();

  sconf->stat1 = sconf->SPI_transfer(SPIRIT1_SPI_WRITE);
  sconf->stat2 = sconf->SPI_transfer(addr);

  int i;
  for(i = 0; i < len; i++){
    sconf->SPI_transfer(buf[i]);
  }

  sconf->SPI_stop();

}

static uint8_t readreg8(SPIRIT1_CONFIG * sconf, uint8_t addr){

  uint8_t val;

  sconf->SPI_start();

  sconf->stat1 = sconf->SPI_transfer(SPIRIT1_SPI_READ);
  sconf->stat2 = sconf->SPI_transfer(addr);

  val = sconf->SPI_transfer(0x00);

  sconf->SPI_stop();

  return val;

}

static void writereg8(SPIRIT1_CONFIG * sconf, uint8_t addr, uint8_t val){

  sconf->SPI_start();

  sconf->stat1 = sconf->SPI_transfer(SPIRIT1_SPI_WRITE);
  sconf->stat2 = sconf->SPI_transfer(addr);

  sconf->SPI_transfer(val);

  sconf->SPI_stop();

}

static void command(SPIRIT1_CONFIG * sconf, uint8_t command){

  sconf->SPI_start();

  sconf->stat1 = sconf->SPI_transfer(SPIRIT1_SPI_CMD);
  sconf->stat2 = sconf->SPI_transfer(command);

  sconf-> SPI_stop();

}

void SPIRIT1_init(SPIRIT1_CONFIG * sconf){

  //

}

void SPIRIT1_transmit_packet(SPIRIT1_CONFIG * sconf, uint8_t * data, uint16_t len){

  writereg8(sconf, SPIRIT1_PCKTLEN1, ((len & 0xFF00) >> 8));
  writereg8(sconf, SPIRIT1_PCKTLEN0, (len & 0x00FF));

  write_block(sconf, SPIRIT1_FIFO, data, len);
  command(sconf, SPIRIT1_CMD_TX);

}

void SPIRIT1_lock_tx(SPIRIT1_CONFIG * sconf){
  command(sconf, SPIRIT1_CMD_LOCKTX);
}

void SPIRIT1_enable_rx(SPIRIT1_CONFIG * sconf){
  command(sconf, SPIRIT1_CMD_RX);
}

void SPIRIT1_enable_persistent_rx(SPIRIT1_CONFIG * sconf){

  uint8_t proto_0 = readreg8(sconf, SPIRIT1_PROTOCOL_0);
  writereg8(sconf, SPIRIT1_PROTOCOL_0, proto_0 | (1 << SPIRIT1_PERS_RX_SHFIT));

}

void SPIRIT1_disable_smps(SPIRIT1_CONFIG * sconf){

  uint8_t pm_conf = readreg8(sconf, SPIRIT1_PM_CONFIG);
  writereg8(sconf, SPIRIT1_PM_CONFIG, pm_conf | (1 << SPIRIT1_DISABLE_SMPS_SHIFT));

}

void SPIRIT1_configure_gpio(SPIRIT1_CONFIG * sconf, uint8_t pin, uint8_t mode){

  writereg8(sconf, SPIRIT1_GPIO0_CONF-pin, mode);

}

void SPIRIT1_configure_irq_mask(SPIRIT1_CONFIG * sconf, uint32_t mask){

  uint8_t irq_mask[4];

  int i;
  for(i = 0; i < 4; i++){
    irq_mask[3-i] = (mask & (0xFF<<(i*8))) >> (i*8);
  }

  write_block(sconf, SPIRIT1_IRQ_MASK_3, irq_mask, 4);

}

void SPIRIT1_configure_pa(SPIRIT1_CONFIG * sconf, uint8_t pa_power){

  writereg8(sconf, SPIRIT1_PA_POWER_8, pa_power);

}

// TODO complete 12 bit 2's complment write
void SPIRIT1_set_crystal_correction(SPIRIT1_CONFIG * sconf, int16_t corr){

  writereg8(sconf, SPIRIT1_FC_OFFSET_0, corr);

}

void SPIRIT1_set_packet_config(SPIRIT1_CONFIG * sconf, SPIRIT1_PACKET_CONFIG * pkt_conf){

  uint8_t pktctrl3 = pkt_conf->len_wid;
  uint8_t pktctrl2 = (pkt_conf->preamble_len << SPIRIT1_PREAMB_LEN_SHIFT) | (pkt_conf->sync_len << SPIRIT1_SYNC_LEN_SHIFT) | pkt_conf->var_len;
  uint8_t pktctrl1 = (pkt_conf->crc_mode << SPIRIT1_CRC_MODE_SHIFT) | (pkt_conf->whitening_en << SPIRIT1_WHITENING_SHIFT) | pkt_conf->fec_en;

  uint8_t pckt_flt_op = pkt_conf->crc_check_en;

  writereg8(sconf, SPIRIT1_PCKTCTRL2, pktctrl3);
  writereg8(sconf, SPIRIT1_PCKTCTRL2, pktctrl2);
  writereg8(sconf, SPIRIT1_PCKTCTRL1, pktctrl1);

  writereg8(sconf, SPIRIT1_PCKT_FLT_OPTIONS, pckt_flt_op);

}

void SPIRIT1_set_base_frequency(SPIRIT1_CONFIG * sconf, SPIRIT1_FREQUENCY_CONFIG * freq_conf){

  // EQN 4
  // Fbase = fXO/((B*D)/2) * SYNT/(2^18)

  uint8_t synth_config[2];

  synth_config[0] = SPIRIT1_SYNTH_CONFIG_0_RES;
  synth_config[1] = freq_conf->refdiv | SPIRIT1_SYNTH_CONFIG_1_RES | freq_conf->vco_hl;

  uint8_t synt_reg[4];

  synt_reg[3] = ((freq_conf->synt & 0x03E00000) >> 21) | (freq_conf->wcp << SPIRIT1_WCP_SHIFT);
  synt_reg[2] = ((freq_conf->synt & 0x001FE000) >> 13);
  synt_reg[1] = ((freq_conf->synt & 0x00001FE0) >> 5);
  synt_reg[0] = ((freq_conf->synt & 0x0000001F) << 3) | ((freq_conf->band & SPIRIT1_BAND_MASK) << SPIRIT1_BAND_SHIFT);

  writereg8(sconf, SPIRIT1_SYNT3, synt_reg[3]);
  writereg8(sconf, SPIRIT1_SYNT2, synt_reg[2]);
  writereg8(sconf, SPIRIT1_SYNT1, synt_reg[1]);
  writereg8(sconf, SPIRIT1_SYNT0, synt_reg[0]);

}

void SPIRIT1_set_modulation(SPIRIT1_CONFIG * sconf, SPIRIT1_MODULATION_CONFIG * modulation){

  // EQN 11
  // DataRate = fclk * ((256+DATA_RATE_M)*2^DATARATE_E)/(2^28)

  uint8_t mod1, mod0;

  mod0 = modulation->datarate_e | modulation->cw | modulation->bt_sel | modulation->mod_type;
  mod1 = modulation->datarate_m;

  uint8_t fdev0;

  fdev0 = (modulation->fdev_e << 4) | modulation->fdev_m; // TODO CLOCK REC ALG

  uint8_t chflt;

  chflt = modulation->chflt_e | (modulation->chflt_m << SPIRIT1_CHFLT_SHIFT);

  writereg8(sconf, SPIRIT1_MOD0, mod0);
  writereg8(sconf, SPIRIT1_MOD1, mod1);

  writereg8(sconf, SPIRIT1_FDEV0, fdev0);

  writereg8(sconf, SPIRIT1_CHFLT, chflt);

}

void SPIRIT1_set_ifoffset(SPIRIT1_CONFIG * sconf, uint8_t offset_ana, uint8_t offset_dig){

  writereg8(sconf, SPIRIT1_IF_OFFSET_DIG, offset_dig);
  writereg8(sconf, SPIRIT1_IF_OFFSET_ANA, offset_ana);

}

uint8_t SPIRIT1_rxfifo_dump(SPIRIT1_CONFIG * sconf, uint8_t * data, uint8_t len){

  uint8_t bytes_available = SPIRIT1_get_rx_fifo_len(sconf);
  uint8_t bytes_to_read = MIN(bytes_available, len);

  read_block(sconf, SPIRIT1_FIFO, data, bytes_to_read);

  return bytes_to_read;

}

void SPIRIT1_get_link_qual(SPIRIT1_CONFIG * sconf, SPIRIT1_LINK_QUAL * link_qual){

  link_qual->rssi = readreg8(sconf, SPIRIT1_RSSI_LEVEL);
  link_qual->pqi = readreg8(sconf, SPIRIT1_LINK_QUALIF_2);
  link_qual->sqi = readreg8(sconf, SPIRIT1_LINK_QUALIF_1) & 0x7F;
  link_qual->agc = readreg8(sconf, SPIRIT1_LINK_QUALIF_0) & 0x0F;
  link_qual->afc = readreg8(sconf, SPIRIT1_AFC_CORR);

}

void SPIRIT1_register_dump(SPIRIT1_CONFIG * sconf, uint8_t * data){

  read_block(sconf, 0x00, data, 0x6F);

}

uint32_t SPIRIT1_get_irq_mask(SPIRIT1_CONFIG * sconf){

  uint8_t irq_mask[4];
  uint32_t val;

  read_block(sconf, SPIRIT1_IRQ_MASK_3, irq_mask, 4);

  val = (irq_mask[0] << 24) | (irq_mask[1] << 16) | (irq_mask[2] << 8) | irq_mask[3];

  return val;

}

uint32_t SPIRIT1_get_irq_status(SPIRIT1_CONFIG * sconf){

  uint8_t irq_stat[4];
  uint32_t val;

  read_block(sconf, SPIRIT1_IRQ_STATUS_3, irq_stat, 4);

  val = (irq_stat[0] << 24) | (irq_stat[1] << 16) | (irq_stat[2] << 8) | irq_stat[3];

  return val;

}

uint8_t SPIRIT1_get_rx_fifo_len(SPIRIT1_CONFIG * sconf){
  return readreg8(sconf, SPIRIT1_LINEAR_FIFO_STATUS_0);
}

uint8_t SPIRIT1_get_tx_fifo_len(SPIRIT1_CONFIG * sconf){
  return readreg8(sconf, SPIRIT1_LINEAR_FIFO_STATUS_1);
}

uint8_t SPIRIT1_get_partnum(SPIRIT1_CONFIG * sconf){

  return readreg8(sconf, SPIRIT1_DEVICE_INFO_PARTNUM);

}

uint8_t SPIRIT1_get_version(SPIRIT1_CONFIG * sconf){

  return readreg8(sconf, SPIRIT1_DEVICE_INFO_VERSION);

}
