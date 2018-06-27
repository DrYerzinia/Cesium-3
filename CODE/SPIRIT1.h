#ifndef __SPIRIT1_H
#define __SPIRIT1_H

#include <stdint.h>
#include <stdbool.h>

// SYNTH SETTINGS

#define SPIRIT1_WCP_SHIFT 5

#define SPIRIT1_BAND_SHIFT 0x00
#define SPIRIT1_BAND_MASK  0x07

#define SPIRIT1_6_BAND  1
#define SPIRIT1_12_BAND 3
#define SPIRIT1_16_BAND 4
#define SPIRIT1_32_BAND 5

#define SPIRIT1_VCO_L_SEL (1<<2)
#define SPIRIT1_VCO_H_SEL (1<<1)

#define SPIRIT1_REFDIV_SHIFT 7

#define SPIRIT1_REFDIV_1 0
#define SPIRIT1_REFDIV_2 (1 << SPIRIT1_REFDIV_SHIFT)

#define SPIRIT1_SYNTH_CONFIG_0_RES 0x20
#define SPIRIT1_SYNTH_CONFIG_1_RES 0x58

// MODULATION SETTINGS

#define SPIRIT1_CHFLT_SHIFT 4

#define SPIRIT1_MOD_SHIFT 4

#define SPIRIT1_MOD_BT_SEL_SHIFT 6

#define SPIRIT1_MOD_CW (1<<7)

// Packet settings

#define SPIRIT1_CRC_MODE_SHIFT 5
#define SPIRIT1_WHITENING_SHIFT 4
#define SPIRIT1_PREAMB_LEN_SHIFT 3
#define SPIRIT1_SYNC_LEN_SHIFT 1

// Radio settings

#define SPIRIT1_PERS_RX_SHFIT 1
#define SPIRIT1_DISABLE_SMPS_SHIFT 5

// IRQ Bits

#define SPIRIT1_IRQ_RX_DATA_READY (1<<0)
#define SPIRIT1_IRQ_TX_DATA_SENT  (1<<2)

// GPIO Modes
#define SPIRIT1_GPIO_nIRQ (0<<SPIRIT1_GPIO_SELECT_SHIFT)

#define SPIRIT1_GPIO_DIG_OUT_LOWPWR 2

#define SPIRIT1_GPIO_SELECT_SHIFT 3

// REGISTER DEFINITIONS

#define SPIRIT1_GPIO0_CONF           0x05
#define SPIRIT1_IF_OFFSET_ANA        0x07
#define SPIRIT1_SYNT3                0x08
#define SPIRIT1_SYNT2                0x09
#define SPIRIT1_SYNT1                0x0A
#define SPIRIT1_SYNT0                0x0B
#define SPIRIT1_IF_OFFSET_DIG        0x0D
#define SPIRIT1_FC_OFFSET_1          0x0E
#define SPIRIT1_FC_OFFSET_0          0x0F
#define SPIRIT1_PA_POWER_8           0x10
#define SPIRIT1_MOD1                 0x1A
#define SPIRIT1_MOD0                 0x1B
#define SPIRIT1_FDEV0                0x1C
#define SPIRIT1_CHFLT                0x1D
#define SPIRIT1_PCKTCTRL4            0x30
#define SPIRIT1_PCKTCTRL3            0x31
#define SPIRIT1_PCKTCTRL2            0x32
#define SPIRIT1_PCKTCTRL1            0x33
#define SPIRIT1_PCKTLEN1             0x34
#define SPIRIT1_PCKTLEN0             0x35
#define SPIRIT1_SYNC4                0x36
#define SPIRIT1_SYNC3                0x37
#define SPIRIT1_SYNC2                0x38
#define SPIRIT1_SYNC1                0x39
#define SPIRIT1_PCKT_FLT_OPTIONS     0x4F
#define SPIRIT1_PROTOCOL_0           0x52
#define SPIRIT1_IRQ_MASK_3           0x90
#define SPIRIT1_IRQ_MASK_2           0x91
#define SPIRIT1_IRQ_MASK_1           0x92
#define SPIRIT1_IRQ_MASK_0           0x93
#define SPIRIT1_PM_CONFIG            0xA4
#define SPIRIT1_AFC_CORR             0xC4
#define SPIRIT1_LINK_QUALIF_2        0xC5
#define SPIRIT1_LINK_QUALIF_1        0xC6
#define SPIRIT1_LINK_QUALIF_0        0xC7
#define SPIRIT1_RSSI_LEVEL           0xC8
#define SPIRIT1_LINEAR_FIFO_STATUS_1 0xE6
#define SPIRIT1_LINEAR_FIFO_STATUS_0 0xE7
#define SPIRIT1_DEVICE_INFO_PARTNUM  0xF0 // Expected 0x01
#define SPIRIT1_DEVICE_INFO_VERSION  0xF1 // Expected 0x30
#define SPIRIT1_IRQ_STATUS_3         0xFA
#define SPIRIT1_IRQ_STATUS_2         0xFB
#define SPIRIT1_IRQ_STATUS_1         0xFC
#define SPIRIT1_IRQ_STATUS_0         0xFD
#define SPIRIT1_FIFO                 0xFF

// COMMANDS

#define SPIRIT1_CMD_TX              0x60
#define SPIRIT1_CMD_RX              0x61
#define SPIRIT1_CMD_READY           0x62
#define SPIRIT1_CMD_STANDBY         0x63
#define SPIRIT1_CMD_SLEEP           0x64
#define SPIRIT1_CMD_LOCKRX          0x65
#define SPIRIT1_CMD_LOCKTX          0x66
#define SPIRIT1_CMD_SABORT          0x67
#define SPIRIT1_CMD_LDC_RELOAD      0x68
#define SPIRIT1_CMD_SEQUENCE_UPDATE 0x69
#define SPIRIT1_CMD_AES_ENC         0x6A
#define SPIRIT1_CMD_AES_KEY         0x6B
#define SPIRIT1_CMD_AES_DEC         0x6C
#define SPIRIT1_CMD_AES_KEYDEC      0x6D
#define SPIRIT1_CMD_SRES            0x70
#define SPIRIT1_CMD_FLUSHRXFIFO     0x71
#define SPIRIT1_CMD_FLUSHTXFIFO     0x72

// STATES

#define SPIRIT1_STATE_STANDBY 0x40
#define SPIRIT1_STATE_SLEEP   0x36
#define SPIRIT1_STATE_READY   0x03
#define SPIRIT1_STATE_LOCK    0x0F
#define SPIRIT1_STATE_RX      0x33
#define SPIRIT1_STATE_TX      0x5F

// SPI COMMANDS

#define SPIRIT1_SPI_WRITE 0x00
#define SPIRIT1_SPI_READ  0x01
#define SPIRIT1_SPI_CMD   0x80

// OTHER

#define SPIRIT1_FIFO_SIZE 96


typedef struct {

  void (*SPI_start)();
  void (*SPI_stop)();
  uint8_t (*SPI_transfer)(uint8_t);

  uint8_t stat1;
  uint8_t stat2;

} SPIRIT1_CONFIG;

typedef enum {
  BASIC = 0,
  WMBUS = 2,
  STACK = 3
} SPIRIT1_PACKET_TYPE;

typedef enum {
  NONE      = 0,
  CRC07     = 1,
  CRC8005   = 2,
  CRC1021   = 3,
  CRC864CBF = 4
} SPIRIT1_CRC_MODE;

typedef enum {
  FSK2    = 0,
  GFSK    = (1 << SPIRIT1_MOD_SHIFT),
  ASK_OOK = (2 << SPIRIT1_MOD_SHIFT),
  MSK     = (3 << SPIRIT1_MOD_SHIFT)
} SPIRIT1_MOD_MODE;

typedef enum {
  BT_SEL_1   = 0,
  BT_SEL_0_5 = (1 << SPIRIT1_MOD_BT_SEL_SHIFT)
} SPIRIT1_MOD_BT;

typedef struct {

  SPIRIT1_PACKET_TYPE pkt_type;

  uint8_t addr_len;
  uint8_t ctrl_len;
  uint8_t preamble_len;
  uint8_t sync_len;

  bool var_len;

  uint8_t len_wid;

  SPIRIT1_CRC_MODE crc_mode;

  bool whitening_en;
  bool fec_en;
  bool crc_check_en;

} SPIRIT1_PACKET_CONFIG;

typedef struct {

  uint8_t datarate_m;
  uint8_t datarate_e;
  uint8_t fdev_m;
  uint8_t fdev_e;
  uint8_t chflt_m;
  uint8_t chflt_e;
  uint8_t cw;
  SPIRIT1_MOD_BT bt_sel;
  SPIRIT1_MOD_MODE mod_type;

} SPIRIT1_MODULATION_CONFIG;

typedef struct {

  uint32_t synt;
  uint8_t band;
  uint8_t refdiv;
  uint8_t wcp;
  uint8_t vco_hl;

} SPIRIT1_FREQUENCY_CONFIG;

typedef struct {

  uint8_t rssi;
  uint8_t pqi;
  uint8_t sqi;
  uint8_t agc;
  uint8_t afc;

} SPIRIT1_LINK_QUAL;

void SPIRIT1_init(SPIRIT1_CONFIG * sconf);

void SPIRIT1_transmit_packet(SPIRIT1_CONFIG * sconf, uint8_t * data, uint16_t len);

void SPIRIT1_lock_tx(SPIRIT1_CONFIG * sconf);
void SPIRIT1_enable_rx(SPIRIT1_CONFIG * sconf);

void SPIRIT1_enable_persistent_rx(SPIRIT1_CONFIG * sconf);
void SPIRIT1_disable_smps(SPIRIT1_CONFIG * sconf);
void SPIRIT1_configure_gpio(SPIRIT1_CONFIG * sconf, uint8_t pin, uint8_t function);
void SPIRIT1_configure_irq_mask(SPIRIT1_CONFIG * sconf, uint32_t mask);
void SPIRIT1_configure_pa(SPIRIT1_CONFIG * sconf, uint8_t pa_power);
void SPIRIT1_set_crystal_correction(SPIRIT1_CONFIG * sconf, int16_t corr);

void SPIRIT1_set_packet_config(SPIRIT1_CONFIG * sconf, SPIRIT1_PACKET_CONFIG * pkt_conf);
void SPIRIT1_set_base_frequency(SPIRIT1_CONFIG * sconf, SPIRIT1_FREQUENCY_CONFIG * freq_conf);
void SPIRIT1_set_modulation(SPIRIT1_CONFIG * sconf, SPIRIT1_MODULATION_CONFIG * modulation);

void SPIRIT1_set_ifoffset(SPIRIT1_CONFIG * sconf, uint8_t offset_ana, uint8_t offset_dig);

uint8_t SPIRIT1_rxfifo_dump(SPIRIT1_CONFIG * sconf, uint8_t * data, uint8_t len);
void SPIRIT1_get_link_qual(SPIRIT1_CONFIG * sconf, SPIRIT1_LINK_QUAL * link_qual);

void SPIRIT1_register_dump(SPIRIT1_CONFIG * sconf, uint8_t * data);
uint32_t SPIRIT1_get_irq_mask(SPIRIT1_CONFIG * sconf);
uint32_t SPIRIT1_get_irq_status(SPIRIT1_CONFIG * sconf);
uint8_t SPIRIT1_get_rx_fifo_len(SPIRIT1_CONFIG * sconf);
uint8_t SPIRIT1_get_tx_fifo_len(SPIRIT1_CONFIG * sconf);
uint8_t SPIRIT1_get_partnum(SPIRIT1_CONFIG * sconf);
uint8_t SPIRIT1_get_version(SPIRIT1_CONFIG * sconf);

#endif
