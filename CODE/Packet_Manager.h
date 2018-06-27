#ifndef PACKET_MANAGER_H_
#define PACKET_MANAGER_H_

#include <stdint.h>
#include <string.h>

#include "driverlib.h"

typedef enum {

    IDLE,
    BUSY

} Consumer_State;

typedef enum {

    FRESH,
    BEING_CONSUMED,
    CONSUMED

} Packet_State;

typedef struct {

    Packet_State state;
    uint16_t packet_offset;
    uint16_t packet_length;

} Packet_Info;

typedef struct {

    uint8_t * buffer;
    uint16_t buffer_length;
    uint16_t write_location;
    uint16_t last_pkt_start_location;

    Packet_Info * packet_list;
    uint16_t packet_list_offset;
    uint16_t packet_list_length;

} Producer;

typedef struct {

    Producer * producer;
    uint16_t packet_list_offset;

} Producer_Consumer_State;

typedef struct {

    Consumer_State state;
    Producer_Consumer_State * producer_list;

} Consumer;

void Packet_Manger_init();
void Packet_Manger_process();

void SLIP_RX_produce_byte(uint8_t byte);
void SLIP_TX_consume_byte();

#endif /* PACKET_MANAGER_H_ */
