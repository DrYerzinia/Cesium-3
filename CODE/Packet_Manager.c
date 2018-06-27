#include "Packet_Manager.h"

// Debug

typedef enum {
    DISABLED,
    SELF,
    OTHER
} Loopback_Mode;

Loopback_Mode slip_loopback_mode = DISABLED;
Loopback_Mode radio_loopback_mode = DISABLED;

#define MAX_PACKET_SIZE 256

#define COMMAND_LED_OFF 0
#define COMMAND_LED_ON 1

// Producer indexes for consumers
#define SLIP_TX_PRODUCER_COUNT 4
#define SLIP_TX_to_SLIP_RX 0

#define Internal_Message_PRODUCER_COUNT 4
#define SLIP_RX_to_Internal_Message 0

Producer SLIP_RX;
Producer UHF_RX;
Producer SBAND_RX;
Producer Internal_Message_Prod;

Consumer SLIP_TX;
Consumer UHF_TX;
Consumer SBAND_TX;
Consumer Internal_Message_Cons;

uint8_t SLIP_RX_buffer[1024];
uint8_t UHF_RX_buffer[512];
uint8_t SBAND_RX_buffer[512];
uint8_t Internal_Message_buffer[256];

Packet_Info SLIP_RX_packet_list[10];
Packet_Info SBAND_RX_packet_list[5];
Packet_Info UHF_RX_packet_list[5];
Packet_Info Internal_Message_packet_list[2];

Producer_Consumer_State SLIP_TX_producer_list[SLIP_TX_PRODUCER_COUNT];
Producer_Consumer_State Internal_Message_producer_list[Internal_Message_PRODUCER_COUNT];

void Packet_Manger_init(){

    // Initalize Producers
    SLIP_RX.buffer = SLIP_RX_buffer;
    SLIP_RX.buffer_length = 1024;
    SLIP_RX.last_pkt_start_location = 0;
    SLIP_RX.packet_list = SLIP_RX_packet_list;
    SLIP_RX.packet_list_length = 10;
    SLIP_RX.packet_list_offset = 0;
    SLIP_RX.write_location = 0;

    UHF_RX.buffer = UHF_RX_buffer;
    UHF_RX.buffer_length = 512;
    UHF_RX.packet_list = UHF_RX_packet_list;
    UHF_RX.packet_list_length = 5;
    UHF_RX.packet_list_offset = 0;
    UHF_RX.write_location = 0;

    SBAND_RX.buffer = SBAND_RX_buffer;
    SBAND_RX.buffer_length = 512;
    SBAND_RX.packet_list = SBAND_RX_packet_list;
    SBAND_RX.packet_list_length = 5;
    SBAND_RX.packet_list_offset = 0;
    SBAND_RX.write_location = 0;

    Internal_Message_Prod.buffer = Internal_Message_buffer;
    Internal_Message_Prod.buffer_length = 256;
    Internal_Message_Prod.packet_list = Internal_Message_packet_list;
    Internal_Message_Prod.packet_list_length = 2;
    Internal_Message_Prod.packet_list_offset = 0;
    Internal_Message_Prod.write_location = 0;

    // Initialize Consumers
    SLIP_TX.producer_list = SLIP_TX_producer_list;
    SLIP_TX.state = IDLE;
    SLIP_TX_producer_list[SLIP_TX_to_SLIP_RX].producer = &SLIP_RX;
    SLIP_TX_producer_list[SLIP_TX_to_SLIP_RX].packet_list_offset = 0;

    Internal_Message_Cons.producer_list = Internal_Message_producer_list;
    Internal_Message_Cons.state = IDLE;
    Internal_Message_producer_list[SLIP_RX_to_Internal_Message].producer = &SLIP_RX;
    Internal_Message_producer_list[SLIP_RX_to_Internal_Message].packet_list_offset = 0;

}

static inline advance_producer(Producer * p){

    uint16_t plen = p->write_location - p->last_pkt_start_location;
    if(plen == 0) return;

    // Set new packet parameters in new packet list
    p->packet_list[p->packet_list_offset].packet_offset = p->last_pkt_start_location;
    p->packet_list[p->packet_list_offset].packet_length = plen;
    p->packet_list[p->packet_list_offset].state = FRESH;

    // Check to see if we have room for another packet before end of buffer and prepare to start next packet
    if(p->buffer_length-p->write_location < MAX_PACKET_SIZE){
        p->write_location = 0;
    }
    p->last_pkt_start_location = p->write_location;

    // Advance the packet list and allow consumers to access last packet
    p->packet_list_offset++;
    if(p->packet_list_offset == p->packet_list_length){ // Wrap the packet list
        p->packet_list_offset = 0;
    }

    // TODO: will be difficult to check overflow.  Will have to advance buffer to find
    // first non consumed packet in list and see its offset to see if we are going to
    // overrun it

}

typedef enum {
    START,
    NORMAL,
    ESCAPE,
    END
} SLIP_STATE;

SLIP_STATE slip_rx_state;

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

void SLIP_RX_produce_byte(uint8_t byte){

    switch(slip_rx_state){
        case START:
        case NORMAL:
            switch(byte){
                case SLIP_END:
                    slip_rx_state = NORMAL;
                    advance_producer(&SLIP_RX);
                    break;
                case SLIP_ESC:
                    slip_rx_state = ESCAPE;
                    break;
                default:
                    SLIP_RX.buffer[SLIP_RX.write_location++] = byte;
                    break;
            }
            break;
        case ESCAPE:
            switch(byte){
                case SLIP_ESC_END:
                    SLIP_RX.buffer[SLIP_RX.write_location++] = SLIP_END;
                    break;
                case SLIP_ESC_ESC:
                    SLIP_RX.buffer[SLIP_RX.write_location++] = SLIP_ESC;
                    break;
                default:
                    // TODO ERROR STATE
                    break;
            }
            slip_rx_state = NORMAL;
            break;

    }

}

Packet_Info * SLIP_TX_pinfo = 0;
uint8_t * SLIP_TX_data;
uint16_t SLIP_TX_count;

SLIP_STATE slip_tx_state;

void SLIP_TX_consume_byte(){

    if(SLIP_TX_pinfo == 0) return;

    if(slip_tx_state == START){
        HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = SLIP_END;
        slip_tx_state = NORMAL;
    }

    if(SLIP_TX_count == SLIP_TX_pinfo->packet_length){

        if(slip_tx_state == NORMAL){

            HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = SLIP_END;
            slip_tx_state = END;
            return;

        } else if(slip_tx_state == END){

            SLIP_TX_pinfo = 0;
            SLIP_TX.state = IDLE;

        }

        return;

    }

    uint8_t byte = SLIP_TX_data[SLIP_TX_pinfo->packet_offset+SLIP_TX_count];

    switch(slip_tx_state){
        case NORMAL:
            switch(byte){
                case SLIP_END:
                case SLIP_ESC:
                    slip_tx_state = ESCAPE;
                    HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = SLIP_ESC;
                    break;
                default:
                    HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = byte;
                    SLIP_TX_count++;
                    break;
            }
            break;
        case ESCAPE:
            switch(byte){
                case SLIP_END:
                    HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = SLIP_ESC_END;
                    break;
                case SLIP_ESC:
                    HWREG16(EUSCI_A0_BASE + OFS_UCAxTXBUF) = SLIP_ESC_ESC;
                    break;
            }
            SLIP_TX_count++;
            slip_tx_state = NORMAL;
            break;

    }

}

static inline void SLIP_TX_consume_SLIP_RX(){
    // Loopback debug enabled
    if(slip_loopback_mode == SELF){

        // SLIP TX not currently DMAing anything
        if(SLIP_TX.state == IDLE){

            // If we are behind the producer in packets
            Producer_Consumer_State * pcs = &(SLIP_TX.producer_list[SLIP_TX_to_SLIP_RX]);
            while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

                Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

                // If fresh
                if(pinfo->state == FRESH){

                    // Set variables for SLIP TX to read from
                    SLIP_TX_pinfo = pinfo;
                    SLIP_TX_data = pcs->producer->buffer;
                    SLIP_TX_count = 0;

                    slip_tx_state = START;
                    SLIP_TX_consume_byte();

                    pinfo->state = BEING_CONSUMED;
                    SLIP_TX.state = BUSY;

                    break;

                }

                pcs->packet_list_offset++;
                if(pcs->packet_list_offset == pcs->producer->packet_list_length){
                    pcs->packet_list_offset = 0;
                }

            }
        }
    }
}

uint8_t internal_ip[] = {0xc0, 0xa8, 0x01, 0x8d}; // 192.168.1.141

static inline void Internal_Message_consume_SLIP_RX(){

    if(slip_loopback_mode == DISABLED){

        Producer_Consumer_State * pcs = &(Internal_Message_Cons.producer_list[SLIP_RX_to_Internal_Message]);
        while(pcs->packet_list_offset != pcs->producer->packet_list_offset){

            Packet_Info * pinfo = &(pcs->producer->packet_list[pcs->packet_list_offset]);

            // If fresh
            if(pinfo->state == FRESH){

                uint16_t offset = pinfo->packet_offset;
                uint8_t * data = pcs->producer->buffer;

                if(memcmp(data+offset+12, internal_ip, 4) == 0){ // Check that IP address matches target

                    // Get command byte
                    uint8_t cmd = data[offset+28];
                    switch(cmd){
                        case COMMAND_LED_ON:
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                            break;
                        case COMMAND_LED_OFF:
                            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                            break;
                        default:
                            break;
                    }

                }

            }

            pcs->packet_list_offset++;
            if(pcs->packet_list_offset == pcs->producer->packet_list_length){
                pcs->packet_list_offset = 0;
            }

        }

    }

}

void Packet_Manger_process(){

    // Our messages to the ground are very important and should happen seldomly

    // 1 ) UHF TX             from   Internal Message
    // 2 ) S-Band TX          from   Internal Message

    // Clearing TX buffer from CDH is next highest priority

    // 3 ) UHF TX             from   SLIP RX            (Skip if SLIP Loopback Mode ALL)
    // 4 ) S-Band TX          from   SLIP RX            (Skip if SLIP Loopback Mode ALL)

    // We want our messages to CDH to be before radio traffic and they should happen seldomly (Buffer Full Warning)

    // 5 ) SLIP TX            from   Internal Message

    // Clearing out radio RX buffers is important

    // 6 ) SLIP TX            from   S-Band RX          (Skip if RADIO Loopback Mode not NONE)
    // 7 ) SLIP TX            from   UHF RX             (Skip if RADIO Loopback Mode not NONE)

    // Worry about reading in internal messages after things are on there way to radios
    // 8 ) Internal Message   from   SLIP RX            (Skip if SLIP  Loopback Mode ALL)
    // 9 ) Internal Message   from   UHF RX             (Skip if RADIO Loopback Mode not NONE)
    // 10) Internal Message   from   S-Band RX          (Skip if RADIO Loopback Mode not NONE)
    Internal_Message_consume_SLIP_RX();

    // Loopback test modes lowest priority

    // 11) SLIP TX            from   SLIP RX            (If SLIP Loopback Mode ALL)
    SLIP_TX_consume_SLIP_RX();

    // 12) UHF TX             from   UHF RX             (If RADIO Loopback Mode SELF)
    // 13) UHF TX             from   S-Band RX          (If RADIO Loopback Mode OTHER)

    // 14) S-Band TX          from   S-Band RX          (If RADIO Loopback Mode SELF)
    // 15) S-Band TX          from   UHF RX             (If RADIO Loopback Mode OTHER)

}
