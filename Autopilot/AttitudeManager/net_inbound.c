/**
 * @file net_inbound.c
 * @author Chris Hajduk
 * @date Sep 2015
 * @brief Implementation for handling UART input from the XBEE telemetry datalink
 * @copyright Waterloo Aerial Robotics Group 2016 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 * @see http://ftp1.digi.com/support/documentation/90002173_R.pdf for the Xbee manual
 */

#include "net.h"
#include <stdlib.h>
#include "main.h"
#include "UART2.h"

/** Defines the start value of a new incoming valid telemetry packet **/
#define START_DELIMITER 0x7E

/** The index in the raw telemetry packet that indicates the length of the data stream */
#define DATA_LENGTH_INDEX 1
/** The index in the raw telemetry packet at which the command number is contained. Data should come after this */
#define DATA_COMMAND_NUM_INDEX 15
/** Number of bytes between when the data ends and rawpacket ends */
#define DATA_END_PADDING_OFFSET 12

/**
 * @brief Cirular buffer queue that contains an array of parsed commands retrieved from the telemetry link.
 * Will not accept any more commands if the buffer is full.
 */
typedef struct {
	struct command* commands[INBOUND_QUEUE_SIZE];
	short start_pos;
	short end_pos;
} CommandBuffer;

/**
 * RawPacket statuses. Every raw packet is initalized with EMPTY, meaning it can be written to.
 * As the packet is being written to, its status is set to BUSY. After the packet has been completely
 * written to, it is set to READY, afterwhich the packet payload will be parsed and put into the command buffer.
 */
typedef enum {
    /** Can write new data here **/
    EMPTY = 0,
    /** Data being written here **/
    BUSY = 1,
    /** Data Ready **/
    READY = 2
} RawPacketStatus;

/**
 * Represents a RawPacket, containing the payload, the current packet status and the length and payload byte position
 * of the containing payload
 */
typedef struct {
	char payload[MAX_PACKET_SIZE + 1];
	RawPacketStatus status;
        short latest_byte; //which byte in the packets payload we're writing to
        short length;
} RawPacket;

/**
 * @brief Circular buffer storage for packets received from the UART2 port, before the are parsed into commands and placed in the command buffer.
 * A UART2 interrupt transfers data byte by byte. This is used as a temporary buffer storage for the raw packets we receive, until
 * they are parsed as commands and placed into the command buffer as executed by the inboundBufferMaintanance function.
 * Will not receive any more packets if the buffer is full.
 */
typedef struct {
	RawPacket packets[RAW_PACKET_BUFFER_SIZE]; /** **/
	short curr_packet; /** which packet in the buffer  we're currently writing to **/
} RawPacketBuffer;

/**
 * Initializes the internal input buffer queue by setting all its command pointers to null.
 * @param buffer The command buffer to initialize
 **/
static void initCommandInputBuffer(CommandBuffer* buffer){
	for(int i = 0;i<INBOUND_QUEUE_SIZE;i++){
		buffer->commands[i] = NULL;
	}
	buffer->start_pos = 0;
	buffer->end_pos = 0;
}

/**
 * Initializes a raw packet buffer by setting the status of all its packets to EMPTY
 * @param buffer The raw packet buffer to initialize
 */
static void initRawPacketBuffer(RawPacketBuffer* buffer){
	for (int i =0; i< RAW_PACKET_BUFFER_SIZE; i++){
		buffer->packets[i].status = EMPTY;
		buffer->packets[i].latest_byte = 0;
		buffer->packets[i].length = 0;
	}
	buffer->curr_packet = 0;
}

// initialize our two buffers
static RawPacketBuffer rawPacketBuffer;
static CommandBuffer commandBuffer;

initRawPacketBuffer(&rawPacketBuffer);
initCommandBuffer(&commandBuffer);

/**
 * Destroys and deallocates a command
 * @param The command to remove
 */
void destroyCommand(struct command *cmd) {
    free(cmd);
    cmd = NULL;
}

/**
 * Pops a command out of the command buffer queue
 * @return Pointer reference to the command popped
 */
struct command* popCommand() {
    struct command* cmd = inBuffer[inbuff_start];
    if ( ! cmd ) return 0; //if its an already deleted(deallocated) pointer/command, do nothing
    inBuffer[inbuff_start] = 0;
    inbuff_start = ( inbuff_start + 1 ) % INBOUND_QUEUE_SIZE;
    return cmd;
}

int pushCommand(struct command* cmd) {
    if ( inbuff_end == ( inbuff_start - 1 + INBOUND_QUEUE_SIZE ) % INBOUND_QUEUE_SIZE ) {
        return 0;   // If no room for commands, fail
    }
    inBuffer[inbuff_end] = cmd;     // Insert cmd at end of queue
    inbuff_end = ( inbuff_end + 1 ) % INBOUND_QUEUE_SIZE; // increment handling wrap
    return 1;
}

/** Parses a raw packet from the telemetry and converts it into a command
 * @return The parsed command containing the command number and data
 */
struct command* createCommand( char* rawPacket ) {
    struct command* cmd = malloc(sizeof(struct command));
    if ( ! cmd ) {  // Malloc failed ?
        return 0;
    }
    cmd->data_length = rawPacket[DATA_LENGTH_INDEX];
    cmd->cmd = rawPacket[DATA_COMMAND_NUM_INDEX];
    int j = 0;
    //Data comes right after the Command Number, in this case at the 16th byte
    for (int i = DATA_COMMAND_NUM_INDEX + 1; i < 15 + cmd->data_length - DATA_END_PADDING_OFFSET; i++ ){
        cmd->data[j++] = rawPacket[i];
    }
    cmd->data[j] = '\0';// Null terminate the string so can use SendUart
    return cmd;
}

// Return 1 if packet is valid
int checkPacket(char* rawPacket) {
    unsigned int i = 2;
    char packetLength = rawPacket[i++];
    char checksum = 0;
    char packetChecksum = 0;

    for (i = 3; i < packetLength + 3; i++){
        checksum += rawPacket[i];
    }
    packetChecksum = 0xFF - rawPacket[i];
    if (checksum == packetChecksum){
        return 1;                       // TODO: checksums are for suckers, because fuck you, thats why

    }
    else{
        return 0;
    }
}

void inboundBufferMaintenance(void) {
    int i;
    for ( i = 0; i < RAW_PACKET_BUFFER_SIZE; i++ ) {
        if ( rawPacketStatus[i] == READY && checkPacket(rawPackets[i]) ) {
            struct command* cmd = createCommand( rawPackets[i] );
            if ( cmd ) {            // create command was successful ?
                pushCommand( cmd ); // queue it up
                rawPacketStatus[i] = EMPTY;         // buffer is now good for writing another packet
            }
        }
    }
    if ( rawPacketStatus[0] == EMPTY ) {
        rawPacketStatus[0] = BUSY;
    }
}

/**
 * @brief Writes a byte to the raw buffer.
 * @param The byte to write
 * @param The reference to the actual RawPacketBuffer instance of which the data will be written to
 *
 * Will perform error checking on the incoming bytes as documented in the XBEE API documentation.
 * The first byte in a packet will be checked so that it starts with the START_DELIMETER, indicating
 * that this byte is the begining of a new packet. Then the length will be parsed from the 2 subsequent
 * bytes, and error-checked to make sure its positive. Afterwords the rest of the packet contents will
 * be written to the raw buffer until the data length determined before is reached
 * @see Page 83 of the Xbee documentation listed above for the API format
 */
static void writeByteToRawBuffer(unsigned char data, RawPacketBuffer* buffer){
	//copy the packet over for easier reading
	RawPacket* curr_packet = buffer->packets[buffer->curr_packet];

	//Note: If the current packet status is READY, then we've run out of buffer space and will do nothing

	// in a good packet, the first byte must be the START_DELIMITER. If not, its bad data and we should do nothing
	if (curr_packet->status == EMPTY){
		if (data == START_DELIMITER){
			curr_packet->payload[0] = data;
			curr_packet->latest_byte = 1;
			curr_packet->status = BUSY; //set this as BUSY, indicating we're writing data to it
			return;
		}
	} else if (curr_packet->status == BUSY){
		//The data length is a 2-byte value that comes after the start delimiter
		if (curr_packet->latest_byte == DATA_LENGTH_INDEX){
			curr_packet->length = ((short)data << 8); //the first byte of the length (msb)
		} else if (curr_packet->latest_byte == DATA_LENGTH_INDEX + 1){
			curr_packet->length |= (short)data; //last byte of the length (lsb)

			//do some error checking on the length, since it must be positive
			if (curr_packet->length <= 0){
				curr_packet->status = EMPTY;
				return;
			}
		}

		//write the data to the buffer
		curr_packet->payload[curr_packet->latest_byte] = data;
		curr_packet->latest_byte++;

		//check if this was the last byte of data for the packet based on its length
		if ((curr_packet->latest_byte > DATA_LENGTH_INDEX + 1) && (curr_packet->latest_byte - DATA_LENGTH_INDEX - 2) == curr_packet->length){
			curr_packet->status = READY; //this the status of this packet as ready, so that it may be transferred to the command buffer later
			//this is a circular buffer, so have the pointer go back to the start if it exceeds the size of the buffer
			buffer->curr_packet = (buffer->curr_packet + 1) % INBOUND_QUEUE_SIZE;
		}
	}
}

/**
 * @brief Interrupt service routine for when we receive data from our UART2 connection to the XBEE.
 * The byte received will be taken from the appropriate register and then copied over to the
 * raw packet buffer.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {
    unsigned char data = U2RXREG;
    writeByteToRawBuffer(data, &rawPacketBuffer);
    resetUART2RXInterrupt(); //enable subsequent UART2 interrupts
}
