/**
 * @file net_inbound.c
 * @author Chris Hajduk
 * @date Sep 2015
 * @brief Implementation for handling UART input from the XBEE telemetry datalink * @copyright Waterloo Aerial Robotics Group 2016 \n
 *  https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE
 * @see http://ftp1.digi.com/support/documentation/90002173_R.pdf for the Xbee manual
 */

#include "net.h"
#include <stdlib.h>
#include "main.h"
#include "UART2.h"

/** Defines the start value of a new incoming valid telemetry packet.
 * This is defined in the XBEE APi docs **/
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
    char checksum;
} RawPacket;

/**
 * @brief Circular buffer storage for packets received from the UART2 port, before the are parsed into commands and placed in the command buffer.
 * A UART2 interrupt transfers data byte by byte. This is used as a temporary buffer storage for the raw packets we receive, until
 * they are parsed as commands and placed into the command buffer as executed by the inboundBufferMaintanance function.
 * Will not receive any more packets if the buffer is full.
 */
typedef struct {
	RawPacket packets[RAW_PACKET_BUFFER_SIZE];
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

void destroyCommand(struct command* cmd) {
    free(cmd);
}

struct command* popCommand() {
    struct command* cmd = commandBuffer->commands[commandBuffer->start_pos];
    commandBuffer->commands[commandBuffer->start_pos] = NULL; //set to null so we dont have any dangling pointers
    commandBuffer->start_pos = (commandBuffer->start_pos + 1) % INBOUND_QUEUE_SIZE;
    return cmd;
}

/**
 * Pushes a command onto the command buffer queue.
 * This method is only used internally by the inboundBufferMaintenance function, so no need to expose this function
 * @param cmd Command to insert into the buffer
 * @return 1 if command successfully pushed, or 0 otherwise
 */
static char pushCommand(struct command* cmd) {
    short insert_position = (commandBuffer->end_pos + 1) % INBOUND_QUEUE_SIZE;

    //check if we've run out of space on our command buffer. If so return false/0
    if (insert_position == commandBuffer->start_pos){
        return 0;
    }
    //otherwise we'll add the command
    commandBuffer->end_pos = insert_position;
    commandBuffer->commands[insert_position] = cmd;
    return 1;
}

/**
 * Parses a raw packet from the telemetry and converts it into a command
 * @return A pointer to the parsed command, or null if it could not be created
 */
static struct command* createCommand(RawPacket* packet) {
    struct command* cmd = malloc(sizeof(struct command));
    if (! cmd) { //malloc might fail in case we run out of memory. We don't want to fatally crash because of this
        return NULL;
    }
    cmd->data_length = packet->length;
    cmd->cmd = packet->payload[DATA_COMMAND_NUM_INDEX];
    int j = 0;
    //Data comes right after the Command Number, in this case at the 16th byte
    for (int i = DATA_COMMAND_NUM_INDEX + 1; i < 15 + cmd->data_length - DATA_END_PADDING_OFFSET; i++ ){
        cmd->data[j++] = packet->payload[i];
    }
    cmd->data[j] = '\0';// Null terminate the string so can use SendUart
    return cmd;
}

/**
 * @brief Checks validity of a raw packet based on its checksum.
 * @return 1 if packet is valid, 0 otherwise
 * According to the XBEE API documentation, the last byte of a packet is the checksum, and does
 * not count towards the length specified by the first 2 bytes of the packet. The checksum
 * is the result of adding all the data bytes, whilst only keeping the 8 least significant bits. Then
 * the result is subtracted from 255, or 0xFF. To verify data integrity, we add all the data bytes
 * whilst keeping only the latest 8 bits, and add the checksum from the last byte of the payload.
 * If it equals to 0xFF, then our data is intact and usable.
 * @see Page 85 of the Xbee manual listed above
 */
static char isValidRawPacket(RawPacket* packet){
    unsigned char checksum = 0;
    //we don't want to count the actual checksum value, hence the -1 after the length
    for (int i = DATA_LENGTH_INDEX + 2; i < packet->length - 1; i++){
        checksum += packet->payload[i];
    }

    return (checksum + packet->checksum) == 0xFF;
}

void inboundBufferMaintenance(void) {
    for (int i = 0; i < RAW_PACKET_BUFFER_SIZE; i++) {
        RawPacket* packet = rawPacketBuffer->packets[i];
        //check if packet is ready to be transferred
        if (packet->status == READY){
            if(isValidPacket(packet)){ //check integrity of the packet based on its checksum
                //create the command
                struct command* cmd = createCommand(packet);
                //Check if the command was successfully created. If so, push it to the CommandBuffer.
                //If the command could not be created due to a shortage of memory, we'll discard the command
                if(cmd != NULL){
                    if(!pushCommand(cmd)){ //if we were not able to push the command to the command buffer, it means that its full and we should wait
                        destroyCommand(cmd);
                        return;
                    }
                }
            }
            packet->status = EMPTY;
        }
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
		//if we've written more than 3 bytes AND we've written the length of the data PLUS 3 bytes for the start,
		//plus 1 byte for the checksum, which doesnt count towards the length of the packet
		if ((curr_packet->latest_byte > DATA_LENGTH_INDEX + 1) && (curr_packet->latest_byte - 2 - (DATA_LENGTH_INDEX + 1)) == curr_packet->length){
			curr_packet->status = READY; //this the status of this packet as ready, so that it may be transferred to the command buffer later
			curr_packet->checksum = curr_packet->payload[curr_packet->latest_byte - 1]; //save the checksum for future reference for next time
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
