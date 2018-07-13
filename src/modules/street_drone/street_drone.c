/****************************************************************************
 *
 *   Copyright (c) 2018 Linaro Ltd. All rights reserved.
 *   Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file street_drone.c
 *
 * Street Drone CAN Transport Layer.
 *
 * @author Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <px4_module.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <nuttx/can/can.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_vect.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <poll.h>
#include <modules/px4iofirmware/protocol.h>

#define BUFLEN 11
#define POLYNOMIAL 0x1D
#define MSG_DLC 8
#define STR_CTRL_1_ID 0x100
#define CUS_CTRL_1_ID 0x101

static int fd;
uint8_t alive = 0;
struct can_msg_s txmsg;
struct can_msg_s *rxmsg;
struct can_msg_s ctrlmsg = {
	.cm_hdr = {
		.ch_dlc = MSG_DLC,
		.ch_rtr = false,
		.ch_id = STR_CTRL_1_ID
	},
};
char rxbuffer[BUFLEN];

int can_devinit(void);
uint8_t crc8(uint8_t newByte, uint8_t crc);
void cmd_cus_ctrl_1(struct can_msg_s *canmsg, uint8_t str_req, uint8_t thr_req,
			bool str_auto, bool thr_auto);

/*
 * Routine to calculate CRC8 for CAN data
 *
 */
uint8_t crc8(uint8_t newByte, uint8_t crc)
{
	int i;

	crc = crc ^ newByte;
	for (i = 0; i < 8; i++){
		crc = (crc & 0x80)? (crc << 1) ^ POLYNOMIAL : (crc <<1);
	}

	return crc;
}

/*
 * Routine to construct CAN frame: ‘Customer_Control_1’
 * 
 * ID: 0x101
 * Data Frame
 * 
 * Signals:
 * ========
 *
 * CRC: Min:0, Max:255, Length:8bits, Datatype:uint8
 * Alive: Min:0, Max:255, Length:8bits, Datatype:uint8
 * Steer_Request: Min:-100, Length:8bits, Max:100, Datatype:int8
 * Torque_Request: Min:-100, Length:8bits, Max:100, Datatype:int8
 * Reserved: 0, Length:8bits, Datatype:uint8
 * Reserved: 0, Length:8bits, Datatype:uint8
 * Reserved: 0, Length:8bits, Datatype:uint8
 * Steer_Automation_Requested: Length:1bit, Datatype:uint8
 * Torque_Automation_Requested: Length:1bit, Datatype:uint8
 */
void cmd_cus_ctrl_1(struct can_msg_s *canmsg, uint8_t str_req, uint8_t thr_req,
			bool str_auto, bool thr_auto)
{
	uint8_t str_thr_auto = 0;
	uint8_t crc = 0xFF;

	if (str_auto)
		str_thr_auto |= (1 << 0);
	if (thr_auto)
		str_thr_auto |= (1 << 1);

	crc = crc8(alive, crc);
	crc = crc8(str_req, crc);
	crc = crc8(thr_req, crc);
	crc = crc8(0x00, crc);
	crc = crc8(0x00, crc);
	crc = crc8(0x00, crc);
	crc = crc8(str_thr_auto, crc);

	/* CAN frame header */
	canmsg->cm_hdr.ch_id = CUS_CTRL_1_ID;
	canmsg->cm_hdr.ch_rtr = false;
	canmsg->cm_hdr.ch_dlc = MSG_DLC;
	canmsg->cm_hdr.ch_unused = 0;

	PX4_INFO("CRC: %d", crc);
	PX4_INFO("Str Req: %d", str_req);
	PX4_INFO("Torque req: %d", thr_req);
	/* CAN frame data */
	canmsg->cm_data[0] = crc;
	canmsg->cm_data[1] = alive;
	canmsg->cm_data[2] = str_req;
	canmsg->cm_data[3] = thr_req;
	canmsg->cm_data[4] = 0x00;
	canmsg->cm_data[5] = 0x00;
	canmsg->cm_data[6] = 0x00;
	canmsg->cm_data[7] = str_thr_auto;

	/* this will wrap around to 0 after reaching 255 */
	alive++;
}

__EXPORT int street_drone_main(int argc, char *argv[]);
int street_drone_main(int argc, char *argv[])
{
	ssize_t nbytes;
	ssize_t msgsize, minsize;
	uint8_t rx_data[CAN_MAXDATALEN];
	uint8_t str_req, thr_req, str_auto, thr_auto;
	uint8_t str_max, str_min, thr_max, thr_min;
	int ret, nread, msglen, i, found = 0;
	
	int send_topic = orb_subscribe(ORB_ID(debug_vect));
        px4_pollfd_struct_t fds[1] = {};
        fds[0].fd = send_topic;
        fds[0].events = POLLIN;

//	struct named_value_int_s int_value;
//	memset(&int_value, 0, sizeof(int_value));
//	orb_advert_t int_value_fd = orb_advertise(ORB_ID(named_value_int), &int_value);

	fflush(stdout);

	if (argc < 2) {
		exit(1);
	} else if (!strcmp(argv[1], "send")) {
		PX4_INFO("Initializing the CAN device");
		/* Initialize the CAN device */
		ret = can_devinit();
		if (ret != OK) {
			PX4_ERR("Unable to initialize CAN device");
      			return -1;
    		}

		PX4_INFO("Opening the CAN device");
		/* Open the CAN device */
		fd = open("/dev/can0", O_RDWR);
		if (fd < 0) {
			PX4_ERR("Unable to open CAN device");
			return -1;
		}

		while (1) {
			ret = px4_poll(fds, 1, 1000);
			if (ret < 0)
				return -1;

			if (fds[0].revents & POLLIN) {
				struct debug_vect_s data;
				orb_copy(ORB_ID(debug_vect), send_topic, &data);
//				memcpy(int_value.name, name, 10);
//				PX4_INFO("Pusblishing data\n");
//				int_value.value = data.x;
//				orb_publish(ORB_ID(named_value_int), int_value_fd, &int_value);

				PX4_INFO("Constructing the CAN frame for transmission");
				msgsize = CAN_MSGLEN(MSG_DLC);

				str_req = data.str_req;
				thr_req = data.thr_req;
				str_auto = data.str_auto;
				thr_auto = data.thr_auto;
				cmd_cus_ctrl_1(&txmsg, str_req, thr_req, str_auto, thr_auto);

				PX4_INFO("Sending the CAN frame");
				/* Send CAN frame */
				nbytes = write(fd, &txmsg, msgsize);
				if (nbytes != msgsize) {
					PX4_ERR("Unable to write to CAN device");
					close(fd);
					return -1;
				}

				PX4_INFO("Sent bytes: %u successfully", nbytes);
			}
		}
	} else if (!strcmp(argv[1], "recv")) {
		PX4_INFO("Reading the CAN frames");
		/* Read CAN frames */
		minsize = CAN_MSGLEN(0);
		msgsize = sizeof(struct can_msg_s);
		nread = read(fd, rxbuffer, BUFLEN);
		if (nread < minsize || nread > msgsize) {
			PX4_ERR("Unable to read from CAN device");
			close(fd);
			return -1;
		}
		
		PX4_INFO("Received bytes: %u successfully", nread);
		for (i = 0; i <= nread - minsize; i += msglen) {	
  			/* Get the next message from the RX buffer */
			rxmsg  = (struct can_msg_s *)&rxbuffer[i];
			msglen = CAN_MSGLEN(8);

			DEBUGASSERT(i + msglen < BUFLEN);

			/* 
			 * FIXME: Ideally we should compare the received header with
			 * StreetDrone_Control_1 header stored in ctrlmsg
		 	 */
			if (!memcmp(&rxmsg->cm_hdr, &txmsg.cm_hdr, sizeof(struct can_hdr_s))) {
				found = 1;
				break;
			}
		}
		if (found) {
			printf("Recv ID: %4u DLC: %u\n", rxmsg->cm_hdr.ch_id, rxmsg->cm_hdr.ch_dlc);
			memcpy(&rx_data, &rxmsg->cm_data, sizeof(rxmsg->cm_data));
			str_max = (rx_data[2] > 127 ? (rx_data[2] - 256) : rx_data[2]);
			str_min = (rx_data[3] > 127 ? (rx_data[3] - 256) : rx_data[3]);
			thr_max = (rx_data[4] > 127 ? (rx_data[4] - 256) : rx_data[4]);
			thr_min = (rx_data[5] > 127 ? (rx_data[5] - 256) : rx_data[5]);
			
			PX4_INFO("StreetDrone_Control_1 CAN frame received!");
			PX4_INFO("Steer Request Max: %u", str_max);	
			PX4_INFO("Steer Request Min: %u", str_min);	
			PX4_INFO("Torque Request Min: %u", thr_max);	
			PX4_INFO("Torque Request Min: %u", thr_min);
			PX4_INFO("Steer Automation State: %u", ((rx_data[6] >> 4) & 0x0F));
			PX4_INFO("Torque Automation State: %u", (rx_data[6] & 0x0F));
		} else {
			PX4_WARN("StreetDrone_Control_1 CAN frame not recieved");
		}
	}

	close(fd);
	exit(0);
}
