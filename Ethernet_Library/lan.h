/*
 * lan.h
 * Description: Includes protocols(Ethernet,ARP,IP,ICMP,UDP) Header file
 * Created on: May 20, 2024
 * Author: @ Hexabitz
 ******************************************************************************
 * @attention: this driver requires to set up SPI
 *
 * Copyright (c) 2024 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */
#pragma once

#include <string.h>
#include "enc28j60.h"
extern uint8_t mac_addr[6];
extern uint32_t Local_IP;
extern uint32_t ip_mask;
extern uint32_t Remote_IP;
extern uint32_t ip_dest;
extern uint8_t Local_PORT;
extern uint8_t Remote_PORT;

/*
 * Config
 */

#define WITH_ICMP

#define ARP_CACHE_SIZE		3

#define MAC_ADDR            {0x00,0x0C,0xD1,0xED,0x34,0x2}

#define IP_ADDR				inet_addr(192,168,1,2)//ethernet
#define IP_SUBNET_MASK		inet_addr(255,255,255,0)
#define IP_DEFAULT_GATEWAY	inet_addr(192,168,1,3)//lap
#define IP_DEST             IP_DEFAULT_GATEWAY

#define TO_PORT             1//lap
#define FROM_PORT           2//ethernet

#define IP_PACKET_TTL		64


/*
 * BE conversion
 */

#define htons(a)			((((a)>>8)&0xff)|(((a)<<8)&0xff00))
#define ntohs(a)			htons(a)

#define htonl(a)			( (((a)>>24)&0xff) | (((a)>>8)&0xff00) |\
								(((a)<<8)&0xff0000) | (((a)<<24)&0xff000000) )
#define ntohl(a)			htonl(a)

#define inet_addr(a,b,c,d)	( ((uint32_t)a) | ((uint32_t)b << 8) |\
								((uint32_t)c << 16) | ((uint32_t)d << 24) )

/*
 * Ethernet
 */
 
#define ETH_TYPE_ARP		htons(0x0806)
#define ETH_TYPE_IP			htons(0x0800)

typedef struct __packed eth_frame {
	uint8_t to_addr[6];
	uint8_t from_addr[6];
	uint16_t type;
	uint8_t data[];
} eth_frame_t;

/*
 * ARP
 */

#define ARP_HW_TYPE_ETH		htons(0x0001)
#define ARP_PROTO_TYPE_IP	htons(0x0800)

#define ARP_TYPE_REQUEST	htons(1)
#define ARP_TYPE_RESPONSE	htons(2)

typedef struct __packed arp_message {
	uint16_t hw_type; //link layer protocol (Ethernet)
	uint16_t proto_type;// network layer protocol (IP)
	uint8_t hw_addr_len;// MAC address length =6
	uint8_t proto_addr_len;// length of IP address =4
	uint16_t type;// message type (request/response)
	uint8_t mac_addr_from[6];// Source MAC address
	uint32_t ip_addr_from;// sender IP address
	uint8_t mac_addr_to[6];// Destination MAC address, zeros if unknown
	uint32_t ip_addr_to;// destination IP address
} arp_message_t;

typedef struct __packed arp_cache_entry {
	uint32_t ip_addr;
	uint8_t mac_addr[6];
} arp_cache_entry_t;

/*
 * IP
 */

#define IP_PROTOCOL_ICMP	1
#define IP_PROTOCOL_TCP		6
#define IP_PROTOCOL_UDP		17

typedef struct __packed ip_packet {
	uint8_t ver_head_len;// header version and length =0x45
	uint8_t tos;//Service Type
	uint16_t total_len;//length of the whole packet
	uint16_t fragment_id;//fragment ID
	uint16_t flags_framgent_offset;// fragment offset
	uint8_t ttl;//TTL
	uint8_t protocol;//protocol code
	uint16_t cksum;//header checksum
	uint32_t from_addr;//Sender's IP address
	uint32_t to_addr;//recipient's IP address
	uint8_t data[];
} ip_packet_t;


/*
 * ICMP
 */

#define ICMP_TYPE_ECHO_RQ	8
#define ICMP_TYPE_ECHO_RPLY	0

typedef struct __packed icmp_echo_packet {
	uint8_t type;
	uint8_t code;
	uint16_t cksum;
	uint16_t id;
	uint16_t seq;
	uint8_t data[];
} icmp_echo_packet_t;


/*
 * UDP
 */

typedef struct __packed udp_packet {
	uint16_t from_port;
	uint16_t to_port;
	uint16_t len;
	uint16_t cksum;
	uint8_t data[];
} udp_packet_t;


/*
 * LAN
 */

extern uint8_t net_buf[];
extern arp_cache_entry_t arp_cache[ARP_CACHE_SIZE];
void lan_init();
void lan_echo();
void lan_poll(uint8_t* pData,uint16_t* length);
void udp_packet(eth_frame_t *frame, uint16_t len);
uint8_t udp_send(eth_frame_t *frame, uint16_t len);
void udp_reply(eth_frame_t *frame, uint16_t len);
void udp_answer(eth_frame_t *frame, uint16_t len,char *answer);

void  ether_send_udp(char *data ,uint16_t length);
void lan_poll(uint8_t* pData,uint16_t* length);
