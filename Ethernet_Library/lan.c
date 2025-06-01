/*
 * lan.c
 * Description: Includes protocols(Ethernet,ARP,IP,ICMP,UDP) Source file
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
#include "lan.h"

uint8_t mac_addr[6] = MAC_ADDR;
uint32_t Local_IP = IP_ADDR;
uint32_t ip_mask = IP_SUBNET_MASK;
uint32_t Remote_IP = IP_DEFAULT_GATEWAY;
uint32_t ip_dest=IP_DEST;
uint8_t net_buf[ENC28J60_MAXFRAME];
uint8_t Local_PORT=FROM_PORT;
uint8_t Remote_PORT=TO_PORT;

uint8_t arp_cache_wr;
arp_cache_entry_t arp_cache[ARP_CACHE_SIZE];
uint8_t data_watch[1460];

void eth_send(eth_frame_t *frame, uint16_t len);
void eth_reply(eth_frame_t *frame, uint16_t len);
uint8_t *arp_resolve(uint32_t node_ip_addr);
uint8_t ip_send(eth_frame_t *frame, uint16_t len);
void ip_reply(eth_frame_t *frame, uint16_t len);
uint16_t ip_cksum(uint32_t sum, uint8_t *buf, size_t len);
void  ether_send_udp(char *data ,uint16_t length);

/*
 * UDP
 */
void  ether_send_udp(char *data ,uint16_t length)
{
	 static uint8_t buf[576];
	 eth_frame_t *send=(void*)buf;
	 memset( send->to_addr ,0xFF,6);
	 memcpy(send->from_addr, mac_addr, 6);
	 memcpy(send->data,data , length);
	 send->type=ETH_TYPE_IP;
	 udp_send(send,length);
}
// send UDP packet
// fields must be set:
//	- ip.dst
//	- udp.src_port
//	- udp.dst_port
// uint16_t len is UDP data payload length
uint8_t udp_send(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (ip_packet_t*)(frame->data);
	udp_packet_t *udp = (udp_packet_t*)(ip->data);

	memcpy((udp->data), (frame->data), len);

	len += sizeof(udp_packet_t);

	ip->protocol = IP_PROTOCOL_UDP;
	ip->from_addr = Local_IP;
	ip->to_addr = ip_dest;
	udp->len = htons(len);
	udp->cksum = 0;
	udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP,(uint8_t*)udp-8, len+8);
    udp->from_port=htons(Local_PORT);
    udp->to_port=htons(Remote_PORT);

	return ip_send(frame, len);
}

// reply to UDP packet
// len is UDP data payload length
void udp_reply(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (ip_packet_t*)(frame->data);
	udp_packet_t *udp = (udp_packet_t*)(ip->data);
	uint16_t temp;

	// Calculate the length of the entire packet
	len += sizeof(udp_packet_t);
    udp->from_port=htons(Local_PORT);
    udp->to_port=htons(Remote_PORT);
	// Swap the port of the sender and receiver
	temp = udp->from_port;
	udp->from_port = udp->to_port;
	udp->to_port =temp;

	// Packet length
	udp->len = htons(len);

     // Calculate checksum from pseudo header + data
	 // Pseudo header = packet length + protocol + IP addresses + normal udp header
	 // packet length + protocol is passed as the initial value for
	 // checksum calculation
	 // take ip addresses from the header of the IP packet (udp packet - 8)
	udp->cksum = 0;
	udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP, 
	(uint8_t*)udp-8, len+8);

	ip_reply(frame, len);
}

// process UDP packet
void udp_filter(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (void*)(frame->data);
	udp_packet_t *udp = (void*)(ip->data);
	memcpy(data_watch,udp->data,len);
	// Check header length
	if(len >= sizeof(udp_packet_t))
	{
	// Give the package to the application
	 udp_reply(frame, ntohs(udp->len) -
	sizeof(udp_packet_t));

	}
}

void udp_filter_answer(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (void*)(frame->data);
	udp_packet_t *udp = (void*)(ip->data);
	memcpy(data_watch,udp->data,len);
}

/*
 * ICMP
 */

#ifdef WITH_ICMP

// process ICMP packet
void icmp_filter(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *packet = (void*)frame->data;
	icmp_echo_packet_t *icmp = (void*)packet->data;

	// Check packet length
	if(len >= sizeof(icmp_echo_packet_t) )
	{
	// Received an Echo Request
	if(icmp->type == ICMP_TYPE_ECHO_RQ)
	{
		// Change packet type to response
		icmp->type = ICMP_TYPE_ECHO_RPLY;
		// Update the checksum,
	   //  we only changed one field in the batch,
	  //   so it is not necessary to recalculate completely
		icmp->cksum += 8; // update cksum

		// Send the packet back
		ip_reply(frame, len);
	}
	}
}

#endif


/*
 * IP
 */

// calculate IP checksum
uint16_t ip_cksum(uint32_t sum, uint8_t *buf, size_t len)
{
	while(len >= 2)
	{
		sum += ((uint16_t)*buf << 8) | *(buf+1);
		buf += 2;
		len -= 2;
	}

	if(len)
		sum += (uint16_t)*buf << 8;

	while(sum >> 16)
		sum = (sum & 0xffff) + (sum >> 16);

	return ~htons((uint16_t)sum);
}

// send IP packet
// fields must be set:
//	- ip.dst
//	- ip.proto
// len is IP packet payload length
uint8_t ip_send(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *ip = (void*)(frame->data);
	uint32_t route_ip;
	uint8_t *mac_addr_to;

	// apply route
	if( ((ip->to_addr ^ Local_IP) & ip_mask) == 0 )
		route_ip = ip->to_addr;
	else
		route_ip = Remote_IP;

	// resolve mac address
	mac_addr_to = arp_resolve(route_ip);
//		return 0;

	// send packet
	len += sizeof(ip_packet_t);

	memcpy(frame->to_addr, mac_addr_to, 6);
	frame->type = ETH_TYPE_IP;

	ip->ver_head_len = 0x45;
	ip->tos = 0;
	ip->total_len = htons(len);
	ip->fragment_id = 0;
	ip->flags_framgent_offset = 0;
	ip->ttl = IP_PACKET_TTL;
	ip->cksum = 0;
	ip->from_addr = Local_IP;
	ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

	eth_send(frame, len);
	return 1;
}

// send IP packet back
// len is IP packet payload length
void ip_reply(eth_frame_t *frame, uint16_t len)
{
	ip_packet_t *packet = (void*)(frame->data);

	len += sizeof(ip_packet_t);

	packet->total_len = htons(len);
	packet->fragment_id = 0;
	packet->flags_framgent_offset = 0;
	packet->ttl = IP_PACKET_TTL;
	packet->cksum = 0;
	packet->to_addr = packet->from_addr;
	packet->from_addr = Local_IP;
	packet->cksum = ip_cksum(0, (void*)packet, sizeof(ip_packet_t));

	eth_reply((void*)frame, len);
}

// process IP packet
void ip_filter(eth_frame_t *frame, uint16_t len,uint8_t mode)
{
	ip_packet_t *packet = (void*)(frame->data);
	
	//if(len >= sizeof(ip_packet_t))
	//{
	if( (packet->ver_head_len == 0x45) &&
	(packet->to_addr == Local_IP) )
	{
		len = ntohs(packet->total_len) -
		sizeof(ip_packet_t);

		switch(packet->protocol)
		{
	#ifdef WITH_ICMP
		case IP_PROTOCOL_ICMP:
			icmp_filter(frame, len);
		break;
	#endif
		case IP_PROTOCOL_UDP  :
			if(mode==1)udp_filter(frame, len);
			if(mode==0)udp_filter_answer(frame, len);
		break;
		}
	}
	//}
}


/*
 * ARP
 */

// search ARP cache
uint8_t *arp_search_cache(uint32_t node_ip_addr)
{
	uint8_t i;
	for(i = 0; i < ARP_CACHE_SIZE; ++i)
	{
		if(arp_cache[i].ip_addr == node_ip_addr)
			return arp_cache[i].mac_addr;
	}
	return 0;
}

// resolve MAC address
// returns 0 if still resolving
// (invalidates net_buffer if not resolved)
uint8_t *arp_resolve(uint32_t node_ip_addr)
{
	eth_frame_t *frame = (void*)net_buf;
	arp_message_t *msg = (void*)(frame->data);
	uint8_t *mac;

	// search arp cache
	if((mac = arp_search_cache(node_ip_addr)))
		return mac;

	// send request
	memset(frame->to_addr, 0xff, 6);
	frame->type = ETH_TYPE_ARP;

	msg->hw_type = ARP_HW_TYPE_ETH;
	msg->proto_type = ARP_PROTO_TYPE_IP;
	msg->hw_addr_len = 6;
	msg->proto_addr_len = 4;
	msg->type = ARP_TYPE_REQUEST;
	memcpy(msg->mac_addr_from, mac_addr, 6);
	msg->ip_addr_from = Local_IP;
	memset(msg->mac_addr_to, 0x00, 6);
	msg->ip_addr_to = node_ip_addr;

	eth_send(frame, sizeof(arp_message_t));
	return 0;
}

// process arp packet
void arp_filter(eth_frame_t *frame, uint16_t len)
{
	arp_message_t *msg = (void*)(frame->data);

	if(len >= sizeof(arp_message_t))
	{
		if( (msg->hw_type == ARP_HW_TYPE_ETH) &&
			(msg->proto_type == ARP_PROTO_TYPE_IP) &&
			(msg->ip_addr_to == Local_IP) )
		{
			switch(msg->type)
			{
			case ARP_TYPE_REQUEST:
				msg->type = ARP_TYPE_RESPONSE;
				memcpy(msg->mac_addr_to, msg->mac_addr_from, 6);
				memcpy(msg->mac_addr_from, mac_addr, 6);
				msg->ip_addr_to = msg->ip_addr_from;
				msg->ip_addr_from = Local_IP;
				eth_reply(frame, sizeof(arp_message_t));
				break;
			case ARP_TYPE_RESPONSE:
				if(!arp_search_cache(msg->ip_addr_from))
				{
					arp_cache[arp_cache_wr].ip_addr = msg->ip_addr_from;
					memcpy(arp_cache[arp_cache_wr].mac_addr, msg->mac_addr_from, 6);
					arp_cache_wr++;
					if(arp_cache_wr == ARP_CACHE_SIZE)
						arp_cache_wr = 0;
				}
				break;
			}
		}
	}
}


/*
 * Ethernet
 */

// send Ethernet frame
// fields must be set:
//	- frame.to_addr
//	- frame.type
void eth_send(eth_frame_t *frame, uint16_t len)
{
	memcpy(frame->from_addr, mac_addr, 6);
	enc28j60_send_packet((void*)frame, len +
		sizeof(eth_frame_t));
}

// send Ethernet frame back
//Send a response to the Ethernet frame
// (suitable for a server application -
// received a request, swapped the sender and recipient addresses,
// sent back)
void eth_reply(eth_frame_t *frame, uint16_t len)
{

	memcpy(frame->to_addr, frame->from_addr, 6);
	memcpy(frame->from_addr, mac_addr, 6);
	enc28j60_send_packet((void*)frame, len + 
		sizeof(eth_frame_t));
}

// Handler for received Ethernet frames
void eth_filter(eth_frame_t *frame, uint16_t len)
{
	// Check frame length
   //  We will not check the recipient's address,
   //  rely on the ENC28J60 packet filter
	if(len >= sizeof(eth_frame_t))
	{
	switch(frame->type)
	{
	case ETH_TYPE_ARP:
		arp_filter(frame, len - sizeof(eth_frame_t));
		break;
	case ETH_TYPE_IP:
		ip_filter(frame, len - sizeof(eth_frame_t),1);
		break;
	}
	}
}
/*
 * LAN
 */

void lan_init()
{
	enc28j60_init(mac_addr);
}

void lan_echo()
{

	static uint8_t net_buf[512];
	uint16_t len;
	eth_frame_t *frame = (eth_frame_t*)net_buf;

	while((len = enc28j60_recv_packet(net_buf, sizeof(net_buf))))
	{
		    eth_filter(frame, len);

	}
}

void lan_poll(uint8_t* pData,uint16_t* length)
{
	static uint8_t net_buf[512];
	uint16_t len;
	eth_frame_t *frame = (eth_frame_t*)net_buf;

	while((len = enc28j60_recv_packet(net_buf, sizeof(net_buf))))
	{
		if(len >= sizeof(eth_frame_t))
		{
			switch(frame->type)
			{
			case ETH_TYPE_ARP:
				arp_filter(frame, len - sizeof(eth_frame_t));
				break;
			case ETH_TYPE_IP:
				ip_filter(frame, len - sizeof(eth_frame_t),0);
				ip_packet_t *ip = (void*)(frame->data);
				udp_packet_t *udp = (void*)(ip->data);
				*length = ntohs(ip->total_len) -
					sizeof(ip_packet_t)-8;
				////////////////////
				uint8_t myMac[6] ={0};
				memcpy(&myMac,mac_addr,6);
				uint8_t macBroadcast[6]= {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
				uint8_t poort=udp->to_port>>8;

				if(poort!=Local_PORT)
				{
					*length=0;
					len=0;
					break;
				}
				else if (ip->from_addr==ip_dest&&(memcmp(frame->to_addr, myMac, 6) == 0||memcmp(frame->to_addr,macBroadcast, 6) == 0))
				{

					*length = ntohs(ip->total_len)-
					sizeof(ip_packet_t)-8;
					memcpy(pData,udp->data,*length);
					break;
				}
				else
				{
					*length=0;
					len=0;
					break;
				}

			}
		}
	}
}
