/**
 * mdns.c
 * This provides mDNS related functionality.
 *
 */

/**************************************************************************************************************
 * INCLUDES
 **************************************************************************************************************/
#include "lwip/opt.h"
#include "lwip/udp.h"
#include "lwip/def.h"
#include "lwip/memp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "arch/perf.h"
#include "lwip/udp.h"
#include "lwip/mdns.h"

#include <stdlib.h>
#include <string.h>
#include "wiced_network.h"

/**************************************************************************************************************
 * CONSTANTS
 **************************************************************************************************************/
#define SERVICE_QUERY_NAME      "_services._dns-sd._udp.local."
#define SERVICE_HTTP      "_easylink._tcp.local."

#define MDNS_DEFAULT_TTL 300  // this is the default value
//#define MDNS_DEFAULT_TTL 30 //

#define DNS_MESSAGE_IS_A_RESPONSE           0x8000
#define DNS_MESSAGE_OPCODE                  0x7800
#define DNS_MESSAGE_AUTHORITATIVE           0x0400
#define DNS_MESSAGE_TRUNCATION              0x0200
#define DNS_MESSAGE_RECURSION_DESIRED       0x0100
#define DNS_MESSAGE_RECURSION_AVAILABLE     0x0080
#define DNS_MESSAGE_RESPONSE_CODE           0x000F

typedef enum
{
    DNS_NO_ERROR        = 0,
    DNS_FORMAT_ERROR    = 1,
    DNS_SERVER_FAILURE  = 2,
    DNS_NAME_ERROR      = 3,
    DNS_NOT_IMPLEMENTED = 4,
    DNS_REFUSED         = 5
} dns_message_response_code_t;

typedef enum
{
    RR_TYPE_A      = 1,     // A - Host Address
    RR_TYPE_NS     = 2,
    RR_TYPE_MD     = 3,
    RR_TYPE_MF     = 4,
    RR_TYPE_CNAME  = 5,
    RR_TYPE_SOA    = 6,
    RR_TYPE_MB     = 7,
    RR_TYPE_MG     = 8,
    RR_TYPE_MR     = 9,
    RR_TYPE_NULL   = 10,
    RR_TYPE_WKS    = 11,
    RR_TYPE_PTR    = 12,    // PTR - Domain Name pointer
    RR_TYPE_HINFO  = 13,    // HINFO - Host Information
    RR_TYPE_MINFO  = 14,
    RR_TYPE_MX     = 15,
    RR_TYPE_TXT    = 16,
    RR_TYPE_SRV    = 33,    // SRV - Service Location Record
    RR_QTYPE_AXFR  = 252,
    RR_QTYPE_MAILB = 253,
    RR_QTYPE_AILA  = 254,
    RR_QTYPE_ANY   = 255
} dns_resource_record_type_t;

typedef enum
{
    RR_CLASS_IN  = 1,
    RR_CLASS_CS  = 2,
    RR_CLASS_CH  = 3,
    RR_CLASS_HS  = 4,
    RR_CLASS_ALL = 255
} dns_resource_record_class_t;

#define RR_CACHE_FLUSH   0x8000

/**************************************************************************************************************
 * STRUCTURES
 **************************************************************************************************************/

typedef struct
{
    uint8_t* start_of_name;
    uint8_t* start_of_packet; // Used for compressed names;
} dns_name_t;

typedef struct
{
    uint16_t id;
    uint16_t flags;
    uint16_t question_count;
    uint16_t answer_count;
    uint16_t name_server_count;
    uint16_t additional_record_count;
} dns_message_header_t;

typedef struct
{
    dns_message_header_t* header; // Also used as start of packet for compressed names
    uint8_t* iter;
    uint8_t* end;
} dns_message_iterator_t;

typedef struct
{
    uint16_t question_type;
    uint16_t question_class;
} dns_question_t;

typedef struct
{
    uint16_t record_type;
    uint16_t record_class;
    uint32_t ttl;
    uint16_t rd_length;
    dns_message_iterator_t rdata;
} dns_record_t;

typedef struct
{
	char* hostname;
	char* instance_name;
	char* service_name;
    char* txt_att;
	uint16_t	port;
	char		instance_name_suffix[4]; // This variable should only be modified by the DNS-SD library
} dns_sd_service_record_t;

/**************************************************************************************************************
 * FUNCTION DECLARATIONS
 **************************************************************************************************************/

/**************************************************************************************************************
 * VARIABLES
 **************************************************************************************************************/
static dns_sd_service_record_t*   available_services	 = NULL;
static uint8_t					  available_service_count;

static int dns_create_message( dns_message_iterator_t* message, uint16_t size )
{
	message->header = (dns_message_header_t*) malloc( size );
	if ( message->header == NULL )
	{
		return 0;
	}

	message->iter = (uint8_t*) message->header + sizeof(dns_message_header_t);
    return 1;
}

static void dns_free_message( dns_message_iterator_t* message )
{
	free(message->header);
	message->header = NULL;
}

static void _dns_write_string( dns_message_iterator_t* iter, const char* src )
{
	uint8_t* segment_length_pointer;
	uint8_t  segment_length=0;

	while ( *src != 0 && (unsigned char)*src != 0xC0)
	{
		/* Remember where we need to store the segment length and reset the counter*/
		segment_length_pointer = iter->iter++;
		segment_length = 0;

        
        
		/* Copy bytes until '.' or end of string*/
		while ( *src != '.' && *src != 0 && (unsigned char)*src != 0xC0)
		{
		    if (*src == '/')
                src++; // skip '/'
           
			*iter->iter++ = *src++;
			++segment_length;
		}
        
		/* Store the length of the segment*/
		*segment_length_pointer = segment_length;

		/* Check if we stopped because of a '.', if so, skip it*/
		if ( *src == '.' )
		{
			++src;
		}
        
	}

    if ((unsigned char)*src == 0xC0) { // compress name
        *iter->iter++ = *src++;
        *iter->iter++ = *src++;
    } else {
        /* Add the ending null */
	    *iter->iter++ = 0;
    }
}

static void _dns_write_name( dns_message_iterator_t* iter, const char* src )
{
	_dns_write_string( iter, src );
}

static void _dns_write_uint16( dns_message_iterator_t* iter, uint16_t data )
{
	// We cannot assume the uint8_t alignment of iter->iter so we can't just typecast and assign
	iter->iter[0] = data >> 8;
	iter->iter[1] = data & 0xFF;
	iter->iter += 2;
}

static void _dns_write_uint32( dns_message_iterator_t* iter, uint32_t data )
{
	iter->iter[0] = data >> 24;
	iter->iter[1] = data >> 16;
	iter->iter[2] = data >> 8;
	iter->iter[3] = data & 0xFF;
	iter->iter += 4;
}

static void _dns_write_bytes( dns_message_iterator_t* iter, uint8_t* data, uint16_t length )
{
	int a = 0;

	for ( a = 0; a < length; ++a )
	{
		iter->iter[a] = data[a];
	}
	iter->iter += length;
}

static uint16_t _dns_read_uint16( dns_message_iterator_t* iter )
{
	uint16_t temp = (uint16_t) ( *iter->iter++ ) << 8;
	temp += (uint16_t) ( *iter->iter++ );
	return temp;
}

static void _dns_skip_name( dns_message_iterator_t* iter )
{
	while ( *iter->iter != 0 )
	{
		// Check if the name is compressed
		if ( *iter->iter & 0xC0 )
		{
			iter->iter += 1; // Normally this should be 2, but we have a ++ outside the while loop
			break;
		}
		else
		{
			iter->iter += (uint32_t) *iter->iter + 1;
		}
        
        if (iter->iter > iter->end)
            break;
	}
	// Skip the null uint8_t
	++iter->iter;
}

static int _dns_get_next_question( dns_message_iterator_t* iter, dns_question_t* q, dns_name_t* name )
{
	// Set the name pointers and then skip it
	name->start_of_name   = (uint8_t*) iter->iter;
	name->start_of_packet = (uint8_t*) iter->header;
	_dns_skip_name( iter );

    if (iter->iter > iter->end)
        return 0;
    
	// Read the type and class
	q->question_type  = _dns_read_uint16( iter );
	q->question_class = _dns_read_uint16( iter );
    return 1;
}

static int _dns_compare_name_to_string( dns_name_t* name, const char* string )
{
	uint8_t section_length;
	int finished = 0;
	int result   = 1;
	uint8_t* buffer 	  = name->start_of_name;

	while ( !finished )
	{
		// Check if the name is compressed. If so, find the uncompressed version
		while ( *buffer & 0xC0 )
		{
			uint16_t offset = ( *buffer++ ) << 8;
			offset += *buffer;
			offset &= 0x3FFF;
			buffer = name->start_of_packet + offset;
		}

		// Compare section
		section_length = *( buffer++ );
		if ( strncmp( (char*) buffer, string, section_length ) )
		{
			result	 = 0;
			finished = 1;
		}
		string += section_length + 1;
		buffer += section_length;

		// Check if we've finished comparing
		if ( *buffer == 0 || *string == 0 )
		{
			finished = 1;
			// Check if one of the strings has more data
			if ( *buffer != 0 || *string != 0 )
			{
				result = 0;
			}
		}
	}

	return result;
}

static void _dns_write_header( dns_message_iterator_t* iter, uint16_t id, uint16_t flags, uint16_t question_count, uint16_t answer_count, uint16_t authorative_count )
{
	memset( iter->header, 0, sizeof(dns_message_header_t) );
	iter->header->id				= htons(id);
	iter->header->flags 			= htons(flags);
	iter->header->question_count	= htons(question_count);
	iter->header->name_server_count = htons(authorative_count);
	iter->header->answer_count		= htons(answer_count);
}

static void _dns_write_record( dns_message_iterator_t* iter, const char* name, uint16_t record_class, uint16_t record_type, uint32_t ttl, uint8_t* rdata )
{
	uint8_t* rd_length;
	uint8_t* temp_ptr;

	/* Write the name, type, class, TTL*/
	_dns_write_name	( iter, name );
	_dns_write_uint16( iter, record_type );
	_dns_write_uint16( iter, record_class );
	_dns_write_uint32( iter, ttl );

	/* Keep track of where we store the rdata length*/
	rd_length	= iter->iter;
	iter->iter += 2;
	temp_ptr	= iter->iter;

	switch ( record_type )
	{
		case RR_TYPE_A:
			_dns_write_bytes( iter, rdata, 4 );
			break;
			
		case RR_TYPE_PTR:
		case RR_TYPE_TXT:
			_dns_write_name( iter, (const char*) rdata );
			break;

		case RR_TYPE_SRV:
			/* Set priority and weight to 0*/
			_dns_write_uint16( iter, 0 );
			_dns_write_uint16( iter, 0 );

			/* Write the port*/
			_dns_write_uint16( iter, ( (dns_sd_service_record_t*) rdata )->port );

			/* Write the hostname*/
			_dns_write_string( iter, ( (dns_sd_service_record_t*) rdata )->hostname );
			break;
		default:
			break;
	}
	// Write the rdata length
	rd_length[0] = ( iter->iter - temp_ptr ) >> 8;
	rd_length[1] = ( iter->iter - temp_ptr ) & 0xFF;
}

static void register_services(char *instance_name, char *hostname, char *txt_att, uint16_t port)
{
    int len;
    
	available_service_count = 1;
	available_services = (void *)malloc(sizeof(dns_sd_service_record_t) * 1);

	available_services->hostname = (char*)strdup(hostname);
	available_services->instance_name = (char*)strdup(instance_name);
    len = strlen(instance_name);
    available_services->instance_name = (char*)malloc(len+3);//// 0xc00c+\0
    memcpy(available_services->instance_name, instance_name, len);
    available_services->instance_name[len]= 0xc0;
    available_services->instance_name[len+1]= 0x0c;
    available_services->instance_name[len+2]= 0;
	available_services->service_name = SERVICE_HTTP;
    available_services->txt_att = (char*)strdup(txt_att);
	available_services->port = port;
}
static void mdns_send_message(struct netif *iface, dns_message_iterator_t* message )
{
    uint8_t *data = (uint8_t *)message->header;
    int len = (int)( (int)message->iter - (int)message->header );
    struct pbuf *r;
    ip_addr_t dst;
    ip_addr_t src  = *IP_ADDR_ANY;
    struct udp_hdr *udphdr;
    
    
      /* allocate new packet buffer with space for link headers */
      r = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
      if (r == NULL) {
        return;
      }

    memcpy(r->payload, data, len);
    pbuf_header(r, sizeof(struct udp_hdr));
    udphdr = (struct udp_hdr *)r->payload;
    udphdr->src = udphdr->dest = htons(MDNS_PORT);
    udphdr->len = htons(len+sizeof(struct udp_hdr));
    udphdr->chksum = 0;

    dst.addr = (MDNS_ADDR); // send to MDNS ip address
    ip_output_if(r, &src, &dst, 255, 0, IP_PROTO_UDP, iface);
    dst.addr = (0xFFFFFFFF); // send to broadcast address
    ip_output_if(r, &src, &dst, 255, 0, IP_PROTO_UDP, iface);
    
    pbuf_free(r);

}

static void mdns_process_query(struct netif * iface, dns_name_t* name, 
                               dns_question_t* question, dns_message_iterator_t* source )
{
    dns_message_iterator_t response;

    memset( &response, 0, sizeof(dns_message_iterator_t) );

    switch ( question->question_type )
    {
        case RR_QTYPE_ANY:
        case RR_TYPE_A:
            if ( _dns_compare_name_to_string( name, available_services->hostname ) )
            {
            	uint32_t myip = (iface->ip_addr.addr);
				
                if (dns_create_message( &response, 256 ) == 1) {
                    _dns_write_header( &response, source->header->id, 0x8400, 0, 1, 0 );
                    _dns_write_record( &response, available_services->hostname, RR_CLASS_IN | RR_CACHE_FLUSH, RR_TYPE_A, MDNS_DEFAULT_TTL, (uint8_t*)&myip);
                    mdns_send_message(iface, &response );
                    dns_free_message( &response );
                }
            }

            break;
    }

}

static void process_dns_questions(struct netif *iface, dns_message_iterator_t* iter )
{
    dns_name_t name;
    dns_question_t question;
    dns_message_iterator_t response;
    int a = 0;
    int question_processed;

    memset( &response, 0, sizeof(dns_message_iterator_t) );

    for ( a = 0; a < htons(iter->header->question_count); ++a )
    {
        if (iter->iter > iter->end)
            break;
        
        if (_dns_get_next_question( iter, &question, &name ) == 0)
            break;
        
        question_processed = 0;
        switch ( question.question_type )
        {
            case RR_TYPE_PTR:
                if ( available_services != NULL )
                {
                    // Check if its a query for all available services
                    if ( _dns_compare_name_to_string( &name, SERVICE_QUERY_NAME ) )
                    {
                    	int b = 0;
						
                        if (dns_create_message( &response, 512 ) == 1) {
    						_dns_write_header(&response, iter->header->id, 0x8400, 0, 
    										 available_service_count, 0 );
                            
                            for ( b = 0; b < available_service_count; ++b )
                            {
                                _dns_write_record( &response, SERVICE_QUERY_NAME, RR_CLASS_IN, RR_TYPE_PTR, MDNS_DEFAULT_TTL, (uint8_t*) available_services[b].service_name );
                            }
                            mdns_send_message(iface, &response );
                            dns_free_message( &response );

                            question_processed = 1;
                        }
                    }
                    // else check if its one of our records
                    else
                    {
                        int b = 0;
                        for ( b = 0; b < available_service_count; ++b )
                        {
                            if ( _dns_compare_name_to_string( &name, available_services[b].service_name ) )
                            {
                            	uint32_t myip = (iface->ip_addr.addr);
                                // Send the PTR, TXT, SRV and A records
                                if (dns_create_message( &response, 512 ) == 1) {
                                    _dns_write_header( &response, iter->header->id, 0x8400, 0, 4, 0 );
                                    _dns_write_record( &response, available_services[b].service_name, RR_CLASS_IN, RR_TYPE_PTR, MDNS_DEFAULT_TTL, (uint8_t*) available_services[b].instance_name );
                                    _dns_write_record( &response, available_services[b].instance_name, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_TXT, MDNS_DEFAULT_TTL, (uint8_t*) available_services->txt_att );
                                    _dns_write_record( &response, available_services[b].instance_name, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_SRV, MDNS_DEFAULT_TTL, (uint8_t*) &available_services[b]);
                                    //
                                    _dns_write_record( &response, available_services[b].hostname, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_A, MDNS_DEFAULT_TTL, (uint8_t*) &myip);
                                    mdns_send_message(iface, &response );
                                    dns_free_message( &response );
                                    question_processed = 1;
                                }
                            }
                        }
                    }
                }
                break;
        }

        if ( !question_processed )
        {
            mdns_process_query(iface, &name, &question, iter );
        }
    }
}

void mdns_handler(struct netif *iface, uint8_t* pkt, int pkt_len)
{
    dns_message_iterator_t iter;

    iter.header = (dns_message_header_t*) pkt;
    iter.iter   = (uint8_t*) iter.header + sizeof(dns_message_header_t);
    iter.end = pkt + pkt_len;
    // Check if the message is a response (otherwise its a query)
    if ( ntohs(iter.header->flags) & DNS_MESSAGE_IS_A_RESPONSE )
    {
        return ;
    }
    else
    {
        
        process_dns_questions(iface, &iter);
    }
}

// Join IGMP group, add supported mdns type
void init_mdns(char *instance_name, char *hostname, char *txt_att, uint16_t port)
{
	register_services(instance_name, hostname, txt_att, port);
}
#define msleep wiced_rtos_delay_milliseconds
//extern void msleep(unsigned int ms);
void send_mdns_up(void)
{
    dns_message_iterator_t response;
    int b = 0;
    struct netif *iface = &IP_HANDLE(WICED_STA_INTERFACE);
	uint32_t myip = (iface->ip_addr.addr);

    // Send the PTR, TXT, SRV and A records
    if (dns_create_message( &response, 512 ) == 0)
        return;
    
    _dns_write_header( &response, 0, 0x8400, 0, 4, 0 );
    _dns_write_record( &response, available_services[b].service_name, RR_CLASS_IN, RR_TYPE_PTR, MDNS_DEFAULT_TTL, (uint8_t*) available_services[b].instance_name );
    _dns_write_record( &response, available_services[b].instance_name, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_TXT, MDNS_DEFAULT_TTL, (uint8_t*) available_services->txt_att );
    _dns_write_record( &response, available_services[b].instance_name, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_SRV, MDNS_DEFAULT_TTL, (uint8_t*) &available_services[b]);
    _dns_write_record( &response, available_services[b].hostname, RR_CACHE_FLUSH|RR_CLASS_IN, RR_TYPE_A, MDNS_DEFAULT_TTL, (uint8_t*) &myip);

    for(b=0; b<5; b++) {
        mdns_send_message(iface, &response );
        msleep(10);
    }
    dns_free_message( &response );

}
extern void send_arikiss_rpt(char value);

void send_arikiss_rpt(char value)
{
    int i = 0;
    struct pbuf *r, q;
    ip_addr_t dst;
    ip_addr_t src  = *IP_ADDR_ANY;
    struct udp_hdr *udphdr;
    struct netif *iface = &IP_HANDLE(WICED_STA_INTERFACE);
    
      /* allocate new packet buffer with space for link headers */
      r = pbuf_alloc(PBUF_TRANSPORT, 1, PBUF_RAM);
      if (r == NULL) {
        return;
      }

    memcpy(r->payload, &value, 1);
    pbuf_header(r, sizeof(struct udp_hdr));
    udphdr = (struct udp_hdr *)r->payload;
    udphdr->src = udphdr->dest = htons(10000);
    udphdr->len = htons(1+sizeof(struct udp_hdr));
    udphdr->chksum = 0;

    dst.addr = (0xFFFFFFFF); // send to broadcast address
    memcpy(&q, r, sizeof(struct pbuf));
    while(1) {
        memcpy(r, &q, sizeof(struct pbuf));
        ip_output_if(r, &src, &dst, 255, 0, IP_PROTO_UDP, iface);
        msleep(100);
        i++;
        if (i == 20)
            break;
    }
    pbuf_free(r);
}

