#ifndef AFW_KERNEL_H
#define AFW_KERNEL_H


#define HTTP_CMD_GET				htonl(('G' << 24) | ('E' << 16) | ('T' << 8) | ' ')
#define HTTP_CMD_GET_LOWER			htonl(('g' << 24) | ('e' << 16) | ('t' << 8) | ' ')

#define HTTP_CMD_POST				htonl(('P' << 24) | ('O' << 16) | ('S' << 8) | 'T')
#define HTTP_CMD_POST_LOWER			htonl(('p' << 24) | ('o' << 16) | ('s' << 8) | 't')

#define HTTP_CMD_CONNECT			htonl(('C' << 24) | ('O' << 16) | ('N' << 8) | 'N')
#define HTTP_CMD_CONNECT_LOWER		htonl(('c' << 24) | ('o' << 16) | ('n' << 8) | 'n')

#define HTTP_CMD_PUT				htonl(('P' << 24) | ('U' << 16) | ('T' << 8) | ' ')
#define HTTP_CMD_PUT_LOWER			htonl(('p' << 24) | ('u' << 16) | ('t' << 8) | ' ')

#define DNS_PORT					53
#define HTTP_PORT					80
#define HTTPS_PORT					443
#define HTTP_PROXY					8080

#define MAX_DOMAIN_LEN				256
#define MAX_DNS_ADDR				16
#define MAX_RULE_DNS_ADDR			256
#define MAX_RULE_DNS_ADDR_LEN		(MAX_RULE_DNS_ADDR * sizeof(unsigned int))

#define MAX_RULE_DNS_ADDR6			64
#define MAX_RULE_DNS_ADDR6_LEN		(MAX_RULE_DNS_ADDR6 * IPV6_ADDR_LEN)

#define MAX_DNS_NODE_NUM			10000
#define DEF_HTTP_PORT				80

#define AFW_PKT_LOG_NUM				1000
#define MAX_LOG_DOMAIN_LEN			32

#define MAX_DEBUG_PKT_NUM			16
#define MAX_DEBUG_PKT_LEN			2048
#define DNS_COMPRESS_FLAG			0xc0


enum {
	AFW_PKT_TYPE_IPV4,
	AFW_PKT_TYPE_IPV4_HTTP,
	AFW_PKT_TYPE_IPV4_HTTP_PROXY,
	AFW_PKT_TYPE_IPV4_DNS,
	AFW_PKT_TYPE_IPV6,
	AFW_PKT_TYPE_IPV6_HTTP,
	AFW_PKT_TYPE_IPV6_HTTP_PROXY,
	AFW_PKT_TYPE_IPV6_DNS,
	AFW_PKT_TYPE_MAX,
};

enum {
	AFW_PKT_ACTION_STOP,
	AFW_PKT_ACTION_ACCEPT,
	AFW_PKT_ACTION_MAX,
};


typedef struct _afw_conf {
	unsigned char afw_enable;
	unsigned char black_enable;
	unsigned char white_enable;
	unsigned char parse_input;

	unsigned char pass_dns;
	unsigned char pid_cache;
	unsigned short pid_tmout;

	unsigned char tcp_reset;
}afw_conf_st;

typedef struct _afw_stat {
	/* stats of dynamic malloc memory */
	unsigned int malloc_rule;
	unsigned int malloc_rfc_field;
	unsigned int malloc_rfc_cbm;
	unsigned int malloc_dns_node;
	unsigned int malloc_dns_addr;

	unsigned int parse_dns_req_failed;
	unsigned int parse_dns_rep_failed;
	unsigned int parse_dns_no_answer;
	unsigned int parse_dns_no_answer2;
	unsigned int dns_addr_too_much;
	unsigned int dns_rule_ipv4_full;
	unsigned int dns_node_ipv4_full;
	unsigned int dns_rule_ipv6_full;
	unsigned int dns_node_ipv6_full;
	unsigned int dns_node_reach_limit;
	unsigned int tcp_reset_num;

	/* stats of input ipv4 pkt */
	unsigned long ipv4_input_total_num;
	unsigned long ipv4_input_total_len;
	unsigned long ipv4_input_nonlinear;
	unsigned long ipv4_input_tcp_num;
	unsigned long ipv4_input_udp_num;
	unsigned long ipv4_input_other_num;
	unsigned long ipv4_input_dns_num;

	/* stats of input ipv6 pkt */
	unsigned long ipv6_input_total_num;
	unsigned long ipv6_input_total_len;
	unsigned long ipv6_input_nonlinear;
	unsigned long ipv6_input_tcp_num;
	unsigned long ipv6_input_udp_num;
	unsigned long ipv6_input_other_num;
	unsigned long ipv6_input_dns_num;

	/* stats of output ipv4 pkt num */
	unsigned long ipv4_total_num;
	unsigned long ipv4_invalid_num;
	unsigned long ipv4_tcp_num;
	unsigned long ipv4_udp_num;
	unsigned long ipv4_other_num;
	unsigned long ipv4_http_num;
	unsigned long ipv4_http_proxy_num;
	unsigned long ipv4_dns_num;

	/* stats of output ipv4 pkt len */
	unsigned long ipv4_total_len;
	unsigned long ipv4_tcp_len;
	unsigned long ipv4_udp_len;
	unsigned long ipv4_other_len;
	unsigned long ipv4_http_len;
	unsigned long ipv4_http_proxy_len;
	unsigned long ipv4_dns_len;

	/* stats of output ipv6 pkt num */
	unsigned long ipv6_total_num;
	unsigned long ipv6_invalid_num;
	unsigned long ipv6_tcp_num;
	unsigned long ipv6_udp_num;
	unsigned long ipv6_other_num;
	unsigned long ipv6_http_num;
	unsigned long ipv6_http_proxy_num;
	unsigned long ipv6_dns_num;

	/* stats of output ipv6 pkt len */
	unsigned long ipv6_total_len;
	unsigned long ipv6_tcp_len;
	unsigned long ipv6_udp_len;
	unsigned long ipv6_other_len;
	unsigned long ipv6_http_len;
	unsigned long ipv6_http_proxy_len;
	unsigned long ipv6_dns_len;

	/* stats of ipv4 rule hit */
	unsigned long ipv4_black_rule_uid_hit;
	unsigned long ipv4_black_rule_ipv4_hit;
	unsigned long ipv4_black_rule_dns_hit;
	unsigned long ipv4_black_rule_domain_hit;
	unsigned long ipv4_black_rule_port_hit;
	unsigned long ipv4_black_rule_host_hit;
	unsigned long ipv4_white_rule_uid_hit;
	unsigned long ipv4_white_rule_ipv4_hit;
	unsigned long ipv4_white_rule_dns_hit;
	unsigned long ipv4_white_rule_domain_hit;
	unsigned long ipv4_white_rule_port_hit;
	unsigned long ipv4_white_rule_host_hit;

	unsigned long ipv4_pass_dns_hit;
	unsigned long ipv4_pid_cache_hit;

	/* stats of ipv6 rule hit */
	unsigned long ipv6_black_rule_uid_hit;
	unsigned long ipv6_black_rule_ipv6_hit;
	unsigned long ipv6_black_rule_dns_hit;
	unsigned long ipv6_black_rule_domain_hit;
	unsigned long ipv6_black_rule_port_hit;
	unsigned long ipv6_black_rule_host_hit;
	unsigned long ipv6_white_rule_uid_hit;
	unsigned long ipv6_white_rule_ipv6_hit;
	unsigned long ipv6_white_rule_dns_hit;
	unsigned long ipv6_white_rule_domain_hit;
	unsigned long ipv6_white_rule_port_hit;
	unsigned long ipv6_white_rule_host_hit;

	unsigned long ipv6_pass_dns_hit;
	unsigned long ipv6_pid_cache_hit;
}afw_stat_st;

typedef struct _afw_pkt_log {
	unsigned char dip[16];

	unsigned short dport;
	unsigned char action;
	unsigned char pkt_type;

	unsigned int uid;
	unsigned int rule_idx;
	unsigned long jiffies;

	char domain[MAX_LOG_DOMAIN_LEN];
	unsigned char domain_len;
	unsigned char proto;
	unsigned short dport_domain;

	unsigned char comm[16];
	unsigned int pid;
}afw_pkt_log_st;

typedef struct _afw_dns_header {
	unsigned short id;
	unsigned short flags;
	unsigned short query_num;
	unsigned short answer_num;
	unsigned short auth_num;
	unsigned short extra_num;
}__attribute__((packed)) afw_dns_header_st;

typedef struct _afw_dns_answer {
	//unsigned short name;
	unsigned short type;
	unsigned short class;
	unsigned int time;
	unsigned short len;
}__attribute__((packed)) afw_dns_answer_st;

typedef struct _afw_debug {
	unsigned char pkt[MAX_DEBUG_PKT_NUM][MAX_DEBUG_PKT_LEN];
	unsigned int pkt_len[MAX_DEBUG_PKT_NUM];
	unsigned int pkt_num;
}afw_debug_st;

extern afw_conf_st afw_conf;
extern afw_stat_st afw_stat;
extern afw_debug_st afw_debug;
extern afw_pkt_log_st afw_pkt_log[AFW_PKT_LOG_NUM];
extern unsigned int afw_pkt_log_num;


#endif
