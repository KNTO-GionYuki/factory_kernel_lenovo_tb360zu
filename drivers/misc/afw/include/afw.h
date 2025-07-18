#ifndef AFW_H
#define AFW_H
#include "args.h"


#define AFW_DEV_NAME		"afw"
#define AFW_DEV_MAJOR		137


#define AFW_ALIGN_4(len) (((len) + 3) & ~3)


enum {
	AFW_RULE_TYPE_BLACK,
	AFW_RULE_TYPE_WHITE,
	AFW_RULE_TYPE_AFW,
	AFW_RULE_TYPE_PARSE_INPUT,
	AFW_RULE_TYPE_PASS_DNS,
	AFW_RULE_TYPE_PID_CACHE,
	AFW_RULE_TYPE_TCP_RESET,
	AFW_RULE_TYPE_ALL,
	AFW_RULE_TYPE_MAX,
};

enum {
	AFW_RULE_KIND_DEF_HOST,
	AFW_RULE_KIND_DEF_PORT,
	AFW_RULE_KIND_IPV4,
	AFW_RULE_KIND_IPV6,
	AFW_RULE_KIND_DOMAIN,
	AFW_RULE_KIND_UID,
	AFW_RULE_KIND_ALL,
	AFW_RULE_KIND_MAX,
};

enum {
	AFW_CMD_SET_AFW_ENABLE,
	AFW_CMD_GET_AFW_ENABLE,
	AFW_CMD_SET_BLACK_ENABLE,
	AFW_CMD_GET_BLACK_ENABLE,
	AFW_CMD_SET_WHITE_ENABLE,
	AFW_CMD_GET_WHITE_ENABLE,
	AFW_CMD_SET_PARSE_INPUT_ENABLE,
	AFW_CMD_GET_PARSE_INPUT_ENABLE,

	AFW_CMD_SET_PASS_DNS_ENABLE,
	AFW_CMD_GET_PASS_DNS_ENABLE,
	AFW_CMD_SET_PID_CACHE_ENABLE,
	AFW_CMD_GET_PID_CACHE_ENABLE,
	AFW_CMD_SET_TCP_RESET_ENABLE,
	AFW_CMD_GET_TCP_RESET_ENABLE,

	AFW_CMD_ADD_RULE,
	AFW_CMD_GET_RULE_NUM,
	AFW_CMD_GET_RULE,
	AFW_CMD_FLUSH_RULE,
	AFW_CMD_TEST_RULE,

	AFW_CMD_SET_PID_TIMEOUT,
	AFW_CMD_GET_PID_TIMEOUT,

	AFW_CMD_MAX,
};


typedef struct _afw_req {
	unsigned char cmd;
	unsigned char type;					/* black, white */
	unsigned char kind;					/* host, port, ipv4, ipv6, domain */
	unsigned char enable;

	char *raw_rule;
	unsigned int raw_rule_len;

	unsigned int uid;
	unsigned int timeout;

	range_u32_st ip;
	range_ipv6_st ipv6;

	char *domain;
	unsigned short domain_len;
	unsigned char domain_any;
	unsigned char domain_level;

	range_u16_st *port;
	unsigned int port_num;

	unsigned int rule_num;
	unsigned int max_rule_len;
	char *data;
}afw_req_st;


#endif


