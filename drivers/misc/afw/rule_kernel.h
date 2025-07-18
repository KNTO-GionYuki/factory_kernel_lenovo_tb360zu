#ifndef RULE_KERNEL_H
#define RULE_KERNEL_H


#if 0
#define RULE_NUM					(10000 + 100)
#define BITS_PER_LONG				64
#define CBM_SIZE					((RULE_NUM + BITS_PER_LONG - 1) / BITS_PER_LONG)
#define CBM_NUM_MAX					(RULE_NUM * 2)
#endif

#define MAX_PORT					65536
#define MAX_NORMAL_UID				65536

#define MAX_PID						100000

#define TEST_NUM					10000
#define IPV6_ADDR_LEN				16
#define IPV6_RFC_FU16_NUM			1
#define MAX_IPV4_LEN				15				/* xxx.xxx.xxx.xxx */
#define MIN_IPV4_LEN				7				/* x.x.x.x */

#define AFW_HASH_SIZE				(1 << 14)
#define AFW_HASH_MASK				(AFW_HASH_SIZE - 1)


enum {
	TYPE_DPORT,

	TYPE_DIP_H1,
	TYPE_DIP_H2,
	TYPE_DIP_H3,
	TYPE_DIP_H4,

	TYPE_DIP_L1,
	TYPE_DIP_L2,
	TYPE_DIP_L3,
	TYPE_DIP_L4,

	TYPE_MAX,
};

enum {
	RULE_IDX_DNS_START = RULE_NUM,
	RULE_IDX_SPEC_START = 100000,
	RULE_IDX_BLACK_UID = RULE_IDX_SPEC_START,
	RULE_IDX_WHITE_UID,
	RULE_IDX_BLACK_PORT,
	RULE_IDX_WHITE_PORT,
	RULE_IDX_BLACK_HOST,
	RULE_IDX_WHITE_HOST,
	RULE_IDX_PASS_DNS,
	RULE_IDX_PID_CACHE,
	RULE_IDX_MAX,
};


typedef struct _afw_rule {
	unsigned char type;
	unsigned char kind;
	unsigned short res;

	unsigned int rule_idx;
	unsigned int rule_len;
	unsigned int hit_num;
	atomic_t refcnt;

	char *raw_rule;
	unsigned int raw_rule_len;

	unsigned int uid;

	range_u32_st ip;
	range_ipv6_st ipv6;

	char *domain;
	unsigned short domain_len;
	unsigned char domain_any;
	unsigned char domain_level;

	range_u16_st *port;
	unsigned int port_num;

	unsigned short dns_ipv4_num;
	unsigned short new_dns_ipv4_idx;
	unsigned int *dns_ipv4;

	unsigned short dns_ipv6_num;
	unsigned short new_dns_ipv6_idx;
	unsigned char *dns_ipv6;

	struct _afw_rule *next;

	char data[0];
}afw_rule_st;

typedef struct _key {
    unsigned int ip;
    unsigned short port;
}key_st;

typedef struct _afw_dns_node {
	unsigned int domain_len;
	unsigned char domain[MAX_DOMAIN_LEN];

	unsigned short num;
	unsigned short new_idx;
	unsigned int ipv4[MAX_DNS_ADDR];

	unsigned short num_ipv6;
	unsigned short new_idx_ipv6;
	unsigned char ipv6[MAX_DNS_ADDR][IPV6_ADDR_LEN];

	struct list_head next;
}afw_dns_node_st;


extern afw_rule_st *black_rule[RULE_NUM];
extern unsigned int black_rule_num;
extern unsigned int black_rule_max_len;
extern afw_rule_st *white_rule[RULE_NUM];
extern unsigned int white_rule_num;
extern unsigned int white_rule_max_len;

extern afw_rule_st *black_rule_ipv4[RULE_NUM];
extern unsigned int black_rule_ipv4_num;
extern afw_rule_st *white_rule_ipv4[RULE_NUM];
extern unsigned int white_rule_ipv4_num;
extern rfc_rule_st rfc_black_rule_ipv4;
extern rfc_rule_st rfc_white_rule_ipv4;

extern afw_rule_st *black_rule_ipv6[RULE_NUM];
extern unsigned int black_rule_ipv6_num;
extern afw_rule_st *white_rule_ipv6[RULE_NUM];
extern unsigned int white_rule_ipv6_num;
extern rfc_rule_st rfc_black_rule_ipv6;
extern rfc_rule_st rfc_white_rule_ipv6;

extern afw_rule_st *black_rule_domain[RULE_NUM];
extern unsigned int black_rule_domain_num;
extern afw_rule_st *white_rule_domain[RULE_NUM];
extern unsigned int white_rule_domain_num;
extern afw_rule_st *black_rule_domain_hash[AFW_HASH_SIZE];
extern afw_rule_st *white_rule_domain_hash[AFW_HASH_SIZE];

extern unsigned char black_rule_uid[MAX_NORMAL_UID];
extern unsigned int black_rule_uid_large[RULE_NUM];
extern unsigned int black_rule_uid_num;
extern unsigned int black_rule_uid_large_num;
extern unsigned char white_rule_uid[MAX_NORMAL_UID];
extern unsigned int white_rule_uid_large[RULE_NUM];
extern unsigned int white_rule_uid_num;
extern unsigned int white_rule_uid_large_num;

extern unsigned char def_black_port_rule[MAX_PORT];
extern unsigned int black_rule_port_num;
extern unsigned char def_white_port_rule[MAX_PORT];
extern unsigned int white_rule_port_num;

extern unsigned int def_host_rule;
extern unsigned int black_rule_host_num;
extern unsigned int white_rule_host_num;

extern struct kmem_cache *dns_cache;
extern struct list_head dns_node_hash[AFW_HASH_SIZE];
extern unsigned int afw_dns_node_num;

extern afw_rule_st *black_rule_dns[RULE_NUM];
extern unsigned int black_rule_dns_num;
extern afw_rule_st *white_rule_dns[RULE_NUM];
extern unsigned int white_rule_dns_num;

extern afw_rule_st *black_rule_dns_ipv6[RULE_NUM];
extern unsigned int black_rule_dns_ipv6_num;
extern afw_rule_st *white_rule_dns_ipv6[RULE_NUM];
extern unsigned int white_rule_dns_ipv6_num;

extern unsigned char pid_cache[MAX_PID];
extern unsigned long pid_jiffies[MAX_PID];


extern int afw_add_dns_node(char *domain, int domain_len, unsigned int *ipv4, int num);
extern int afw_add_dns_node_ipv6(char *domain, int domain_len, unsigned char ipv6[][IPV6_ADDR_LEN], int num);

extern int afw_rule_init(void);
extern void afw_rule_exit(void);

extern int afw_add_rule(afw_req_st *req);
extern int afw_get_rule_num(afw_req_st *req);
extern int afw_get_rule(afw_req_st *req);
extern int afw_flush_rule(afw_req_st *req);
extern int afw_test_rule(afw_req_st *req);

extern int afw_match_ipv4(unsigned int dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, 
	unsigned short dport_domain, unsigned int pkt_type, unsigned int uid);
extern int afw_match_ipv6(unsigned char *dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, 
	unsigned short dport_domain, unsigned int pkt_type, unsigned int uid);


#endif


