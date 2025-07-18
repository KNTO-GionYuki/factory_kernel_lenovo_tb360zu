#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/netfilter.h>
#include <linux/jhash.h>

#include "include/afw.h"
#include "rfc.h"
#include "afw_kernel.h"
#include "rule_kernel.h"


DEFINE_RWLOCK(afw_lock);


afw_rule_st *black_rule[RULE_NUM];
unsigned int black_rule_num;
unsigned int black_rule_max_len;
afw_rule_st *white_rule[RULE_NUM];
unsigned int white_rule_num;
unsigned int white_rule_max_len;

afw_rule_st *black_rule_ipv4[RULE_NUM];
unsigned int black_rule_ipv4_num;
afw_rule_st *white_rule_ipv4[RULE_NUM];
unsigned int white_rule_ipv4_num;
rfc_rule_st rfc_black_rule_ipv4;
rfc_rule_st rfc_white_rule_ipv4;

afw_rule_st *black_rule_ipv6[RULE_NUM];
unsigned int black_rule_ipv6_num;
afw_rule_st *white_rule_ipv6[RULE_NUM];
unsigned int white_rule_ipv6_num;
rfc_rule_st rfc_black_rule_ipv6;
rfc_rule_st rfc_white_rule_ipv6;

afw_rule_st *black_rule_domain[RULE_NUM];
unsigned int black_rule_domain_num;
afw_rule_st *white_rule_domain[RULE_NUM];
unsigned int white_rule_domain_num;
afw_rule_st *black_rule_domain_hash[AFW_HASH_SIZE];
afw_rule_st *white_rule_domain_hash[AFW_HASH_SIZE];

unsigned char black_rule_uid[MAX_NORMAL_UID];
unsigned int black_rule_uid_large[RULE_NUM];
unsigned int black_rule_uid_num;
unsigned int black_rule_uid_large_num;
unsigned char white_rule_uid[MAX_NORMAL_UID];
unsigned int white_rule_uid_large[RULE_NUM];
unsigned int white_rule_uid_num;
unsigned int white_rule_uid_large_num;

unsigned char def_black_port_rule[MAX_PORT];
unsigned int black_rule_port_num;
unsigned char def_white_port_rule[MAX_PORT];
unsigned int white_rule_port_num;

unsigned int def_host_rule;
unsigned int black_rule_host_num;
unsigned int white_rule_host_num;

key_st keys[TEST_NUM];


struct kmem_cache *dns_cache;
struct list_head dns_node_hash[AFW_HASH_SIZE];
unsigned int afw_dns_node_num;

afw_rule_st *black_rule_dns[RULE_NUM];
unsigned int black_rule_dns_num;
afw_rule_st *white_rule_dns[RULE_NUM];
unsigned int white_rule_dns_num;

afw_rule_st *black_rule_dns_ipv6[RULE_NUM];
unsigned int black_rule_dns_ipv6_num;
afw_rule_st *white_rule_dns_ipv6[RULE_NUM];
unsigned int white_rule_dns_ipv6_num;

unsigned char pid_cache[MAX_PID];
unsigned long pid_jiffies[MAX_PID];


static afw_dns_node_st *
malloc_dns_node(void)
{
	afw_dns_node_st *node;

	node = (afw_dns_node_st *)kmem_cache_alloc(dns_cache, GFP_ATOMIC);
	if(!node) {
		printk("%s[%d]: kmem_cache_alloc failed\n", __FILE__, __LINE__);
		goto out;
	}
	memset(node, 0, sizeof(*node));

	afw_dns_node_num ++; 
	afw_stat.malloc_dns_node += sizeof(*node);

out:
	return node;
}


static void
free_dns_node(afw_dns_node_st *node)
{
	if(!node) {
		goto out;
	}

	kmem_cache_free(dns_cache, node);
	afw_dns_node_num --; 
	afw_stat.malloc_dns_node -= sizeof(*node);

out:
	return;
}


static int
afw_dns_init(void)
{
	int ret;
	int i;

	afw_dns_node_num = 0;
	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		INIT_LIST_HEAD(&dns_node_hash[i]);
	}

	dns_cache = kmem_cache_create("afw_dns", sizeof(afw_dns_node_st), 0, 0, NULL);
	if(!dns_cache) {
		printk("%s[%d]: kmem_cache_create failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	ret = 0;
out:
	return ret;
}


static void
afw_dns_exit(void)
{
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	int i;

	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		head = &dns_node_hash[i];
		list_for_each_safe(pos, next, head) {
			node = list_entry(pos, afw_dns_node_st, next);
			list_del(&node->next);
			free_dns_node(node);
		}
	}

	kmem_cache_destroy(dns_cache);
	return;
}


static int afw_match_domain_hash(char *domain, int domain_len, unsigned int dport_domain, int type, unsigned int *rule_idx);


static void
afw_add_rule_dns(char *domain, int domain_len, unsigned int *ipv4, int num, int type)
{
	int ret;
	afw_rule_st *rule;
	unsigned int rule_idx;
	int i;
	int j;

	rule_idx = 0;
	ret = afw_match_domain_hash(domain, domain_len, MAX_PORT, type, &rule_idx);
	if(ret) {
		goto out;
	}

	rule_idx = rule_idx % RULE_NUM;
	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule[rule_idx];
	}else {
		rule = white_rule[rule_idx];
	}

	if(!rule) {
		goto out;
	}

	if(!rule->dns_ipv4) {
		rule->dns_ipv4 = (unsigned int *)kmalloc(MAX_RULE_DNS_ADDR_LEN, GFP_ATOMIC);
		if(!rule->dns_ipv4) {
			printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
			goto out;
		}

		memset(rule->dns_ipv4, 0, MAX_RULE_DNS_ADDR_LEN);
		rule->dns_ipv4_num = 0;
		afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR_LEN;

		if(type == AFW_RULE_TYPE_BLACK) {
			black_rule_dns[black_rule_dns_num] = rule;
			black_rule_dns_num ++;
			atomic_inc(&rule->refcnt);
		}else {
			white_rule_dns[white_rule_dns_num] = rule;
			white_rule_dns_num ++;
			atomic_inc(&rule->refcnt);
		}
	}

	for(i = 0; i < num; i ++) {
		for(j = 0; j < rule->dns_ipv4_num; j ++) {
			if(rule->dns_ipv4[j] == ipv4[i]) {
				break;
			}
		}

		if(j < rule->dns_ipv4_num) {		/* already exist */
			continue;
		}

		if(rule->dns_ipv4_num < MAX_RULE_DNS_ADDR) {
			rule->dns_ipv4[rule->dns_ipv4_num] = ipv4[i];
			rule->dns_ipv4_num ++;
		}else {
			if(rule->new_dns_ipv4_idx >= MAX_RULE_DNS_ADDR) {
				rule->new_dns_ipv4_idx = 0;
			}

			rule->dns_ipv4[rule->new_dns_ipv4_idx] = ipv4[i];
			rule->new_dns_ipv4_idx ++;

			afw_stat.dns_rule_ipv4_full ++;
		}
	}

out:
	return;
}


static void
afw_add_rule_dns_ipv6(char *domain, int domain_len, unsigned char ipv6[][IPV6_ADDR_LEN], int num, int type)
{
	int ret;
	afw_rule_st *rule;
	unsigned int rule_idx;
	int i;
	int j;

	rule_idx = 0;
	ret = afw_match_domain_hash(domain, domain_len, MAX_PORT, type, &rule_idx);
	if(ret) {
		goto out;
	}

	rule_idx = rule_idx % RULE_NUM;
	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule[rule_idx];
	}else {
		rule = white_rule[rule_idx];
	}

	if(!rule) {
		goto out;
	}

	if(!rule->dns_ipv6) {
		rule->dns_ipv6 = (unsigned char *)kmalloc(MAX_RULE_DNS_ADDR6_LEN, GFP_ATOMIC);
		if(!rule->dns_ipv6) {
			printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
			goto out;
		}

		memset(rule->dns_ipv6, 0, MAX_RULE_DNS_ADDR6_LEN);
		rule->dns_ipv6_num = 0;
		afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR6_LEN;

		if(type == AFW_RULE_TYPE_BLACK) {
			black_rule_dns_ipv6[black_rule_dns_ipv6_num] = rule;
			black_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}else {
			white_rule_dns_ipv6[white_rule_dns_ipv6_num] = rule;
			white_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}
	}

	for(i = 0; i < num; i ++) {
		for(j = 0; j < rule->dns_ipv6_num; j ++) {
			if(!memcmp(&rule->dns_ipv6[j * IPV6_ADDR_LEN], ipv6[i], IPV6_ADDR_LEN)) {
				break;
			}
		}

		if(j < rule->dns_ipv6_num) {		/* already exist */
			continue;
		}

		if(rule->dns_ipv6_num < MAX_RULE_DNS_ADDR6) {
			memcpy(&rule->dns_ipv6[rule->dns_ipv6_num * IPV6_ADDR_LEN], ipv6[i], IPV6_ADDR_LEN);
			rule->dns_ipv6_num ++;
		}else {
			if(rule->new_dns_ipv6_idx >= MAX_RULE_DNS_ADDR6) {
				rule->new_dns_ipv6_idx = 0;
			}

			memcpy(&rule->dns_ipv6[rule->new_dns_ipv6_idx * IPV6_ADDR_LEN], ipv6[i], IPV6_ADDR_LEN);
			rule->new_dns_ipv6_idx ++;

			afw_stat.dns_rule_ipv6_full ++;
		}
	}

out:
	return;
}


int
afw_add_dns_node(char *domain, int domain_len, unsigned int *ipv4, int num)
{
	int ret;
	unsigned int hash;
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	int i;
	int j;

	write_lock_bh(&afw_lock);

	afw_add_rule_dns(domain, domain_len, ipv4, num, AFW_RULE_TYPE_BLACK);
	afw_add_rule_dns(domain, domain_len, ipv4, num, AFW_RULE_TYPE_WHITE);

	if(afw_dns_node_num >= MAX_DNS_NODE_NUM) {
		afw_stat.dns_node_reach_limit ++;
		ret = -1;
		goto out;
	}

	if(!domain_len) {
		ret = 0;
		goto out;
	}

	if(num > MAX_DNS_ADDR) {
		num = MAX_DNS_ADDR;
	}

	hash = jhash(domain, domain_len, 0);
	hash = hash & AFW_HASH_MASK;

	head = &dns_node_hash[hash];
	list_for_each_safe(pos, next, head) {
		node = list_entry(pos, afw_dns_node_st, next);

		if(node->domain_len != domain_len) {
			continue;
		}

		if(memcmp(node->domain, domain, domain_len)) {
			continue;
		}

		for(i = 0; i < num; i ++) {
			for(j = 0; j < node->num; j ++) {
				if(node->ipv4[j] == ipv4[i]) {
					break;
				}
			}

			if(j < node->num) {				/* already exist */
				continue;
			}

			if(node->num < MAX_DNS_ADDR) {
				node->ipv4[node->num] = ipv4[i];
				node->num ++;
			}else {
				if(node->new_idx >= MAX_DNS_ADDR) {
					node->new_idx = 0;
				}

				node->ipv4[node->new_idx] = ipv4[i];		/* overwrite the oldest */
				node->new_idx ++;

				afw_stat.dns_node_ipv4_full ++;
			}
		}

		ret = 0;
		goto out;
	}

	node = malloc_dns_node();
	if(!node) {
		printk("%s[%d]: malloc_dns_node failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	node->domain_len = domain_len;
	memcpy(node->domain, domain, domain_len);
	node->num = num;
	for(i = 0; i < num; i ++) {
		node->ipv4[i] = ipv4[i];
	}
	list_add(&node->next, head);

	ret = 0;
out:
	write_unlock_bh(&afw_lock);
	return ret;
}


int
afw_add_dns_node_ipv6(char *domain, int domain_len, unsigned char ipv6[][IPV6_ADDR_LEN], int num)
{
	int ret;
	unsigned int hash;
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	int i;
	int j;

	write_lock_bh(&afw_lock);

	afw_add_rule_dns_ipv6(domain, domain_len, ipv6, num, AFW_RULE_TYPE_BLACK);
	afw_add_rule_dns_ipv6(domain, domain_len, ipv6, num, AFW_RULE_TYPE_WHITE);

	if(afw_dns_node_num >= MAX_DNS_NODE_NUM) {
		afw_stat.dns_node_reach_limit ++;
		ret = -1;
		goto out;
	}

	if(!domain_len) {
		ret = 0;
		goto out;
	}

	if(num > MAX_DNS_ADDR) {
		num = MAX_DNS_ADDR;
	}

	hash = jhash(domain, domain_len, 0);
	hash = hash & AFW_HASH_MASK;

	head = &dns_node_hash[hash];
	list_for_each_safe(pos, next, head) {
		node = list_entry(pos, afw_dns_node_st, next);

		if(node->domain_len != domain_len) {
			continue;
		}

		if(memcmp(node->domain, domain, domain_len)) {
			continue;
		}

		for(i = 0; i < num; i ++) {
			for(j = 0; j < node->num_ipv6; j ++) {
				if(!memcmp(node->ipv6[j], ipv6[i], IPV6_ADDR_LEN)) {
					break;
				}
			}

			if(j < node->num_ipv6) {				/* already exist */
				continue;
			}

			if(node->num_ipv6 < MAX_DNS_ADDR) {
				memcpy(node->ipv6[node->num_ipv6], ipv6[i], IPV6_ADDR_LEN);
				node->num_ipv6 ++;
			}else {
				if(node->new_idx_ipv6 >= MAX_DNS_ADDR) {
					node->new_idx_ipv6 = 0;
				}

				memcpy(node->ipv6[node->new_idx_ipv6], ipv6[i], IPV6_ADDR_LEN);				/* overwrite the oldest */
				node->new_idx_ipv6 ++;

				afw_stat.dns_node_ipv6_full ++;
			}
		}

		ret = 0;
		goto out;
	}

	node = malloc_dns_node();
	if(!node) {
		printk("%s[%d]: malloc_dns_node failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	node->domain_len = domain_len;
	memcpy(node->domain, domain, domain_len);
	node->num_ipv6 = num;
	for(i = 0; i < num; i ++) {
		memcpy(node->ipv6[i], ipv6[i], IPV6_ADDR_LEN);
	}
	list_add(&node->next, head);

	ret = 0;
out:
	write_unlock_bh(&afw_lock);
	return ret;
}


static int
build_rule_dns(afw_rule_st *rule)
{
	int ret;
	unsigned int hash;
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	int find;
	int i;

	hash = jhash(rule->domain, rule->domain_len, 0);
	hash = hash & AFW_HASH_MASK;

	find = 0;
	head = &dns_node_hash[hash];
	list_for_each_safe(pos, next, head) {
		node = list_entry(pos, afw_dns_node_st, next);

		if(node->domain_len != rule->domain_len) {
			continue;
		}

		if(memcmp(node->domain, rule->domain, rule->domain_len)) {
			continue;
		}

		find = 1;
		break;
	}

	if(!find) {
		ret = 0;
		goto out;
	}

	if(node->num) {
		rule->dns_ipv4 = (unsigned int *)kmalloc(MAX_RULE_DNS_ADDR_LEN, GFP_ATOMIC);
		if(!rule->dns_ipv4) {
			printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
			ret = -1;
			goto out;
		}

	memset(rule->dns_ipv4, 0, MAX_RULE_DNS_ADDR_LEN);
	rule->dns_ipv4_num = 0;
	afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR_LEN;

	for(i = 0; i < node->num; i ++) {
		rule->dns_ipv4[i] = node->ipv4[i];
	}
	rule->dns_ipv4_num = node->num;

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_dns[black_rule_dns_num] = rule;
		black_rule_dns_num ++;
		atomic_inc(&rule->refcnt);
	}else {
		white_rule_dns[white_rule_dns_num] = rule;
		white_rule_dns_num ++;
		atomic_inc(&rule->refcnt);
		}
	}

	if(node->num_ipv6) {
		rule->dns_ipv6 = (unsigned char *)kmalloc(MAX_RULE_DNS_ADDR6_LEN, GFP_ATOMIC);
		if(!rule->dns_ipv6) {
			printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
			ret = -1;
			goto out;
		}

		memset(rule->dns_ipv6, 0, MAX_RULE_DNS_ADDR6_LEN);
		rule->dns_ipv6_num = 0;
		afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR6_LEN;

		for(i = 0; i < node->num_ipv6; i ++) {
			memcpy(&rule->dns_ipv6[i * IPV6_ADDR_LEN], node->ipv6[i], IPV6_ADDR_LEN);
		}
		rule->dns_ipv6_num = node->num_ipv6;

		if(rule->type == AFW_RULE_TYPE_BLACK) {
			black_rule_dns_ipv6[black_rule_dns_ipv6_num] = rule;
			black_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}else {
			white_rule_dns_ipv6[white_rule_dns_ipv6_num] = rule;
			white_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}
	}

	ret = 0;
out:
	return ret;
}


static int
build_rule_dns_any(afw_rule_st *rule)
{
	int ret;
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	int idx;
	int i;
	int j;
	int k;

	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		head = &dns_node_hash[i];
		list_for_each_safe(pos, next, head) {
			node = list_entry(pos, afw_dns_node_st, next);

			if(node->domain_len < rule->domain_len) {
				continue;
			}

			idx = node->domain_len - rule->domain_len;
			if(memcmp(&node->domain[idx], rule->domain, rule->domain_len)) {
				continue;
			}

			if(node->num) {
				if(!rule->dns_ipv4) {
					rule->dns_ipv4 = (unsigned int *)kmalloc(MAX_RULE_DNS_ADDR_LEN, GFP_ATOMIC);
					if(!rule->dns_ipv4) {
						printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
						ret = -1;
						goto out;
					}

				memset(rule->dns_ipv4, 0, MAX_RULE_DNS_ADDR_LEN);
				rule->dns_ipv4_num = 0;
				afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR_LEN;
			}

			for(j = 0; j < node->num; j ++) {
				for(k = 0; k < rule->dns_ipv4_num; k ++) {
					if(rule->dns_ipv4[k] == node->ipv4[j]) {
						break;
					}
				}

				if(k < rule->dns_ipv4_num) {		/* already exist */
					continue;
				}

				if(rule->dns_ipv4_num < MAX_RULE_DNS_ADDR) {
					rule->dns_ipv4[rule->dns_ipv4_num] = node->ipv4[j];
					rule->dns_ipv4_num ++;
				}else {
					if(rule->new_dns_ipv4_idx >= MAX_RULE_DNS_ADDR) {
						rule->new_dns_ipv4_idx = 0;
					}

					rule->dns_ipv4[rule->new_dns_ipv4_idx] = node->ipv4[j];
					rule->new_dns_ipv4_idx ++;

						afw_stat.dns_rule_ipv4_full ++;
					}
				}
			}

			if(node->num_ipv6) {
				if(!rule->dns_ipv6) {
					rule->dns_ipv6 = (unsigned char *)kmalloc(MAX_RULE_DNS_ADDR6_LEN, GFP_ATOMIC);
					if(!rule->dns_ipv6) {
						printk("%s[%d]: kmalloc failed\n", __FILE__, __LINE__);
						ret = -1;
						goto out;
					}

					memset(rule->dns_ipv6, 0, MAX_RULE_DNS_ADDR6_LEN);
					rule->dns_ipv6_num = 0;
					afw_stat.malloc_dns_addr += MAX_RULE_DNS_ADDR6_LEN;
				}

				for(j = 0; j < node->num_ipv6; j ++) {
					for(k = 0; k < rule->dns_ipv6_num; k ++) {
						if(!memcmp(&rule->dns_ipv6[k * IPV6_ADDR_LEN], node->ipv6[j], IPV6_ADDR_LEN)) {
							break;
						}
					}

					if(k < rule->dns_ipv6_num) {		/* already exist */
						continue;
					}

					if(rule->dns_ipv6_num < MAX_RULE_DNS_ADDR6) {
						memcpy(&rule->dns_ipv6[rule->dns_ipv6_num * IPV6_ADDR_LEN], node->ipv6[j], IPV6_ADDR_LEN);
						rule->dns_ipv6_num ++;
					}else {
						if(rule->new_dns_ipv6_idx >= MAX_RULE_DNS_ADDR6) {
							rule->new_dns_ipv6_idx = 0;
						}

						memcpy(&rule->dns_ipv6[rule->new_dns_ipv6_idx * IPV6_ADDR_LEN], node->ipv6[j], IPV6_ADDR_LEN);
						rule->new_dns_ipv6_idx ++;

						afw_stat.dns_rule_ipv6_full ++;
					}
				}
			}
		}
	}

	if(rule->dns_ipv4) {
		if(rule->type == AFW_RULE_TYPE_BLACK) {
			black_rule_dns[black_rule_dns_num] = rule;
			black_rule_dns_num ++;
			atomic_inc(&rule->refcnt);
		}else {
			white_rule_dns[white_rule_dns_num] = rule;
			white_rule_dns_num ++;
			atomic_inc(&rule->refcnt);
		}
	}

	if(rule->dns_ipv6) {
		if(rule->type == AFW_RULE_TYPE_BLACK) {
			black_rule_dns_ipv6[black_rule_dns_ipv6_num] = rule;
			black_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}else {
			white_rule_dns_ipv6[white_rule_dns_ipv6_num] = rule;
			white_rule_dns_ipv6_num ++;
			atomic_inc(&rule->refcnt);
		}
	}

	ret = 0;
out:
	return ret;
}


static int
afw_inet_addr(char *data, int len, unsigned int *ipv4)
{
	int ret;
	unsigned int ip;
	unsigned int shift;
	unsigned int addr;
	unsigned int dot_num;
	int i;

	if(len > MAX_IPV4_LEN || len < MIN_IPV4_LEN) {
		ret = -1;
		goto out;
	}

	addr = 0;
	shift = 24;
	ip = 0;
	dot_num = 0;
	for(i = 0; i < len; i ++) {
		if(data[i] == '.') {
			if(addr > 255 || dot_num >= 3 || !i) {
				ret = -1;
				goto out;
			}

			ip |= (addr << shift);
			shift -= 8;
			dot_num ++;
			addr = 0;

			continue;
		}
		if(data[i] < '0' || data[i] > '9') {
			ret = -1;
			goto out;
		}
		addr = addr * 10 + (data[i] - '0');
	}

	if(addr > 255 || dot_num != 3) {
		ret = -1;
		goto out;
	}
	ip |= addr;

	*ipv4 = ip;
	ret = 0;
out:
	return ret;
}


static void
afw_log_pkt_ipv4(unsigned int dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, unsigned short dport_domain, 
	unsigned int pkt_type, unsigned int rule_idx, unsigned int action, unsigned int uid)
{
	int num;
	int len;

	num = afw_pkt_log_num;
	if(afw_pkt_log_num >= AFW_PKT_LOG_NUM) {
		afw_pkt_log_num = 0;
		num = 0;
	}

	*(unsigned int *)afw_pkt_log[num].dip = dip;

	afw_pkt_log[num].dport = dport;
	afw_pkt_log[num].action = action;
	afw_pkt_log[num].pkt_type = pkt_type;

	afw_pkt_log[num].uid = uid;
	afw_pkt_log[num].proto = proto;
	afw_pkt_log[num].rule_idx = rule_idx;
	afw_pkt_log[num].jiffies = jiffies;

	afw_pkt_log[num].domain_len = domain_len;
	len = domain_len;
	if(len >= MAX_LOG_DOMAIN_LEN) {
		len = MAX_LOG_DOMAIN_LEN - 1;
	}
	memcpy(afw_pkt_log[num].domain, domain, len);
	afw_pkt_log[num].domain[len] = 0;
	afw_pkt_log[num].dport_domain = dport_domain;

	afw_pkt_log[num].pid = current->pid;
	memcpy(afw_pkt_log[num].comm, current->comm, sizeof(afw_pkt_log[num].comm));

	if(action == AFW_PKT_ACTION_STOP) {
		if(rule_idx < RULE_NUM && black_rule[rule_idx]) {
			black_rule[rule_idx]->hit_num ++;
		}
	}else {
		if(rule_idx < RULE_NUM && white_rule[rule_idx]) {
			white_rule[rule_idx]->hit_num ++;
		}
	}

	afw_pkt_log_num ++;

	return;
}


static void
afw_log_pkt_ipv6(unsigned char *dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, unsigned short dport_domain, 
	unsigned int pkt_type, unsigned int rule_idx, unsigned int action, unsigned int uid)
{
	int num;
	int len;

	num = afw_pkt_log_num;
	if(afw_pkt_log_num >= AFW_PKT_LOG_NUM) {
		afw_pkt_log_num = 0;
		num = 0;
	}

	memcpy(afw_pkt_log[num].dip, dip, 16);

	afw_pkt_log[num].dport = dport;
	afw_pkt_log[num].action = action;
	afw_pkt_log[num].pkt_type = pkt_type;

	afw_pkt_log[num].uid = uid;
	afw_pkt_log[num].proto = proto;
	afw_pkt_log[num].rule_idx = rule_idx;
	afw_pkt_log[num].jiffies = jiffies;

	afw_pkt_log[num].domain_len = domain_len;
	len = domain_len;
	if(len >= MAX_LOG_DOMAIN_LEN) {
		len = MAX_LOG_DOMAIN_LEN - 1;
	}
	memcpy(afw_pkt_log[num].domain, domain, len);
	afw_pkt_log[num].domain[len] = 0;
	afw_pkt_log[num].dport_domain = dport_domain;

	afw_pkt_log[num].pid = current->pid;
	memcpy(afw_pkt_log[num].comm, current->comm, sizeof(afw_pkt_log[num].comm));

	if(action == AFW_PKT_ACTION_STOP) {
		if(rule_idx < RULE_NUM && black_rule[rule_idx]) {
			black_rule[rule_idx]->hit_num ++;
		}
	}else {
		if(rule_idx < RULE_NUM && white_rule[rule_idx]) {
			white_rule[rule_idx]->hit_num ++;
		}
	}

	afw_pkt_log_num ++;

	return;
}


static int
afw_rule_len(afw_req_st *req)
{
	int len;

	len = sizeof(afw_rule_st);
	if(req->raw_rule) {
		len += AFW_ALIGN_4(req->raw_rule_len + 1);
	}

	if(req->domain) {
		len += AFW_ALIGN_4(req->domain_len + 1);
	}

	if(req->port) {
		len += (req->port_num * sizeof(range_u16_st));
	}

	return len;
}


static afw_rule_st*
afw_malloc_rule(afw_req_st *req)
{
	afw_rule_st *rule;
	int len;
	int offset;

	len = afw_rule_len(req);
	rule = (afw_rule_st *)kmalloc(len, GFP_ATOMIC);
	if(!rule) {
		printk("%s[%d]: kmalloc failed, %d\n", __FILE__, __LINE__, len);
		goto out;
	}
	memset(rule, 0, len);
	atomic_set(&rule->refcnt, 1);
	rule->rule_len = len;

	offset = 0;
	if(req->raw_rule) {
		rule->raw_rule = &rule->data[offset];
		offset += AFW_ALIGN_4(req->raw_rule_len + 1);
	}

	if(req->domain) {
		rule->domain= &rule->data[offset];
		offset += AFW_ALIGN_4(req->domain_len + 1);
	}

	if(req->port) {
		rule->port = (range_u16_st *)&rule->data[offset];
	}

	afw_stat.malloc_rule += rule->rule_len;
out:
	return rule;
}


static void
afw_free_rule(afw_rule_st *rule)
{
	if(rule && atomic_dec_and_test(&rule->refcnt)) {		
		afw_stat.malloc_rule -= rule->rule_len;
		if(rule->dns_ipv4) {
			kfree(rule->dns_ipv4);
			afw_stat.malloc_dns_addr -= MAX_RULE_DNS_ADDR_LEN;
		}
		if(rule->dns_ipv6) {
			kfree(rule->dns_ipv6);
			afw_stat.malloc_dns_addr -= MAX_RULE_DNS_ADDR6_LEN;
		}
		kfree(rule);
	}
	return;
}


static int
afw_init_black_rule(void)
{
	int ret;
	int i;

	black_rule_num = 0;
	black_rule_max_len = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule[i] = NULL;
	}

	black_rule_ipv4_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_ipv4[i] = NULL;
	}

	black_rule_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_ipv6[i] = NULL;
	}

	black_rule_domain_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_domain[i] = NULL;
	}
	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		black_rule_domain_hash[i] = NULL;
	}

	black_rule_dns_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_dns[i] = NULL;
	}

	black_rule_dns_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_dns_ipv6[i] = NULL;
	}

	black_rule_uid_num = 0;
	black_rule_uid_large_num = 0;
	for(i = 0; i < MAX_NORMAL_UID; i ++) {
		black_rule_uid[i] = 0;
	}
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_uid_large[i] = 0;
	}

	black_rule_port_num = 0;
	for(i = 0; i < MAX_PORT; i ++) {
		def_black_port_rule[i] = 0;
	}

	black_rule_host_num = 0;

	ret = rfc_init_rule(&rfc_black_rule_ipv4, RULE_NUM, 0, TYPE_MAX);
	if(ret) {
		printk("%s[%d]: rfc_init_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = rfc_init_rule(&rfc_black_rule_ipv6, RULE_NUM, 0, IPV6_RFC_FU16_NUM);
	if(ret) {
		printk("%s[%d]: rfc_init_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out_free;
	}

	ret = 0;
	goto out;
out_free:
	rfc_free_rule(&rfc_black_rule_ipv4);
out:
	return ret;
}


int
afw_init_white_rule(void)
{
	int ret;
	int i;

	white_rule_num = 0;
	white_rule_max_len = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule[i] = NULL;
	}

	white_rule_ipv4_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_ipv4[i] = NULL;
	}

	white_rule_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_ipv6[i] = NULL;
	}

	white_rule_domain_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_domain[i] = NULL;
	}
	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		white_rule_domain_hash[i] = NULL;
	}

	white_rule_dns_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_dns[i] = NULL;
	}

	white_rule_dns_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_dns_ipv6[i] = NULL;
	}

	white_rule_uid_num = 0;
	white_rule_uid_large_num = 0;
	for(i = 0; i < MAX_NORMAL_UID; i ++) {
		white_rule_uid[i] = 0;
	}
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_uid_large[i] = 0;
	}

	white_rule_port_num = 0;
	for(i = 0; i < MAX_PORT; i ++) {
		def_white_port_rule[i] = 0;
	}

	white_rule_host_num = 0;

	ret = rfc_init_rule(&rfc_white_rule_ipv4, RULE_NUM, 0, TYPE_MAX);
	if(ret) {
		printk("%s[%d]: rfc_init_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = rfc_init_rule(&rfc_white_rule_ipv6, RULE_NUM, 0, IPV6_RFC_FU16_NUM);
	if(ret) {
		printk("%s[%d]: rfc_init_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out_free;
	}

	ret = 0;
	goto out;
out_free:
	rfc_free_rule(&rfc_white_rule_ipv4);
out:
	return ret;
}


static void
afw_free_black_rule(void)
{
	afw_rule_st *rule;
	afw_rule_st *next;
	int i;

	rfc_free_rule(&rfc_black_rule_ipv4);
	rfc_free_rule(&rfc_black_rule_ipv6);

	black_rule_num = 0;
	black_rule_max_len = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule[i]) {
			afw_free_rule(black_rule[i]);
			black_rule[i] = NULL;
		}
	}

	black_rule_ipv4_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule_ipv4[i]) {
			afw_free_rule(black_rule_ipv4[i]);
			black_rule_ipv4[i] = NULL;
		}
	}

	black_rule_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule_ipv6[i]) {
			afw_free_rule(black_rule_ipv6[i]);
			black_rule_ipv6[i] = NULL;
		}
	}

	black_rule_domain_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule_domain[i]) {
			afw_free_rule(black_rule_domain[i]);
			black_rule_domain[i] = NULL;
		}
	}

	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		rule = black_rule_domain_hash[i];
		while(rule) {
			next = rule->next;
			afw_free_rule(rule);
			rule = next;
		}
		black_rule_domain_hash[i] = NULL;
	}

	black_rule_dns_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule_dns[i]) {
			afw_free_rule(black_rule_dns[i]);
			black_rule_dns[i] = NULL;
		}
	}

	black_rule_dns_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(black_rule_dns_ipv6[i]) {
			afw_free_rule(black_rule_dns_ipv6[i]);
			black_rule_dns_ipv6[i] = NULL;
		}
	}

	black_rule_uid_num = 0;
	black_rule_uid_large_num = 0;
	for(i = 0; i < MAX_NORMAL_UID; i ++) {
		black_rule_uid[i] = 0;
	}
	for(i = 0; i < RULE_NUM; i ++) {
		black_rule_uid_large[i] = 0;
	}

	black_rule_port_num = 0;
	for(i = 0; i < MAX_PORT; i ++) {
		def_black_port_rule[i] = 0;
	}

	black_rule_host_num = 0;

	return;
}


static void
afw_free_white_rule(void)
{
	afw_rule_st *rule;
	afw_rule_st *next;
	int i;

	rfc_free_rule(&rfc_white_rule_ipv4);
	rfc_free_rule(&rfc_white_rule_ipv6);

	white_rule_num = 0;
	white_rule_max_len = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule[i]) {
			afw_free_rule(white_rule[i]);
			white_rule[i] = NULL;
		}
	}

	white_rule_ipv4_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule_ipv4[i]) {
			afw_free_rule(white_rule_ipv4[i]);
			white_rule_ipv4[i] = NULL;
		}
	}

	white_rule_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule_ipv6[i]) {
			afw_free_rule(white_rule_ipv6[i]);
			white_rule_ipv6[i] = NULL;
		}
	}

	white_rule_domain_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule_domain[i]) {
			afw_free_rule(white_rule_domain[i]);
			white_rule_domain[i] = NULL;
		}
	}

	for(i = 0; i < AFW_HASH_SIZE; i ++) {
		rule = white_rule_domain_hash[i];
		while(rule) {
			next = rule->next;
			afw_free_rule(rule);
			rule = next;
		}
		white_rule_domain_hash[i] = NULL;
	}

	white_rule_dns_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule_dns[i]) {
			afw_free_rule(white_rule_dns[i]);
			white_rule_dns[i] = NULL;
		}
	}

	white_rule_dns_ipv6_num = 0;
	for(i = 0; i < RULE_NUM; i ++) {
		if(white_rule_dns_ipv6[i]) {
			afw_free_rule(white_rule_dns_ipv6[i]);
			white_rule_dns_ipv6[i] = NULL;
		}
	}

	white_rule_uid_num = 0;
	white_rule_uid_large_num = 0;
	for(i = 0; i < MAX_NORMAL_UID; i ++) {
		white_rule_uid[i] = 0;
	}
	for(i = 0; i < RULE_NUM; i ++) {
		white_rule_uid_large[i] = 0;
	}

	white_rule_port_num = 0;
	for(i = 0; i < MAX_PORT; i ++) {
		def_white_port_rule[i] = 0;
	}

	white_rule_host_num = 0;

	return;
}


static int
build_dport(afw_rule_st *rule, rfc_rule_st *rfc_rule)
{
	int ret;
	unsigned short port1;
	unsigned short port2;
	int chunk_idx;
	int rule_idx;
	int i;
	int j;

	rule_idx = rule->rule_idx;

	if(!rule->port || !rule->port_num) {		/* mean any port */
		for(i = 1; i < MAX_PORT; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DPORT]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}
		ret = 0;
		goto out;
	}

	for(i = 0; i < rule->port_num; i ++) {
		port1 = rule->port[i].v1;
		port2 = rule->port[i].v2;
		for(j = port1; j <= port2; j ++) {
			chunk_idx = j;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DPORT]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}
	}

	ret = 0;
out:
	return ret;
}


static int
build_dip(afw_rule_st *rule, rfc_rule_st *rfc_rule)
{
	int ret;
	unsigned int ip_h1;
	unsigned int ip_h2;
	unsigned int ip_l1;
	unsigned int ip_l2;

	int chunk_idx;
	int rule_idx;
	int i;

	ip_h1 = rule->ip.v1 >> 16;
	ip_l1 = rule->ip.v1 & 0xffff;

	ip_h2 = rule->ip.v2 >> 16;
	ip_l2 = rule->ip.v2 & 0xffff;

	/* build dip_h1 */
	rule_idx = rule->rule_idx;
	if(ip_h1 == ip_h2) {
		chunk_idx = ip_h1;
		ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_H1]);
		if(ret) {
			printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}

		/* build dip_l1 */
		for(i = ip_l1; i <= ip_l2; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_L1]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}

	}

	/* build dip_h2 */
	if(ip_h1 <= ip_h2 - 1) {
		chunk_idx = ip_h1;
		ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_H2]);
		if(ret) {
			printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}

		/* build dip_l2 */
		for(i = ip_l1; i < RFC_MAX_U16; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_L2]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}
	}

	/* build dip_h3 */
	if(ip_h1 <= ip_h2 - 1) {
		chunk_idx = ip_h2;
		ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_H3]);
		if(ret) {
			printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}

		/* build dip_l3 */
		for(i = 0; i <= ip_l2; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_L3]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}
	}

	if(ip_h1 < ip_h2 - 1) {
		/* build dip_h4 */
		for(i = ip_h1 + 1; i < ip_h2; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_H4]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}

		/* build dip_l4 */
		for(i = 0; i < RFC_MAX_U16; i ++) {
			chunk_idx = i;
			ret = rfc_build_cbm_u16(chunk_idx, rule_idx, rfc_rule->fu16[TYPE_DIP_L4]);
			if(ret) {
				printk("%s[%d]: rfc_build_cbm_u16 failed, %d\n", __FILE__, __LINE__, ret);
				goto out;
			}
		}
	}

	ret = 0;
out:
	return ret;
}


static int
build_rule_def_host(afw_rule_st *rule)
{
	def_host_rule = rule->type;

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_host_num ++;
	}else {
		white_rule_host_num ++;
	}

	return 0;
}


static int
build_rule_def_port(afw_rule_st *rule)
{
	int ret;
	unsigned short port1;
	unsigned short port2;
	int i;
	int j;

	if(!rule->port) {
		printk("%s[%d]: check parameter failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	for(i = 0; i < rule->port_num; i ++) {
		port1 = rule->port[i].v1;
		port2 = rule->port[i].v2;

		for(j = port1; j <= port2; j ++) {
			if(rule->type == AFW_RULE_TYPE_BLACK) {
				def_black_port_rule[j] = 1;
			}else {
				def_white_port_rule[j] = 1;
			}
		}
	}

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_port_num ++;
	}else {
		white_rule_port_num ++;
	}

	ret = 0;
out:
	return ret;
}


static int
build_rule_ipv4(afw_rule_st *rule)
{
	int ret;
	rfc_rule_st *rfc_rule;

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_ipv4[black_rule_ipv4_num] = rule;
		black_rule_ipv4_num ++;
		atomic_inc(&rule->refcnt);

		rfc_rule = &rfc_black_rule_ipv4;
	}else {
		white_rule_ipv4[white_rule_ipv4_num] = rule;
		white_rule_ipv4_num ++;
		atomic_inc(&rule->refcnt);

		rfc_rule = &rfc_white_rule_ipv4;
	}

	ret = build_dport(rule, rfc_rule);
	if(ret) {
		printk("%s[%d]: build dport failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = build_dip(rule, rfc_rule);
	if(ret) {
		printk("%s[%d]: build dip failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	rfc_rule->cur_bitmap_size = rule->rule_idx / RFC_BITS_PER_LONG + 1;

	ret = 0;
out:
	return ret;
}


static int
build_rule_ipv6(afw_rule_st *rule)
{
	int ret;
	rfc_rule_st *rfc_rule;

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_ipv6[black_rule_ipv6_num] = rule;
		black_rule_ipv6_num ++;
		atomic_inc(&rule->refcnt);

		rfc_rule = &rfc_black_rule_ipv6;
	}else {
		white_rule_ipv6[white_rule_ipv6_num] = rule;
		white_rule_ipv6_num ++;
		atomic_inc(&rule->refcnt);

		rfc_rule = &rfc_white_rule_ipv6;
	}

	ret = build_dport(rule, rfc_rule);
	if(ret) {
		printk("%s[%d]: build dport failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	rfc_rule->cur_bitmap_size = rule->rule_idx / RFC_BITS_PER_LONG + 1;

out:
	return 0;
}


static int
build_rule_domain(afw_rule_st *rule)
{
	int ret;
	unsigned int hash;

	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_domain[black_rule_domain_num] = rule;
		black_rule_domain_num ++;
		atomic_inc(&rule->refcnt);
	}else {
		white_rule_domain[white_rule_domain_num] = rule;
		white_rule_domain_num ++;
		atomic_inc(&rule->refcnt);
	}

	hash = jhash(rule->domain, rule->domain_len, 0);
	hash = hash & AFW_HASH_MASK;
	if(rule->type == AFW_RULE_TYPE_BLACK) {
		rule->next = black_rule_domain_hash[hash];
		black_rule_domain_hash[hash] = rule;
		atomic_inc(&rule->refcnt);
	}else {
		rule->next = white_rule_domain_hash[hash];
		white_rule_domain_hash[hash] = rule;
		atomic_inc(&rule->refcnt);
	}

	if(rule->domain_any) {
		ret = build_rule_dns_any(rule);
	}else {
		ret = build_rule_dns(rule);
	}

	return ret;
}


static int
build_rule_uid(afw_rule_st *rule)
{
	unsigned int uid;

	uid = rule->uid;
	if(rule->type == AFW_RULE_TYPE_BLACK) {
		black_rule_uid_num ++;
		if(uid < MAX_NORMAL_UID) {
			black_rule_uid[uid] = 1;
		}else {
			black_rule_uid_large[black_rule_uid_large_num] = uid;
			black_rule_uid_large_num ++;
		}
	}else {
		white_rule_uid_num ++;
		if(uid < MAX_NORMAL_UID) {
			white_rule_uid[uid] = 1;
		}else {
			white_rule_uid_large[white_rule_uid_large_num] = uid;
			white_rule_uid_large_num ++;
		}
	}

	return 0;
}


static int
build_rule(afw_rule_st *rule)
{
	int ret;

	switch(rule->kind) {
		case AFW_RULE_KIND_DEF_HOST:
			ret = build_rule_def_host(rule);
			goto out;

		case AFW_RULE_KIND_DEF_PORT:
			ret = build_rule_def_port(rule);
			goto out;

		case AFW_RULE_KIND_IPV4:
			ret = build_rule_ipv4(rule);
			goto out;

		case AFW_RULE_KIND_IPV6:
			ret = build_rule_ipv6(rule);
			goto out;

		case AFW_RULE_KIND_DOMAIN:
			ret = build_rule_domain(rule);
			goto out;

		case AFW_RULE_KIND_UID:
			ret = build_rule_uid(rule);
			goto out;

		default:
			printk("%s[%d]: unexpected rule kind, %d\n", __FILE__, __LINE__, rule->kind);
			ret = -1;
			goto out;
	}

	ret = 0;
out:
	return ret;
}


int
afw_add_rule(afw_req_st *req)
{
	int ret;
	int len;
	afw_rule_st *rule;

	if(!req || !req->raw_rule) {
		printk("%s[%d]: check parameter failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	if(req->type != AFW_RULE_TYPE_BLACK && req->type != AFW_RULE_TYPE_WHITE) {
		printk("%s[%d]: check parameter failed, %d\n", __FILE__, __LINE__, req->type);
		ret = -1;
		goto out;
	}

	if(req->kind >= AFW_RULE_KIND_MAX) {
		printk("%s[%d]: check parameter failed, %d\n", __FILE__, __LINE__, req->kind);
		ret = -1;
		goto out;
	}

	if(black_rule_num + white_rule_num >= RULE_NUM) {
		printk("%s[%d]: check rule num failed, %d, %d\n", __FILE__, __LINE__, black_rule_num, white_rule_num);
		ret = -1;
		goto out;
	}

	rule = afw_malloc_rule(req);
	if(!rule) {
		printk("%s[%d]: afw_malloc_rule failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	rule->type = req->type;
	rule->kind = req->kind;
	rule->ip = req->ip;
	rule->ipv6 = req->ipv6;
	rule->uid = req->uid;

	if(req->type == AFW_RULE_TYPE_BLACK) {
		rule->rule_idx = black_rule_num;
	}else {
		rule->rule_idx = white_rule_num;
	}

	len = req->raw_rule_len;
	ret = copy_from_user(rule->raw_rule, req->raw_rule, len);
	if(ret) {
		printk("%s[%d]: copy_from_user failed, %d\n", __FILE__, __LINE__, ret);
		afw_free_rule(rule);
		ret = -1;
		goto out;
	}
	rule->raw_rule[len] = 0;
	rule->raw_rule_len = req->raw_rule_len;

	if(req->domain) {
		len = req->domain_len;
		ret = copy_from_user(rule->domain, req->domain, len);
		if(ret) {
			printk("%s[%d]: copy_from_user failed, %d\n", __FILE__, __LINE__, ret);
			afw_free_rule(rule);
			ret = -1;
			goto out;
		}
		rule->domain[len] = 0;

		rule->domain_len = req->domain_len;
		rule->domain_any = req->domain_any;
		rule->domain_level = req->domain_level;
	}

	if(req->port) {
		len = req->port_num * sizeof(range_u16_st);
		ret = copy_from_user(rule->port, req->port, len);
		if(ret) {
			printk("%s[%d]: copy_from_user failed, %d\n", __FILE__, __LINE__, ret);
			afw_free_rule(rule);
			ret = -1;
			goto out;
		}

		rule->port_num = req->port_num;
	}

	write_lock_bh(&afw_lock);
	ret = build_rule(rule);
	if(ret) {
		printk("%s[%d]: build_rule failed, %d\n", __FILE__, __LINE__, ret);
		afw_free_rule(rule);
		goto out_unlock;
	}

	if(req->type == AFW_RULE_TYPE_BLACK) {
		black_rule[black_rule_num] = rule;
		black_rule_num ++;
		if(rule->raw_rule_len > black_rule_max_len) {
			black_rule_max_len = rule->raw_rule_len;
		}
	}else {
		white_rule[white_rule_num] = rule;
		white_rule_num ++;
		if(rule->raw_rule_len > white_rule_max_len) {
			white_rule_max_len = rule->raw_rule_len;
		}
	}

	ret = 0;
out_unlock:
	write_unlock_bh(&afw_lock);
out:
	return ret;
}


int
afw_get_rule_num(afw_req_st *req)
{
	int ret;

	if(!req) {
		printk("%s[%d]: check parameter failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	if(req->type != AFW_RULE_TYPE_BLACK && req->type != AFW_RULE_TYPE_WHITE) {
		printk("%s[%d]: check parameter failed, %d\n", __FILE__, __LINE__, req->type);
		ret = -1;
		goto out;
	}

	if(req->type == AFW_RULE_TYPE_BLACK) {
		req->rule_num = black_rule_num;	
		req->max_rule_len = black_rule_max_len;	
	}else {
		req->rule_num = white_rule_num;	
		req->max_rule_len = white_rule_max_len;	
	}

	ret = 0;
out:
	return ret;
}


int
afw_get_rule(afw_req_st *req)
{
	int ret;
	afw_rule_st **rules;
	int len;
	int num;
	int i;

	if(!req || !req->data) {
		printk("%s[%d]: check parameter failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	if(req->type != AFW_RULE_TYPE_BLACK && req->type != AFW_RULE_TYPE_WHITE) {
		printk("%s[%d]: check parameter failed, %d\n", __FILE__, __LINE__, req->type);
		ret = -1;
		goto out;
	}

	read_lock_bh(&afw_lock);
	if(req->type == AFW_RULE_TYPE_BLACK) {
		if(req->rule_num < black_rule_num || req->max_rule_len < black_rule_max_len) {
			printk("%s[%d]: check parameter failed, %d, %d\n", __FILE__, __LINE__, req->rule_num, req->max_rule_len);
			ret = -1;
			goto out_unlock;
		}
		rules = black_rule;
		num = black_rule_num;
	}else {
		if(req->rule_num < white_rule_num || req->max_rule_len < white_rule_max_len) {
			printk("%s[%d]: check parameter failed, %d, %d\n", __FILE__, __LINE__, req->rule_num, req->max_rule_len);
			ret = -1;
			goto out_unlock;
		}
		rules = white_rule;
		num = white_rule_num;
	}
	
	len = 0;
	for(i = 0; i < num; i ++) {
		if(!rules[i]->raw_rule) {
			continue;
		}
		read_unlock_bh(&afw_lock);
		ret = copy_to_user(&req->data[len], rules[i]->raw_rule, rules[i]->raw_rule_len);
		read_lock_bh(&afw_lock);
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out_unlock;
		}
		len += req->max_rule_len;
	}
	req->rule_num = num;

	ret = 0;
out_unlock:
	read_unlock_bh(&afw_lock);
out:
	return ret;
}


int
afw_flush_rule(afw_req_st *req)
{
	int ret;

	memset(pid_cache, 0, sizeof(pid_cache));
	memset(pid_jiffies, 0, sizeof(pid_jiffies));

	write_lock_bh(&afw_lock);
	if(req->type == AFW_RULE_TYPE_BLACK || req->type == AFW_RULE_TYPE_ALL) {
		afw_free_black_rule();
		ret = afw_init_black_rule();
		if(ret) {
			printk("%s[%d]: afw_init_black_rule failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
	}

	if(req->type == AFW_RULE_TYPE_WHITE || req->type == AFW_RULE_TYPE_ALL) {
		afw_free_white_rule();
		ret = afw_init_white_rule();
		if(ret) {
			printk("%s[%d]: afw_init_white_rule failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
	}

	def_host_rule = AFW_RULE_TYPE_WHITE;

	ret = 0;
out:
	write_unlock_bh(&afw_lock);
	return ret;
}


static int afw_match_ipv4_slow(unsigned int dip, unsigned short dport, int type, unsigned int *rule_idx);


static int
afw_test_rule_ipv4(int type)
{
	int ret;
	afw_rule_st **afw_rule;
	rfc_rule_st *rfc_rule;
	unsigned int afw_rule_num;

	unsigned int ip;
	unsigned int ip1;
	unsigned int ip2;
	unsigned short port;
	unsigned short port1;
	unsigned short port2;

	rfc_field_u16_st **fu16;
	cbm_st *cbm = NULL;
	cbm_st *cbm2 = NULL;
	unsigned long bitmap;
	int cur_bitmap_size;

	int eqidh1;
	int eqidh2;
	int eqidh3;
	int eqidh4;

	int eqidl1;
	int eqidl2;
	int eqidl3;
	int eqidl4;

	int eqid_dport;

	unsigned long long start;
	unsigned long long end;
	unsigned long long normal_cost;
	unsigned long long fast_cost;
	unsigned long long match_slow_cost;

	int match_normal_num;
	int match_fast_num;
	int match_slow_num;
	int flag;

	unsigned int rule_idx;
	int idx;
	int i;
	int j;
	int k;

	if(type == AFW_RULE_TYPE_BLACK) {
		printk("=============================test black ipv4 rule=========================================\n");
		afw_rule = black_rule_ipv4;
		afw_rule_num = black_rule_ipv4_num;
		rfc_rule = &rfc_black_rule_ipv4;
	}else if(type == AFW_RULE_TYPE_WHITE) {
		printk("=============================test whilte ipv4 rule=========================================\n");
		afw_rule = white_rule_ipv4;
		afw_rule_num = white_rule_ipv4_num;
		rfc_rule = &rfc_white_rule_ipv4;
	}else {
		printk("%s[%d]: invalid type, type[%d]\n", __FILE__, __LINE__, type);
		ret = -1;
		goto out;
	}

	//rfc_print_rule(rfc_rule);

	#if 0
	if(!rfc_rule->fu16) {
		printk("%s[%d]: rfc_rule->fiedl_u16 is null\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}
	#endif

	cbm = malloc_cbm(1);
	if(!cbm) {
		printk("%s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	cbm2 = malloc_cbm(1);
	if(!cbm2) {
		printk("%s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	fu16 = rfc_rule->fu16;
	cur_bitmap_size = rfc_rule->cur_bitmap_size;

	for(i = 1; i <= 100; i ++) {
		for(j = 1; j <= 100; j ++) {
			idx = (i - 1) * 100 + j - 1;
			keys[idx].ip = 0xc0a80000 | (i << 8) | j;
			keys[idx].port = i * j;
		}
	}

	for(i = 0; i < TEST_NUM; i ++) {
		ip = keys[i].ip;
		port = keys[i].port;

		memset(cbm, 0, sizeof(cbm_st));
		for(j = 0; j < afw_rule_num; j ++) {
			ip1 = afw_rule[j]->ip.v1;
			ip2 = afw_rule[j]->ip.v2;
			if(ip < ip1 || ip > ip2) {
				continue;
			}

			for(k = 0; k < afw_rule[j]->port_num; k ++) {
				port1 = afw_rule[j]->port[k].v1;
				port2 = afw_rule[j]->port[k].v2;
				if(port >= port1 && port <= port2) {
					//printk("key[%d] match rule[%d]\n", i, j);
					set_bit(afw_rule[j]->rule_idx, cbm->bitmap);
					break;
				}
			}
		}

		idx = ip >> 16;
		eqidh1 = fu16[TYPE_DIP_H1]->chunk[idx];
		eqidh2 = fu16[TYPE_DIP_H2]->chunk[idx];
		eqidh3 = fu16[TYPE_DIP_H3]->chunk[idx];
		eqidh4 = fu16[TYPE_DIP_H4]->chunk[idx];

		idx = ip & 0x0000ffff;
		eqidl1 = fu16[TYPE_DIP_L1]->chunk[idx];
		eqidl2 = fu16[TYPE_DIP_L2]->chunk[idx];
		eqidl3 = fu16[TYPE_DIP_L3]->chunk[idx];
		eqidl4 = fu16[TYPE_DIP_L4]->chunk[idx];

		idx = port;
		eqid_dport = fu16[TYPE_DPORT]->chunk[idx];

		memset(cbm2, 0, sizeof(cbm_st));
		for(j = 0; j < cur_bitmap_size; j ++) {
			cbm2->bitmap[j] = (fu16[TYPE_DIP_H1]->cbm[eqidh1]->bitmap[j] & fu16[TYPE_DIP_L1]->cbm[eqidl1]->bitmap[j]);
			cbm2->bitmap[j] |= (fu16[TYPE_DIP_H2]->cbm[eqidh2]->bitmap[j] & fu16[TYPE_DIP_L2]->cbm[eqidl2]->bitmap[j]);
			cbm2->bitmap[j] |= (fu16[TYPE_DIP_H3]->cbm[eqidh3]->bitmap[j] & fu16[TYPE_DIP_L3]->cbm[eqidl3]->bitmap[j]);
			cbm2->bitmap[j] |= (fu16[TYPE_DIP_H4]->cbm[eqidh4]->bitmap[j] & fu16[TYPE_DIP_L4]->cbm[eqidl4]->bitmap[j]);
			cbm2->bitmap[j] &= fu16[TYPE_DPORT]->cbm[eqid_dport]->bitmap[j];
		}

		if(memcmp(cbm, cbm2, sizeof(cbm_st))) {
			printk("===========================!!!NOT MATCH========================\n");
		}
	}

	start = get_cycles();
	match_normal_num = 0;
	for(i = 0; i < TEST_NUM; i ++) {
		ip = keys[i].ip;
		port = keys[i].port;

		flag = 0;
		memset(cbm, 0, sizeof(cbm_st));
		for(j = 0; j < afw_rule_num; j ++) {
			ip1 = afw_rule[j]->ip.v1;
			ip2 = afw_rule[j]->ip.v2;
			if(ip < ip1 || ip > ip2) {
				continue;
			}

			if(!afw_rule[j]->port_num) {
				flag = 1;
				break;
			}

			for(k = 0; k < afw_rule[j]->port_num; k ++) {
				port1 = afw_rule[j]->port[k].v1;
				port2 = afw_rule[j]->port[k].v2;
				if(port >= port1 && port <= port2) {
					set_bit(afw_rule[j]->rule_idx, cbm->bitmap);
					flag = 1;
					break;
				}
			}
		}
		match_normal_num += flag;
	}
	end = get_cycles();
	normal_cost = end - start;


	start = get_cycles();
	match_fast_num = 0;
	for(i = 0; i < TEST_NUM; i ++) {
		ip = keys[i].ip;
		port = keys[i].port;

		idx = ip >> 16;
		eqidh1 = fu16[TYPE_DIP_H1]->chunk[idx];
		eqidh2 = fu16[TYPE_DIP_H2]->chunk[idx];
		eqidh3 = fu16[TYPE_DIP_H3]->chunk[idx];
		eqidh4 = fu16[TYPE_DIP_H4]->chunk[idx];

		idx = ip & 0x0000ffff;
		eqidl1 = fu16[TYPE_DIP_L1]->chunk[idx];
		eqidl2 = fu16[TYPE_DIP_L2]->chunk[idx];
		eqidl3 = fu16[TYPE_DIP_L3]->chunk[idx];
		eqidl4 = fu16[TYPE_DIP_L4]->chunk[idx];

		idx = port;
		eqid_dport = fu16[TYPE_DPORT]->chunk[idx];

		flag = 0;
		bitmap = 0;
		for(j = 0; j < cur_bitmap_size; j ++) {
			bitmap |= (fu16[TYPE_DIP_H1]->cbm[eqidh1]->bitmap[j] & fu16[TYPE_DIP_L1]->cbm[eqidl1]->bitmap[j]);
			bitmap |= (fu16[TYPE_DIP_H2]->cbm[eqidh2]->bitmap[j] & fu16[TYPE_DIP_L2]->cbm[eqidl2]->bitmap[j]);
			bitmap |= (fu16[TYPE_DIP_H3]->cbm[eqidh3]->bitmap[j] & fu16[TYPE_DIP_L3]->cbm[eqidl3]->bitmap[j]);
			bitmap |= (fu16[TYPE_DIP_H4]->cbm[eqidh4]->bitmap[j] & fu16[TYPE_DIP_L4]->cbm[eqidl4]->bitmap[j]);
			bitmap &= fu16[TYPE_DPORT]->cbm[eqid_dport]->bitmap[j];
			if(bitmap) {
				flag = 1;
				break;
			}
		}

		match_fast_num += flag;
	}
	end = get_cycles();
	fast_cost = end - start;


	ip = 0x12345678;
	port = 10000;
	match_slow_num = 0;
	start = get_cycles();
	for(i = 0; i < TEST_NUM; i ++) {
		ret = afw_match_ipv4_slow(ip, port, type, &rule_idx);
		if(!ret) {
			match_slow_num ++;
		}
	}
	end = get_cycles();
	match_slow_cost = end - start;


	printk("\n");
	printk("rules_num:		%d\n", type == AFW_RULE_TYPE_BLACK ? black_rule_ipv4_num : white_rule_ipv4_num);
	printk("test_times:		%d\n", TEST_NUM);
	printk("match_normal_num:	%d\n", match_normal_num);
	printk("match_fast_num:		%d\n", match_fast_num);
	printk("\n");

	printk("rules_num:		%d\n", afw_rule_num);
	printk("test_times:		%d\n", TEST_NUM);
	printk("normal-match-cost:	%lld\n", normal_cost);
	printk("fast-match-cost:	%lld\n", fast_cost);
	printk("\n");

	printk("\n");
	printk("rules_num:		%d\n", type == AFW_RULE_TYPE_BLACK ? black_rule_ipv4_num : white_rule_ipv4_num);
	printk("test_times:		%d\n", TEST_NUM);
	printk("match_slow_num:		%d\n", match_slow_num);
	printk("match_slow_cost:	%lld\n", match_slow_cost);
	printk("\n");


	ret = 0;
out:
	if(cbm) {
		free_cbm(cbm);
	}
	if(cbm2) {
		free_cbm(cbm2);
	}
	return ret;
}


static int afw_match_ipv6_slow(unsigned char *dip, unsigned short dport, int type, unsigned int *rule_idx);
static int afw_match_ipv6_rfc(unsigned char *dip, unsigned short dport, int type, unsigned int *rule_idx);


static int
afw_test_rule_ipv6(int type)
{
	int ret;
	char dip[16];
	unsigned int *p;
	unsigned short dport;
	unsigned int rule_idx;

	unsigned long long start;
	unsigned long long end;
	unsigned long long match_cost;
	unsigned long long match_cost_rfc;
	unsigned int match_num;
	unsigned int match_num_rfc;
	int i;

	memset(dip, 0, sizeof(dip));
	p = (unsigned int *)dip;
	p[0] = ntohl(0xfec00000);
	rule_idx = 0;

	match_num = 0;
	start = get_cycles();
	for(i = 0; i < TEST_NUM; i ++) {
		p[1] = ntohl((i << 16) | i);
		dport = i * (i + 1);
		ret = afw_match_ipv6_slow(dip, dport, type, &rule_idx);
		if(!ret) {
			match_num ++;
		}
	}
	end = get_cycles();
	match_cost = end - start;

	match_num_rfc = 0;
	start = get_cycles();
	for(i = 0; i < TEST_NUM; i ++) {
		p[1] = ntohl((i << 16) | i);
		dport = i * (i + 1);
		ret = afw_match_ipv6_rfc(dip, dport, type, &rule_idx);
		if(!ret) {
			match_num_rfc ++;
		}
	}
	end = get_cycles();
	match_cost_rfc = end - start;


	printk("\n");
	printk("rules_num:		%d\n", type == AFW_RULE_TYPE_BLACK ? black_rule_ipv6_num : white_rule_ipv6_num);
	printk("test_times:		%d\n", TEST_NUM);
	printk("match_num:		%d\n", match_num);
	printk("match_num_rfc:		%d\n", match_num_rfc);
	printk("match_cost:		%lld\n", match_cost);
	printk("match_cost_rfc:		%lld\n", match_cost_rfc);
	printk("\n");

	return 0;
}


static int afw_match_domain(char *domain, int domain_len, unsigned short dport_domain, int type, unsigned int *rule_idx);
static int afw_match_domain_hash(char *domain, int domain_len, unsigned int dport_domain, int type, unsigned int *rule_idx);


static int
afw_test_rule_domain(int type)
{
	int ret;
	char domain[32];
	unsigned int domain_len;
	unsigned short dport_domain;
	unsigned int rule_idx;

	unsigned long long start;
	unsigned long long end;
	unsigned long long match_cost;
	unsigned long long match_cost_hash;
	unsigned int match_num;
	unsigned int match_num_hash;
	int i;

	rule_idx = 0;

	match_num = 0;
	start = get_cycles();
	for(i = 0; i < TEST_NUM; i ++) {
		sprintf(domain, "www-%d-%d.baidu.com", i, i);
		domain_len = strlen(domain);
		dport_domain = i * (i + 1);
		ret = afw_match_domain(domain, domain_len, dport_domain, type, &rule_idx);
		if(!ret) {
			match_num ++;
		}
	}
	end = get_cycles();
	match_cost = end - start;

	match_num_hash = 0;
	start = get_cycles();
	for(i = 0; i < TEST_NUM; i ++) {
		sprintf(domain, "www-%d-%d.baidu.com", i, i);
		domain_len = strlen(domain);
		dport_domain = i * (i + 1);
		ret = afw_match_domain_hash(domain, domain_len, dport_domain, type, &rule_idx);
		if(!ret) {
			match_num_hash ++;
		}
	}
	end = get_cycles();
	match_cost_hash = end - start;

	printk("\n");
	printk("rules_num:		%d\n", type == AFW_RULE_TYPE_BLACK ? black_rule_domain_num : white_rule_domain_num);
	printk("test_times:		%d\n", TEST_NUM);
	printk("match_num:		%d\n", match_num);
	printk("match_num_hash:		%d\n", match_num_hash);
	printk("match_cost:		%lld\n", match_cost);
	printk("match_cost_hash:	%lld\n", match_cost_hash);
	printk("\n");

	return 0;
}


int
afw_test_rule(afw_req_st *req)
{
	int ret;

	if(!req) {
		printk("%s[%d]: check parameter failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	if(req->type == AFW_RULE_TYPE_BLACK || req->type == AFW_RULE_TYPE_ALL) {
		if(req->kind == AFW_RULE_KIND_IPV4) {
			afw_test_rule_ipv4(AFW_RULE_TYPE_BLACK);
		}else if(req->kind == AFW_RULE_KIND_IPV6) {
			afw_test_rule_ipv6(AFW_RULE_TYPE_BLACK);
		}else if(req->kind == AFW_RULE_KIND_DOMAIN) {
			afw_test_rule_domain(AFW_RULE_TYPE_BLACK);
		}else if(req->kind == AFW_RULE_KIND_ALL) {
			afw_test_rule_ipv4(AFW_RULE_TYPE_BLACK);
			afw_test_rule_ipv6(AFW_RULE_TYPE_BLACK);
			afw_test_rule_domain(AFW_RULE_TYPE_BLACK);
		}
	}

	if(req->type == AFW_RULE_TYPE_WHITE || req->type == AFW_RULE_TYPE_ALL) {
		if(req->kind == AFW_RULE_KIND_IPV4) {
			afw_test_rule_ipv4(AFW_RULE_TYPE_WHITE);
		}else if(req->kind == AFW_RULE_KIND_IPV6) {
			afw_test_rule_ipv6(AFW_RULE_TYPE_WHITE);
		}else if(req->kind == AFW_RULE_KIND_DOMAIN) {
			afw_test_rule_domain(AFW_RULE_TYPE_WHITE);
		}else if(req->kind == AFW_RULE_KIND_ALL) {
			afw_test_rule_ipv4(AFW_RULE_TYPE_WHITE);
			afw_test_rule_ipv6(AFW_RULE_TYPE_WHITE);
			afw_test_rule_domain(AFW_RULE_TYPE_WHITE);
		}
	}

	ret = 0;
out:
	return ret;
}


static int
afw_match_ipv4_rfc(unsigned int dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	rfc_rule_st *rule;
	rfc_field_u16_st **fu16;
	unsigned long bitmap;
	int idx;
	int i;

	int eqidh1;
	int eqidh2;
	int eqidh3;
	int eqidh4;

	int eqidl1;
	int eqidl2;
	int eqidl3;
	int eqidl4;

	int eqid_dport;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = &rfc_black_rule_ipv4;
	}else {
		rule = &rfc_white_rule_ipv4;
	}

	#if 0
	if(!rule->fu16) {
		ret = -1;
		goto out;
	}
	#endif
	fu16 = rule->fu16;

	idx = dip >> 16;
	eqidh1 = fu16[TYPE_DIP_H1]->chunk[idx];
	eqidh2 = fu16[TYPE_DIP_H2]->chunk[idx];
	eqidh3 = fu16[TYPE_DIP_H3]->chunk[idx];
	eqidh4 = fu16[TYPE_DIP_H4]->chunk[idx];

	idx = dip & 0x0000ffff;
	eqidl1 = fu16[TYPE_DIP_L1]->chunk[idx];
	eqidl2 = fu16[TYPE_DIP_L2]->chunk[idx];
	eqidl3 = fu16[TYPE_DIP_L3]->chunk[idx];
	eqidl4 = fu16[TYPE_DIP_L4]->chunk[idx];

	idx = dport;
	eqid_dport = fu16[TYPE_DPORT]->chunk[idx];

	bitmap = 0;
	*rule_idx = 0;
	for(i = 0; i < rule->cur_bitmap_size; i ++) {
		bitmap |= (fu16[TYPE_DIP_H1]->cbm[eqidh1]->bitmap[i] & fu16[TYPE_DIP_L1]->cbm[eqidl1]->bitmap[i]);
		bitmap |= (fu16[TYPE_DIP_H2]->cbm[eqidh2]->bitmap[i] & fu16[TYPE_DIP_L2]->cbm[eqidl2]->bitmap[i]);
		bitmap |= (fu16[TYPE_DIP_H3]->cbm[eqidh3]->bitmap[i] & fu16[TYPE_DIP_L3]->cbm[eqidl3]->bitmap[i]);
		bitmap |= (fu16[TYPE_DIP_H4]->cbm[eqidh4]->bitmap[i] & fu16[TYPE_DIP_L4]->cbm[eqidl4]->bitmap[i]);
		bitmap &= fu16[TYPE_DPORT]->cbm[eqid_dport]->bitmap[i];
		if(bitmap) {
			*rule_idx += __ffs(bitmap);
			ret = 0;
			goto out;
		}
		*rule_idx += sizeof(unsigned long) * 8;
	}

	ret = -1;	
out:
	return ret;
}


static int
afw_match_ipv4_slow(unsigned int dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	afw_rule_st **rule;
	unsigned int num;
	unsigned int ip1;
	unsigned int ip2;
	unsigned short port1;
	unsigned short port2;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule_ipv4;
		num = black_rule_ipv4_num;
	}else {
		rule = white_rule_ipv4;
		num = white_rule_ipv4_num;
	}

	for(i = 0; i < num; i ++) {
		if(!rule[i]) {
			continue;
		}

		ip1 = rule[i]->ip.v1;
		ip2 = rule[i]->ip.v2;
		if(dip < ip1 || dip > ip2) {
			continue;
		}

		if(!rule[i]->port) {			/* match any dport */
			ret = 0;
			*rule_idx = rule[i]->rule_idx;
			goto out;
		}

		for(j = 0; j < rule[i]->port_num; j ++) {
			port1 = rule[i]->port[j].v1;
			port2 = rule[i]->port[j].v2;
			if(dport >= port1 && dport <= port2) {
				*rule_idx = rule[i]->rule_idx;
				ret = 0;
				goto out;
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_dns_ipv4(unsigned int dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	afw_rule_st **rule;
	unsigned int num;
	unsigned short port1;
	unsigned short port2;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule_dns;
		num = black_rule_dns_num;
	}else {
		rule = white_rule_dns;
		num = white_rule_dns_num;
	}

	for(i = 0; i < num; i ++) {
		if(!rule[i] || !rule[i]->dns_ipv4) {
			continue;
		}

		for(j = 0; j < rule[i]->dns_ipv4_num; j ++) {
			if(rule[i]->dns_ipv4[j] == dip) {
				break;
			}
		}

		if(j >= rule[i]->dns_ipv4_num) {
			continue;
		}

		if(!rule[i]->port) {			/* match any dport */
			ret = 0;
			*rule_idx = rule[i]->rule_idx + RULE_IDX_DNS_START;
			goto out;
		}

		for(j = 0; j < rule[i]->port_num; j ++) {
			port1 = rule[i]->port[j].v1;
			port2 = rule[i]->port[j].v2;
			if(dport >= port1 && dport <= port2) {
				*rule_idx = rule[i]->rule_idx + RULE_IDX_DNS_START;
				ret = 0;
				goto out;
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_domain(char *domain, int domain_len, unsigned short dport_domain, int type, unsigned int *rule_idx)
{
	int ret;
	int offset;
	afw_rule_st **rule;
	unsigned short port1;
	unsigned short port2;
	unsigned int num;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule_domain;
		num = black_rule_domain_num;
	}else {
		rule = white_rule_domain;
		num = white_rule_domain_num;
	}

	for(i = 0; i < num; i ++) {
		if(!rule[i] || !rule[i]->domain) {
			continue;
		}
		
		if(domain_len < rule[i]->domain_len) {
			continue;
		}

		offset = domain_len - rule[i]->domain_len;
		if(memcmp(rule[i]->domain, &domain[offset], rule[i]->domain_len)) {
			continue;
		}

		if(offset && !rule[i]->domain_any) {
			continue;
		}

		if(!dport_domain) {				/* DNS-req */
			if(rule[i]->port && type == AFW_RULE_TYPE_BLACK) {
				continue;
			}
			*rule_idx = rule[i]->rule_idx;
			ret = 0;
			goto out;
		}

		if(!rule[i]->port) {			/* match any dport */
			*rule_idx = rule[i]->rule_idx;
			ret = 0;
			goto out;
		}

		for(j = 0; j < rule[i]->port_num; j ++) {
			port1 = rule[i]->port[j].v1;
			port2 = rule[i]->port[j].v2;
			if(dport_domain >= port1 && dport_domain <= port2) {
				*rule_idx = rule[i]->rule_idx;
				ret = 0;
				goto out;
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_domain_hash(char *domain, int domain_len, unsigned int dport_domain, int type, unsigned int *rule_idx)
{
	int ret;
	unsigned int hash;
	afw_rule_st **rule_tab;
	afw_rule_st *rule;
	unsigned short port1;
	unsigned short port2;
	int offset;
	int len;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule_tab = black_rule_domain_hash;
	}else {
		rule_tab = white_rule_domain_hash;
	}

	for(i = 0; i < domain_len; i ++) {
		offset = domain_len - 1 - i;
		if((domain[offset] != '.' && offset) || !i) {
			continue;
		}

		if(offset) {
			offset = offset + 1;
			len = i;
		}else {
			len = i + 1;
		}
			
		hash = jhash(&domain[offset], len, 0);
		hash = hash & AFW_HASH_MASK;

		for(rule = rule_tab[hash]; rule; rule = rule->next) {
			if(rule->domain_len != len) {
				continue;
			}

			if(len != domain_len && !rule->domain_any) {
				continue;
			}

			if(memcmp(rule->domain, &domain[offset], len)) {
				continue;
			}

			if(!dport_domain) {				/* DNS-req */
				if(rule->port && type == AFW_RULE_TYPE_BLACK) {
					continue;
				}
				*rule_idx = rule->rule_idx;
				ret = 0;
				goto out;
			}

			if(dport_domain == MAX_PORT) {		/* DNS-rep: add dns rule */
				*rule_idx = rule->rule_idx;
				ret = 0;
				goto out;
			}

			if(!rule->port) {			/* match any dport */
				*rule_idx = rule->rule_idx;
				ret = 0;
				goto out;
			}

			for(j = 0; j < rule->port_num; j ++) {
				port1 = rule->port[j].v1;
				port2 = rule->port[j].v2;
				if(dport_domain >= port1 && dport_domain <= port2) {
					*rule_idx = rule->rule_idx;
					ret = 0;
					goto out;
				}
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_ipv6_slow(unsigned char *dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	afw_rule_st **rule;
	unsigned int num;
	unsigned char *ip1;
	unsigned char *ip2;
	unsigned short port1;
	unsigned short port2;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule_ipv6;
		num = black_rule_ipv6_num;
	}else {
		rule = white_rule_ipv6;
		num = white_rule_ipv6_num;
	}

	for(i = 0; i < num; i ++) {
		if(!rule[i]) {
			continue;
		}

		ip1 = rule[i]->ipv6.v1;
		ip2 = rule[i]->ipv6.v2;
		if(memcmp(dip, ip1, IPV6_ADDR_LEN) < 0 || memcmp(dip, ip2, IPV6_ADDR_LEN) > 0) {
			continue;
		}

		if(!rule[i]->port) {			/* match any dport */
			*rule_idx = rule[i]->rule_idx;
			ret = 0;
			goto out;
		}

		for(j = 0; j < rule[i]->port_num; j ++) {
			port1 = rule[i]->port[j].v1;
			port2 = rule[i]->port[j].v2;
			if(dport >= port1 && dport <= port2) {
				*rule_idx = rule[i]->rule_idx;
				ret = 0;
				goto out;
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_ipv6_rfc(unsigned char *dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	afw_rule_st *rule;
	rfc_rule_st *rfc_rule;
	rfc_field_u16_st **fu16;
	unsigned long *bitmap;
	unsigned long *p1;
	unsigned long *p2;
	unsigned long *p;
	int eqid_dport;
	int idx;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rfc_rule = &rfc_black_rule_ipv6;
	}else {
		rfc_rule = &rfc_white_rule_ipv6;
	}

	fu16 = rfc_rule->fu16;

	idx = dport;
	eqid_dport = fu16[TYPE_DPORT]->chunk[idx];

	*rule_idx = 0;
	bitmap = fu16[TYPE_DPORT]->cbm[eqid_dport]->bitmap;
	for(i = 0; i < rfc_rule->cur_bitmap_size; i ++) {
		if(!bitmap[i]) {
			continue;
		}

		for(j = 0; j < RFC_BITS_PER_LONG; j ++) {
			if(!test_bit(j, &bitmap[i])) {
				continue;
			}

			*rule_idx = sizeof(unsigned long) * 8 * i + j;
			if(type == AFW_RULE_TYPE_BLACK) {
				rule = black_rule[*rule_idx];
			}else {
				rule = white_rule[*rule_idx];
			}

			p = (unsigned long *)dip;
			p1 = (unsigned long *)rule->ipv6.v1;
			p2 = (unsigned long *)rule->ipv6.v2;
			//if(memcmp(dip, rule->ipv6.v1, IPV6_ADDR_LEN) < 0 || memcmp(dip, rule->ipv6.v2, IPV6_ADDR_LEN) > 0) {
			if((p[0] < p1[0]) || (p[0] == p1[0] && p[1] < p1[1]) || (p[0] > p2[0]) || (p[0] == p2[0] && p[1] > p2[1])) {
				continue;
			}

			ret = 0;
			goto out;
		}
	}

	ret = -1;	
out:
	return ret;
}


static int
afw_match_dns_ipv6(unsigned char *dip, unsigned short dport, int type, unsigned int *rule_idx)
{
	int ret;
	afw_rule_st **rule;
	unsigned int num;
	unsigned short port1;
	unsigned short port2;
	int i;
	int j;

	if(type == AFW_RULE_TYPE_BLACK) {
		rule = black_rule_dns_ipv6;
		num = black_rule_dns_ipv6_num;
	}else {
		rule = white_rule_dns_ipv6;
		num = white_rule_dns_ipv6_num;
	}

	for(i = 0; i < num; i ++) {
		if(!rule[i] || !rule[i]->dns_ipv6) {
			continue;
		}

		for(j = 0; j < rule[i]->dns_ipv6_num; j ++) {
			if(!memcmp(&rule[i]->dns_ipv6[j * IPV6_ADDR_LEN], dip, IPV6_ADDR_LEN)) {
				break;
			}
		}

		if(j >= rule[i]->dns_ipv6_num) {
			continue;
		}

		if(!rule[i]->port) {			/* match any dport */
			ret = 0;
			*rule_idx = rule[i]->rule_idx + RULE_IDX_DNS_START;
			goto out;
		}

		for(j = 0; j < rule[i]->port_num; j ++) {
			port1 = rule[i]->port[j].v1;
			port2 = rule[i]->port[j].v2;
			if(dport >= port1 && dport <= port2) {
				*rule_idx = rule[i]->rule_idx + RULE_IDX_DNS_START;
				ret = 0;
				goto out;
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static int
afw_match_uid(unsigned int uid, int type, unsigned int *rule_idx)
{
	int ret;
	int i;

	if(type == AFW_RULE_TYPE_BLACK) {
		if(uid < MAX_NORMAL_UID) {
			if(black_rule_uid[uid]) {
				*rule_idx = RULE_IDX_BLACK_UID;
				ret = 0;
				goto out;
			}
		}else {
			for(i = 0; i < black_rule_uid_large_num; i ++) {
				if(black_rule_uid_large[i] == uid) {
					*rule_idx = RULE_IDX_BLACK_UID;
					ret = 0;
					goto out;
				}
			}
		}
	}else {
		if(uid < MAX_NORMAL_UID) {
			if(white_rule_uid[uid]) {
				*rule_idx = RULE_IDX_WHITE_UID;
				ret = 0;
				goto out;
			}
		}else {
			for(i = 0; i < white_rule_uid_large_num; i ++) {
				if(white_rule_uid_large[i] == uid) {
					*rule_idx = RULE_IDX_WHITE_UID;
					ret = 0;
					goto out;
				}
			}
		}
	}

	ret = -1;
out:
	return ret;
}


static void
afw_set_pid_cache(unsigned short dport, unsigned short dport_domain)
{
	unsigned int pid;

	if(dport != HTTP_PORT && dport != HTTPS_PORT && dport != HTTP_PROXY &&
		dport_domain != HTTP_PORT && dport_domain != HTTPS_PORT && dport_domain != HTTP_PROXY) {
		goto out;
	}

	pid = current->pid;
	if(pid >= MAX_PID) {
		goto out;
	}

	pid_cache[pid] = 1;
	pid_jiffies[pid] = jiffies;

out:
	return;
}


static int
afw_match_pid_cache(unsigned short dport, unsigned short dport_domain)
{
	int ret;
	unsigned int pid;
	unsigned long time;

	if(dport != HTTP_PORT && dport != HTTPS_PORT && dport != HTTP_PROXY &&
		dport_domain != HTTP_PORT && dport_domain != HTTPS_PORT && dport_domain != HTTP_PROXY) {
		ret = -1;
		goto out;
	}

	pid = current->pid;
	if(pid >= MAX_PID) {
		ret = -1;
		goto out;
	}

	if(!pid_cache[pid]) {
		ret = -1;
		goto out;
	}

	time = jiffies - pid_jiffies[pid];
	if(time >= afw_conf.pid_tmout * HZ) {
		ret = -1;
		goto out;
	}

	ret = 0;
out:
	return ret;
}


int
afw_match_ipv4(unsigned int dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, unsigned short dport_domain, 
	unsigned int pkt_type, unsigned int uid)
{
	int ret;
	unsigned int rule_idx;
	unsigned int dip_domain;

	rule_idx = 0;

	read_lock_bh(&afw_lock);

	if(afw_conf.pass_dns) {
		if(dport == DNS_PORT) {
			rule_idx = RULE_IDX_PASS_DNS;
			afw_stat.ipv4_pass_dns_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_uid_num) {
		ret = afw_match_uid(uid, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv4_black_rule_uid_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_ipv4_num) {
		ret = afw_match_ipv4_rfc(dip, dport, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv4_black_rule_ipv4_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_dns_num) {
		ret = afw_match_dns_ipv4(dip, dport, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv4_black_rule_dns_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	dip_domain = 0;
	afw_inet_addr(domain, domain_len, &dip_domain);

	if(afw_conf.black_enable && domain_len) {
		if(!dip_domain && black_rule_domain_num) {
			ret = afw_match_domain_hash(domain, domain_len, dport_domain, AFW_RULE_TYPE_BLACK, &rule_idx);
			if(!ret) {
				afw_stat.ipv4_black_rule_domain_hit ++;
				ret = NF_DROP;
				goto out;
			}
		}else if(dip_domain && black_rule_ipv4_num) {
			ret = afw_match_ipv4_rfc(dip_domain, dport_domain, AFW_RULE_TYPE_BLACK, &rule_idx);
			if(!ret) {
				afw_stat.ipv4_black_rule_ipv4_hit ++;
				ret = NF_DROP;
				goto out;
			}
		}
	}

	if(afw_conf.white_enable && white_rule_uid_num) {
		ret = afw_match_uid(uid, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv4_white_rule_uid_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_ipv4_num) {
		ret = afw_match_ipv4_rfc(dip, dport, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv4_white_rule_ipv4_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_dns_num) {
		ret = afw_match_dns_ipv4(dip, dport, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			if(afw_conf.pid_cache) {
				afw_set_pid_cache(dport, dport_domain);
			}
			afw_stat.ipv4_white_rule_dns_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.white_enable && domain_len) {
		if(!dip_domain && white_rule_domain_num) {
			ret = afw_match_domain_hash(domain, domain_len, dport_domain, AFW_RULE_TYPE_WHITE, &rule_idx);
			if(!ret) {
				afw_stat.ipv4_white_rule_domain_hit ++;
				ret = NF_ACCEPT;
				goto out;
			}
		}else if(dip_domain && white_rule_ipv4_num) {
			ret = afw_match_ipv4_rfc(dip_domain, dport_domain, AFW_RULE_TYPE_WHITE, &rule_idx);
			if(!ret) {
				afw_stat.ipv4_white_rule_ipv4_hit ++;
				ret = NF_ACCEPT;
				goto out;
			}
		}
	}

	if(afw_conf.black_enable && black_rule_port_num) {
		if(def_black_port_rule[dport]) {
			afw_stat.ipv4_black_rule_port_hit ++;
			rule_idx = RULE_IDX_BLACK_PORT;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_port_num) {
		if(def_white_port_rule[dport]) {
			afw_stat.ipv4_white_rule_port_hit ++;
			rule_idx = RULE_IDX_WHITE_PORT;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.pid_cache) {
		ret = afw_match_pid_cache(dport, dport_domain);
		if(!ret) {
			afw_stat.ipv4_pid_cache_hit ++;
			rule_idx = RULE_IDX_PID_CACHE;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(def_host_rule == AFW_RULE_TYPE_BLACK) {
		afw_stat.ipv4_black_rule_host_hit ++;
		rule_idx = RULE_IDX_BLACK_HOST;
		ret = NF_DROP;
	}else {
		afw_stat.ipv4_white_rule_host_hit ++;
		rule_idx = RULE_IDX_WHITE_HOST;
		ret = NF_ACCEPT;
	}

out:
	if(ret == NF_ACCEPT) {
		afw_log_pkt_ipv4(dip, dport, proto, domain, domain_len, dport_domain, pkt_type, rule_idx, AFW_PKT_ACTION_ACCEPT, uid);
	}else {
		afw_log_pkt_ipv4(dip, dport, proto, domain, domain_len, dport_domain, pkt_type, rule_idx, AFW_PKT_ACTION_STOP, uid);
	}

	read_unlock_bh(&afw_lock);
	return ret;
}


int
afw_match_ipv6(unsigned char *dip, unsigned short dport, unsigned char proto, char *domain, int domain_len, unsigned short dport_domain, 
	unsigned int pkt_type, unsigned int uid)
{
	int ret;
	unsigned int rule_idx;

	rule_idx = 0;

	read_lock_bh(&afw_lock);

	if(afw_conf.pass_dns) {
		if(dport == DNS_PORT) {
			rule_idx = RULE_IDX_PASS_DNS;
			afw_stat.ipv6_pass_dns_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_uid_num) {
		ret = afw_match_uid(uid, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_black_rule_uid_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_ipv6_num) {
		ret = afw_match_ipv6_rfc(dip, dport, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_black_rule_ipv6_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_dns_ipv6_num) {
		ret = afw_match_dns_ipv6(dip, dport, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_black_rule_dns_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_domain_num && domain_len) {
		ret = afw_match_domain_hash(domain, domain_len, dport_domain, AFW_RULE_TYPE_BLACK, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_black_rule_domain_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_uid_num) {
		ret = afw_match_uid(uid, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_white_rule_uid_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_ipv6_num) {
		ret = afw_match_ipv6_rfc(dip, dport, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_white_rule_ipv6_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_dns_ipv6_num) {
		ret = afw_match_dns_ipv6(dip, dport, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_white_rule_dns_hit ++;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_domain_num && domain_len) {
		ret = afw_match_domain_hash(domain, domain_len, dport_domain, AFW_RULE_TYPE_WHITE, &rule_idx);
		if(!ret) {
			afw_stat.ipv6_white_rule_domain_hit ++;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(afw_conf.black_enable && black_rule_port_num) {
		if(def_black_port_rule[dport]) {
			afw_stat.ipv6_black_rule_port_hit ++;
			rule_idx = RULE_IDX_BLACK_PORT;
			ret = NF_DROP;
			goto out;
		}
	}

	if(afw_conf.white_enable && white_rule_port_num) {
		if(def_white_port_rule[dport]) {
			afw_stat.ipv6_white_rule_port_hit ++;
			rule_idx = RULE_IDX_WHITE_PORT;
			ret = NF_ACCEPT;
			goto out;
		}
	}

	if(def_host_rule == AFW_RULE_TYPE_BLACK) {
		afw_stat.ipv6_black_rule_host_hit ++;
		rule_idx = RULE_IDX_BLACK_HOST;
		ret = NF_DROP;
	}else {
		afw_stat.ipv6_white_rule_host_hit ++;
		rule_idx = RULE_IDX_WHITE_HOST;
		ret = NF_ACCEPT;
	}

out:
	if(ret == NF_ACCEPT) {
		afw_log_pkt_ipv6(dip, dport, proto, domain, domain_len, dport_domain, pkt_type, rule_idx, AFW_PKT_ACTION_ACCEPT, uid);
	}else {
		afw_log_pkt_ipv6(dip, dport, proto, domain, domain_len, dport_domain, pkt_type, rule_idx, AFW_PKT_ACTION_STOP, uid);
	}

	read_unlock_bh(&afw_lock);
	return ret;
}


int
afw_rule_init(void)
{
	int ret;

	memset(pid_cache, 0, sizeof(pid_cache));
	memset(pid_jiffies, 0, sizeof(pid_jiffies));

	ret = afw_init_black_rule();
	if(ret) {
		printk("%s[%d]: afw_init_black_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = afw_init_white_rule();
	if(ret) {
		printk("%s[%d]: afw_init_white_rule failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = afw_dns_init();
	if(ret) {
		printk("%s[%d]: afw_dns_init failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	def_host_rule = AFW_RULE_TYPE_WHITE;
	memset(keys, 0, sizeof(keys));

	ret = 0;
out:
	return ret;
}


void
afw_rule_exit(void)
{
	afw_dns_exit();
	afw_free_black_rule();
	afw_free_white_rule();

	return;
}


