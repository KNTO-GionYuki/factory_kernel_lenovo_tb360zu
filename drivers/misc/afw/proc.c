#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/in.h>

#include "include/afw.h"
#include "rfc.h"
#include "afw_kernel.h"
#include "rule_kernel.h"
#include "proc.h"


static int
afw_conf_read(struct seq_file *m, void *v)
{
	seq_printf(m, "\n");
	seq_printf(m, "%-32s%d\n", "afw_enable", afw_conf.afw_enable);
	seq_printf(m, "%-32s%d\n", "black_enable", afw_conf.black_enable);
	seq_printf(m, "%-32s%d\n", "white_enable", afw_conf.white_enable);
	seq_printf(m, "%-32s%d\n", "parse_input", afw_conf.parse_input);
	seq_printf(m, "%-32s%d\n", "pass_dns", afw_conf.pass_dns);
	seq_printf(m, "%-32s%d\n", "pid_cache", afw_conf.pid_cache);
	seq_printf(m, "%-32s%d\n", "pid_tmout", afw_conf.pid_tmout);
	seq_printf(m, "%-32s%d\n", "tcp_reset", afw_conf.tcp_reset);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s%d\n", "black_rule_num", black_rule_num);
	seq_printf(m, "%-32s%d\n", "white_rule_num", white_rule_num);
	seq_printf(m, "%-32s%d\n", "afw_dns_node_num", afw_dns_node_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s%d\n", "black_rule_max_len", black_rule_max_len);
	seq_printf(m, "%-32s%d\n", "white_rule_max_len", white_rule_max_len);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s%d\n", "black_rule_uid_num", black_rule_uid_num);
	seq_printf(m, "%-32s%d\n", "black_rule_uid_large_num", black_rule_uid_large_num);
	seq_printf(m, "%-32s%d\n", "black_rule_ipv4_num", black_rule_ipv4_num);
	seq_printf(m, "%-32s%d\n", "black_rule_ipv6_num", black_rule_ipv6_num);
	seq_printf(m, "%-32s%d\n", "black_rule_domain_num", black_rule_domain_num);
	seq_printf(m, "%-32s%d\n", "black_rule_dns_num", black_rule_dns_num);
	seq_printf(m, "%-32s%d\n", "black_rule_port_num", black_rule_port_num);
	seq_printf(m, "%-32s%d\n", "black_rule_host_num", black_rule_host_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s%d\n", "white_rule_uid_num", white_rule_uid_num);
	seq_printf(m, "%-32s%d\n", "white_rule_uid_large_num", white_rule_uid_large_num);
	seq_printf(m, "%-32s%d\n", "white_rule_ipv4_num", white_rule_ipv4_num);
	seq_printf(m, "%-32s%d\n", "white_rule_ipv6_num", white_rule_ipv6_num);
	seq_printf(m, "%-32s%d\n", "white_rule_domain_num", white_rule_domain_num);
	seq_printf(m, "%-32s%d\n", "white_rule_dns_num", white_rule_dns_num);
	seq_printf(m, "%-32s%d\n", "white_rule_port_num", white_rule_port_num);
	seq_printf(m, "%-32s%d\n", "white_rule_host_num", white_rule_host_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s%s\n", "def_host_rule", def_host_rule == AFW_RULE_TYPE_BLACK ? "black" : "white");
	seq_printf(m, "\n");

	return 0;
}


static int
afw_stat_read(struct seq_file *m, void *v)
{
	unsigned int total;

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "debug info:");
	seq_printf(m, "%-32s%d\n", "parse_dns_req_failed", afw_stat.parse_dns_req_failed);
	seq_printf(m, "%-32s%d\n", "parse_dns_rep_failed", afw_stat.parse_dns_rep_failed);
	seq_printf(m, "%-32s%d\n", "parse_dns_no_answer", afw_stat.parse_dns_no_answer);
	seq_printf(m, "%-32s%d\n", "parse_dns_no_answer2", afw_stat.parse_dns_no_answer2);
	seq_printf(m, "%-32s%d\n", "dns_addr_too_much", afw_stat.dns_addr_too_much);
	seq_printf(m, "%-32s%d\n", "dns_rule_ipv4_full", afw_stat.dns_rule_ipv4_full);
	seq_printf(m, "%-32s%d\n", "dns_node_ipv4_full", afw_stat.dns_node_ipv4_full);
	seq_printf(m, "%-32s%d\n", "dns_rule_ipv6_full", afw_stat.dns_rule_ipv6_full);
	seq_printf(m, "%-32s%d\n", "dns_node_ipv6_full", afw_stat.dns_node_ipv6_full);
	seq_printf(m, "%-32s%d\n", "dns_node_reach_limit", afw_stat.dns_node_reach_limit);
	seq_printf(m, "%-32s%d\n", "tcp_reset_num", afw_stat.tcp_reset_num);
	seq_printf(m, "\n");


	total = 0;
	seq_printf(m, "%-32s\n", "malloc memory info:");

	seq_printf(m, "%-32s%d\n", "malloc_rule", afw_stat.malloc_rule);
	total += afw_stat.malloc_rule;

	seq_printf(m, "%-32s%d\n", "malloc_rfc_field", afw_stat.malloc_rfc_field);
	total += afw_stat.malloc_rfc_field;

	seq_printf(m, "%-32s%d\n", "malloc_rfc_cbm", afw_stat.malloc_rfc_cbm);
	total += afw_stat.malloc_rfc_cbm;

	seq_printf(m, "%-32s%d\n", "malloc_dns_node", afw_stat.malloc_dns_node);
	total += afw_stat.malloc_dns_node;

	seq_printf(m, "%-32s%d\n", "malloc_dns_addr", afw_stat.malloc_dns_addr);
	total += afw_stat.malloc_dns_addr;

	seq_printf(m, "%-32s%d\n", "malloc_total", total);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv4 input stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv4_input_total_num", afw_stat.ipv4_input_total_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_total_len", afw_stat.ipv4_input_total_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_nonlinear", afw_stat.ipv4_input_nonlinear);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_tcp_num", afw_stat.ipv4_input_tcp_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_udp_num", afw_stat.ipv4_input_udp_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_other_num", afw_stat.ipv4_input_other_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_input_dns_num", afw_stat.ipv4_input_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv6 input stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv6_input_total_num", afw_stat.ipv6_input_total_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_total_len", afw_stat.ipv6_input_total_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_nonlinear", afw_stat.ipv6_input_nonlinear);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_tcp_num", afw_stat.ipv6_input_tcp_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_udp_num", afw_stat.ipv6_input_udp_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_other_num", afw_stat.ipv6_input_other_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_input_dns_num", afw_stat.ipv6_input_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv4 pkt-num stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv4_total_num", afw_stat.ipv4_total_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_invalid_num", afw_stat.ipv4_invalid_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_tcp_num", afw_stat.ipv4_tcp_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_udp_num", afw_stat.ipv4_udp_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_other_num", afw_stat.ipv4_other_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_http_num", afw_stat.ipv4_http_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_http_proxy_num", afw_stat.ipv4_http_proxy_num);
	seq_printf(m, "%-32s%ld\n", "ipv4_dns_num", afw_stat.ipv4_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv4 pkt-len stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv4_total_len", afw_stat.ipv4_total_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_tcp_len", afw_stat.ipv4_tcp_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_udp_len", afw_stat.ipv4_udp_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_other_len", afw_stat.ipv4_other_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_http_len", afw_stat.ipv4_http_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_http_proxy_len", afw_stat.ipv4_http_proxy_len);
	seq_printf(m, "%-32s%ld\n", "ipv4_dns_len", afw_stat.ipv4_dns_len);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv6 pkt-num stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv6_total_num", afw_stat.ipv6_total_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_invalid_num", afw_stat.ipv6_invalid_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_tcp_num", afw_stat.ipv6_tcp_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_udp_num", afw_stat.ipv6_udp_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_other_num", afw_stat.ipv6_other_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_http_num", afw_stat.ipv6_http_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_http_proxy_num", afw_stat.ipv6_http_proxy_num);
	seq_printf(m, "%-32s%ld\n", "ipv6_dns_num", afw_stat.ipv6_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv6 pkt-len stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv6_total_len", afw_stat.ipv6_total_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_tcp_len", afw_stat.ipv6_tcp_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_udp_len", afw_stat.ipv6_udp_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_other_len", afw_stat.ipv6_other_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_http_len", afw_stat.ipv6_http_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_http_proxy_len", afw_stat.ipv6_http_proxy_len);
	seq_printf(m, "%-32s%ld\n", "ipv6_dns_len", afw_stat.ipv6_dns_len);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv4 pkt rule-hit stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_uid_hit", afw_stat.ipv4_black_rule_uid_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_ipv4_hit", afw_stat.ipv4_black_rule_ipv4_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_dns_hit", afw_stat.ipv4_black_rule_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_domain_hit", afw_stat.ipv4_black_rule_domain_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_port_hit", afw_stat.ipv4_black_rule_port_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_black_rule_host_hit", afw_stat.ipv4_black_rule_host_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_uid_hit", afw_stat.ipv4_white_rule_uid_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_ipv4_hit", afw_stat.ipv4_white_rule_ipv4_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_dns_hit", afw_stat.ipv4_white_rule_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_domain_hit", afw_stat.ipv4_white_rule_domain_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_port_hit", afw_stat.ipv4_white_rule_port_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_white_rule_host_hit", afw_stat.ipv4_white_rule_host_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_pass_dns_hit", afw_stat.ipv4_pass_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv4_pid_cache_hit", afw_stat.ipv4_pid_cache_hit);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "ipv6 pkt rule-hit stats info:");
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_uid_hit", afw_stat.ipv6_black_rule_uid_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_ipv6_hit", afw_stat.ipv6_black_rule_ipv6_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_dns_hit", afw_stat.ipv6_black_rule_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_domain_hit", afw_stat.ipv6_black_rule_domain_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_port_hit", afw_stat.ipv6_black_rule_port_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_black_rule_host_hit", afw_stat.ipv6_black_rule_host_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_uid_hit", afw_stat.ipv6_white_rule_uid_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_ipv6_hit", afw_stat.ipv6_white_rule_ipv6_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_dns_hit", afw_stat.ipv6_white_rule_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_domain_hit", afw_stat.ipv6_white_rule_domain_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_port_hit", afw_stat.ipv6_white_rule_port_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_white_rule_host_hit", afw_stat.ipv6_white_rule_host_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_pass_dns_hit", afw_stat.ipv6_pass_dns_hit);
	seq_printf(m, "%-32s%ld\n", "ipv6_pid_cache_hit", afw_stat.ipv6_pid_cache_hit);
	seq_printf(m, "\n");

	return 0;
}


static int
afw_rfc_read(struct seq_file *m, void *v)
{
	unsigned int total;
	int i;

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "black ipv4-rule rfc info:");
	seq_printf(m, "%-32s%d\n", "max_rule_num", rfc_black_rule_ipv4.max_rule_num);
	seq_printf(m, "%-32s%d\n", "cur_bitmap_size", rfc_black_rule_ipv4.cur_bitmap_size);
	seq_printf(m, "%-32s%d\n", "fu8_num", rfc_black_rule_ipv4.fu8_num);
	seq_printf(m, "%-32s%d\n", "fu16_num", rfc_black_rule_ipv4.fu16_num);

	if(rfc_black_rule_ipv4.fu8_num) {
		seq_printf(m, "%-32s", "fu8_eqid");

		total = 0;
		for(i = 0; i < rfc_black_rule_ipv4.fu8_num; i ++) {
			seq_printf(m, "%06d ", rfc_black_rule_ipv4.fu8[i]->eqid);
			total += rfc_black_rule_ipv4.fu8[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu8_eqid_total", total);
	}

	if(rfc_black_rule_ipv4.fu16_num) {
		seq_printf(m, "%-32s", "fu16_eqid");

		total = 0;
		for(i = 0; i < rfc_black_rule_ipv4.fu16_num; i ++) {
			seq_printf(m, "%-6d ", rfc_black_rule_ipv4.fu16[i]->eqid);
			total += rfc_black_rule_ipv4.fu16[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu16_eqid_total", total);
	}
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "white ipv4-rule rfc info:");
	seq_printf(m, "%-32s%d\n", "max_rule_num", rfc_white_rule_ipv4.max_rule_num);
	seq_printf(m, "%-32s%d\n", "cur_bitmap_size", rfc_white_rule_ipv4.cur_bitmap_size);
	seq_printf(m, "%-32s%d\n", "fu8_num", rfc_white_rule_ipv4.fu8_num);
	seq_printf(m, "%-32s%d\n", "fu16_num", rfc_white_rule_ipv4.fu16_num);

	if(rfc_white_rule_ipv4.fu8_num) {
		seq_printf(m, "%-32s", "fu8_eqid");

		total = 0;
		for(i = 0; i < rfc_white_rule_ipv4.fu8_num; i ++) {
			seq_printf(m, "%06d ", rfc_white_rule_ipv4.fu8[i]->eqid);
			total += rfc_white_rule_ipv4.fu8[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu8_eqid_total", total);
	}

	if(rfc_white_rule_ipv4.fu16_num) {
		seq_printf(m, "%-32s", "fu16_eqid");

		total = 0;
		for(i = 0; i < rfc_white_rule_ipv4.fu16_num; i ++) {
			seq_printf(m, "%-6d ", rfc_white_rule_ipv4.fu16[i]->eqid);
			total += rfc_white_rule_ipv4.fu16[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu16_eqid_total", total);
	}
	seq_printf(m, "\n");

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "black ipv6-rule rfc info:");
	seq_printf(m, "%-32s%d\n", "max_rule_num", rfc_black_rule_ipv6.max_rule_num);
	seq_printf(m, "%-32s%d\n", "cur_bitmap_size", rfc_black_rule_ipv6.cur_bitmap_size);
	seq_printf(m, "%-32s%d\n", "fu8_num", rfc_black_rule_ipv6.fu8_num);
	seq_printf(m, "%-32s%d\n", "fu16_num", rfc_black_rule_ipv6.fu16_num);

	if(rfc_black_rule_ipv6.fu8_num) {
		seq_printf(m, "%-32s", "fu8_eqid");

		total = 0;
		for(i = 0; i < rfc_black_rule_ipv6.fu8_num; i ++) {
			seq_printf(m, "%06d ", rfc_black_rule_ipv6.fu8[i]->eqid);
			total += rfc_black_rule_ipv6.fu8[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu8_eqid_total", total);
	}

	if(rfc_black_rule_ipv6.fu16_num) {
		seq_printf(m, "%-32s", "fu16_eqid");

		total = 0;
		for(i = 0; i < rfc_black_rule_ipv6.fu16_num; i ++) {
			seq_printf(m, "%-6d ", rfc_black_rule_ipv6.fu16[i]->eqid);
			total += rfc_black_rule_ipv6.fu16[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu16_eqid_total", total);
	}
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "white ipv6-rule rfc info:");
	seq_printf(m, "%-32s%d\n", "max_rule_num", rfc_white_rule_ipv6.max_rule_num);
	seq_printf(m, "%-32s%d\n", "cur_bitmap_size", rfc_white_rule_ipv6.cur_bitmap_size);
	seq_printf(m, "%-32s%d\n", "fu8_num", rfc_white_rule_ipv6.fu8_num);
	seq_printf(m, "%-32s%d\n", "fu16_num", rfc_white_rule_ipv6.fu16_num);

	if(rfc_white_rule_ipv6.fu8_num) {
		seq_printf(m, "%-32s", "fu8_eqid");

		total = 0;
		for(i = 0; i < rfc_white_rule_ipv6.fu8_num; i ++) {
			seq_printf(m, "%06d ", rfc_white_rule_ipv6.fu8[i]->eqid);
			total += rfc_white_rule_ipv6.fu8[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu8_eqid_total", total);
	}

	if(rfc_white_rule_ipv6.fu16_num) {
		seq_printf(m, "%-32s", "fu16_eqid");

		total = 0;
		for(i = 0; i < rfc_white_rule_ipv6.fu16_num; i ++) {
			seq_printf(m, "%-6d ", rfc_white_rule_ipv6.fu16[i]->eqid);
			total += rfc_white_rule_ipv6.fu16[i]->eqid;
		}
		seq_printf(m, "\n");

		seq_printf(m, "%-32s%d\n", "fu16_eqid_total", total);
	}
	seq_printf(m, "\n");

	return 0;
}


static int
afw_dns_read(struct seq_file *m, void *v)
{
	int total;
	struct list_head *head;
	struct list_head *pos;
	struct list_head *next;
	afw_dns_node_st *node;
	unsigned char *p;
	int i;
	int j;

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "dns node info:");

	total = 0;
    for(i = 0; i < AFW_HASH_SIZE; i ++) {
		head = &dns_node_hash[i];
		list_for_each_safe(pos, next, head) {
			node = list_entry(pos, afw_dns_node_st, next);

			seq_printf(m, "  domain[%s] ipv4[", node->domain);
			for(j = 0; j < node->num; j ++) {
				seq_printf(m, "%08x ", node->ipv4[j]);
			}
			seq_printf(m, "]\n");

			seq_printf(m, "    ipv6[");
			for(j = 0; j < node->num_ipv6; j ++) {
				p = node->ipv6[j];
				seq_printf(m, "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x, ", 
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
			}
			seq_printf(m, "]\n");

			total ++;
		}
	}
	seq_printf(m, "  dns node num: %d\n", total);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "dynamic black dns rule info:");
	for(i = 0; i < black_rule_dns_num; i ++) {
		if(!black_rule_dns[i]) {
			continue;
		}

		seq_printf(m, "  raw_rule[%s] ipv4[", black_rule_dns[i]->raw_rule);
		for(j = 0; j < black_rule_dns[i]->dns_ipv4_num; j ++) {
			seq_printf(m, "%08x ", black_rule_dns[i]->dns_ipv4[j]);
		}
		seq_printf(m, "]\n");
	}
	seq_printf(m, "  black_rule_dns_num: %d\n", black_rule_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "dynamic white dns rule info:");
	for(i = 0; i < white_rule_dns_num; i ++) {
		if(!white_rule_dns[i]) {
			continue;
		}

		seq_printf(m, "  raw_rule[%s] ipv4[", white_rule_dns[i]->raw_rule);
		for(j = 0; j < white_rule_dns[i]->dns_ipv4_num; j ++) {
			seq_printf(m, "%08x ", white_rule_dns[i]->dns_ipv4[j]);
		}
		seq_printf(m, "]\n");
	}
	seq_printf(m, "  white_rule_dns_num: %d\n", white_rule_dns_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "dynamic black dns ipv6 rule info:");
	for(i = 0; i < black_rule_dns_ipv6_num; i ++) {
		if(!black_rule_dns_ipv6[i]) {
			continue;
		}

		seq_printf(m, "  raw_rule[%s] ipv6[", black_rule_dns_ipv6[i]->raw_rule);
		for(j = 0; j < black_rule_dns_ipv6[i]->dns_ipv6_num; j ++) {
			p = &black_rule_dns_ipv6[i]->dns_ipv6[j * IPV6_ADDR_LEN];
			seq_printf(m, "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x, ", 
				p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		}
		seq_printf(m, "]\n");
	}
	seq_printf(m, "  black_rule_dns_ipv6_num: %d\n", black_rule_dns_ipv6_num);
	seq_printf(m, "\n");

	seq_printf(m, "%-32s\n", "dynamic white dns ipv6 rule info:");
	for(i = 0; i < white_rule_dns_ipv6_num; i ++) {
		if(!white_rule_dns_ipv6[i]) {
			continue;
		}

		seq_printf(m, "  raw_rule[%s] ipv6[", white_rule_dns_ipv6[i]->raw_rule);
		for(j = 0; j < white_rule_dns_ipv6[i]->dns_ipv6_num; j ++) {
			p = &white_rule_dns_ipv6[i]->dns_ipv6[j * IPV6_ADDR_LEN];
			seq_printf(m, "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x, ", 
				p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
		}
		seq_printf(m, "]\n");
	}
	seq_printf(m, "  white_rule_dns_ipv6_num: %d\n", white_rule_dns_ipv6_num);
	seq_printf(m, "\n");

	return 0;
}


static int
afw_debug_read(struct seq_file *m, void *v)
{
	int len;
	int i;
	int j;

	seq_printf(m, "\n");
	seq_printf(m, "%-32s%d\n", "pkt_num", afw_debug.pkt_num);
	seq_printf(m, "\n");

	for(i = 0; i < afw_debug.pkt_num; i ++) {
		len = afw_debug.pkt_len[i];	
		seq_printf(m, "debug pkt info[%d], len[%d]\n", i, len);

		for(j = 0; j < len; j ++) {
			if(j && !(j % 16)) {
				seq_printf(m, "\n");
			}
			seq_printf(m, " %02x", afw_debug.pkt[i][j]);
		}

		seq_printf(m, "\n\n");
	}

	seq_printf(m, "\n");
	return 0;
}


static int
afw_rule_read(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "black rule info: ");

	for(i = 0; i < black_rule_num; i ++) {
		if(!black_rule[i]) {
			continue;
		}
		seq_printf(m, "  black_rule[%d]: %s\n", i, black_rule[i]->raw_rule);
	}

	seq_printf(m, "\n");
	seq_printf(m, "%-32s\n", "white rule info: ");

	for(i = 0; i < white_rule_num; i ++) {
		if(!white_rule[i]) {
			continue;
		}
		seq_printf(m, "  white_rule[%d]: %s\n", i, white_rule[i]->raw_rule);
	}

	seq_printf(m, "\n");
	return 0;
}


static char *action_name[AFW_PKT_ACTION_MAX] =
{
	"STOP",
	"ACCEPT",
};


static char *pkt_name[AFW_PKT_TYPE_MAX] =
{
	"ipv4",
	"ipv4_http",
	"ipv4_http_proxy",
	"ipv4_dns",
	"ipv6",
	"ipv6_http",
	"ipv6_http_proxy",
	"ipv6_dns",
};


static char *rule_name[RULE_IDX_MAX - RULE_IDX_SPEC_START] =
{
	"black_uid",
	"white_uid",
	"def-port-black",
	"def-port-white",
	"def-host-black",
	"def-host-white",
	"pass-dns",
	"pid-cache",
};


static int
afw_pkt_read(struct seq_file *m, void *v)
{
	int idx;
	int action;
	int pkt;
	unsigned int *dip;
	unsigned int rule_idx;
	unsigned char dip_str[48];
	unsigned char rule_str[16];
	unsigned char proto_str[16];
	int i;

	seq_printf(m, "\n");
	seq_printf(m, "HZ[%d]\n", HZ);
	seq_printf(m, "\n");

	idx = 0;
	for(i = afw_pkt_log_num; i < AFW_PKT_LOG_NUM; i ++) {
		if(!afw_pkt_log[i].jiffies) {
			continue;
		}

		dip = (unsigned int *)afw_pkt_log[i].dip;
		action = afw_pkt_log[i].action % AFW_PKT_ACTION_MAX;
		pkt = afw_pkt_log[i].pkt_type % AFW_PKT_TYPE_MAX;
		rule_idx = afw_pkt_log[i].rule_idx;

		if(afw_pkt_log[i].proto == IPPROTO_TCP) {
			strcpy(proto_str, "tcp");
		}else if(afw_pkt_log[i].proto == IPPROTO_UDP) {
			strcpy(proto_str, "udp");
		}else {
			strcpy(proto_str, "other");
		}

		if(pkt <= AFW_PKT_TYPE_IPV4_DNS) {
			snprintf(dip_str, sizeof(dip_str), "%08x", dip[0]);
		}else {
			snprintf(dip_str, sizeof(dip_str), "%08x %08x %08x %08x", htonl(dip[0]), htonl(dip[1]), htonl(dip[2]), htonl(dip[3]));
		}

		if(rule_idx < RULE_IDX_DNS_START) {
			snprintf(rule_str, sizeof(rule_str), "%d", rule_idx);
		}else if(rule_idx < RULE_IDX_SPEC_START) {
			rule_idx -= RULE_IDX_DNS_START;
			snprintf(rule_str, sizeof(rule_str), "%d-dns", rule_idx);
		}else {
			rule_idx -= RULE_IDX_SPEC_START;
			rule_idx = rule_idx % RULE_IDX_MAX;
			snprintf(rule_str, sizeof(rule_str), "%s", rule_name[rule_idx]);
		}

		seq_printf(m, "idx[%05d] uid[%d] dip[%s] dport[%d] proto[%s] domain[%s] domain_len[%d] dport_domain[%d] "
			"pkt_type[%s] rule_idx[%s] action[%s] jiffies[%ld] pid[%d] comm[%s]\n", idx, 
			afw_pkt_log[i].uid, dip_str, afw_pkt_log[i].dport, proto_str, afw_pkt_log[i].domain, afw_pkt_log[i].domain_len, 
			afw_pkt_log[i].dport_domain, pkt_name[pkt], rule_str, action_name[action], jiffies - afw_pkt_log[i].jiffies, 
			afw_pkt_log[i].pid, afw_pkt_log[i].comm);

		idx ++;
	}

	for(i = 0; i < afw_pkt_log_num; i ++) {
		if(!afw_pkt_log[i].jiffies) {
			continue;
		}

		dip = (unsigned int *)afw_pkt_log[i].dip;
		action = afw_pkt_log[i].action % AFW_PKT_ACTION_MAX;
		pkt = afw_pkt_log[i].pkt_type % AFW_PKT_TYPE_MAX;
		rule_idx = afw_pkt_log[i].rule_idx;

		if(afw_pkt_log[i].proto == IPPROTO_TCP) {
			strcpy(proto_str, "tcp");
		}else if(afw_pkt_log[i].proto == IPPROTO_UDP) {
			strcpy(proto_str, "udp");
		}else {
			strcpy(proto_str, "other");
		}

		if(pkt <= AFW_PKT_TYPE_IPV4_DNS) {
			snprintf(dip_str, sizeof(dip_str), "%08x", dip[0]);
		}else {
			snprintf(dip_str, sizeof(dip_str), "%08x %08x %08x %08x", htonl(dip[0]), htonl(dip[1]), htonl(dip[2]), htonl(dip[3]));
		}

		if(rule_idx < RULE_IDX_DNS_START) {
			snprintf(rule_str, sizeof(rule_str), "%d", rule_idx);
		}else if(rule_idx < RULE_IDX_SPEC_START) {
			rule_idx -= RULE_IDX_DNS_START;
			snprintf(rule_str, sizeof(rule_str), "%d-dns", rule_idx);
		}else {
			rule_idx -= RULE_IDX_SPEC_START;
			rule_idx = rule_idx % ( RULE_IDX_MAX % RULE_IDX_SPEC_START );
			snprintf(rule_str, sizeof(rule_str), "%s", rule_name[rule_idx]);
		}

		seq_printf(m, "idx[%05d] uid[%d] dip[%s] dport[%d] proto[%s] domain[%s] domain_len[%d] dport_domain[%d] "
			"pkt_type[%s] rule_idx[%s] action[%s] jiffies[%ld] pid[%d] comm[%s]\n", idx, 
			afw_pkt_log[i].uid, dip_str, afw_pkt_log[i].dport, proto_str, afw_pkt_log[i].domain, afw_pkt_log[i].domain_len, 
			afw_pkt_log[i].dport_domain, pkt_name[pkt], rule_str, action_name[action], jiffies - afw_pkt_log[i].jiffies, 
			afw_pkt_log[i].pid, afw_pkt_log[i].comm);

		idx ++;
	}

	seq_printf(m, "\n");

	return 0;
}


static ssize_t
afw_conf_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_stat_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_pkt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_rfc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_dns_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static ssize_t
afw_rule_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	return count;
}


static int
afw_conf_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_conf_read, NULL);
}


static int
afw_stat_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_stat_read, NULL);
}


static int
afw_pkt_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_pkt_read, NULL);
}


static int
afw_rfc_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_rfc_read, NULL);
}


static int
afw_dns_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_dns_read, NULL);
}


static int
afw_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_debug_read, NULL);
}


static int
afw_rule_open(struct inode *inode, struct file *file)
{
    return single_open(file, afw_rule_read, NULL);
}


static struct file_operations afw_conf_fops = {
    .open = afw_conf_open,
    .read = seq_read,
    .write = afw_conf_write,
};


static struct file_operations afw_stat_fops = {
    .open = afw_stat_open,
    .read = seq_read,
    .write = afw_stat_write,
};


static struct file_operations afw_pkt_fops = {
    .open = afw_pkt_open,
    .read = seq_read,
    .write = afw_pkt_write,
};


static struct file_operations afw_rfc_fops = {
    .open = afw_rfc_open,
    .read = seq_read,
    .write = afw_rfc_write,
};


static struct file_operations afw_dns_fops = {
    .open = afw_dns_open,
    .read = seq_read,
    .write = afw_dns_write,
};


static struct file_operations afw_debug_fops = {
    .open = afw_debug_open,
    .read = seq_read,
    .write = afw_debug_write,
};


static struct file_operations afw_rule_fops = {
    .open = afw_rule_open,
    .read = seq_read,
    .write = afw_rule_write,
};


int
afw_proc_init(void)
{
	int ret;
	struct proc_dir_entry *proc;

	proc = proc_create(AFW_CONF_PROC_FILE, 0644, NULL, &afw_conf_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_CONF_PROC_FILE);
		ret = -1;
        goto out;
    }

	proc = proc_create(AFW_STAT_PROC_FILE, 0644, NULL, &afw_stat_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_STAT_PROC_FILE);
		ret = -1;
        goto out_conf;
    }

	proc = proc_create(AFW_PKT_PROC_FILE, 0644, NULL, &afw_pkt_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_PKT_PROC_FILE);
		ret = -1;
        goto out_stat;
    }

	proc = proc_create(AFW_RFC_PROC_FILE, 0644, NULL, &afw_rfc_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_RFC_PROC_FILE);
		ret = -1;
        goto out_pkt;
    }

	proc = proc_create(AFW_DNS_PROC_FILE, 0644, NULL, &afw_dns_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_DNS_PROC_FILE);
		ret = -1;
        goto out_rfc;
    }

	proc = proc_create(AFW_DEBUG_PROC_FILE, 0644, NULL, &afw_debug_fops);
    if(!proc) {
        pr_info("[AFW] %s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_DEBUG_PROC_FILE);
		ret = -1;
        goto out_dns;
    }

	proc = proc_create(AFW_RULE_FILE, 0644, NULL, &afw_rule_fops);
    if(!proc) {
        printk("%s[%d]: proc_create failed, %s\n", __FILE__, __LINE__, AFW_RULE_FILE);
		ret = -1;
        goto out_debug;
    }

	ret = 0;
	goto out;
out_debug:
	remove_proc_entry(AFW_DEBUG_PROC_FILE, NULL);
out_dns:
	remove_proc_entry(AFW_DNS_PROC_FILE, NULL);
out_rfc:
	remove_proc_entry(AFW_RFC_PROC_FILE, NULL);
out_pkt:
	remove_proc_entry(AFW_PKT_PROC_FILE, NULL);
out_stat:
	remove_proc_entry(AFW_STAT_PROC_FILE, NULL);
out_conf:
	remove_proc_entry(AFW_CONF_PROC_FILE, NULL);
out:
	return ret;
}


void
afw_proc_exit(void)
{
	remove_proc_entry(AFW_DEBUG_PROC_FILE, NULL);
	remove_proc_entry(AFW_DNS_PROC_FILE, NULL);
	remove_proc_entry(AFW_RFC_PROC_FILE, NULL);
	remove_proc_entry(AFW_PKT_PROC_FILE, NULL);
	remove_proc_entry(AFW_STAT_PROC_FILE, NULL);
	remove_proc_entry(AFW_CONF_PROC_FILE, NULL);
	remove_proc_entry(AFW_RULE_FILE, NULL);

	return;
}


