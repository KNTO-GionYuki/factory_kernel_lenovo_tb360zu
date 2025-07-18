#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/netfilter_ipv6.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include "include/afw.h"
#include "rfc.h"
#include "afw_kernel.h"
#include "rule_kernel.h"
#include "proc.h"

extern int nf_register_net_hooks(struct net *net, const struct nf_hook_ops *reg,
                          unsigned int n);
extern void nf_unregister_net_hooks(struct net *net, const struct nf_hook_ops *reg,
                             unsigned int n);
 

afw_conf_st afw_conf;
afw_stat_st afw_stat;
afw_debug_st afw_debug;
afw_pkt_log_st afw_pkt_log[AFW_PKT_LOG_NUM];
unsigned int afw_pkt_log_num;

static struct class *driver_class;
static dev_t afw_client_device_no;
static struct cdev afw_client_cdev;

static int
add_debug_pkt(struct sk_buff *skb)
{
	int ret;
	int len;
	int idx;
	int i;

	if(afw_debug.pkt_num >= MAX_DEBUG_PKT_NUM) {
		ret = 0;
		goto out;
	}

	len = skb->len;
	if(len > MAX_DEBUG_PKT_LEN) {
		len = MAX_DEBUG_PKT_LEN;
	}

	for(i = 0; i < afw_debug.pkt_num; i ++) {
		if(!memcmp(skb->data, afw_debug.pkt[i], len)) {
			ret = 0;
			goto out;
		}
	}

	idx = afw_debug.pkt_num;
	memcpy(afw_debug.pkt[idx], skb->data, len);
	afw_debug.pkt_len[idx] = len;
	afw_debug.pkt_num ++;

	ret = 0;
out:
	return ret;
}


static int
afw_atoi(char *str)
{
	unsigned int ret;
	unsigned int tmp;
	int len;
	int i;
	int j;

	len = strlen(str);
	ret = 0;
	for(i = 0; i < len; i ++) {
		tmp = (str[i] - '0');
		for(j = 0; j < (len - 1 - i); j ++) {
			tmp *= 10;
		}
		ret += tmp;
	}

	return ret;

}


static void
afw_conf_init(void)
{
	memset(&afw_conf, 0, sizeof(afw_conf));
	memset(&afw_stat, 0, sizeof(afw_stat));
	memset(&afw_debug, 0, sizeof(afw_debug));
	memset(afw_pkt_log, 0, sizeof(afw_pkt_log));
	afw_pkt_log_num = 0;

	afw_conf.afw_enable = 0;
	afw_conf.black_enable = 1;
	afw_conf.white_enable = 1;
	afw_conf.parse_input = 1;

	afw_conf.pass_dns = 1;
	afw_conf.pid_cache = 0;
	afw_conf.pid_tmout = 60;

	afw_conf.tcp_reset = 1;

	return;
}


static int
afw_set_enable(afw_req_st *req)
{
	afw_conf.afw_enable = req->enable;
	return 0;
}


static int
afw_get_enable(afw_req_st *req)
{
	req->enable = afw_conf.afw_enable;
	return 0;
}


static int
afw_set_black_enable(afw_req_st *req)
{
	afw_conf.black_enable = req->enable;
	return 0;
}


static int
afw_get_black_enable(afw_req_st *req)
{
	req->enable = afw_conf.black_enable;
	return 0;
}


static int
afw_set_white_enable(afw_req_st *req)
{
	afw_conf.white_enable = req->enable;
	return 0;
}


static int
afw_get_white_enable(afw_req_st *req)
{
	req->enable = afw_conf.white_enable;
	return 0;
}


static int
afw_set_parse_input_enable(afw_req_st *req)
{
	afw_conf.parse_input = req->enable;
	return 0;
}


static int
afw_get_parse_input_enable(afw_req_st *req)
{
	req->enable = afw_conf.parse_input;
	return 0;
}


static int
afw_set_pass_dns_enable(afw_req_st *req)
{
	afw_conf.pass_dns = req->enable;
	return 0;
}


static int
afw_get_pass_dns_enable(afw_req_st *req)
{
	req->enable = afw_conf.pass_dns;
	return 0;
}


static int
afw_set_pid_cache_enable(afw_req_st *req)
{
	afw_conf.pid_cache = req->enable;
	return 0;
}


static int
afw_get_pid_cache_enable(afw_req_st *req)
{
	req->enable = afw_conf.pid_cache;
	return 0;
}


static int
afw_set_pid_timeout(afw_req_st *req)
{
	afw_conf.pid_tmout = req->timeout;
	return 0;
}


static int
afw_get_pid_timeout(afw_req_st *req)
{
	req->timeout = afw_conf.pid_tmout;
	return 0;
}


static int
afw_set_tcp_reset_enable(afw_req_st *req)
{
	afw_conf.tcp_reset = req->enable;
	return 0;
}


static int
afw_get_tcp_reset_enable(afw_req_st *req)
{
	req->enable = afw_conf.tcp_reset;
	return 0;
}


static long
afw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	afw_req_st req;

	ret = copy_from_user(&req, (void *)arg, sizeof(req));
	if(ret) {
		printk("%s[%d]: copy_from_user failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	switch(req.cmd) {
	case AFW_CMD_SET_AFW_ENABLE:
		ret = afw_set_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_AFW_ENABLE:
		ret = afw_get_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_BLACK_ENABLE:
		ret = afw_set_black_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_black_enable failed, %d\n", __FILE__, __LINE__, ret);
		}
		goto out;

	case AFW_CMD_GET_BLACK_ENABLE:
		ret = afw_get_black_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_black_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_WHITE_ENABLE:
		ret = afw_set_white_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_white_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_WHITE_ENABLE:
		ret = afw_get_white_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_white_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_PARSE_INPUT_ENABLE:
		ret = afw_set_parse_input_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_parse_input_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_PARSE_INPUT_ENABLE:
		ret = afw_get_parse_input_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_parse_input_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_PASS_DNS_ENABLE:
		ret = afw_set_pass_dns_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_pass_dns_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_PASS_DNS_ENABLE:
		ret = afw_get_pass_dns_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_pass_dns_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_PID_CACHE_ENABLE:
		ret = afw_set_pid_cache_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_pid_cache_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_PID_CACHE_ENABLE:
		ret = afw_get_pid_cache_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_pid_cache_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_TCP_RESET_ENABLE:
		ret = afw_set_tcp_reset_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_set_tcp_reset_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_TCP_RESET_ENABLE:
		ret = afw_get_tcp_reset_enable(&req);
		if(ret) {
			printk("%s[%d]: afw_get_tcp_reset_enable failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_ADD_RULE:
		ret = afw_add_rule(&req);
		if(ret) {
			printk("%s[%d]: afw_add_rule failed, %d\n", __FILE__, __LINE__, ret);
		}
		goto out;

	case AFW_CMD_GET_RULE_NUM:
		ret = afw_get_rule_num(&req);
		if(ret) {
			printk("%s[%d]: afw_get_rule_num failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_RULE:
		ret = afw_get_rule(&req);
		if(ret) {
			printk("%s[%d]: afw_get_rule failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_FLUSH_RULE:
		ret = afw_flush_rule(&req);
		if(ret) {
			printk("%s[%d]: afw_flush_rule failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_TEST_RULE:
		ret = afw_test_rule(&req);
		if(ret) {
			printk("%s[%d]: afw_test_rule failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_SET_PID_TIMEOUT:
		ret = afw_set_pid_timeout(&req);
		if(ret) {
			printk("%s[%d]: afw_set_pidtime failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	case AFW_CMD_GET_PID_TIMEOUT:
		ret = afw_get_pid_timeout(&req);
		if(ret) {
			printk("%s[%d]: afw_get_pidtime failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		ret = copy_to_user((void *)arg, &req, sizeof(req));
		if(ret) {
			printk("%s[%d]: copy_to_user failed, %d\n", __FILE__, __LINE__, ret);
			goto out;
		}
		goto out;

	default:
		ret = -1;
		goto out;
	}

	ret = 0;
out:
	return ret;
}


static struct file_operations afw_fops = {
	owner: THIS_MODULE,
	unlocked_ioctl:afw_ioctl,
};


/* data example
 * 1)Host: www.baidu.com\r\n
 * 2)Host: www.baidu.com:443\r\n
 */
static int
parse_http_host(char *data, int dlen, char *domain, unsigned int *domain_len, unsigned short *port)
{
	int ret;
	int start;
	int end;
	int start_port;
	int end_port;
	char buf[16];
	int len;
	int i;

	if(dlen <= 7) {
		ret = -1;
		goto out;
	}

	for(i = 0; i < dlen - 4; i ++) {
		if((data[i] == 'H' || data[i] == 'h') &&
			(data[i+1] == 'O' || data[i+1] == 'o') &&
			(data[i+2] == 'S' || data[i+2] == 's') &&
			(data[i+3] == 'T' || data[i+3] == 't') &&
			(data[i+4] == ':')) {
			break;
		}
	}

	if(i >= (dlen - 4)) {
		ret = -1;
		goto out;
	}

	i += 5;
	start = 0;
	end = 0;
	start_port = 0;
	end_port = 0;
	for(; i < dlen - 1; i ++) {
		if(data[i] == ' ') {
			continue;
		}

		if(!start) {
			start = i;
		}

		if(data[i] == ':') {
			end = i;
			start_port = i + 1;
		}

		if(data[i] == '\r' && data[i+1] == '\n') {
			if(!end) {
				end = i;
			}else {
				end_port = i;
			}
			break;
		}
	}

	if(end > start) {
		len = end - start;
		if(len >= MAX_DOMAIN_LEN) {
			ret = -1;
			goto out;
		}
		memcpy(domain, &data[start], len);
		domain[len] = 0;
		*domain_len = len;
	}

	if(end_port > start_port) {
		len = end_port - start_port;
		if(len >= sizeof(buf)) {
			ret = -1;
			goto out;
		}
		memcpy(buf, &data[start_port], len);
		buf[len] = 0;
		*port = afw_atoi(buf);
	}

	ret = 0;
out:
	return ret;
}


static int
parse_dns_req(char *data, int dlen, char *domain, unsigned int *domain_len, int is_udp)
{
	int ret;
	int len;
	int length;
	int left;
	int i;

	if(dlen <= 12) {
		afw_stat.parse_dns_req_failed ++;
		ret = -1;
		goto out;
	}

	if(is_udp) {
		data += 12;
		dlen -= 12;
	}else {
		data += 14;
		dlen -= 14;
	}

	length = 0;
	left = dlen;
	for(i = 0; i < dlen; i ++) {
		len = data[0];
		if(!len) {
			break;
		}

		if(left < len) {
			afw_stat.parse_dns_req_failed ++;
			ret = -1;
			goto out;
		}
		left --;
		data ++;
		if(i) {
			if((length + 1) >= MAX_DOMAIN_LEN) {
				afw_stat.parse_dns_req_failed ++;
				ret = -1;
				goto out;
			}

			*domain = '.';
			domain ++;
			length ++;
		}

		if((length + len) >= MAX_DOMAIN_LEN) {
			afw_stat.parse_dns_req_failed ++;
			ret = -1;
			goto out;
		}

		memcpy(domain, data, len);
		left -= len;
		data += len;
		domain += len;
		length += len;

		i += len;
	}

	*domain = 0;
	*domain_len = length;

	ret = 0;
out:
	return ret;
}


static int
parse_dns_rep(char *data, int dlen, char *domain, unsigned int *domain_len, unsigned int *dns_ipv4, unsigned int *dns_ipv4_num, 
	unsigned char dns_ipv6[][IPV6_ADDR_LEN], unsigned int *dns_ipv6_num)
{
	int ret;
	afw_dns_header_st *dns_header;
	afw_dns_answer_st *dns_answer;
	int len;
	int num;
	int length;
	int left;
	int type;
	int idx;
	int idx6;
	int i;

	if(dlen <= 12) {
		afw_stat.parse_dns_rep_failed ++;
		ret = -1;
		goto out;
	}

	dns_header = (afw_dns_header_st *)data;
	num = ntohs(dns_header->query_num);
	if(num != 1) {
		afw_stat.parse_dns_rep_failed ++;
		ret = -1;
		goto out;
	}

	num = ntohs(dns_header->answer_num);
	if(!num) {
		afw_stat.parse_dns_no_answer ++;
		ret = 0;
		goto out;
	}

	if(num > MAX_DNS_ADDR) {
		num = MAX_DNS_ADDR;
		afw_stat.dns_addr_too_much ++;
	}

	data += sizeof(*dns_header);			/* skip dns header */
	dlen -= sizeof(*dns_header);

	length = 0;
	left = dlen;
	for(i = 0; i < dlen; i ++) {
		len = data[0];
		if(!len) {
			left --;
			data ++;
			break;
		}

		if(left < len) {
			afw_stat.parse_dns_rep_failed ++;
			ret = -1;
			goto out;
		}
		left --;
		data ++;
		if(i) {
			if((length + 1) >= MAX_DOMAIN_LEN) {
				afw_stat.parse_dns_rep_failed ++;
				ret = -1;
				goto out;
			}

			*domain = '.';
			domain ++;
			length ++;
		}

		if((length + len) >= MAX_DOMAIN_LEN) {
			afw_stat.parse_dns_rep_failed ++;
			ret = -1;
			goto out;
		}

		memcpy(domain, data, len);
		left -= len;
		data += len;
		domain += len;
		length += len;

		i += len;
	}

	left -= 4;				/* skip type and class in queries */
	data += 4;

	*domain = 0;
	*domain_len = length;

	idx = 0;
	idx6 = 0;
	for(i = 0; i < num; i ++) {
		if((data[0] & DNS_COMPRESS_FLAG) == DNS_COMPRESS_FLAG) {
			data += 2;
			left -= 2;
		}else {
			len = strlen(data);
			data += len + 1;
			left -= len + 1;
		}

		if(left < sizeof(*dns_answer)) {
			afw_stat.parse_dns_rep_failed ++;
			ret = -1;
			goto out;
		}

		dns_answer = (afw_dns_answer_st *)data;
		data += sizeof(*dns_answer);
		left -= sizeof(*dns_answer);

		type = ntohs(dns_answer->type);
		len = ntohs(dns_answer->len);
		if(type == 1 && len == 4) {
			if(left < 4) {
				afw_stat.parse_dns_rep_failed ++;
				ret = -1;
				goto out;
			}
			dns_ipv4[idx] = ntohl(*(unsigned int *)data);
			idx ++;
		}else if(type == 0x1c && len == 16) {
			if(left < 16) {
				afw_stat.parse_dns_rep_failed ++;
				ret = -1;
				goto out;
			}
			memcpy(dns_ipv6[idx6], data, 16);
			idx6 ++;
		}
		data += len;
		left -= len;
	}

	if(!idx && !idx6) {
		afw_stat.parse_dns_no_answer2 ++;
	}

	*dns_ipv4_num = idx;
	*dns_ipv6_num = idx6;
	ret = 0;
out:
	return ret;
}


static unsigned int
afw_output_ipv4(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	int ret;
	kuid_t kuid;
	unsigned int uid;
	struct iphdr *iph;
	struct tcphdr *tcph;
	struct udphdr *udph;

	unsigned int dip;
	unsigned short dport;
	unsigned short dport_domain;
	unsigned char *data;
	unsigned int sign;
	unsigned char domain[MAX_DOMAIN_LEN];
	unsigned int domain_len;
	unsigned int pkt_type;
	int dlen;

	afw_stat.ipv4_total_num ++;
	afw_stat.ipv4_total_len += skb->len;

	if(!afw_conf.afw_enable) {
		ret = NF_ACCEPT;
		goto out;
	}

	iph = ip_hdr(skb);
	dip = ntohl(iph->daddr);

	dlen = ntohs(iph->tot_len);
	if(dlen < skb->len) {
		afw_stat.ipv4_invalid_num ++;
		ret = NF_ACCEPT;
		goto out;
	}

	kuid = current_uid();
	uid = kuid.val;

	domain[0] = 0;
	domain_len = 0;
	dport_domain = 0;
	pkt_type = AFW_PKT_TYPE_IPV4;
	if(iph->protocol == IPPROTO_TCP) {
		afw_stat.ipv4_tcp_num ++;
		afw_stat.ipv4_tcp_len += skb->len;

		tcph = (struct tcphdr *)((void *)iph + (iph->ihl << 2));
		dport = ntohs(tcph->dest);

		#if 0
		if(!black_rule_domain_num && !white_rule_domain_num) {
			goto match;
		}
		#endif

		data = (void *)tcph + (tcph->doff << 2);
		dlen = dlen - (iph->ihl << 2) - (tcph->doff << 2);
		if(dport == DNS_PORT) {
			afw_stat.ipv4_dns_num ++;
			afw_stat.ipv4_dns_len += skb->len;

			parse_dns_req(data, dlen, domain, &domain_len, 0);
			pkt_type = AFW_PKT_TYPE_IPV4_DNS;

			#if 0
			if(domain_len) {
				printk("ipv4-dns-req: domain[%s], domain_len[%d]\n", domain, domain_len);
			}
			#endif

			goto match;
		}

		sign = *(unsigned int *)data;
		if(sign == HTTP_CMD_GET || sign == HTTP_CMD_GET_LOWER || 
			sign == HTTP_CMD_POST || sign == HTTP_CMD_POST_LOWER || 
			sign == HTTP_CMD_CONNECT || sign == HTTP_CMD_CONNECT_LOWER || 
			sign == HTTP_CMD_PUT || sign == HTTP_CMD_PUT_LOWER) {
			parse_http_host(data, dlen, domain, &domain_len, &dport_domain);
			if(!dport_domain) {
				afw_stat.ipv4_http_num ++;
				afw_stat.ipv4_http_len += skb->len;

				dport_domain = DEF_HTTP_PORT;
				pkt_type = AFW_PKT_TYPE_IPV4_HTTP;
			}else {
				afw_stat.ipv4_http_proxy_num ++;
				afw_stat.ipv4_http_proxy_len += skb->len;

				pkt_type = AFW_PKT_TYPE_IPV4_HTTP_PROXY;
			}
			#if 0 
			if(domain_len) {
				printk("ipv4-http: domain[%s], domain_len[%d], dport_domain[%d]\n", domain, domain_len, dport_domain);
			}
			#endif
		}
	}else if(iph->protocol == IPPROTO_UDP) {
		afw_stat.ipv4_udp_num ++;
		afw_stat.ipv4_udp_len += skb->len;

		udph = (struct udphdr *)((void *)iph + (iph->ihl << 2));
		dport = ntohs(udph->dest);

		if(!black_rule_domain_num && !white_rule_domain_num) {
			goto match;
		}

		data = (void *)&udph[1];
		dlen = dlen - (iph->ihl << 2) - sizeof(*udph);
		if(dport == DNS_PORT) {
			afw_stat.ipv4_dns_num ++;
			afw_stat.ipv4_dns_len += skb->len;

			parse_dns_req(data, dlen, domain, &domain_len, 1);
			pkt_type = AFW_PKT_TYPE_IPV4_DNS;
			#if 0
			if(domain_len) {
				printk("ipv4-dns-req: domain[%s], domain_len[%d]\n", domain, domain_len);
			}
			#endif
		}
	}else {
		afw_stat.ipv4_other_num ++;
		afw_stat.ipv4_other_len += skb->len;

		ret = NF_ACCEPT;
		goto out;
	}

match:
	ret = afw_match_ipv4(dip, dport, iph->protocol, domain, domain_len, dport_domain, pkt_type, uid);
	if(afw_conf.tcp_reset && ret == NF_DROP && iph->protocol == IPPROTO_TCP) {
		if(state && state->sk && state->sk->sk_state == TCP_SYN_SENT) {
			afw_stat.tcp_reset_num ++;
			ret |= (ECONNREFUSED << NF_VERDICT_QBITS);
		}
	}
out:
	return ret;
}


static unsigned int
afw_input_ipv4(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	int ret;
	struct iphdr *iph;
	struct udphdr *udph;
	unsigned char *data;
	unsigned char domain[MAX_DOMAIN_LEN];
	unsigned int domain_len;
	unsigned int dns_ipv4[MAX_DNS_ADDR];
	unsigned int dns_ipv4_num;
	unsigned char dns_ipv6[MAX_DNS_ADDR][IPV6_ADDR_LEN];
	unsigned int dns_ipv6_num;
	int dlen;
	
	afw_stat.ipv4_input_total_num ++;
	afw_stat.ipv4_input_total_len += skb->len;

	if(!afw_conf.parse_input) {
		ret = NF_ACCEPT;
		goto out;
	}

	#if 0
	if(!black_rule_domain_num && !white_rule_domain_num) {
		ret = NF_ACCEPT;
		goto out;
	}
	#endif

	if(skb_is_nonlinear(skb)) {
		afw_stat.ipv4_input_nonlinear ++;
		skb_linearize(skb);
	}
	iph = ip_hdr(skb);

	if(iph->protocol == IPPROTO_TCP) {
		afw_stat.ipv4_input_tcp_num ++;
	}else if(iph->protocol == IPPROTO_UDP) {
		afw_stat.ipv4_input_udp_num ++;
	}else {
		afw_stat.ipv4_input_other_num ++;
	}

	if(iph->protocol != IPPROTO_UDP) {
		ret = NF_ACCEPT;
		goto out;
	}

	udph = (struct udphdr *)((void *)iph + (iph->ihl << 2));
	if(udph->source != htons(DNS_PORT)) {
		ret = NF_ACCEPT;
		goto out;
	}

	afw_stat.ipv4_input_dns_num ++;

	dlen = ntohs(iph->tot_len);
	if(dlen > skb->len) {
		ret = NF_ACCEPT;
		goto out;
	}

	data = (void *)&udph[1];
	dlen = dlen - (iph->ihl << 2) - sizeof(*udph);

	domain[0] = 0;
	domain_len = 0;
	dns_ipv4_num = 0;
	memset(dns_ipv4, 0, sizeof(dns_ipv4));
	dns_ipv6_num = 0;
	memset(dns_ipv6, 0, sizeof(dns_ipv6));
	ret = parse_dns_rep(data, dlen, domain, &domain_len, dns_ipv4, &dns_ipv4_num, dns_ipv6, &dns_ipv6_num);
	if(ret) {
		add_debug_pkt(skb);
		ret = NF_ACCEPT;
		goto out;
	}

	if(!dns_ipv4_num) {
		add_debug_pkt(skb);
	}

	#if 0
	if(domain_len) {
		printk("ipv4-dns-rep: domain[%s], domain_len[%d], dns_ipv4[%08x, %08x, %08x, %08x], dns_ipv4_num[%d]\n", 
			domain, domain_len, dns_ipv4[0], dns_ipv4[1], dns_ipv4[2], dns_ipv4[3], dns_ipv4_num);
	}
	#endif

	afw_add_dns_node(domain, domain_len, dns_ipv4, dns_ipv4_num);
	if(dns_ipv6_num) {
		afw_add_dns_node_ipv6(domain, domain_len, dns_ipv6, dns_ipv6_num);
	}
	ret = NF_ACCEPT;
out:
	return ret;
}


static unsigned int
afw_output_ipv6(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	int ret;
	kuid_t kuid;
	unsigned int uid;
	struct ipv6hdr *iph;
	struct tcphdr *tcph;
	struct udphdr *udph;

	struct in6_addr *dip;
	unsigned short dport;
	unsigned short dport_domain;
	unsigned char *data;
	unsigned int sign;
	unsigned char domain[MAX_DOMAIN_LEN];
	unsigned int domain_len;
	unsigned int pkt_type;
	int dlen;

	afw_stat.ipv6_total_num ++;
	afw_stat.ipv6_total_len += skb->len;

	if(!afw_conf.afw_enable) {
		ret = NF_ACCEPT;
		goto out;
	}

	iph = ipv6_hdr(skb);
	dip = &iph->daddr;

	dlen = ntohs(iph->payload_len);
	if((dlen + sizeof(*iph)) < skb->len) {
		afw_stat.ipv6_invalid_num ++;
		ret = NF_ACCEPT;
		goto out;
	}

	kuid = current_uid();
	uid = kuid.val;

	domain[0] = 0;
	domain_len = 0;
	dport_domain = 0;
	pkt_type = AFW_PKT_TYPE_IPV6;
	if(iph->nexthdr == IPPROTO_TCP) {
		afw_stat.ipv6_tcp_num ++;
		afw_stat.ipv6_tcp_len += skb->len;

		tcph = (struct tcphdr *)&iph[1];
		dport = ntohs(tcph->dest);

		#if 0
		if(!black_rule_domain_num && !white_rule_domain_num) {
			goto match;
		}
		#endif

		data = (void *)tcph + (tcph->doff << 2);
		dlen = dlen - (tcph->doff << 2);
		if(dport == DNS_PORT) {
			afw_stat.ipv6_dns_num ++;
			afw_stat.ipv6_dns_len += skb->len;

			parse_dns_req(data, dlen, domain, &domain_len, 0);
			pkt_type = AFW_PKT_TYPE_IPV6_DNS;

			#if 0
			if(domain_len) {
				printk("ipv6-dns-req: domain[%s], domain_len[%d]\n", domain, domain_len);
			}
			#endif

			goto match;
		}

		sign = *(unsigned int *)data;
		if(sign == HTTP_CMD_GET || sign == HTTP_CMD_GET_LOWER || 
			sign == HTTP_CMD_POST || sign == HTTP_CMD_POST_LOWER || 
			sign == HTTP_CMD_CONNECT || sign == HTTP_CMD_CONNECT_LOWER || 
			sign == HTTP_CMD_PUT || sign == HTTP_CMD_PUT_LOWER) {
			parse_http_host(data, dlen, domain, &domain_len, &dport_domain);
			if(!dport_domain) {
				afw_stat.ipv6_http_num ++;
				afw_stat.ipv6_http_len += skb->len;

				dport_domain = dport;
				pkt_type = AFW_PKT_TYPE_IPV6_HTTP;
			}else {
				afw_stat.ipv6_http_proxy_num ++;
				afw_stat.ipv6_http_proxy_len += skb->len;

				pkt_type = AFW_PKT_TYPE_IPV6_HTTP_PROXY;
			}
			#if 0
			if(domain_len) {
				printk("ipv6-http: domain[%s], domain_len[%d], dport_domain[%d]\n", domain, domain_len, dport_domain);
			}
			#endif
		}
	}else if(iph->nexthdr == IPPROTO_UDP) {
		afw_stat.ipv6_udp_num ++;
		afw_stat.ipv6_udp_len += skb->len;

		udph = (struct udphdr *)&iph[1];
		dport = ntohs(udph->dest);

		if(!black_rule_domain_num && !white_rule_domain_num) {
			goto match;
		}

		data = (void *)&udph[1];
		dlen = dlen - sizeof(*udph);
		if(dport == DNS_PORT) {
			afw_stat.ipv6_dns_num ++;
			afw_stat.ipv6_dns_len += skb->len;

			parse_dns_req(data, dlen, domain, &domain_len, 1);
			pkt_type = AFW_PKT_TYPE_IPV6_DNS;
			#if 0
			if(domain_len) {
				printk("ipv6-dns-req: domain[%s], domain_len[%d]\n", domain, domain_len);
			}
			#endif
		}
	}else {
		afw_stat.ipv6_other_num ++;
		afw_stat.ipv6_other_len += skb->len;

		ret = NF_ACCEPT;
		goto out;
	}

match:
	ret = afw_match_ipv6((unsigned char *)dip, dport, iph->nexthdr, domain, domain_len, dport_domain, pkt_type, uid);
out:
	return ret;
}


static unsigned int
afw_input_ipv6(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	int ret;
	struct ipv6hdr *iph;
	struct udphdr *udph;
	unsigned char *data;
	unsigned char domain[MAX_DOMAIN_LEN];
	unsigned int domain_len;
	unsigned int dns_ipv4[MAX_DNS_ADDR];
	unsigned int dns_ipv4_num;
	unsigned char dns_ipv6[MAX_DNS_ADDR][IPV6_ADDR_LEN];
	unsigned int dns_ipv6_num;
	int dlen;
	
	afw_stat.ipv6_input_total_num ++;
	afw_stat.ipv6_input_total_len += skb->len;

	if(!afw_conf.parse_input) {
		ret = NF_ACCEPT;
		goto out;
	}

	#if 0
	if(!black_rule_domain_num && !white_rule_domain_num) {
		ret = NF_ACCEPT;
		goto out;
	}
	#endif

	if(skb_is_nonlinear(skb)) {
		afw_stat.ipv6_input_nonlinear ++;
		skb_linearize(skb);
	}
	iph = ipv6_hdr(skb);

	if(iph->nexthdr == IPPROTO_TCP) {
		afw_stat.ipv6_input_tcp_num ++;
	}else if(iph->nexthdr == IPPROTO_UDP) {
		afw_stat.ipv6_input_udp_num ++;
	}else {
		afw_stat.ipv6_input_other_num ++;
	}

	if(iph->nexthdr != IPPROTO_UDP) {
		ret = NF_ACCEPT;
		goto out;
	}

	udph = (struct udphdr *)&iph[1];
	if(udph->source != htons(DNS_PORT)) {
		ret = NF_ACCEPT;
		goto out;
	}

	afw_stat.ipv6_input_dns_num ++;

	dlen = ntohs(iph->payload_len);
	if((dlen + sizeof(*iph)) > skb->len) {
		ret = NF_ACCEPT;
		goto out;
	}

	data = (void *)&udph[1];
	dlen = dlen - sizeof(*udph);

	domain[0] = 0;
	domain_len = 0;
	dns_ipv4_num = 0;
	memset(dns_ipv4, 0, sizeof(dns_ipv4));
	dns_ipv6_num = 0;
	memset(dns_ipv6, 0, sizeof(dns_ipv6));
	ret = parse_dns_rep(data, dlen, domain, &domain_len, dns_ipv4, &dns_ipv4_num, dns_ipv6, &dns_ipv6_num);
	if(ret) {
		add_debug_pkt(skb);
		ret = NF_ACCEPT;
		goto out;
	}

	if(!dns_ipv4_num) {
		add_debug_pkt(skb);
	}

	#if 0
	if(domain_len) {
		printk("ipv6-dns-rep: domain[%s], domain_len[%d], dns_ipv4[%08x, %08x, %08x, %08x], dns_ipv4_num[%d]\n", 
			domain, domain_len, dns_ipv4[0], dns_ipv4[1], dns_ipv4[2], dns_ipv4[3], dns_ipv4_num);
	}
	#endif

	afw_add_dns_node(domain, domain_len, dns_ipv4, dns_ipv4_num);
	if(dns_ipv6_num) {
		afw_add_dns_node_ipv6(domain, domain_len, dns_ipv6, dns_ipv6_num);
	}
	ret = NF_ACCEPT;
out:
	return ret;
}


static const struct nf_hook_ops afw_ops[] = {
	{
		.hook = afw_output_ipv4,
		.pf = NFPROTO_IPV4,
		.hooknum = NF_INET_LOCAL_OUT,
		.priority = NF_IP_PRI_FIRST,
	},	

	{
		.hook = afw_input_ipv4,
		.pf = NFPROTO_IPV4,
		.hooknum = NF_INET_LOCAL_IN,
		.priority = NF_IP_PRI_FIRST,
	},	

	{
		.hook = afw_output_ipv6,
		.pf = NFPROTO_IPV6,
		.hooknum = NF_INET_LOCAL_OUT,
		.priority = NF_IP6_PRI_FIRST,
	},

	{
		.hook = afw_input_ipv6,
		.pf = NFPROTO_IPV6,
		.hooknum = NF_INET_LOCAL_IN,
		.priority = NF_IP6_PRI_FIRST,
	},
};


static int afw_probe(struct platform_device *pdev)
{
	pr_info("[AFW] afw afw_probe.\n");
	return 0;
}

static int afw_remove(struct platform_device *pdev)
{
	pr_info("[AFW] afw afw_remove.\n");
	return 0;
}

static struct platform_driver afw_driver = {
	.probe = afw_probe,
	.remove = afw_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "afw",
		.owner = THIS_MODULE,
	},
};


int
afw_init(void)
{
	int ret;
	struct device *class_dev = NULL;
	pr_info("[AFW] afw module_init.\n");
	afw_conf_init();
	pr_info("[AFW] afw_conf_init returned.\n");
	ret = rfc_init();
	if(ret) {
		pr_info("%s[%d]: rfc_init failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}
	pr_info("[AFW] afw rfc_init returned. ret: %d\n", ret);
	ret = afw_rule_init();
	if(ret) {
		pr_info("%s[%d]: afw_rule_init failed, %d\n", __FILE__, __LINE__, ret);
		goto out_rfc;
	}
	pr_info("[AFW] afw afw_rule_init returned. ret: %d\n", ret);
	ret = afw_proc_init();
	if(ret) {
		pr_info("%s[%d]: afw_proc_init failed, %d\n", __FILE__, __LINE__, ret);
		goto out_rule;
	}
	pr_info("[AFW] afw afw_proc_init returned. ret: %d\n", ret);

	ret = alloc_chrdev_region(&afw_client_device_no,
						0, 1, AFW_DEV_NAME);
	if(ret) {
		pr_info("%s[%d]: alloc_chrdev_region failed, %d\n", __FILE__, __LINE__, ret);
		goto out_proc; //TODO
	}

	pr_info("[AFW] afw register_chrdev returned. ret: %d\n", ret);
	ret = platform_driver_register(&afw_driver);
	if (ret) {
		pr_info("[AFW] afw register driver returned. ret: %d\n", ret);
		goto out_dev;
	}
	pr_info("[AFW] afw platform_driver_register returned. ret: %d\n", ret);

	driver_class = class_create(THIS_MODULE, AFW_DEV_NAME);
	if (IS_ERR(driver_class)) {
		ret = -ENOMEM;
		pr_info("class_create failed %x\n", ret);
		goto out_driver;
	}

	class_dev = device_create(driver_class, NULL,
				afw_client_device_no, NULL, AFW_DEV_NAME);

	if (class_dev == NULL) {
		pr_info("[AFW] afw class_device_create failed %d\n", ret);
		ret = -ENOMEM;
		goto out_class;
	}

	cdev_init(&afw_client_cdev, &afw_fops);
	afw_client_cdev.owner = THIS_MODULE;

	ret = cdev_add(&afw_client_cdev,
				MKDEV(MAJOR(afw_client_device_no), 0), 1);

	if (ret < 0) {
		pr_info("[AFW] afw cdev_add failed %d\n", ret);
		goto out_device;
	}

	ret = nf_register_net_hooks(&init_net, afw_ops, ARRAY_SIZE(afw_ops));
	if(ret < 0) {
		pr_info("%s[%d]: nf_register_net_hooks failed, %d\n", __FILE__, __LINE__, ret);
		goto out_cdev;
	}
	pr_info("[AFW] afw nf_register_net_hooks returned. ret: %d\n", ret);
	ret = 0;
	goto out;
out_cdev:

out_device:
	device_destroy(driver_class, afw_client_device_no);
out_class:
	class_destroy(driver_class);
out_driver:
	platform_driver_unregister(&afw_driver);
out_dev:
	unregister_chrdev(AFW_DEV_MAJOR, AFW_DEV_NAME);
out_proc:
	afw_proc_exit();
out_rule:
	afw_rule_exit();
out_rfc:
	rfc_exit();
out:
	pr_info("[AFW] afw module_init returned. ret: %d\n", ret);
	return ret;
}


void
afw_cleanup(void)
{
	nf_unregister_net_hooks(&init_net, afw_ops, ARRAY_SIZE(afw_ops));
	unregister_chrdev(AFW_DEV_MAJOR, AFW_DEV_NAME);
	platform_driver_unregister(&afw_driver);
	afw_proc_exit();
	afw_rule_exit();
	rfc_exit();

	return;
}


module_init(afw_init);
module_exit(afw_cleanup);

MODULE_DESCRIPTION("IOMMU API for ARM architected SMMU implementations");
MODULE_AUTHOR("Will Deacon <will.deacon@arm.com>");
MODULE_LICENSE("GPL v2");
