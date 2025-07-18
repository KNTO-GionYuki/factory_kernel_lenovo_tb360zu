#ifndef PROC_H
#define PROC_H


#define AFW_CONF_PROC_FILE		"afw_conf"
#define AFW_STAT_PROC_FILE		"afw_stat"
#define AFW_PKT_PROC_FILE		"afw_pkt"
#define AFW_RFC_PROC_FILE		"afw_rfc"
#define AFW_DNS_PROC_FILE		"afw_dns"
#define AFW_DEBUG_PROC_FILE		"afw_debug"
#define AFW_RULE_FILE			"afw_rule"


extern int afw_proc_init(void);
extern void afw_proc_exit(void);


#endif
