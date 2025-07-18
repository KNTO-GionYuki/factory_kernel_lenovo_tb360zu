#ifndef AFW_LIB_H
#define AFW_LIB_H


typedef struct _afw_rule_tab {
	unsigned int num;
	char **rules;
	char data[0];
}afw_rule_tab_st;


extern void afw_free_rule_tab(afw_rule_tab_st *rule_tab);

extern int afw_set_enable(int enable);
extern int afw_is_enalbe();
extern int afw_set_black_rule_enable(int enable);
extern int afw_is_black_rule_enable();
extern int afw_set_white_rule_enable(int enable);
extern int afw_is_white_rule_enable();
extern int afw_set_parse_input_enable(int enable);
extern int afw_is_parse_input_enable();

extern int afw_set_pass_dns_enable(int enable);
extern int afw_is_pass_dns_enable();
extern int afw_set_pid_cache_enable(int enable);
extern int afw_is_pid_cache_enable();
extern int afw_set_tcp_reset_enable(int enable);
extern int afw_is_tcp_reset_enable();

extern int afw_add_white_rule(char *rule);
extern afw_rule_tab_st *afw_get_white_rules();
extern int afw_add_black_rule(char *rule);
extern afw_rule_tab_st *afw_get_black_rules();
extern int afw_flush_rules(int type);
extern int afw_flush_all_rules();
extern int afw_test_rules(int type, int kind);

extern int afw_set_pid_timeout(int timeout);
extern int afw_get_pid_timeout(int *timeout);

#endif
