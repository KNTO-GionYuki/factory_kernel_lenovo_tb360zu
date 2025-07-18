#ifndef ARGS_H
#define ARGS_H

typedef struct _range_u16 {
	unsigned short v1;
	unsigned short v2;
}range_u16_st;

typedef struct _range_u32 {
	unsigned int v1;
	unsigned int v2;
}range_u32_st;

typedef struct _range_ipv6 {
    unsigned char v1[16];
    unsigned char v2[16];
}range_ipv6_st;


typedef struct _id_name {
	unsigned int id;
	char *name;
}id_name_st;


extern int is_number(char *arg);
extern int multi_range_num(char *arg);
extern int domain_level(char *arg);

extern int parse_u16(char *arg, unsigned short *num);
extern int parse_u16_range(char *arg, range_u16_st *range);
extern int parse_u16_multi_range(char *arg, range_u16_st *range, unsigned int *num);

extern int parse_u32(char *arg, unsigned int *num);
extern int parse_u32_range(char *arg, range_u32_st *range);
extern int parse_u32_multi_range(char *arg, range_u32_st *range, unsigned int *num);

extern int is_ip_net(char *arg);

extern int parse_ip(char *arg, unsigned int *ip);
extern int parse_ip_net(char *arg, range_u32_st *range);
extern int parse_ip_range(char *arg, range_u32_st *range);
extern int parse_ip_multi_range(char *arg, range_u32_st *range, unsigned int *num);

extern int parse_ipv6(char *arg, void *ip);
extern int parse_ipv6_net(char *arg, range_ipv6_st *range);
extern int parse_ipv6_range(char *arg, range_ipv6_st *range);
extern int parse_ipv6_multi_range(char *arg, range_ipv6_st *range, unsigned int *num);

extern int parse_name2_id(char *arg, id_name_st *id_name, unsigned int num, unsigned int *id);


#endif
