#ifndef RFC_H
#define RFC_H


#define RFC_MAX_U8					256
#define RFC_MAX_U16					65536
#define RFC_BITS_PER_LONG			(sizeof(unsigned long) * 8)

#define RULE_NUM					(10000 + 100)
#define BITMAP_SIZE					((RULE_NUM + RFC_BITS_PER_LONG - 1) / RFC_BITS_PER_LONG)

#define MAX_FIELD_NUM_U8			16
#define MAX_FIELD_NUM_U16			16


enum {
	RFC_FIELD_TYPE_U8,
	RFC_FIELD_TYPE_U16,
};


typedef struct _cbm {							/* Chunk BitMap */
	unsigned int ref;
	unsigned int min;
	unsigned int max;
	unsigned int num;
	unsigned long bitmap[BITMAP_SIZE];
}cbm_st;

typedef struct _rfc_field_u8 {
	unsigned int chunk[RFC_MAX_U8];
	cbm_st *cbm[RFC_MAX_U8];
	unsigned int eqid;
	//rwlock_t lock;
}rfc_field_u8_st;

typedef struct _rfc_field_u16 {
	unsigned int chunk[RFC_MAX_U16];
	cbm_st *cbm[RFC_MAX_U16];
	unsigned int eqid;
}rfc_field_u16_st;

typedef struct _rfc_rule {
	unsigned int max_rule_num;
	unsigned int cur_bitmap_size;

	rwlock_t lock;

	unsigned int fu8_num;
	rfc_field_u8_st *fu8[MAX_FIELD_NUM_U8];

	unsigned int fu16_num;
	rfc_field_u16_st *fu16[MAX_FIELD_NUM_U16];	
}rfc_rule_st;


extern int rfc_init(void);
extern void rfc_exit(void);

extern cbm_st * malloc_cbm(int zero);
extern void free_cbm(cbm_st *cbm);

extern int rfc_init_rule(rfc_rule_st *rule, int max_rule_num, int fu8_num, int fu16_num);
extern void rfc_free_rule(rfc_rule_st *rule);

extern int rfc_build_cbm(int chunk_idx, int rule_idx, unsigned int *chunk, cbm_st **cbm_tab, 
	unsigned int *eqid_num, int chunk_size);
extern int rfc_build_cbm_u8(int chunk_idx, int rule_idx, rfc_field_u8_st *fu8);
extern int rfc_build_cbm_u16(int chunk_idx, int rule_idx, rfc_field_u16_st *fu16);

extern void rfc_print_rule(rfc_rule_st *rule);


#endif


