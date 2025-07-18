#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>

#include "rfc.h"
#include "afw_kernel.h"


static struct kmem_cache *cbm_cache;


static void*
rfc_malloc_field(int type, int num)
{
	void *p;
	int len;

	p = NULL;
	if(!num) {
		pr_info("[AFW] %s[%d]: num is 0\n", __FILE__, __LINE__);
		goto out;
	}

	if(type == RFC_FIELD_TYPE_U8) {
		len = sizeof(rfc_field_u8_st) * num;
	}else if(type == RFC_FIELD_TYPE_U16) {
		len = sizeof(rfc_field_u16_st) * num;
	}else {
		pr_info("[AFW] %s[%d]: type is invalid, %d\n", __FILE__, __LINE__, type);
		goto out;
	}

	p = kmalloc(len, GFP_ATOMIC);
	if(p) {
		memset(p, 0, len);
		afw_stat.malloc_rfc_field += len;
	}
out:
	return p;
}


static void
rfc_free_field(void *field, int type, int num)
{
	int len = 0;

	if(field) {
		if(type == RFC_FIELD_TYPE_U8) {
			len = sizeof(rfc_field_u8_st) * num;
		}else if(type == RFC_FIELD_TYPE_U16) {
			len = sizeof(rfc_field_u16_st) * num;
		}
		afw_stat.malloc_rfc_field -= len;

		kfree(field);
	}
	return;
}


cbm_st *
malloc_cbm(int zero)
{
	cbm_st *cbm;

	cbm = (cbm_st *)kmem_cache_alloc(cbm_cache, GFP_ATOMIC);
	if(!cbm) {
		pr_info("[AFW] %s[%d]: kmem_cache_alloc failed\n", __FILE__, __LINE__);
		goto out;
	}

	if(zero) {
		memset(cbm, 0, sizeof(*cbm));
	}

	afw_stat.malloc_rfc_cbm += sizeof(*cbm);

out:
	return cbm;
}


void
free_cbm(cbm_st *cbm)
{
	if(!cbm) {
		goto out;
	}

	kmem_cache_free(cbm_cache, cbm);
	afw_stat.malloc_rfc_cbm -= sizeof(*cbm);

out:
	return;
}


void
rfc_free_rule(rfc_rule_st *rule)
{
	int i;
	int j;

	if(!rule) {
		goto out;
	}

	if(rule->fu8_num) {
		for(i = 0; i < rule->fu8_num; i ++) {
			for(j = 0; j < RFC_MAX_U8; j ++) {
				if(rule->fu8[i]->cbm[j]) {
					free_cbm(rule->fu8[i]->cbm[j]);
				}
			}
		}

		for(i = 0; i < rule->fu8_num; i ++) {
			rfc_free_field(rule->fu8[i], RFC_FIELD_TYPE_U8, 1);
			rule->fu8[i] = NULL;
		}
		rule->fu8_num = 0;
	}
	
	if(rule->fu16_num) {
		for(i = 0; i < rule->fu16_num; i ++) {
			for(j = 0; j < RFC_MAX_U16; j ++) {
				if(rule->fu16[i]->cbm[j]) {
					free_cbm(rule->fu16[i]->cbm[j]);
				}
			}
		}

		for(i = 0; i < rule->fu16_num; i ++) {
			rfc_free_field(rule->fu16[i], RFC_FIELD_TYPE_U16, 1);
			rule->fu16[i] = NULL;
		}
		rule->fu16_num = 0;
	}

out:
	return;
}


int
rfc_init_rule(rfc_rule_st *rule, int max_rule_num, int fu8_num, int fu16_num)
{
	int ret;
	cbm_st *cbm;
	int i;
	int j;

	if(fu8_num > MAX_FIELD_NUM_U8 || fu16_num > MAX_FIELD_NUM_U16) {
		pr_info("[AFW] %s[%d]: check parameter failed, fu8_num[%d], fu16_num[%d]\n", __FILE__, __LINE__, fu8_num, fu16_num);
		ret = -1;
		goto out;
	}

	rule->max_rule_num = max_rule_num;
	rule->fu8_num = fu8_num;
	rule->fu16_num = fu16_num;

	if(fu8_num) {
		for(i = 0; i < fu8_num; i ++) {
			rule->fu8[i] = rfc_malloc_field(RFC_FIELD_TYPE_U8, 1);
			if(!rule->fu8[i]) {
				pr_info("[AFW] %s[%d]: rfc_malloc_field failed\n", __FILE__, __LINE__);
				ret = -1;
				goto out_free;
			}
		}

		for(i = 0; i < fu8_num; i ++) {
			cbm = malloc_cbm(1);
			if(!cbm) {
				pr_info("[AFW] %s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
				ret = -1;
				goto out_free;
			}

			rule->fu8[i]->eqid = 0;
			for(j = 0; j < RFC_MAX_U8; j ++) {
				rule->fu8[i]->chunk[j] = 0;
				cbm->ref ++;
			}
			rule->fu8[i]->cbm[0] = cbm;
			rule->fu8[i]->eqid ++;
		}
	}

	if(fu16_num) {
		for(i = 0; i < fu16_num; i ++) {
			rule->fu16[i] = rfc_malloc_field(RFC_FIELD_TYPE_U16, 1);
			if(!rule->fu16[i]) {
				pr_info("[AFW] %s[%d]: rfc_malloc_field failed\n", __FILE__, __LINE__);
				ret = -1;
				goto out_free;
			}
		}

		for(i = 0; i < fu16_num; i ++) {
			cbm = malloc_cbm(1);
			if(!cbm) {
				pr_info("[AFW] %s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
				ret = -1;
				goto out_free;
			}

			rule->fu16[i]->eqid = 0;
			for(j = 0; j < RFC_MAX_U16; j ++) {
				rule->fu16[i]->chunk[j] = 0;
				cbm->ref ++;
			}
			rule->fu16[i]->cbm[0] = cbm;
			rule->fu16[i]->eqid ++;
		}
	}

	ret = 0;
	goto out;
out_free:
	rfc_free_rule(rule);
out:
	return ret;
}


int
rfc_build_cbm(int chunk_idx, int rule_idx, unsigned int *chunk, cbm_st **cbm_tab, unsigned int *eqid_num, int chunk_size)
{
	int ret;
	int len;

	cbm_st *cbm;
	cbm_st *cbm2;
	cbm_st *cbm_tmp;
	unsigned int eqid;
	int idx;
	int i;

	idx = chunk[chunk_idx];
	cbm = cbm_tab[idx];

	if(!cbm) {
		pr_info("[AFW] %s[%d]: cbm is null\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	if(test_bit(rule_idx, cbm->bitmap)) {
		ret = 0;
		goto out;
	}

	cbm_tmp = malloc_cbm(0);
	if(!cbm_tmp) {
		pr_info("[AFW] %s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	memcpy(cbm_tmp, cbm, sizeof(cbm_st));
	set_bit(rule_idx, cbm_tmp->bitmap);
	if(!cbm_tmp->num) {
		cbm_tmp->min = rule_idx;
	}
	cbm_tmp->max = rule_idx;
	cbm_tmp->num ++;

	for(eqid = 0; eqid < *eqid_num; eqid ++) {
		cbm2 = cbm_tab[eqid];	

		if(cbm_tmp->num != cbm2->num || cbm_tmp->min != cbm2->min || cbm_tmp->max != cbm2->max) {
			continue;
		}

		len = cbm2->max / 8 + 1;
		if(!memcmp(cbm_tmp->bitmap, cbm2->bitmap, len)) {
			break;
		}
	}

	if(eqid >= *eqid_num) {
		if(cbm->ref == 1) {
			set_bit(rule_idx, cbm->bitmap);
			if(!cbm->num) {
				cbm->min = rule_idx;
			}
			cbm->max = rule_idx;
			cbm->num ++;
		}else {
			cbm2 = malloc_cbm(0);
			if(!cbm2) {
				pr_info("[AFW] %s[%d]: malloc_cbm failed\n", __FILE__, __LINE__);
				free_cbm(cbm_tmp);
				ret = -1;
				goto out;
			}

			memcpy(cbm2, cbm_tmp, sizeof(cbm_st));
			cbm2->ref = 1;

			cbm_tab[*eqid_num] = cbm2;
			chunk[chunk_idx] = *eqid_num;
			(*eqid_num) ++;
			cbm->ref --;
		}
	}else {
		chunk[chunk_idx] = eqid;
		cbm2->ref ++;

		cbm->ref --;
		if(!cbm->ref) {
			for(i = idx; i < *eqid_num - 1; i ++) {
				cbm_tab[i] = cbm_tab[i + 1];
			}

			free_cbm(cbm);
			(*eqid_num) --;
			cbm_tab[*eqid_num] = NULL;

			for(i = 0; i < chunk_size; i ++) {
				if(chunk[i] > idx) {
					chunk[i] --;
				}
			}
		}
	}

	free_cbm(cbm_tmp);
	ret = 0;
out:
	return ret;
}


int
rfc_build_cbm_u8(int chunk_idx, int rule_idx, rfc_field_u8_st *fu8)
{
	int ret;
	unsigned int *chunk;
	cbm_st **cbm_tab;
	unsigned int *eqid_num;

	chunk = fu8->chunk;
	cbm_tab = fu8->cbm;
	eqid_num = &fu8->eqid;

	ret = rfc_build_cbm(chunk_idx, rule_idx, chunk, cbm_tab, eqid_num, RFC_MAX_U8);
	if(ret) {
		pr_info("[AFW] %s[%d]: rfc_build_cbm failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = 0;
out:
	return ret;
}


int
rfc_build_cbm_u16(int chunk_idx, int rule_idx, rfc_field_u16_st *fu16)
{
	int ret;
	unsigned int *chunk;
	cbm_st **cbm_tab;
	unsigned int *eqid_num;

	chunk = fu16->chunk;
	cbm_tab = fu16->cbm;
	eqid_num = &fu16->eqid;

	ret = rfc_build_cbm(chunk_idx, rule_idx, chunk, cbm_tab, eqid_num, RFC_MAX_U16);
	if(ret) {
		pr_info("[AFW] %s[%d]: rfc_build_cbm failed, %d\n", __FILE__, __LINE__, ret);
		goto out;
	}

	ret = 0;
out:
	return ret;
}

void
rfc_print_rule(rfc_rule_st *rule)
{
	int i;

	if(!rule) {
		goto out;
	}

	pr_info("[AFW] max_rule_num	%d\n", rule->max_rule_num);
	pr_info("[AFW] cur_bitmap_size	 %d\n", rule->cur_bitmap_size);
	pr_info("[AFW] fu8_num	%d\n", rule->fu8_num);
	pr_info("[AFW] fu16_num	%d\n", rule->fu16_num);

	if(rule->fu8_num) {
		pr_info("fu8 info:\n");
		for(i = 0; i < rule->fu8_num; i ++) {
			pr_info("[AFW] fu8[%02d], eqid[%d]: \n", i, rule->fu8[i]->eqid);
		}
	}

	if(rule->fu16_num) {
		pr_info("[AFW] fu16 info:\n");
		for(i = 0; i < rule->fu16_num; i ++) {
			pr_info("[AFW] fu16[%02d], eqid[%d]: \n", i, rule->fu16[i]->eqid);
		}
	}

out:
	return;
}


int
rfc_init(void)
{
	int ret;

	cbm_cache = kmem_cache_create("afw_cbm", sizeof(cbm_st), 0, 0, NULL);
	if(!cbm_cache) {
		pr_info("[AFW] %s[%d]: kmem_cache_create failed\n", __FILE__, __LINE__);
		ret = -1;
		goto out;
	}

	ret = 0;
out:
	return ret;
}


void
rfc_exit(void)
{
	kmem_cache_destroy(cbm_cache);
	return;	
}


