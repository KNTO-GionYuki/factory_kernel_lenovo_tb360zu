/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __QRTR_H_
#define __QRTR_H_

#include <linux/types.h>

struct sk_buff;

/* endpoint node id auto assignment */
#define QRTR_EP_NID_AUTO (-1)
#define QRTR_EP_NET_ID_AUTO (1)

#define QRTR_DEL_PROC_MAGIC	0xe111
/*Linden code for JLINDEN-1590 by xiongle at 20230209 start*/
#define MAX_NON_WAKE_SVC_LEN    5
/*Linden code for JLINDEN-1590 by xiongle at 20230209 end*/

/**
 * struct qrtr_endpoint - endpoint handle
 * @xmit: Callback for outgoing packets
 *
 * The socket buffer passed to the xmit function becomes owned by the endpoint
 * driver.  As such, when the driver is done with the buffer, it should
 * call kfree_skb() on failure, or consume_skb() on success.
 */
struct qrtr_endpoint {
	int (*xmit)(struct qrtr_endpoint *ep, struct sk_buff *skb);
	/* private: not for endpoint use */
	struct qrtr_node *node;
};
/*Linden code for JLINDEN-1590 by xiongle at 20230209 start*/
int qrtr_endpoint_register(struct qrtr_endpoint *ep, unsigned int net_id,
			   bool rt, u32 *svc_arr);
/*Linden code for JLINDEN-1590 by xiongle at 20230209 end*/
void qrtr_endpoint_unregister(struct qrtr_endpoint *ep);

int qrtr_endpoint_post(struct qrtr_endpoint *ep, const void *data, size_t len);

void qrtr_ns_init(void);

void qrtr_ns_remove(void);

int qrtr_peek_pkt_size(const void *data);

unsigned int qrtr_get_service_id(unsigned int node_id, unsigned int port_id);
#endif
