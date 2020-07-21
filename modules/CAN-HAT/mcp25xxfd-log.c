// SPDX-License-Identifier: GPL-2.0
//
// mcp25xxfd - Microchip MCP25xxFD Family CAN controller driver
//
// Copyright (c) 2019 Pengutronix,
//                    Marc Kleine-Budde <kernel@pengutronix.de>
//

#include "mcp25xxfd.h"
#include "mcp25xxfd-log.h"

static inline int mcp25xxfd_log_get_index(const struct mcp25xxfd_priv *priv, int n)
{
	return n & (ARRAY_SIZE(priv->log) - 1);
}

struct mcp25xxfd_log *___mcp25xxfd_log(struct mcp25xxfd_priv *priv, const char *func, canid_t can_id)
{
	struct mcp25xxfd_log *log;
	int cnt;

	cnt = mcp25xxfd_log_get_index(priv, atomic_add_return(1, &priv->cnt));

	log = &priv->log[cnt];
	log->func = func;
	log->can_id = can_id;
	log->tef_head = priv->tef.head;
	log->tef_tail = priv->tef.tail;
	log->tx_head = priv->tx->head;
	log->tx_tail = priv->tx->tail;
	log->rx_head = priv->rx[0]->head;
	log->rx_tail = priv->rx[0]->tail;
	log->flags = 0;

	return log;
}

static void mcp25xxfd_log_dump_one(const struct mcp25xxfd_priv *priv, const struct mcp25xxfd_log *last_log, const struct mcp25xxfd_log *log, int n)
{
	pr_info("%04d: %30s: ",
		n, log->func);

	if (log->can_id != -1 &&
	    last_log->can_id != log->can_id)
		pr_cont("id=%03x ", log->can_id);
	else
		pr_cont(" ---   ");

	/* RX */

	if (last_log->rx_head != log->rx_head)
		pr_cont("rx_h=%08x/%02x ", log->rx_head, log->rx_head & (priv->rx[0]->obj_num - 1));
	else
		pr_cont("   ---           ");

	if (log->hw_rx_head != -1 &&
	    last_log->hw_rx_head != log->hw_rx_head)
		pr_cont("hw_rx_h=%02x ", log->hw_rx_head);
	else
		pr_cont("      ---  ");

	if (last_log->rx_tail != log->rx_tail)
		pr_cont("rx_t=%08x/%02x ", log->rx_tail, log->rx_tail & (priv->rx[0]->obj_num - 1));
	else
		pr_cont("   ---           ");

	if (log->hw_rx_tail != -1 &&
	    last_log->hw_rx_tail != log->hw_rx_tail)
		pr_cont("hw_rx_t=%02x ", log->hw_rx_tail);
	else
		pr_cont("      ---  ");

	if (log->rx_offset != 255 &&
	    last_log->rx_offset != log->rx_offset)
		pr_cont("rx_o=%02x ", log->rx_offset);
	else
		pr_cont("   ---  ");

	if (log->rx_len != 255 &&
	    last_log->rx_len != log->rx_len)
		pr_cont("rx_l=%2d ", log->rx_len);
	else
		pr_cont("   ---  ");


	/* TEF */

	if (last_log->tef_head != log->tef_head)
		pr_cont("tef_h=%08x/%02x ", log->tef_head, log->tef_head & (priv->tx->obj_num - 1));
	else
		pr_cont("    ---           ");

	if (log->hw_tx_ci != -1 &&
	    last_log->hw_tx_ci != log->hw_tx_ci)
		pr_cont("hw_tx_ci=%02x ", log->hw_tx_ci);
	else
		pr_cont("       ---  ");

	if (last_log->tef_tail != log->tef_tail)
		pr_cont("tef_t=%08x/%02x ", log->tef_tail, log->tef_tail & (priv->tx->obj_num - 1));
	else
		pr_cont("    ---           ");

	if (log->hw_tef_tail != -1 &&
	    last_log->hw_tef_tail != log->hw_tef_tail)
		pr_cont("hw_tef_t=%02x ", log->hw_tef_tail);
	else
		pr_cont("       ---  ");

	/* TX */

	if (last_log->tx_head != log->tx_head)
		pr_cont("tx_h=%08x/%02x ", log->tx_head, log->tx_head & (priv->tx->obj_num - 1));
	else
		pr_cont("   ---           ");

	if (last_log->tx_tail != log->tx_tail)
		pr_cont("tx_t=%08x/%02x ", log->tx_tail, log->tx_tail & (priv->tx->obj_num - 1));
	else
		pr_cont("   ---           ");

	pr_cont("%s%s%s%s%s\n",
		log->flags & MCP25XXFD_LOG_STOP ? "s" : " ",
		log->flags & MCP25XXFD_LOG_WAKE ? "w" : " ",
		log->flags & MCP25XXFD_LOG_BUSY ? "b" : " ",
		log->flags & MCP25XXFD_LOG_TXMAB ? "T" : " ",
		log->flags & MCP25XXFD_LOG_RXMAB ? "R" : " ");
}

void mcp25xxfd_log_dump(const struct mcp25xxfd_priv *priv)
{
	int index, i;

	index = atomic_read(&priv->cnt) + 1;

	for (i = 0; i < ARRAY_SIZE(priv->log); i++) {
		int index_last;

		index = mcp25xxfd_log_get_index(priv, index);
		index_last = mcp25xxfd_log_get_index(priv, index - 1);

		mcp25xxfd_log_dump_one(priv, &priv->log[index_last], &priv->log[index], i);
		index++;
	}
}
