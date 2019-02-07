/*
 * Copyright (c) 2016-2017, Mellanox Technologies. All rights reserved.
 * Copyright (c) 2016-2017, Dave Watson <davejwatson@fb.com>. All rights reserved.
 * Copyright (c) 2016-2017, Lance Chao <lancerchao@fb.com>. All rights reserved.
 * Copyright (c) 2016, Fridolin Pokorny <fridolin.pokorny@gmail.com>. All rights reserved.
 * Copyright (c) 2016, Nikos Mavrogiannopoulos <nmav@gnutls.org>. All rights reserved.
 * Copyright (c) 2018, Covalent IO, Inc. http://covalent.io
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/sched/signal.h>
#include <linux/module.h>
#include <crypto/aead.h>

#include <net/strparser.h>
#include <net/tls.h>

#define MAX_IV_SIZE	TLS_CIPHER_AES_GCM_128_IV_SIZE

static int __skb_nsg(struct sk_buff *skb, int offset, int len,
                     unsigned int recursion_level)
{
        int start = skb_headlen(skb);
        int i, chunk = start - offset;
        struct sk_buff *frag_iter;
        int elt = 0;

        if (unlikely(recursion_level >= 24))
                return -EMSGSIZE;

        if (chunk > 0) {
                if (chunk > len)
                        chunk = len;
                elt++;
                len -= chunk;
                if (len == 0)
                        return elt;
                offset += chunk;
        }

        for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
                int end;

                WARN_ON(start > offset + len);

                end = start + skb_frag_size(&skb_shinfo(skb)->frags[i]);
                chunk = end - offset;
                if (chunk > 0) {
                        if (chunk > len)
                                chunk = len;
                        elt++;
                        len -= chunk;
                        if (len == 0)
                                return elt;
                        offset += chunk;
                }
                start = end;
        }

        if (unlikely(skb_has_frag_list(skb))) {
                skb_walk_frags(skb, frag_iter) {
                        int end, ret;

                        WARN_ON(start > offset + len);

                        end = start + frag_iter->len;
                        chunk = end - offset;
                        if (chunk > 0) {
                                if (chunk > len)
                                        chunk = len;
                                ret = __skb_nsg(frag_iter, offset - start, chunk,
                                                recursion_level + 1);
                                if (unlikely(ret < 0))
                                        return ret;
                                elt += ret;
                                len -= chunk;
                                if (len == 0)
                                        return elt;
                                offset += chunk;
                        }
                        start = end;
                }
        }
        BUG_ON(len);
        return elt;
}

/* Return the number of scatterlist elements required to completely map the
 * skb, or -EMSGSIZE if the recursion depth is exceeded.
 */
static int skb_nsg(struct sk_buff *skb, int offset, int len)
{
        return __skb_nsg(skb, offset, len, 0);
}

static void tls_decrypt_done(struct crypto_async_request *req, int err)
{
	struct aead_request *aead_req = (struct aead_request *)req;
	struct scatterlist *sgout = aead_req->dst;
	struct tls_sw_context_rx *ctx;
	struct tls_context *tls_ctx;
	struct scatterlist *sg;
	struct sk_buff *skb;
	unsigned int pages;
	int pending;

	skb = (struct sk_buff *)req->data;
	tls_ctx = tls_get_ctx(skb->sk);
	ctx = tls_sw_ctx_rx(tls_ctx);
	pending = atomic_dec_return(&ctx->decrypt_pending);

	/* Propagate if there was an err */
	if (err) {
		ctx->async_wait.err = err;
		tls_err_abort(skb->sk, err);
	}

	/* After using skb->sk to propagate sk through crypto async callback
	 * we need to NULL it again.
	 */
	skb->sk = NULL;

	/* Release the skb, pages and memory allocated for crypto req */
	kfree_skb(skb);

	/* Skip the first S/G entry as it points to AAD */
	for_each_sg(sg_next(sgout), sg, UINT_MAX, pages) {
		if (!sg)
			break;
		put_page(sg_page(sg));
	}

	kfree(aead_req);

	if (!pending && READ_ONCE(ctx->async_notify))
		complete(&ctx->async_wait.completion);
}

static int tls_do_decryption(struct sock *sk,
			     struct sk_buff *skb,
			     struct scatterlist *sgin,
			     struct scatterlist *sgout,
			     char *iv_recv,
			     size_t data_len,
			     struct aead_request *aead_req,
			     bool async)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	int ret;

	aead_request_set_tfm(aead_req, ctx->aead_recv);
	aead_request_set_ad(aead_req, TLS_AAD_SPACE_SIZE);
	aead_request_set_crypt(aead_req, sgin, sgout,
			       data_len + tls_ctx->rx.tag_size,
			       (u8 *)iv_recv);

	if (async) {
		/* Using skb->sk to push sk through to crypto async callback
		 * handler. This allows propagating errors up to the socket
		 * if needed. It _must_ be cleared in the async handler
		 * before kfree_skb is called. We _know_ skb->sk is NULL
		 * because it is a clone from strparser.
		 */
		skb->sk = sk;
		aead_request_set_callback(aead_req,
					  CRYPTO_TFM_REQ_MAY_BACKLOG,
					  tls_decrypt_done, skb);
		atomic_inc(&ctx->decrypt_pending);
	} else {
		aead_request_set_callback(aead_req,
					  CRYPTO_TFM_REQ_MAY_BACKLOG,
					  crypto_req_done, &ctx->async_wait);
	}

	ret = crypto_aead_decrypt(aead_req);
	if (ret == -EINPROGRESS) {
		if (async)
			return ret;

		ret = crypto_wait_req(ret, &ctx->async_wait);
	}

	if (async)
		atomic_dec(&ctx->decrypt_pending);

	return ret;
}

static void tls_trim_both_msgs(struct sock *sk, int target_size)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec;

	sk_msg_trim(sk, &rec->msg_plaintext, target_size);
	if (target_size > 0)
		target_size += tls_ctx->tx.overhead_size;
	sk_msg_trim(sk, &rec->msg_encrypted, target_size);
}

static int tls_alloc_encrypted_msg(struct sock *sk, int len)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec;
	struct sk_msg *msg_en = &rec->msg_encrypted;

	return sk_msg_alloc(sk, msg_en, len, 0);
}

static int tls_clone_plaintext_msg(struct sock *sk, int required)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec;
	struct sk_msg *msg_pl = &rec->msg_plaintext;
	struct sk_msg *msg_en = &rec->msg_encrypted;
	int skip, len;

	/* We add page references worth len bytes from encrypted sg
	 * at the end of plaintext sg. It is guaranteed that msg_en
	 * has enough required room (ensured by caller).
	 */
	len = required - msg_pl->sg.size;

	/* Skip initial bytes in msg_en's data to be able to use
	 * same offset of both plain and encrypted data.
	 */
	skip = tls_ctx->tx.prepend_size + msg_pl->sg.size;

	return sk_msg_clone(sk, msg_pl, msg_en, skip, len);
}

static struct tls_rec *tls_get_rec(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct sk_msg *msg_pl, *msg_en;
	struct tls_rec *rec;
	int mem_size;

	mem_size = sizeof(struct tls_rec) + crypto_aead_reqsize(ctx->aead_send);

	rec = kzalloc(mem_size, sk->sk_allocation);
	if (!rec)
		return NULL;

	msg_pl = &rec->msg_plaintext;
	msg_en = &rec->msg_encrypted;

	sk_msg_init(msg_pl);
	sk_msg_init(msg_en);

	sg_init_table(rec->sg_aead_in, 2);
	sg_set_buf(&rec->sg_aead_in[0], rec->aad_space,
		   sizeof(rec->aad_space));
	sg_unmark_end(&rec->sg_aead_in[1]);

	sg_init_table(rec->sg_aead_out, 2);
	sg_set_buf(&rec->sg_aead_out[0], rec->aad_space,
		   sizeof(rec->aad_space));
	sg_unmark_end(&rec->sg_aead_out[1]);

	return rec;
}

static void tls_free_rec(struct sock *sk, struct tls_rec *rec)
{
	sk_msg_free(sk, &rec->msg_encrypted);
	sk_msg_free(sk, &rec->msg_plaintext);
	kfree(rec);
}

static void tls_free_open_rec(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec;

	if (rec) {
		tls_free_rec(sk, rec);
		ctx->open_rec = NULL;
	}
}

int tls_tx_records(struct sock *sk, int flags)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec, *tmp;
	struct sk_msg *msg_en;
	int tx_flags, rc = 0;

	if (tls_is_partially_sent_record(tls_ctx)) {
		rec = list_first_entry(&ctx->tx_list,
				       struct tls_rec, list);

		if (flags == -1)
			tx_flags = rec->tx_flags;
		else
			tx_flags = flags;

		rc = tls_push_partial_record(sk, tls_ctx, tx_flags);
		if (rc)
			goto tx_err;

		/* Full record has been transmitted.
		 * Remove the head of tx_list
		 */
		list_del(&rec->list);
		sk_msg_free(sk, &rec->msg_plaintext);
		kfree(rec);
	}

	/* Tx all ready records */
	list_for_each_entry_safe(rec, tmp, &ctx->tx_list, list) {
		if (READ_ONCE(rec->tx_ready)) {
			if (flags == -1)
				tx_flags = rec->tx_flags;
			else
				tx_flags = flags;

			msg_en = &rec->msg_encrypted;
			rc = tls_push_sg(sk, tls_ctx,
					 &msg_en->sg.data[msg_en->sg.curr],
					 0, tx_flags);
			if (rc)
				goto tx_err;

			list_del(&rec->list);
			sk_msg_free(sk, &rec->msg_plaintext);
			kfree(rec);
		} else {
			break;
		}
	}

tx_err:
	if (rc < 0 && rc != -EAGAIN)
		tls_err_abort(sk, EBADMSG);

	return rc;
}

static void tls_encrypt_done(struct crypto_async_request *req, int err)
{
	struct aead_request *aead_req = (struct aead_request *)req;
	struct sock *sk = req->data;
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct scatterlist *sge;
	struct sk_msg *msg_en;
	struct tls_rec *rec;
	bool ready = false;
	int pending;

	rec = container_of(aead_req, struct tls_rec, aead_req);
	msg_en = &rec->msg_encrypted;

	sge = sk_msg_elem(msg_en, msg_en->sg.curr);
	sge->offset -= tls_ctx->tx.prepend_size;
	sge->length += tls_ctx->tx.prepend_size;

	/* Check if error is previously set on socket */
	if (err || sk->sk_err) {
		rec = NULL;

		/* If err is already set on socket, return the same code */
		if (sk->sk_err) {
			ctx->async_wait.err = sk->sk_err;
		} else {
			ctx->async_wait.err = err;
			tls_err_abort(sk, err);
		}
	}

	if (rec) {
		struct tls_rec *first_rec;

		/* Mark the record as ready for transmission */
		smp_store_mb(rec->tx_ready, true);

		/* If received record is at head of tx_list, schedule tx */
		first_rec = list_first_entry(&ctx->tx_list,
					     struct tls_rec, list);
		if (rec == first_rec)
			ready = true;
	}

	pending = atomic_dec_return(&ctx->encrypt_pending);

	if (!pending && READ_ONCE(ctx->async_notify))
		complete(&ctx->async_wait.completion);

	if (!ready)
		return;

	/* Schedule the transmission */
	if (!test_and_set_bit(BIT_TX_SCHEDULED, &ctx->tx_bitmask))
		schedule_delayed_work(&ctx->tx_work.work, 1);
}

static int tls_do_encryption(struct sock *sk,
			     struct tls_context *tls_ctx,
			     struct tls_sw_context_tx *ctx,
			     struct aead_request *aead_req,
			     size_t data_len, u32 start)
{
	struct tls_rec *rec = ctx->open_rec;
	struct sk_msg *msg_en = &rec->msg_encrypted;
	struct scatterlist *sge = sk_msg_elem(msg_en, start);
	int rc;

	memcpy(rec->iv_data, tls_ctx->tx.iv, sizeof(rec->iv_data));

	sge->offset += tls_ctx->tx.prepend_size;
	sge->length -= tls_ctx->tx.prepend_size;

	msg_en->sg.curr = start;

	aead_request_set_tfm(aead_req, ctx->aead_send);
	aead_request_set_ad(aead_req, TLS_AAD_SPACE_SIZE);
	aead_request_set_crypt(aead_req, rec->sg_aead_in,
			       rec->sg_aead_out,
			       data_len, rec->iv_data);

	aead_request_set_callback(aead_req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				  tls_encrypt_done, sk);

	/* Add the record in tx_list */
	list_add_tail((struct list_head *)&rec->list, &ctx->tx_list);
	atomic_inc(&ctx->encrypt_pending);

	rc = crypto_aead_encrypt(aead_req);
	if (!rc || rc != -EINPROGRESS) {
		atomic_dec(&ctx->encrypt_pending);
		sge->offset -= tls_ctx->tx.prepend_size;
		sge->length += tls_ctx->tx.prepend_size;
	}

	if (!rc) {
		WRITE_ONCE(rec->tx_ready, true);
	} else if (rc != -EINPROGRESS) {
		list_del(&rec->list);
		return rc;
	}

	/* Unhook the record from context if encryption is not failure */
	ctx->open_rec = NULL;
	tls_advance_record_sn(sk, &tls_ctx->tx);
	return rc;
}

static int tls_split_open_record(struct sock *sk, struct tls_rec *from,
				 struct tls_rec **to, struct sk_msg *msg_opl,
				 struct sk_msg *msg_oen, u32 split_point,
				 u32 tx_overhead_size, u32 *orig_end)
{
	u32 i, j, bytes = 0, apply = msg_opl->apply_bytes;
	struct scatterlist *sge, *osge, *nsge;
	u32 orig_size = msg_opl->sg.size;
	struct scatterlist tmp = { };
	struct sk_msg *msg_npl;
	struct tls_rec *new;
	int ret;

	new = tls_get_rec(sk);
	if (!new)
		return -ENOMEM;
	ret = sk_msg_alloc(sk, &new->msg_encrypted, msg_opl->sg.size +
			   tx_overhead_size, 0);
	if (ret < 0) {
		tls_free_rec(sk, new);
		return ret;
	}

	*orig_end = msg_opl->sg.end;
	i = msg_opl->sg.start;
	sge = sk_msg_elem(msg_opl, i);
	while (apply && sge->length) {
		if (sge->length > apply) {
			u32 len = sge->length - apply;

			get_page(sg_page(sge));
			sg_set_page(&tmp, sg_page(sge), len,
				    sge->offset + apply);
			sge->length = apply;
			bytes += apply;
			apply = 0;
		} else {
			apply -= sge->length;
			bytes += sge->length;
		}

		sk_msg_iter_var_next(i);
		if (i == msg_opl->sg.end)
			break;
		sge = sk_msg_elem(msg_opl, i);
	}

	msg_opl->sg.end = i;
	msg_opl->sg.curr = i;
	msg_opl->sg.copybreak = 0;
	msg_opl->apply_bytes = 0;
	msg_opl->sg.size = bytes;

	msg_npl = &new->msg_plaintext;
	msg_npl->apply_bytes = apply;
	msg_npl->sg.size = orig_size - bytes;

	j = msg_npl->sg.start;
	nsge = sk_msg_elem(msg_npl, j);
	if (tmp.length) {
		memcpy(nsge, &tmp, sizeof(*nsge));
		sk_msg_iter_var_next(j);
		nsge = sk_msg_elem(msg_npl, j);
	}

	osge = sk_msg_elem(msg_opl, i);
	while (osge->length) {
		memcpy(nsge, osge, sizeof(*nsge));
		sg_unmark_end(nsge);
		sk_msg_iter_var_next(i);
		sk_msg_iter_var_next(j);
		if (i == *orig_end)
			break;
		osge = sk_msg_elem(msg_opl, i);
		nsge = sk_msg_elem(msg_npl, j);
	}

	msg_npl->sg.end = j;
	msg_npl->sg.curr = j;
	msg_npl->sg.copybreak = 0;

	*to = new;
	return 0;
}

static void tls_merge_open_record(struct sock *sk, struct tls_rec *to,
				  struct tls_rec *from, u32 orig_end)
{
	struct sk_msg *msg_npl = &from->msg_plaintext;
	struct sk_msg *msg_opl = &to->msg_plaintext;
	struct scatterlist *osge, *nsge;
	u32 i, j;

	i = msg_opl->sg.end;
	sk_msg_iter_var_prev(i);
	j = msg_npl->sg.start;

	osge = sk_msg_elem(msg_opl, i);
	nsge = sk_msg_elem(msg_npl, j);

	if (sg_page(osge) == sg_page(nsge) &&
	    osge->offset + osge->length == nsge->offset) {
		osge->length += nsge->length;
		put_page(sg_page(nsge));
	}

	msg_opl->sg.end = orig_end;
	msg_opl->sg.curr = orig_end;
	msg_opl->sg.copybreak = 0;
	msg_opl->apply_bytes = msg_opl->sg.size + msg_npl->sg.size;
	msg_opl->sg.size += msg_npl->sg.size;

	sk_msg_free(sk, &to->msg_encrypted);
	sk_msg_xfer_full(&to->msg_encrypted, &from->msg_encrypted);

	kfree(from);
}

static int tls_push_record(struct sock *sk, int flags,
			   unsigned char record_type)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec, *tmp = NULL;
	u32 i, split_point, uninitialized_var(orig_end);
	struct sk_msg *msg_pl, *msg_en;
	struct aead_request *req;
	bool split;
	int rc;

	if (!rec)
		return 0;

	msg_pl = &rec->msg_plaintext;
	msg_en = &rec->msg_encrypted;

	split_point = msg_pl->apply_bytes;
	split = split_point && split_point < msg_pl->sg.size;
	if (split) {
		rc = tls_split_open_record(sk, rec, &tmp, msg_pl, msg_en,
					   split_point, tls_ctx->tx.overhead_size,
					   &orig_end);
		if (rc < 0)
			return rc;
		sk_msg_trim(sk, msg_en, msg_pl->sg.size +
			    tls_ctx->tx.overhead_size);
	}

	rec->tx_flags = flags;
	req = &rec->aead_req;

	i = msg_pl->sg.end;
	sk_msg_iter_var_prev(i);
	sg_mark_end(sk_msg_elem(msg_pl, i));

	i = msg_pl->sg.start;
	sg_chain(rec->sg_aead_in, 2, rec->inplace_crypto ?
		 &msg_en->sg.data[i] : &msg_pl->sg.data[i]);

	i = msg_en->sg.end;
	sk_msg_iter_var_prev(i);
	sg_mark_end(sk_msg_elem(msg_en, i));

	i = msg_en->sg.start;
	sg_chain(rec->sg_aead_out, 2, &msg_en->sg.data[i]);

	tls_make_aad(rec->aad_space, msg_pl->sg.size,
		     tls_ctx->tx.rec_seq, tls_ctx->tx.rec_seq_size,
		     record_type);

	tls_fill_prepend(tls_ctx,
			 page_address(sg_page(&msg_en->sg.data[i])) +
			 msg_en->sg.data[i].offset, msg_pl->sg.size,
			 record_type);

	tls_ctx->pending_open_record_frags = false;

	rc = tls_do_encryption(sk, tls_ctx, ctx, req, msg_pl->sg.size, i);
	if (rc < 0) {
		if (rc != -EINPROGRESS) {
			tls_err_abort(sk, EBADMSG);
			if (split) {
				tls_ctx->pending_open_record_frags = true;
				tls_merge_open_record(sk, rec, tmp, orig_end);
			}
		}
		return rc;
	} else if (split) {
		msg_pl = &tmp->msg_plaintext;
		msg_en = &tmp->msg_encrypted;
		sk_msg_trim(sk, msg_en, msg_pl->sg.size +
			    tls_ctx->tx.overhead_size);
		tls_ctx->pending_open_record_frags = true;
		ctx->open_rec = tmp;
	}

	return tls_tx_records(sk, flags);
}

static int bpf_exec_tx_verdict(struct sk_msg *msg, struct sock *sk,
			       bool full_record, u8 record_type,
			       size_t *copied, int flags)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct sk_msg msg_redir = { };
	struct sk_psock *psock;
	struct sock *sk_redir;
	struct tls_rec *rec;
	int err = 0, send;
	bool enospc;

	psock = sk_psock_get(sk);
	if (!psock)
		return tls_push_record(sk, flags, record_type);
more_data:
	enospc = sk_msg_full(msg);
	if (psock->eval == __SK_NONE)
		psock->eval = sk_psock_msg_verdict(sk, psock, msg);
	if (msg->cork_bytes && msg->cork_bytes > msg->sg.size &&
	    !enospc && !full_record) {
		err = -ENOSPC;
		goto out_err;
	}
	msg->cork_bytes = 0;
	send = msg->sg.size;
	if (msg->apply_bytes && msg->apply_bytes < send)
		send = msg->apply_bytes;

	switch (psock->eval) {
	case __SK_PASS:
		err = tls_push_record(sk, flags, record_type);
		if (err < 0) {
			*copied -= sk_msg_free(sk, msg);
			tls_free_open_rec(sk);
			goto out_err;
		}
		break;
	case __SK_REDIRECT:
		sk_redir = psock->sk_redir;
		memcpy(&msg_redir, msg, sizeof(*msg));
		if (msg->apply_bytes < send)
			msg->apply_bytes = 0;
		else
			msg->apply_bytes -= send;
		sk_msg_return_zero(sk, msg, send);
		msg->sg.size -= send;
		release_sock(sk);
		err = tcp_bpf_sendmsg_redir(sk_redir, &msg_redir, send, flags);
		lock_sock(sk);
		if (err < 0) {
			*copied -= sk_msg_free_nocharge(sk, &msg_redir);
			msg->sg.size = 0;
		}
		if (msg->sg.size == 0)
			tls_free_open_rec(sk);
		break;
	case __SK_DROP:
	default:
		sk_msg_free_partial(sk, msg, send);
		if (msg->apply_bytes < send)
			msg->apply_bytes = 0;
		else
			msg->apply_bytes -= send;
		if (msg->sg.size == 0)
			tls_free_open_rec(sk);
		*copied -= send;
		err = -EACCES;
	}

	if (likely(!err)) {
		bool reset_eval = !ctx->open_rec;

		rec = ctx->open_rec;
		if (rec) {
			msg = &rec->msg_plaintext;
			if (!msg->apply_bytes)
				reset_eval = true;
		}
		if (reset_eval) {
			psock->eval = __SK_NONE;
			if (psock->sk_redir) {
				sock_put(psock->sk_redir);
				psock->sk_redir = NULL;
			}
		}
		if (rec)
			goto more_data;
	}
 out_err:
	sk_psock_put(sk, psock);
	return err;
}

static int tls_sw_push_pending_record(struct sock *sk, int flags)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec = ctx->open_rec;
	struct sk_msg *msg_pl;
	size_t copied;

	if (!rec)
		return 0;

	msg_pl = &rec->msg_plaintext;
	copied = msg_pl->sg.size;
	if (!copied)
		return 0;

	return bpf_exec_tx_verdict(msg_pl, sk, true, TLS_RECORD_TYPE_DATA,
				   &copied, flags);
}

int tls_sw_sendmsg(struct sock *sk, struct msghdr *msg, size_t size)
{
	long timeo = sock_sndtimeo(sk, msg->msg_flags & MSG_DONTWAIT);
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct crypto_tfm *tfm = crypto_aead_tfm(ctx->aead_send);
	bool async_capable = tfm->__crt_alg->cra_flags & CRYPTO_ALG_ASYNC;
	unsigned char record_type = TLS_RECORD_TYPE_DATA;
	bool is_kvec = iov_iter_is_kvec(&msg->msg_iter);
	bool eor = !(msg->msg_flags & MSG_MORE);
	size_t try_to_copy, copied = 0;
	struct sk_msg *msg_pl, *msg_en;
	struct tls_rec *rec;
	int required_size;
	int num_async = 0;
	bool full_record;
	int record_room;
	int num_zc = 0;
	int orig_size;
	int ret = 0;

	if (msg->msg_flags & ~(MSG_MORE | MSG_DONTWAIT | MSG_NOSIGNAL))
		return -ENOTSUPP;

	lock_sock(sk);

	/* Wait till there is any pending write on socket */
	if (unlikely(sk->sk_write_pending)) {
		ret = wait_on_pending_writer(sk, &timeo);
		if (unlikely(ret))
			goto send_end;
	}

	if (unlikely(msg->msg_controllen)) {
		ret = tls_proccess_cmsg(sk, msg, &record_type);
		if (ret) {
			if (ret == -EINPROGRESS)
				num_async++;
			else if (ret != -EAGAIN)
				goto send_end;
		}
	}

	while (msg_data_left(msg)) {
		if (sk->sk_err) {
			ret = -sk->sk_err;
			goto send_end;
		}

		if (ctx->open_rec)
			rec = ctx->open_rec;
		else
			rec = ctx->open_rec = tls_get_rec(sk);
		if (!rec) {
			ret = -ENOMEM;
			goto send_end;
		}

		msg_pl = &rec->msg_plaintext;
		msg_en = &rec->msg_encrypted;

		orig_size = msg_pl->sg.size;
		full_record = false;
		try_to_copy = msg_data_left(msg);
		record_room = TLS_MAX_PAYLOAD_SIZE - msg_pl->sg.size;
		if (try_to_copy >= record_room) {
			try_to_copy = record_room;
			full_record = true;
		}

		required_size = msg_pl->sg.size + try_to_copy +
				tls_ctx->tx.overhead_size;

		if (!sk_stream_memory_free(sk))
			goto wait_for_sndbuf;

alloc_encrypted:
		ret = tls_alloc_encrypted_msg(sk, required_size);
		if (ret) {
			if (ret != -ENOSPC)
				goto wait_for_memory;

			/* Adjust try_to_copy according to the amount that was
			 * actually allocated. The difference is due
			 * to max sg elements limit
			 */
			try_to_copy -= required_size - msg_en->sg.size;
			full_record = true;
		}

		if (!is_kvec && (full_record || eor) && !async_capable) {
			u32 first = msg_pl->sg.end;

			ret = sk_msg_zerocopy_from_iter(sk, &msg->msg_iter,
							msg_pl, try_to_copy);
			if (ret)
				goto fallback_to_reg_send;

			rec->inplace_crypto = 0;

			num_zc++;
			copied += try_to_copy;

			sk_msg_sg_copy_set(msg_pl, first);
			ret = bpf_exec_tx_verdict(msg_pl, sk, full_record,
						  record_type, &copied,
						  msg->msg_flags);
			if (ret) {
				if (ret == -EINPROGRESS)
					num_async++;
				else if (ret == -ENOMEM)
					goto wait_for_memory;
				else if (ret == -ENOSPC)
					goto rollback_iter;
				else if (ret != -EAGAIN)
					goto send_end;
			}
			continue;
rollback_iter:
			copied -= try_to_copy;
			sk_msg_sg_copy_clear(msg_pl, first);
			iov_iter_revert(&msg->msg_iter,
					msg_pl->sg.size - orig_size);
fallback_to_reg_send:
			sk_msg_trim(sk, msg_pl, orig_size);
		}

		required_size = msg_pl->sg.size + try_to_copy;

		ret = tls_clone_plaintext_msg(sk, required_size);
		if (ret) {
			if (ret != -ENOSPC)
				goto send_end;

			/* Adjust try_to_copy according to the amount that was
			 * actually allocated. The difference is due
			 * to max sg elements limit
			 */
			try_to_copy -= required_size - msg_pl->sg.size;
			full_record = true;
			sk_msg_trim(sk, msg_en, msg_pl->sg.size +
				    tls_ctx->tx.overhead_size);
		}

		if (try_to_copy) {
			ret = sk_msg_memcopy_from_iter(sk, &msg->msg_iter,
						       msg_pl, try_to_copy);
			if (ret < 0)
				goto trim_sgl;
		}

		/* Open records defined only if successfully copied, otherwise
		 * we would trim the sg but not reset the open record frags.
		 */
		tls_ctx->pending_open_record_frags = true;
		copied += try_to_copy;
		if (full_record || eor) {
			ret = bpf_exec_tx_verdict(msg_pl, sk, full_record,
						  record_type, &copied,
						  msg->msg_flags);
			if (ret) {
				if (ret == -EINPROGRESS)
					num_async++;
				else if (ret == -ENOMEM)
					goto wait_for_memory;
				else if (ret != -EAGAIN) {
					if (ret == -ENOSPC)
						ret = 0;
					goto send_end;
				}
			}
		}

		continue;

wait_for_sndbuf:
		set_bit(SOCK_NOSPACE, &sk->sk_socket->flags);
wait_for_memory:
		ret = sk_stream_wait_memory(sk, &timeo);
		if (ret) {
trim_sgl:
			tls_trim_both_msgs(sk, orig_size);
			goto send_end;
		}

		if (msg_en->sg.size < required_size)
			goto alloc_encrypted;
	}

	if (!num_async) {
		goto send_end;
	} else if (num_zc) {
		/* Wait for pending encryptions to get completed */
		smp_store_mb(ctx->async_notify, true);

		if (atomic_read(&ctx->encrypt_pending))
			crypto_wait_req(-EINPROGRESS, &ctx->async_wait);
		else
			reinit_completion(&ctx->async_wait.completion);

		WRITE_ONCE(ctx->async_notify, false);

		if (ctx->async_wait.err) {
			ret = ctx->async_wait.err;
			copied = 0;
		}
	}

	/* Transmit if any encryptions have completed */
	if (test_and_clear_bit(BIT_TX_SCHEDULED, &ctx->tx_bitmask)) {
		cancel_delayed_work(&ctx->tx_work.work);
		tls_tx_records(sk, msg->msg_flags);
	}

send_end:
	ret = sk_stream_error(sk, msg->msg_flags, ret);

	release_sock(sk);
	return copied ? copied : ret;
}

int tls_sw_sendpage(struct sock *sk, struct page *page,
		    int offset, size_t size, int flags)
{
	long timeo = sock_sndtimeo(sk, flags & MSG_DONTWAIT);
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	unsigned char record_type = TLS_RECORD_TYPE_DATA;
	struct sk_msg *msg_pl;
	struct tls_rec *rec;
	int num_async = 0;
	size_t copied = 0;
	bool full_record;
	int record_room;
	int ret = 0;
	bool eor;

	if (flags & ~(MSG_MORE | MSG_DONTWAIT | MSG_NOSIGNAL |
		      MSG_SENDPAGE_NOTLAST))
		return -ENOTSUPP;

	/* No MSG_EOR from splice, only look at MSG_MORE */
	eor = !(flags & (MSG_MORE | MSG_SENDPAGE_NOTLAST));

	lock_sock(sk);

	sk_clear_bit(SOCKWQ_ASYNC_NOSPACE, sk);

	/* Wait till there is any pending write on socket */
	if (unlikely(sk->sk_write_pending)) {
		ret = wait_on_pending_writer(sk, &timeo);
		if (unlikely(ret))
			goto sendpage_end;
	}

	/* Call the sk_stream functions to manage the sndbuf mem. */
	while (size > 0) {
		size_t copy, required_size;

		if (sk->sk_err) {
			ret = -sk->sk_err;
			goto sendpage_end;
		}

		if (ctx->open_rec)
			rec = ctx->open_rec;
		else
			rec = ctx->open_rec = tls_get_rec(sk);
		if (!rec) {
			ret = -ENOMEM;
			goto sendpage_end;
		}

		msg_pl = &rec->msg_plaintext;

		full_record = false;
		record_room = TLS_MAX_PAYLOAD_SIZE - msg_pl->sg.size;
		copied = 0;
		copy = size;
		if (copy >= record_room) {
			copy = record_room;
			full_record = true;
		}

		required_size = msg_pl->sg.size + copy +
				tls_ctx->tx.overhead_size;

		if (!sk_stream_memory_free(sk))
			goto wait_for_sndbuf;
alloc_payload:
		ret = tls_alloc_encrypted_msg(sk, required_size);
		if (ret) {
			if (ret != -ENOSPC)
				goto wait_for_memory;

			/* Adjust copy according to the amount that was
			 * actually allocated. The difference is due
			 * to max sg elements limit
			 */
			copy -= required_size - msg_pl->sg.size;
			full_record = true;
		}

		sk_msg_page_add(msg_pl, page, copy, offset);
		sk_mem_charge(sk, copy);

		offset += copy;
		size -= copy;
		copied += copy;

		tls_ctx->pending_open_record_frags = true;
		if (full_record || eor || sk_msg_full(msg_pl)) {
			rec->inplace_crypto = 0;
			ret = bpf_exec_tx_verdict(msg_pl, sk, full_record,
						  record_type, &copied, flags);
			if (ret) {
				if (ret == -EINPROGRESS)
					num_async++;
				else if (ret == -ENOMEM)
					goto wait_for_memory;
				else if (ret != -EAGAIN) {
					if (ret == -ENOSPC)
						ret = 0;
					goto sendpage_end;
				}
			}
		}
		continue;
wait_for_sndbuf:
		set_bit(SOCK_NOSPACE, &sk->sk_socket->flags);
wait_for_memory:
		ret = sk_stream_wait_memory(sk, &timeo);
		if (ret) {
			tls_trim_both_msgs(sk, msg_pl->sg.size);
			goto sendpage_end;
		}

		goto alloc_payload;
	}

	if (num_async) {
		/* Transmit if any encryptions have completed */
		if (test_and_clear_bit(BIT_TX_SCHEDULED, &ctx->tx_bitmask)) {
			cancel_delayed_work(&ctx->tx_work.work);
			tls_tx_records(sk, flags);
		}
	}
sendpage_end:
	ret = sk_stream_error(sk, flags, ret);
	release_sock(sk);
	return copied ? copied : ret;
}

static struct sk_buff *tls_wait_data(struct sock *sk, struct sk_psock *psock,
				     int flags, long timeo, int *err)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct sk_buff *skb;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);

	while (!(skb = ctx->recv_pkt) && sk_psock_queue_empty(psock)) {
		if (sk->sk_err) {
			*err = sock_error(sk);
			return NULL;
		}

		if (sk->sk_shutdown & RCV_SHUTDOWN)
			return NULL;

		if (sock_flag(sk, SOCK_DONE))
			return NULL;

		if ((flags & MSG_DONTWAIT) || !timeo) {
			*err = -EAGAIN;
			return NULL;
		}

		add_wait_queue(sk_sleep(sk), &wait);
		sk_set_bit(SOCKWQ_ASYNC_WAITDATA, sk);
		sk_wait_event(sk, &timeo,
			      ctx->recv_pkt != skb ||
			      !sk_psock_queue_empty(psock),
			      &wait);
		sk_clear_bit(SOCKWQ_ASYNC_WAITDATA, sk);
		remove_wait_queue(sk_sleep(sk), &wait);

		/* Handle signals */
		if (signal_pending(current)) {
			*err = sock_intr_errno(timeo);
			return NULL;
		}
	}

	return skb;
}

static int tls_setup_from_iter(struct sock *sk, struct iov_iter *from,
			       int length, int *pages_used,
			       unsigned int *size_used,
			       struct scatterlist *to,
			       int to_max_pages)
{
	int rc = 0, i = 0, num_elem = *pages_used, maxpages;
	struct page *pages[MAX_SKB_FRAGS];
	unsigned int size = *size_used;
	ssize_t copied, use;
	size_t offset;

	while (length > 0) {
		i = 0;
		maxpages = to_max_pages - num_elem;
		if (maxpages == 0) {
			rc = -EFAULT;
			goto out;
		}
		copied = iov_iter_get_pages(from, pages,
					    length,
					    maxpages, &offset);
		if (copied <= 0) {
			rc = -EFAULT;
			goto out;
		}

		iov_iter_advance(from, copied);

		length -= copied;
		size += copied;
		while (copied) {
			use = min_t(int, copied, PAGE_SIZE - offset);

			sg_set_page(&to[num_elem],
				    pages[i], use, offset);
			sg_unmark_end(&to[num_elem]);
			/* We do not uncharge memory from this API */

			offset = 0;
			copied -= use;

			i++;
			num_elem++;
		}
	}
	/* Mark the end in the last sg entry if newly added */
	if (num_elem > *pages_used)
		sg_mark_end(&to[num_elem - 1]);
out:
	if (rc)
		iov_iter_revert(from, size - *size_used);
	*size_used = size;
	*pages_used = num_elem;

	return rc;
}

/* This function decrypts the input skb into either out_iov or in out_sg
 * or in skb buffers itself. The input parameter 'zc' indicates if
 * zero-copy mode needs to be tried or not. With zero-copy mode, either
 * out_iov or out_sg must be non-NULL. In case both out_iov and out_sg are
 * NULL, then the decryption happens inside skb buffers itself, i.e.
 * zero-copy gets disabled and 'zc' is updated.
 */

static int decrypt_internal(struct sock *sk, struct sk_buff *skb,
			    struct iov_iter *out_iov,
			    struct scatterlist *out_sg,
			    int *chunk, bool *zc)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct strp_msg *rxm = strp_msg(skb);
	int n_sgin, n_sgout, nsg, mem_size, aead_size, err, pages = 0;
	struct aead_request *aead_req;
	struct sk_buff *unused;
	u8 *aad, *iv, *mem = NULL;
	struct scatterlist *sgin = NULL;
	struct scatterlist *sgout = NULL;
	const int data_len = rxm->full_len - tls_ctx->rx.overhead_size;

	if (*zc && (out_iov || out_sg)) {
		if (out_iov)
			n_sgout = iov_iter_npages(out_iov, INT_MAX) + 1;
		else
			n_sgout = sg_nents(out_sg);
		n_sgin = skb_nsg(skb, rxm->offset + tls_ctx->rx.prepend_size,
				 rxm->full_len - tls_ctx->rx.prepend_size);
	} else {
		n_sgout = 0;
		*zc = false;
		n_sgin = skb_cow_data(skb, 0, &unused);
	}

	if (n_sgin < 1)
		return -EBADMSG;

	/* Increment to accommodate AAD */
	n_sgin = n_sgin + 1;

	nsg = n_sgin + n_sgout;

	aead_size = sizeof(*aead_req) + crypto_aead_reqsize(ctx->aead_recv);
	mem_size = aead_size + (nsg * sizeof(struct scatterlist));
	mem_size = mem_size + TLS_AAD_SPACE_SIZE;
	mem_size = mem_size + crypto_aead_ivsize(ctx->aead_recv);

	/* Allocate a single block of memory which contains
	 * aead_req || sgin[] || sgout[] || aad || iv.
	 * This order achieves correct alignment for aead_req, sgin, sgout.
	 */
	mem = kmalloc(mem_size, sk->sk_allocation);
	if (!mem)
		return -ENOMEM;

	/* Segment the allocated memory */
	aead_req = (struct aead_request *)mem;
	sgin = (struct scatterlist *)(mem + aead_size);
	sgout = sgin + n_sgin;
	aad = (u8 *)(sgout + n_sgout);
	iv = aad + TLS_AAD_SPACE_SIZE;

	/* Prepare IV */
	err = skb_copy_bits(skb, rxm->offset + TLS_HEADER_SIZE,
			    iv + TLS_CIPHER_AES_GCM_128_SALT_SIZE,
			    tls_ctx->rx.iv_size);
	if (err < 0) {
		kfree(mem);
		return err;
	}
	memcpy(iv, tls_ctx->rx.iv, TLS_CIPHER_AES_GCM_128_SALT_SIZE);

	/* Prepare AAD */
	tls_make_aad(aad, rxm->full_len - tls_ctx->rx.overhead_size,
		     tls_ctx->rx.rec_seq, tls_ctx->rx.rec_seq_size,
		     ctx->control);

	/* Prepare sgin */
	sg_init_table(sgin, n_sgin);
	sg_set_buf(&sgin[0], aad, TLS_AAD_SPACE_SIZE);
	err = skb_to_sgvec(skb, &sgin[1],
			   rxm->offset + tls_ctx->rx.prepend_size,
			   rxm->full_len - tls_ctx->rx.prepend_size);
	if (err < 0) {
		kfree(mem);
		return err;
	}

	if (n_sgout) {
		if (out_iov) {
			sg_init_table(sgout, n_sgout);
			sg_set_buf(&sgout[0], aad, TLS_AAD_SPACE_SIZE);

			*chunk = 0;
			err = tls_setup_from_iter(sk, out_iov, data_len,
						  &pages, chunk, &sgout[1],
						  (n_sgout - 1));
			if (err < 0)
				goto fallback_to_reg_recv;
		} else if (out_sg) {
			memcpy(sgout, out_sg, n_sgout * sizeof(*sgout));
		} else {
			goto fallback_to_reg_recv;
		}
	} else {
fallback_to_reg_recv:
		sgout = sgin;
		pages = 0;
		*chunk = 0;
		*zc = false;
	}

	/* Prepare and submit AEAD request */
	err = tls_do_decryption(sk, skb, sgin, sgout, iv,
				data_len, aead_req, *zc);
	if (err == -EINPROGRESS)
		return err;

	/* Release the pages in case iov was mapped to pages */
	for (; pages > 0; pages--)
		put_page(sg_page(&sgout[pages]));

	kfree(mem);
	return err;
}

static int decrypt_skb_update(struct sock *sk, struct sk_buff *skb,
			      struct iov_iter *dest, int *chunk, bool *zc)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct strp_msg *rxm = strp_msg(skb);
	int err = 0;

#ifdef CONFIG_TLS_DEVICE
	err = tls_device_decrypted(sk, skb);
	if (err < 0)
		return err;
#endif
	if (!ctx->decrypted) {
		err = decrypt_internal(sk, skb, dest, NULL, chunk, zc);
		if (err < 0) {
			if (err == -EINPROGRESS)
				tls_advance_record_sn(sk, &tls_ctx->rx);

			return err;
		}
	} else {
		*zc = false;
	}

	rxm->offset += tls_ctx->rx.prepend_size;
	rxm->full_len -= tls_ctx->rx.overhead_size;
	tls_advance_record_sn(sk, &tls_ctx->rx);
	ctx->decrypted = true;
	ctx->saved_data_ready(sk);

	return err;
}

int decrypt_skb(struct sock *sk, struct sk_buff *skb,
		struct scatterlist *sgout)
{
	bool zc = true;
	int chunk;

	return decrypt_internal(sk, skb, NULL, sgout, &chunk, &zc);
}

static bool tls_sw_advance_skb(struct sock *sk, struct sk_buff *skb,
			       unsigned int len)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);

	if (skb) {
		struct strp_msg *rxm = strp_msg(skb);

		if (len < rxm->full_len) {
			rxm->offset += len;
			rxm->full_len -= len;
			return false;
		}
		kfree_skb(skb);
	}

	/* Finished with message */
	ctx->recv_pkt = NULL;
	__strp_unpause(&ctx->strp);

	return true;
}

int tls_sw_recvmsg(struct sock *sk,
		   struct msghdr *msg,
		   size_t len,
		   int nonblock,
		   int flags,
		   int *addr_len)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct sk_psock *psock;
	unsigned char control;
	struct strp_msg *rxm;
	struct sk_buff *skb;
	ssize_t copied = 0;
	bool cmsg = false;
	int target, err = 0;
	long timeo;
	bool is_kvec = iov_iter_is_kvec(&msg->msg_iter);
	int num_async = 0;

	flags |= nonblock;

	if (unlikely(flags & MSG_ERRQUEUE))
		return sock_recv_errqueue(sk, msg, len, SOL_IP, IP_RECVERR);

	psock = sk_psock_get(sk);
	lock_sock(sk);

	target = sock_rcvlowat(sk, flags & MSG_WAITALL, len);
	timeo = sock_rcvtimeo(sk, flags & MSG_DONTWAIT);
	do {
		bool zc = false;
		bool async = false;
		int chunk = 0;

		skb = tls_wait_data(sk, psock, flags, timeo, &err);
		if (!skb) {
			if (psock) {
				int ret = __tcp_bpf_recvmsg(sk, psock,
							    msg, len, flags);

				if (ret > 0) {
					copied += ret;
					len -= ret;
					continue;
				}
			}
			goto recv_end;
		}

		rxm = strp_msg(skb);

		if (!cmsg) {
			int cerr;

			cerr = put_cmsg(msg, SOL_TLS, TLS_GET_RECORD_TYPE,
					sizeof(ctx->control), &ctx->control);
			cmsg = true;
			control = ctx->control;
			if (ctx->control != TLS_RECORD_TYPE_DATA) {
				if (cerr || msg->msg_flags & MSG_CTRUNC) {
					err = -EIO;
					goto recv_end;
				}
			}
		} else if (control != ctx->control) {
			goto recv_end;
		}

		if (!ctx->decrypted) {
			int to_copy = rxm->full_len - tls_ctx->rx.overhead_size;

			if (!is_kvec && to_copy <= len &&
			    likely(!(flags & MSG_PEEK)))
				zc = true;

			err = decrypt_skb_update(sk, skb, &msg->msg_iter,
						 &chunk, &zc);
			if (err < 0 && err != -EINPROGRESS) {
				tls_err_abort(sk, EBADMSG);
				goto recv_end;
			}

			if (err == -EINPROGRESS) {
				async = true;
				num_async++;
				goto pick_next_record;
			}

			ctx->decrypted = true;
		}

		if (!zc) {
			chunk = min_t(unsigned int, rxm->full_len, len);

			err = skb_copy_datagram_msg(skb, rxm->offset, msg,
						    chunk);
			if (err < 0)
				goto recv_end;
		}

pick_next_record:
		copied += chunk;
		len -= chunk;
		if (likely(!(flags & MSG_PEEK))) {
			u8 control = ctx->control;

			/* For async, drop current skb reference */
			if (async)
				skb = NULL;

			if (tls_sw_advance_skb(sk, skb, chunk)) {
				/* Return full control message to
				 * userspace before trying to parse
				 * another message type
				 */
				msg->msg_flags |= MSG_EOR;
				if (control != TLS_RECORD_TYPE_DATA)
					goto recv_end;
			} else {
				break;
			}
		} else {
			/* MSG_PEEK right now cannot look beyond current skb
			 * from strparser, meaning we cannot advance skb here
			 * and thus unpause strparser since we'd loose original
			 * one.
			 */
			break;
		}

		/* If we have a new message from strparser, continue now. */
		if (copied >= target && !ctx->recv_pkt)
			break;
	} while (len);

recv_end:
	if (num_async) {
		/* Wait for all previously submitted records to be decrypted */
		smp_store_mb(ctx->async_notify, true);
		if (atomic_read(&ctx->decrypt_pending)) {
			err = crypto_wait_req(-EINPROGRESS, &ctx->async_wait);
			if (err) {
				/* one of async decrypt failed */
				tls_err_abort(sk, err);
				copied = 0;
			}
		} else {
			reinit_completion(&ctx->async_wait.completion);
		}
		WRITE_ONCE(ctx->async_notify, false);
	}

	release_sock(sk);
	if (psock)
		sk_psock_put(sk, psock);
	return copied ? : err;
}

ssize_t tls_sw_splice_read(struct socket *sock,  loff_t *ppos,
			   struct pipe_inode_info *pipe,
			   size_t len, unsigned int flags)
{
	struct tls_context *tls_ctx = tls_get_ctx(sock->sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct strp_msg *rxm = NULL;
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	ssize_t copied = 0;
	int err = 0;
	long timeo;
	int chunk;
	bool zc = false;

	lock_sock(sk);

	timeo = sock_rcvtimeo(sk, flags & MSG_DONTWAIT);

	skb = tls_wait_data(sk, NULL, flags, timeo, &err);
	if (!skb)
		goto splice_read_end;

	/* splice does not support reading control messages */
	if (ctx->control != TLS_RECORD_TYPE_DATA) {
		err = -ENOTSUPP;
		goto splice_read_end;
	}

	if (!ctx->decrypted) {
		err = decrypt_skb_update(sk, skb, NULL, &chunk, &zc);

		if (err < 0) {
			tls_err_abort(sk, EBADMSG);
			goto splice_read_end;
		}
		ctx->decrypted = true;
	}
	rxm = strp_msg(skb);

	chunk = min_t(unsigned int, rxm->full_len, len);
	copied = skb_splice_bits(skb, sk, rxm->offset, pipe, chunk, flags);
	if (copied < 0)
		goto splice_read_end;

	if (likely(!(flags & MSG_PEEK)))
		tls_sw_advance_skb(sk, skb, copied);

splice_read_end:
	release_sock(sk);
	return copied ? : err;
}

bool tls_sw_stream_read(const struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	bool ingress_empty = true;
	struct sk_psock *psock;

	rcu_read_lock();
	psock = sk_psock(sk);
	if (psock)
		ingress_empty = list_empty(&psock->ingress_msg);
	rcu_read_unlock();

	return !ingress_empty || ctx->recv_pkt;
}

static int tls_read_size(struct strparser *strp, struct sk_buff *skb)
{
	struct tls_context *tls_ctx = tls_get_ctx(strp->sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	char header[TLS_HEADER_SIZE + MAX_IV_SIZE];
	struct strp_msg *rxm = strp_msg(skb);
	size_t cipher_overhead;
	size_t data_len = 0;
	int ret;

	/* Verify that we have a full TLS header, or wait for more data */
	if (rxm->offset + tls_ctx->rx.prepend_size > skb->len)
		return 0;

	/* Sanity-check size of on-stack buffer. */
	if (WARN_ON(tls_ctx->rx.prepend_size > sizeof(header))) {
		ret = -EINVAL;
		goto read_failure;
	}

	/* Linearize header to local buffer */
	ret = skb_copy_bits(skb, rxm->offset, header, tls_ctx->rx.prepend_size);

	if (ret < 0)
		goto read_failure;

	ctx->control = header[0];

	data_len = ((header[4] & 0xFF) | (header[3] << 8));

	cipher_overhead = tls_ctx->rx.tag_size + tls_ctx->rx.iv_size;

	if (data_len > TLS_MAX_PAYLOAD_SIZE + cipher_overhead) {
		ret = -EMSGSIZE;
		goto read_failure;
	}
	if (data_len < cipher_overhead) {
		ret = -EBADMSG;
		goto read_failure;
	}

	if (header[1] != TLS_VERSION_MINOR(tls_ctx->crypto_recv.info.version) ||
	    header[2] != TLS_VERSION_MAJOR(tls_ctx->crypto_recv.info.version)) {
		ret = -EINVAL;
		goto read_failure;
	}

#ifdef CONFIG_TLS_DEVICE
	handle_device_resync(strp->sk, TCP_SKB_CB(skb)->seq + rxm->offset,
			     *(u64*)tls_ctx->rx.rec_seq);
#endif
	return data_len + TLS_HEADER_SIZE;

read_failure:
	tls_err_abort(strp->sk, ret);

	return ret;
}

static void tls_queue(struct strparser *strp, struct sk_buff *skb)
{
	struct tls_context *tls_ctx = tls_get_ctx(strp->sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);

	ctx->decrypted = false;

	ctx->recv_pkt = skb;
	strp_pause(strp);

	ctx->saved_data_ready(strp->sk);
}

static void tls_data_ready(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);
	struct sk_psock *psock;

	strp_data_ready(&ctx->strp);

	psock = sk_psock_get(sk);
	if (psock && !list_empty(&psock->ingress_msg)) {
		ctx->saved_data_ready(sk);
		sk_psock_put(sk, psock);
	}
}

void tls_sw_free_resources_tx(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);
	struct tls_rec *rec, *tmp;

	/* Wait for any pending async encryptions to complete */
	smp_store_mb(ctx->async_notify, true);
	if (atomic_read(&ctx->encrypt_pending))
		crypto_wait_req(-EINPROGRESS, &ctx->async_wait);

	release_sock(sk);
	cancel_delayed_work_sync(&ctx->tx_work.work);
	lock_sock(sk);

	/* Tx whatever records we can transmit and abandon the rest */
	tls_tx_records(sk, -1);

	/* Free up un-sent records in tx_list. First, free
	 * the partially sent record if any at head of tx_list.
	 */
	if (tls_ctx->partially_sent_record) {
		struct scatterlist *sg = tls_ctx->partially_sent_record;

		while (1) {
			put_page(sg_page(sg));
			sk_mem_uncharge(sk, sg->length);

			if (sg_is_last(sg))
				break;
			sg++;
		}

		tls_ctx->partially_sent_record = NULL;

		rec = list_first_entry(&ctx->tx_list,
				       struct tls_rec, list);
		list_del(&rec->list);
		sk_msg_free(sk, &rec->msg_plaintext);
		kfree(rec);
	}

	list_for_each_entry_safe(rec, tmp, &ctx->tx_list, list) {
		list_del(&rec->list);
		sk_msg_free(sk, &rec->msg_encrypted);
		sk_msg_free(sk, &rec->msg_plaintext);
		kfree(rec);
	}

	crypto_free_aead(ctx->aead_send);
	tls_free_open_rec(sk);

	kfree(ctx);
}

void tls_sw_release_resources_rx(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);

	if (ctx->aead_recv) {
		kfree_skb(ctx->recv_pkt);
		ctx->recv_pkt = NULL;
		crypto_free_aead(ctx->aead_recv);
		strp_stop(&ctx->strp);
		write_lock_bh(&sk->sk_callback_lock);
		sk->sk_data_ready = ctx->saved_data_ready;
		write_unlock_bh(&sk->sk_callback_lock);
		release_sock(sk);
		strp_done(&ctx->strp);
		lock_sock(sk);
	}
}

void tls_sw_free_resources_rx(struct sock *sk)
{
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_rx *ctx = tls_sw_ctx_rx(tls_ctx);

	tls_sw_release_resources_rx(sk);

	kfree(ctx);
}

/* The work handler to transmitt the encrypted records in tx_list */
static void tx_work_handler(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct tx_work *tx_work = container_of(delayed_work,
					       struct tx_work, work);
	struct sock *sk = tx_work->sk;
	struct tls_context *tls_ctx = tls_get_ctx(sk);
	struct tls_sw_context_tx *ctx = tls_sw_ctx_tx(tls_ctx);

	if (!test_and_clear_bit(BIT_TX_SCHEDULED, &ctx->tx_bitmask))
		return;

	lock_sock(sk);
	tls_tx_records(sk, -1);
	release_sock(sk);
}

int tls_set_sw_offload(struct sock *sk, struct tls_context *ctx, int tx)
{
	struct tls_crypto_info *crypto_info;
	struct tls12_crypto_info_aes_gcm_128 *gcm_128_info;
	struct tls_sw_context_tx *sw_ctx_tx = NULL;
	struct tls_sw_context_rx *sw_ctx_rx = NULL;
	struct cipher_context *cctx;
	struct crypto_aead **aead;
	struct strp_callbacks cb;
	u16 nonce_size, tag_size, iv_size, rec_seq_size;
	char *iv, *rec_seq;
	int rc = 0;

	if (!ctx) {
		rc = -EINVAL;
		goto out;
	}

	if (tx) {
		if (!ctx->priv_ctx_tx) {
			sw_ctx_tx = kzalloc(sizeof(*sw_ctx_tx), GFP_KERNEL);
			if (!sw_ctx_tx) {
				rc = -ENOMEM;
				goto out;
			}
			ctx->priv_ctx_tx = sw_ctx_tx;
		} else {
			sw_ctx_tx =
				(struct tls_sw_context_tx *)ctx->priv_ctx_tx;
		}
	} else {
		if (!ctx->priv_ctx_rx) {
			sw_ctx_rx = kzalloc(sizeof(*sw_ctx_rx), GFP_KERNEL);
			if (!sw_ctx_rx) {
				rc = -ENOMEM;
				goto out;
			}
			ctx->priv_ctx_rx = sw_ctx_rx;
		} else {
			sw_ctx_rx =
				(struct tls_sw_context_rx *)ctx->priv_ctx_rx;
		}
	}

	if (tx) {
		crypto_init_wait(&sw_ctx_tx->async_wait);
		crypto_info = &ctx->crypto_send.info;
		cctx = &ctx->tx;
		aead = &sw_ctx_tx->aead_send;
		INIT_LIST_HEAD(&sw_ctx_tx->tx_list);
		INIT_DELAYED_WORK(&sw_ctx_tx->tx_work.work, tx_work_handler);
		sw_ctx_tx->tx_work.sk = sk;
	} else {
		crypto_init_wait(&sw_ctx_rx->async_wait);
		crypto_info = &ctx->crypto_recv.info;
		cctx = &ctx->rx;
		aead = &sw_ctx_rx->aead_recv;
	}

	switch (crypto_info->cipher_type) {
	case TLS_CIPHER_AES_GCM_128: {
		nonce_size = TLS_CIPHER_AES_GCM_128_IV_SIZE;
		tag_size = TLS_CIPHER_AES_GCM_128_TAG_SIZE;
		iv_size = TLS_CIPHER_AES_GCM_128_IV_SIZE;
		iv = ((struct tls12_crypto_info_aes_gcm_128 *)crypto_info)->iv;
		rec_seq_size = TLS_CIPHER_AES_GCM_128_REC_SEQ_SIZE;
		rec_seq =
		 ((struct tls12_crypto_info_aes_gcm_128 *)crypto_info)->rec_seq;
		gcm_128_info =
			(struct tls12_crypto_info_aes_gcm_128 *)crypto_info;
		break;
	}
	default:
		rc = -EINVAL;
		goto free_priv;
	}

	/* Sanity-check the IV size for stack allocations. */
	if (iv_size > MAX_IV_SIZE || nonce_size > MAX_IV_SIZE) {
		rc = -EINVAL;
		goto free_priv;
	}

	cctx->prepend_size = TLS_HEADER_SIZE + nonce_size;
	cctx->tag_size = tag_size;
	cctx->overhead_size = cctx->prepend_size + cctx->tag_size;
	cctx->iv_size = iv_size;
	cctx->iv = kmalloc(iv_size + TLS_CIPHER_AES_GCM_128_SALT_SIZE,
			   GFP_KERNEL);
	if (!cctx->iv) {
		rc = -ENOMEM;
		goto free_priv;
	}
	memcpy(cctx->iv, gcm_128_info->salt, TLS_CIPHER_AES_GCM_128_SALT_SIZE);
	memcpy(cctx->iv + TLS_CIPHER_AES_GCM_128_SALT_SIZE, iv, iv_size);
	cctx->rec_seq_size = rec_seq_size;
	cctx->rec_seq = kmemdup(rec_seq, rec_seq_size, GFP_KERNEL);
	if (!cctx->rec_seq) {
		rc = -ENOMEM;
		goto free_iv;
	}

	if (!*aead) {
		*aead = crypto_alloc_aead("gcm(aes)", 0, 0);
		if (IS_ERR(*aead)) {
			rc = PTR_ERR(*aead);
			*aead = NULL;
			goto free_rec_seq;
		}
	}

	ctx->push_pending_record = tls_sw_push_pending_record;

	rc = crypto_aead_setkey(*aead, gcm_128_info->key,
				TLS_CIPHER_AES_GCM_128_KEY_SIZE);
	if (rc)
		goto free_aead;

	rc = crypto_aead_setauthsize(*aead, cctx->tag_size);
	if (rc)
		goto free_aead;

	if (sw_ctx_rx) {
		/* Set up strparser */
		memset(&cb, 0, sizeof(cb));
		cb.rcv_msg = tls_queue;
		cb.parse_msg = tls_read_size;

		strp_init(&sw_ctx_rx->strp, sk, &cb);

		write_lock_bh(&sk->sk_callback_lock);
		sw_ctx_rx->saved_data_ready = sk->sk_data_ready;
		sk->sk_data_ready = tls_data_ready;
		write_unlock_bh(&sk->sk_callback_lock);

		strp_check_rcv(&sw_ctx_rx->strp);
	}

	goto out;

free_aead:
	crypto_free_aead(*aead);
	*aead = NULL;
free_rec_seq:
	kfree(cctx->rec_seq);
	cctx->rec_seq = NULL;
free_iv:
	kfree(cctx->iv);
	cctx->iv = NULL;
free_priv:
	if (tx) {
		kfree(ctx->priv_ctx_tx);
		ctx->priv_ctx_tx = NULL;
	} else {
		kfree(ctx->priv_ctx_rx);
		ctx->priv_ctx_rx = NULL;
	}
out:
	return rc;
}
