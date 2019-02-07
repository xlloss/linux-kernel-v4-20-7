/*
 * linux/net/sunrpc/auth.c
 *
 * Generic RPC client authentication API.
 *
 * Copyright (C) 1996, Olaf Kirch <okir@monad.swb.de>
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cred.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/hash.h>
#include <linux/sunrpc/clnt.h>
#include <linux/sunrpc/gss_api.h>
#include <linux/spinlock.h>

#if IS_ENABLED(CONFIG_SUNRPC_DEBUG)
# define RPCDBG_FACILITY	RPCDBG_AUTH
#endif

#define RPC_CREDCACHE_DEFAULT_HASHBITS	(4)
struct rpc_cred_cache {
	struct hlist_head	*hashtable;
	unsigned int		hashbits;
	spinlock_t		lock;
};

static unsigned int auth_hashbits = RPC_CREDCACHE_DEFAULT_HASHBITS;

static const struct rpc_authops __rcu *auth_flavors[RPC_AUTH_MAXFLAVOR] = {
	[RPC_AUTH_NULL] = (const struct rpc_authops __force __rcu *)&authnull_ops,
	[RPC_AUTH_UNIX] = (const struct rpc_authops __force __rcu *)&authunix_ops,
	NULL,			/* others can be loadable modules */
};

static LIST_HEAD(cred_unused);
static unsigned long number_cred_unused;

#define MAX_HASHTABLE_BITS (14)
static int param_set_hashtbl_sz(const char *val, const struct kernel_param *kp)
{
	unsigned long num;
	unsigned int nbits;
	int ret;

	if (!val)
		goto out_inval;
	ret = kstrtoul(val, 0, &num);
	if (ret)
		goto out_inval;
	nbits = fls(num - 1);
	if (nbits > MAX_HASHTABLE_BITS || nbits < 2)
		goto out_inval;
	*(unsigned int *)kp->arg = nbits;
	return 0;
out_inval:
	return -EINVAL;
}

static int param_get_hashtbl_sz(char *buffer, const struct kernel_param *kp)
{
	unsigned int nbits;

	nbits = *(unsigned int *)kp->arg;
	return sprintf(buffer, "%u", 1U << nbits);
}

#define param_check_hashtbl_sz(name, p) __param_check(name, p, unsigned int);

static const struct kernel_param_ops param_ops_hashtbl_sz = {
	.set = param_set_hashtbl_sz,
	.get = param_get_hashtbl_sz,
};

module_param_named(auth_hashtable_size, auth_hashbits, hashtbl_sz, 0644);
MODULE_PARM_DESC(auth_hashtable_size, "RPC credential cache hashtable size");

static unsigned long auth_max_cred_cachesize = ULONG_MAX;
module_param(auth_max_cred_cachesize, ulong, 0644);
MODULE_PARM_DESC(auth_max_cred_cachesize, "RPC credential maximum total cache size");

static u32
pseudoflavor_to_flavor(u32 flavor) {
	if (flavor > RPC_AUTH_MAXFLAVOR)
		return RPC_AUTH_GSS;
	return flavor;
}

int
rpcauth_register(const struct rpc_authops *ops)
{
	const struct rpc_authops *old;
	rpc_authflavor_t flavor;

	if ((flavor = ops->au_flavor) >= RPC_AUTH_MAXFLAVOR)
		return -EINVAL;
	old = cmpxchg((const struct rpc_authops ** __force)&auth_flavors[flavor], NULL, ops);
	if (old == NULL || old == ops)
		return 0;
	return -EPERM;
}
EXPORT_SYMBOL_GPL(rpcauth_register);

int
rpcauth_unregister(const struct rpc_authops *ops)
{
	const struct rpc_authops *old;
	rpc_authflavor_t flavor;

	if ((flavor = ops->au_flavor) >= RPC_AUTH_MAXFLAVOR)
		return -EINVAL;

	old = cmpxchg((const struct rpc_authops ** __force)&auth_flavors[flavor], ops, NULL);
	if (old == ops || old == NULL)
		return 0;
	return -EPERM;
}
EXPORT_SYMBOL_GPL(rpcauth_unregister);

static const struct rpc_authops *
rpcauth_get_authops(rpc_authflavor_t flavor)
{
	const struct rpc_authops *ops;

	if (flavor >= RPC_AUTH_MAXFLAVOR)
		return NULL;

	rcu_read_lock();
	ops = rcu_dereference(auth_flavors[flavor]);
	if (ops == NULL) {
		rcu_read_unlock();
		request_module("rpc-auth-%u", flavor);
		rcu_read_lock();
		ops = rcu_dereference(auth_flavors[flavor]);
		if (ops == NULL)
			goto out;
	}
	if (!try_module_get(ops->owner))
		ops = NULL;
out:
	rcu_read_unlock();
	return ops;
}

static void
rpcauth_put_authops(const struct rpc_authops *ops)
{
	module_put(ops->owner);
}

/**
 * rpcauth_get_pseudoflavor - check if security flavor is supported
 * @flavor: a security flavor
 * @info: a GSS mech OID, quality of protection, and service value
 *
 * Verifies that an appropriate kernel module is available or already loaded.
 * Returns an equivalent pseudoflavor, or RPC_AUTH_MAXFLAVOR if "flavor" is
 * not supported locally.
 */
rpc_authflavor_t
rpcauth_get_pseudoflavor(rpc_authflavor_t flavor, struct rpcsec_gss_info *info)
{
	const struct rpc_authops *ops = rpcauth_get_authops(flavor);
	rpc_authflavor_t pseudoflavor;

	if (!ops)
		return RPC_AUTH_MAXFLAVOR;
	pseudoflavor = flavor;
	if (ops->info2flavor != NULL)
		pseudoflavor = ops->info2flavor(info);

	rpcauth_put_authops(ops);
	return pseudoflavor;
}
EXPORT_SYMBOL_GPL(rpcauth_get_pseudoflavor);

/**
 * rpcauth_get_gssinfo - find GSS tuple matching a GSS pseudoflavor
 * @pseudoflavor: GSS pseudoflavor to match
 * @info: rpcsec_gss_info structure to fill in
 *
 * Returns zero and fills in "info" if pseudoflavor matches a
 * supported mechanism.
 */
int
rpcauth_get_gssinfo(rpc_authflavor_t pseudoflavor, struct rpcsec_gss_info *info)
{
	rpc_authflavor_t flavor = pseudoflavor_to_flavor(pseudoflavor);
	const struct rpc_authops *ops;
	int result;

	ops = rpcauth_get_authops(flavor);
	if (ops == NULL)
		return -ENOENT;

	result = -ENOENT;
	if (ops->flavor2info != NULL)
		result = ops->flavor2info(pseudoflavor, info);

	rpcauth_put_authops(ops);
	return result;
}
EXPORT_SYMBOL_GPL(rpcauth_get_gssinfo);

/**
 * rpcauth_list_flavors - discover registered flavors and pseudoflavors
 * @array: array to fill in
 * @size: size of "array"
 *
 * Returns the number of array items filled in, or a negative errno.
 *
 * The returned array is not sorted by any policy.  Callers should not
 * rely on the order of the items in the returned array.
 */
int
rpcauth_list_flavors(rpc_authflavor_t *array, int size)
{
	const struct rpc_authops *ops;
	rpc_authflavor_t flavor, pseudos[4];
	int i, len, result = 0;

	rcu_read_lock();
	for (flavor = 0; flavor < RPC_AUTH_MAXFLAVOR; flavor++) {
		ops = rcu_dereference(auth_flavors[flavor]);
		if (result >= size) {
			result = -ENOMEM;
			break;
		}

		if (ops == NULL)
			continue;
		if (ops->list_pseudoflavors == NULL) {
			array[result++] = ops->au_flavor;
			continue;
		}
		len = ops->list_pseudoflavors(pseudos, ARRAY_SIZE(pseudos));
		if (len < 0) {
			result = len;
			break;
		}
		for (i = 0; i < len; i++) {
			if (result >= size) {
				result = -ENOMEM;
				break;
			}
			array[result++] = pseudos[i];
		}
	}
	rcu_read_unlock();

	dprintk("RPC:       %s returns %d\n", __func__, result);
	return result;
}
EXPORT_SYMBOL_GPL(rpcauth_list_flavors);

struct rpc_auth *
rpcauth_create(const struct rpc_auth_create_args *args, struct rpc_clnt *clnt)
{
	struct rpc_auth	*auth = ERR_PTR(-EINVAL);
	const struct rpc_authops *ops;
	u32 flavor = pseudoflavor_to_flavor(args->pseudoflavor);

	ops = rpcauth_get_authops(flavor);
	if (ops == NULL)
		goto out;

	auth = ops->create(args, clnt);

	rpcauth_put_authops(ops);
	if (IS_ERR(auth))
		return auth;
	if (clnt->cl_auth)
		rpcauth_release(clnt->cl_auth);
	clnt->cl_auth = auth;

out:
	return auth;
}
EXPORT_SYMBOL_GPL(rpcauth_create);

void
rpcauth_release(struct rpc_auth *auth)
{
	if (!refcount_dec_and_test(&auth->au_count))
		return;
	auth->au_ops->destroy(auth);
}

static DEFINE_SPINLOCK(rpc_credcache_lock);

/*
 * On success, the caller is responsible for freeing the reference
 * held by the hashtable
 */
static bool
rpcauth_unhash_cred_locked(struct rpc_cred *cred)
{
	if (!test_and_clear_bit(RPCAUTH_CRED_HASHED, &cred->cr_flags))
		return false;
	hlist_del_rcu(&cred->cr_hash);
	return true;
}

static bool
rpcauth_unhash_cred(struct rpc_cred *cred)
{
	spinlock_t *cache_lock;
	bool ret;

	if (!test_bit(RPCAUTH_CRED_HASHED, &cred->cr_flags))
		return false;
	cache_lock = &cred->cr_auth->au_credcache->lock;
	spin_lock(cache_lock);
	ret = rpcauth_unhash_cred_locked(cred);
	spin_unlock(cache_lock);
	return ret;
}

/*
 * Initialize RPC credential cache
 */
int
rpcauth_init_credcache(struct rpc_auth *auth)
{
	struct rpc_cred_cache *new;
	unsigned int hashsize;

	new = kmalloc(sizeof(*new), GFP_KERNEL);
	if (!new)
		goto out_nocache;
	new->hashbits = auth_hashbits;
	hashsize = 1U << new->hashbits;
	new->hashtable = kcalloc(hashsize, sizeof(new->hashtable[0]), GFP_KERNEL);
	if (!new->hashtable)
		goto out_nohashtbl;
	spin_lock_init(&new->lock);
	auth->au_credcache = new;
	return 0;
out_nohashtbl:
	kfree(new);
out_nocache:
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(rpcauth_init_credcache);

/*
 * Setup a credential key lifetime timeout notification
 */
int
rpcauth_key_timeout_notify(struct rpc_auth *auth, struct rpc_cred *cred)
{
	if (!cred->cr_auth->au_ops->key_timeout)
		return 0;
	return cred->cr_auth->au_ops->key_timeout(auth, cred);
}
EXPORT_SYMBOL_GPL(rpcauth_key_timeout_notify);

bool
rpcauth_cred_key_to_expire(struct rpc_auth *auth, struct rpc_cred *cred)
{
	if (auth->au_flags & RPCAUTH_AUTH_NO_CRKEY_TIMEOUT)
		return false;
	if (!cred->cr_ops->crkey_to_expire)
		return false;
	return cred->cr_ops->crkey_to_expire(cred);
}
EXPORT_SYMBOL_GPL(rpcauth_cred_key_to_expire);

char *
rpcauth_stringify_acceptor(struct rpc_cred *cred)
{
	if (!cred->cr_ops->crstringify_acceptor)
		return NULL;
	return cred->cr_ops->crstringify_acceptor(cred);
}
EXPORT_SYMBOL_GPL(rpcauth_stringify_acceptor);

/*
 * Destroy a list of credentials
 */
static inline
void rpcauth_destroy_credlist(struct list_head *head)
{
	struct rpc_cred *cred;

	while (!list_empty(head)) {
		cred = list_entry(head->next, struct rpc_cred, cr_lru);
		list_del_init(&cred->cr_lru);
		put_rpccred(cred);
	}
}

static void
rpcauth_lru_add_locked(struct rpc_cred *cred)
{
	if (!list_empty(&cred->cr_lru))
		return;
	number_cred_unused++;
	list_add_tail(&cred->cr_lru, &cred_unused);
}

static void
rpcauth_lru_add(struct rpc_cred *cred)
{
	if (!list_empty(&cred->cr_lru))
		return;
	spin_lock(&rpc_credcache_lock);
	rpcauth_lru_add_locked(cred);
	spin_unlock(&rpc_credcache_lock);
}

static void
rpcauth_lru_remove_locked(struct rpc_cred *cred)
{
	if (list_empty(&cred->cr_lru))
		return;
	number_cred_unused--;
	list_del_init(&cred->cr_lru);
}

static void
rpcauth_lru_remove(struct rpc_cred *cred)
{
	if (list_empty(&cred->cr_lru))
		return;
	spin_lock(&rpc_credcache_lock);
	rpcauth_lru_remove_locked(cred);
	spin_unlock(&rpc_credcache_lock);
}

/*
 * Clear the RPC credential cache, and delete those credentials
 * that are not referenced.
 */
void
rpcauth_clear_credcache(struct rpc_cred_cache *cache)
{
	LIST_HEAD(free);
	struct hlist_head *head;
	struct rpc_cred	*cred;
	unsigned int hashsize = 1U << cache->hashbits;
	int		i;

	spin_lock(&rpc_credcache_lock);
	spin_lock(&cache->lock);
	for (i = 0; i < hashsize; i++) {
		head = &cache->hashtable[i];
		while (!hlist_empty(head)) {
			cred = hlist_entry(head->first, struct rpc_cred, cr_hash);
			rpcauth_unhash_cred_locked(cred);
			/* Note: We now hold a reference to cred */
			rpcauth_lru_remove_locked(cred);
			list_add_tail(&cred->cr_lru, &free);
		}
	}
	spin_unlock(&cache->lock);
	spin_unlock(&rpc_credcache_lock);
	rpcauth_destroy_credlist(&free);
}

/*
 * Destroy the RPC credential cache
 */
void
rpcauth_destroy_credcache(struct rpc_auth *auth)
{
	struct rpc_cred_cache *cache = auth->au_credcache;

	if (cache) {
		auth->au_credcache = NULL;
		rpcauth_clear_credcache(cache);
		kfree(cache->hashtable);
		kfree(cache);
	}
}
EXPORT_SYMBOL_GPL(rpcauth_destroy_credcache);


#define RPC_AUTH_EXPIRY_MORATORIUM (60 * HZ)

/*
 * Remove stale credentials. Avoid sleeping inside the loop.
 */
static long
rpcauth_prune_expired(struct list_head *free, int nr_to_scan)
{
	struct rpc_cred *cred, *next;
	unsigned long expired = jiffies - RPC_AUTH_EXPIRY_MORATORIUM;
	long freed = 0;

	list_for_each_entry_safe(cred, next, &cred_unused, cr_lru) {

		if (nr_to_scan-- == 0)
			break;
		if (refcount_read(&cred->cr_count) > 1) {
			rpcauth_lru_remove_locked(cred);
			continue;
		}
		/*
		 * Enforce a 60 second garbage collection moratorium
		 * Note that the cred_unused list must be time-ordered.
		 */
		if (!time_in_range(cred->cr_expire, expired, jiffies))
			continue;
		if (!rpcauth_unhash_cred(cred))
			continue;

		rpcauth_lru_remove_locked(cred);
		freed++;
		list_add_tail(&cred->cr_lru, free);
	}
	return freed ? freed : SHRINK_STOP;
}

static unsigned long
rpcauth_cache_do_shrink(int nr_to_scan)
{
	LIST_HEAD(free);
	unsigned long freed;

	spin_lock(&rpc_credcache_lock);
	freed = rpcauth_prune_expired(&free, nr_to_scan);
	spin_unlock(&rpc_credcache_lock);
	rpcauth_destroy_credlist(&free);

	return freed;
}

/*
 * Run memory cache shrinker.
 */
static unsigned long
rpcauth_cache_shrink_scan(struct shrinker *shrink, struct shrink_control *sc)

{
	if ((sc->gfp_mask & GFP_KERNEL) != GFP_KERNEL)
		return SHRINK_STOP;

	/* nothing left, don't come back */
	if (list_empty(&cred_unused))
		return SHRINK_STOP;

	return rpcauth_cache_do_shrink(sc->nr_to_scan);
}

static unsigned long
rpcauth_cache_shrink_count(struct shrinker *shrink, struct shrink_control *sc)

{
	return number_cred_unused * sysctl_vfs_cache_pressure / 100;
}

static void
rpcauth_cache_enforce_limit(void)
{
	unsigned long diff;
	unsigned int nr_to_scan;

	if (number_cred_unused <= auth_max_cred_cachesize)
		return;
	diff = number_cred_unused - auth_max_cred_cachesize;
	nr_to_scan = 100;
	if (diff < nr_to_scan)
		nr_to_scan = diff;
	rpcauth_cache_do_shrink(nr_to_scan);
}

/*
 * Look up a process' credentials in the authentication cache
 */
struct rpc_cred *
rpcauth_lookup_credcache(struct rpc_auth *auth, struct auth_cred * acred,
		int flags, gfp_t gfp)
{
	LIST_HEAD(free);
	struct rpc_cred_cache *cache = auth->au_credcache;
	struct rpc_cred	*cred = NULL,
			*entry, *new;
	unsigned int nr;

	nr = auth->au_ops->hash_cred(acred, cache->hashbits);

	rcu_read_lock();
	hlist_for_each_entry_rcu(entry, &cache->hashtable[nr], cr_hash) {
		if (!entry->cr_ops->crmatch(acred, entry, flags))
			continue;
		if (flags & RPCAUTH_LOOKUP_RCU) {
			if (test_bit(RPCAUTH_CRED_NEW, &entry->cr_flags) ||
			    refcount_read(&entry->cr_count) == 0)
				continue;
			cred = entry;
			break;
		}
		cred = get_rpccred(entry);
		if (cred)
			break;
	}
	rcu_read_unlock();

	if (cred != NULL)
		goto found;

	if (flags & RPCAUTH_LOOKUP_RCU)
		return ERR_PTR(-ECHILD);

	new = auth->au_ops->crcreate(auth, acred, flags, gfp);
	if (IS_ERR(new)) {
		cred = new;
		goto out;
	}

	spin_lock(&cache->lock);
	hlist_for_each_entry(entry, &cache->hashtable[nr], cr_hash) {
		if (!entry->cr_ops->crmatch(acred, entry, flags))
			continue;
		cred = get_rpccred(entry);
		if (cred)
			break;
	}
	if (cred == NULL) {
		cred = new;
		set_bit(RPCAUTH_CRED_HASHED, &cred->cr_flags);
		refcount_inc(&cred->cr_count);
		hlist_add_head_rcu(&cred->cr_hash, &cache->hashtable[nr]);
	} else
		list_add_tail(&new->cr_lru, &free);
	spin_unlock(&cache->lock);
	rpcauth_cache_enforce_limit();
found:
	if (test_bit(RPCAUTH_CRED_NEW, &cred->cr_flags) &&
	    cred->cr_ops->cr_init != NULL &&
	    !(flags & RPCAUTH_LOOKUP_NEW)) {
		int res = cred->cr_ops->cr_init(auth, cred);
		if (res < 0) {
			put_rpccred(cred);
			cred = ERR_PTR(res);
		}
	}
	rpcauth_destroy_credlist(&free);
out:
	return cred;
}
EXPORT_SYMBOL_GPL(rpcauth_lookup_credcache);

struct rpc_cred *
rpcauth_lookupcred(struct rpc_auth *auth, int flags)
{
	struct auth_cred acred;
	struct rpc_cred *ret;
	const struct cred *cred = current_cred();

	dprintk("RPC:       looking up %s cred\n",
		auth->au_ops->au_name);

	memset(&acred, 0, sizeof(acred));
	acred.uid = cred->fsuid;
	acred.gid = cred->fsgid;
	acred.group_info = cred->group_info;
	ret = auth->au_ops->lookup_cred(auth, &acred, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(rpcauth_lookupcred);

void
rpcauth_init_cred(struct rpc_cred *cred, const struct auth_cred *acred,
		  struct rpc_auth *auth, const struct rpc_credops *ops)
{
	INIT_HLIST_NODE(&cred->cr_hash);
	INIT_LIST_HEAD(&cred->cr_lru);
	refcount_set(&cred->cr_count, 1);
	cred->cr_auth = auth;
	cred->cr_ops = ops;
	cred->cr_expire = jiffies;
	cred->cr_uid = acred->uid;
}
EXPORT_SYMBOL_GPL(rpcauth_init_cred);

struct rpc_cred *
rpcauth_generic_bind_cred(struct rpc_task *task, struct rpc_cred *cred, int lookupflags)
{
	dprintk("RPC: %5u holding %s cred %p\n", task->tk_pid,
			cred->cr_auth->au_ops->au_name, cred);
	return get_rpccred(cred);
}
EXPORT_SYMBOL_GPL(rpcauth_generic_bind_cred);

static struct rpc_cred *
rpcauth_bind_root_cred(struct rpc_task *task, int lookupflags)
{
	struct rpc_auth *auth = task->tk_client->cl_auth;
	struct auth_cred acred = {
		.uid = GLOBAL_ROOT_UID,
		.gid = GLOBAL_ROOT_GID,
	};

	dprintk("RPC: %5u looking up %s cred\n",
		task->tk_pid, task->tk_client->cl_auth->au_ops->au_name);
	return auth->au_ops->lookup_cred(auth, &acred, lookupflags);
}

static struct rpc_cred *
rpcauth_bind_new_cred(struct rpc_task *task, int lookupflags)
{
	struct rpc_auth *auth = task->tk_client->cl_auth;

	dprintk("RPC: %5u looking up %s cred\n",
		task->tk_pid, auth->au_ops->au_name);
	return rpcauth_lookupcred(auth, lookupflags);
}

static int
rpcauth_bindcred(struct rpc_task *task, struct rpc_cred *cred, int flags)
{
	struct rpc_rqst *req = task->tk_rqstp;
	struct rpc_cred *new;
	int lookupflags = 0;

	if (flags & RPC_TASK_ASYNC)
		lookupflags |= RPCAUTH_LOOKUP_NEW;
	if (cred != NULL)
		new = cred->cr_ops->crbind(task, cred, lookupflags);
	else if (flags & RPC_TASK_ROOTCREDS)
		new = rpcauth_bind_root_cred(task, lookupflags);
	else
		new = rpcauth_bind_new_cred(task, lookupflags);
	if (IS_ERR(new))
		return PTR_ERR(new);
	put_rpccred(req->rq_cred);
	req->rq_cred = new;
	return 0;
}

void
put_rpccred(struct rpc_cred *cred)
{
	if (cred == NULL)
		return;
	rcu_read_lock();
	if (refcount_dec_and_test(&cred->cr_count))
		goto destroy;
	if (refcount_read(&cred->cr_count) != 1 ||
	    !test_bit(RPCAUTH_CRED_HASHED, &cred->cr_flags))
		goto out;
	if (test_bit(RPCAUTH_CRED_UPTODATE, &cred->cr_flags) != 0) {
		cred->cr_expire = jiffies;
		rpcauth_lru_add(cred);
		/* Race breaker */
		if (unlikely(!test_bit(RPCAUTH_CRED_HASHED, &cred->cr_flags)))
			rpcauth_lru_remove(cred);
	} else if (rpcauth_unhash_cred(cred)) {
		rpcauth_lru_remove(cred);
		if (refcount_dec_and_test(&cred->cr_count))
			goto destroy;
	}
out:
	rcu_read_unlock();
	return;
destroy:
	rcu_read_unlock();
	cred->cr_ops->crdestroy(cred);
}
EXPORT_SYMBOL_GPL(put_rpccred);

__be32 *
rpcauth_marshcred(struct rpc_task *task, __be32 *p)
{
	struct rpc_cred	*cred = task->tk_rqstp->rq_cred;

	dprintk("RPC: %5u marshaling %s cred %p\n",
		task->tk_pid, cred->cr_auth->au_ops->au_name, cred);

	return cred->cr_ops->crmarshal(task, p);
}

__be32 *
rpcauth_checkverf(struct rpc_task *task, __be32 *p)
{
	struct rpc_cred	*cred = task->tk_rqstp->rq_cred;

	dprintk("RPC: %5u validating %s cred %p\n",
		task->tk_pid, cred->cr_auth->au_ops->au_name, cred);

	return cred->cr_ops->crvalidate(task, p);
}

static void rpcauth_wrap_req_encode(kxdreproc_t encode, struct rpc_rqst *rqstp,
				   __be32 *data, void *obj)
{
	struct xdr_stream xdr;

	xdr_init_encode(&xdr, &rqstp->rq_snd_buf, data);
	encode(rqstp, &xdr, obj);
}

int
rpcauth_wrap_req(struct rpc_task *task, kxdreproc_t encode, void *rqstp,
		__be32 *data, void *obj)
{
	struct rpc_cred *cred = task->tk_rqstp->rq_cred;

	dprintk("RPC: %5u using %s cred %p to wrap rpc data\n",
			task->tk_pid, cred->cr_ops->cr_name, cred);
	if (cred->cr_ops->crwrap_req)
		return cred->cr_ops->crwrap_req(task, encode, rqstp, data, obj);
	/* By default, we encode the arguments normally. */
	rpcauth_wrap_req_encode(encode, rqstp, data, obj);
	return 0;
}

static int
rpcauth_unwrap_req_decode(kxdrdproc_t decode, struct rpc_rqst *rqstp,
			  __be32 *data, void *obj)
{
	struct xdr_stream xdr;

	xdr_init_decode(&xdr, &rqstp->rq_rcv_buf, data);
	return decode(rqstp, &xdr, obj);
}

int
rpcauth_unwrap_resp(struct rpc_task *task, kxdrdproc_t decode, void *rqstp,
		__be32 *data, void *obj)
{
	struct rpc_cred *cred = task->tk_rqstp->rq_cred;

	dprintk("RPC: %5u using %s cred %p to unwrap rpc data\n",
			task->tk_pid, cred->cr_ops->cr_name, cred);
	if (cred->cr_ops->crunwrap_resp)
		return cred->cr_ops->crunwrap_resp(task, decode, rqstp,
						   data, obj);
	/* By default, we decode the arguments normally. */
	return rpcauth_unwrap_req_decode(decode, rqstp, data, obj);
}

bool
rpcauth_xmit_need_reencode(struct rpc_task *task)
{
	struct rpc_cred *cred = task->tk_rqstp->rq_cred;

	if (!cred || !cred->cr_ops->crneed_reencode)
		return false;
	return cred->cr_ops->crneed_reencode(task);
}

int
rpcauth_refreshcred(struct rpc_task *task)
{
	struct rpc_cred	*cred;
	int err;

	cred = task->tk_rqstp->rq_cred;
	if (cred == NULL) {
		err = rpcauth_bindcred(task, task->tk_msg.rpc_cred, task->tk_flags);
		if (err < 0)
			goto out;
		cred = task->tk_rqstp->rq_cred;
	}
	dprintk("RPC: %5u refreshing %s cred %p\n",
		task->tk_pid, cred->cr_auth->au_ops->au_name, cred);

	err = cred->cr_ops->crrefresh(task);
out:
	if (err < 0)
		task->tk_status = err;
	return err;
}

void
rpcauth_invalcred(struct rpc_task *task)
{
	struct rpc_cred *cred = task->tk_rqstp->rq_cred;

	dprintk("RPC: %5u invalidating %s cred %p\n",
		task->tk_pid, cred->cr_auth->au_ops->au_name, cred);
	if (cred)
		clear_bit(RPCAUTH_CRED_UPTODATE, &cred->cr_flags);
}

int
rpcauth_uptodatecred(struct rpc_task *task)
{
	struct rpc_cred *cred = task->tk_rqstp->rq_cred;

	return cred == NULL ||
		test_bit(RPCAUTH_CRED_UPTODATE, &cred->cr_flags) != 0;
}

static struct shrinker rpc_cred_shrinker = {
	.count_objects = rpcauth_cache_shrink_count,
	.scan_objects = rpcauth_cache_shrink_scan,
	.seeks = DEFAULT_SEEKS,
};

int __init rpcauth_init_module(void)
{
	int err;

	err = rpc_init_authunix();
	if (err < 0)
		goto out1;
	err = rpc_init_generic_auth();
	if (err < 0)
		goto out2;
	err = register_shrinker(&rpc_cred_shrinker);
	if (err < 0)
		goto out3;
	return 0;
out3:
	rpc_destroy_generic_auth();
out2:
	rpc_destroy_authunix();
out1:
	return err;
}

void rpcauth_remove_module(void)
{
	rpc_destroy_authunix();
	rpc_destroy_generic_auth();
	unregister_shrinker(&rpc_cred_shrinker);
}
