/*
 * CAAM/SEC 4.x QI transport/backend driver
 * Queue Interface backend functionality
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 */

#include <linux/cpumask.h>
#include <linux/kthread.h>
#include <linux/fsl_qman.h>

#include "regs.h"
#include "qi.h"
#include "desc.h"
#include "intern.h"
#include "desc_constr.h"

#define CAAM_REQ_CGR_THRESHOLD	0x1000000
#define PRE_HDR_LEN		2	/* Length in u32 words */
#define PREHDR_RSLS_SHIFT	31

/*
 * The jobs are processed by the driver against a driver context.
 * With every cryptographic context, a driver context is attached.
 * The driver context contains data for private use by driver.
 * For the applications, this is an opaque structure.
 */

struct caam_drv_ctx {
	u32 prehdr[PRE_HDR_LEN];	/* Preheader placed before shrd desc */
	u32 sh_desc[MAX_SDLEN];		/* Shared descriptor */
	dma_addr_t context_a; /* shared descriptor dma address */
	struct qman_fq *req_fq;		/* Request frame queue to caam */
	struct qman_fq *rsp_fq;		/* Response frame queue from caam */
	int cpu;			/* cpu on which to recv caam rsp */
	struct device *qidev;		/* device pointer for QI backend */
} ____cacheline_aligned;

/*
 * percpu private data structure to main list of pending responses expected
 * on each cpu.
 */
struct caam_qi_pcpu_priv {
	struct napi_struct irqtask;	/* IRQ task for QI backend */
	struct net_device net_dev;	/* netdev used by NAPI */
	struct qman_fq rsp_fq;		/* Response FQ from CAAM */
	spinlock_t listlock ____cacheline_aligned; /* for protecting
						    * simultaneous access
						    * to bklog_list */
	struct list_head bklog_list;	/* List of pending responses*/
	atomic_t pending;		/* Number of pending responses
					 * from CAAM on this cpu */
} ____cacheline_aligned;

static DEFINE_PER_CPU(struct caam_qi_pcpu_priv, pcpu_qipriv);

struct caam_qi_priv {
	bool sec_congested;		/* Indicates whether SEC is congested */
	bool cpu_congested;		/* Indicates whether CPU is congested */
	struct qman_cgr rsp_cgr;	/* QMAN response CGR */
	struct qman_cgr req_cgr;	/* QMAN request CGR */
	struct platform_device *qi_pdev; /* Platform device for QI backend */
};

static struct caam_qi_priv qipriv ____cacheline_aligned;

/*
 * CPU from where the module initialised. This is required because
 * QMAN driver requires CGRs to be removed from same CPU from where
 * they were originally allocated
 */
static int mod_init_cpu;

bool caam_drv_ctx_busy(struct caam_drv_ctx *drv_ctx)
{
	int pending;

	if (qipriv.sec_congested)
		return true;

	pending = atomic_read(&per_cpu(pcpu_qipriv.pending, drv_ctx->cpu));
	if (pending >= JOBR_DEPTH)
		return true;

	return false;
}
EXPORT_SYMBOL(caam_drv_ctx_busy);

int caam_qi_enqueue(struct device *qidev, struct caam_drv_req *req)
{
	struct qm_fd fd;
	int ret;
	size_t size;
	struct list_head *list;
	int num_retries = 0;

	fd.cmd = 0;
	fd.format = qm_fd_compound;
	fd.cong_weight = req->fd_sgt[1].length;

	size = 2 * sizeof(struct qm_sg_entry);

	fd.addr = dma_map_single(qidev, req->fd_sgt, size , DMA_BIDIRECTIONAL);
	if (dma_mapping_error(qidev, fd.addr)) {
		dev_err(qidev, "DMA mapping error for QI enqueue request\n");
		return -EIO;
	}

	req->hwaddr = qm_fd_addr(&fd);
	list = &per_cpu(pcpu_qipriv.bklog_list, req->drv_ctx->cpu);

	spin_lock(&per_cpu(pcpu_qipriv.listlock, req->drv_ctx->cpu));
	list_add_tail(&req->hdr__, list);
	spin_unlock(&per_cpu(pcpu_qipriv.listlock, req->drv_ctx->cpu));
	atomic_inc(&per_cpu(pcpu_qipriv.pending, req->drv_ctx->cpu));

	do {
		ret = qman_enqueue(req->drv_ctx->req_fq, &fd, 0);
		if (likely(!ret))
			return 0;

		if (-EBUSY != ret)
			break;
		num_retries++;
	} while (num_retries < 10000);

	dev_err(qidev, "qman_enqueue failed: %d\n", ret);

	spin_lock(&per_cpu(pcpu_qipriv.listlock, req->drv_ctx->cpu));
	list_del(&req->hdr__);
	spin_unlock(&per_cpu(pcpu_qipriv.listlock, req->drv_ctx->cpu));
	atomic_dec(&per_cpu(pcpu_qipriv.pending, req->drv_ctx->cpu));

	dma_unmap_single(qidev, fd.addr, size, DMA_BIDIRECTIONAL);
	return ret;
}
EXPORT_SYMBOL(caam_qi_enqueue);

struct caam_drv_req *lookup_drv_req(const struct qm_fd *fd, int cpu)
{
	struct list_head *pos, *list, *n;
	struct caam_drv_req *req;

	list = &per_cpu(pcpu_qipriv.bklog_list, cpu);
	list_for_each_safe(pos, n, list) {
		req = container_of(pos, struct caam_drv_req, hdr__);

		if (req->hwaddr == qm_fd_addr(fd)) {
			BUG_ON(req->drv_ctx->cpu != cpu);

			spin_lock(&per_cpu(pcpu_qipriv.listlock,
					   req->drv_ctx->cpu));
			list_del(&req->hdr__);
			spin_unlock(&per_cpu(pcpu_qipriv.listlock,
					     req->drv_ctx->cpu));
			atomic_dec(&per_cpu(pcpu_qipriv.pending,
					    req->drv_ctx->cpu));
			return req;
		}
	}

	return NULL;
}


static struct caam_drv_req *fd_to_drv_req(const struct qm_fd *fd)
{
	struct caam_drv_req *req;
	const cpumask_t *cpus = qman_affine_cpus();
	int i;

	/*
	 * First check on this_cpu since this is likely case of normal caam
	 * response path.
	 */
	req = lookup_drv_req(fd, smp_processor_id());
	if (likely(req))
		return req;
	/*
	 * If drv_req is not found on this_cpu, then try searching on other
	 * portal owning cpus. This is required to handle ERN callbacks and
	 * volatile dequeues. These may be issued on a CPU which is different
	 * than the one associated with the drv_req's drv_ctx.
	 */
	for_each_cpu(i, cpus) {
		if (i == smp_processor_id())
			continue;	/* Already checked */
		req = lookup_drv_req(fd, i);

		if (req)
			return req;
	}

	return NULL;
}

static void caam_fq_ern_cb(struct qman_portal *qm, struct qman_fq *fq,
			   const struct qm_mr_entry *msg)
{
	const struct qm_fd *fd;
	struct caam_drv_req *drv_req;
	size_t size;
	struct device *qidev = &per_cpu(pcpu_qipriv.net_dev,
					smp_processor_id()).dev;

	fd = &msg->ern.fd;

	if (qm_fd_compound != fd->format) {
		dev_err(qidev, "Non compound FD from CAAM\n");
		return;
	}

	drv_req = fd_to_drv_req(fd);
	if (!drv_req) {
		dev_err(qidev,
			"Can't find original request for caam response\n");
		return;
	}

	size = 2 * sizeof(struct qm_sg_entry);
	dma_unmap_single(drv_req->drv_ctx->qidev, fd->addr,
			 size, DMA_BIDIRECTIONAL);

	drv_req->cbk(drv_req, -EIO);
}

static enum qman_cb_dqrr_result	caam_req_fq_dqrr_cb(struct qman_portal *p,
					struct qman_fq *req_fq,
					const struct qm_dqrr_entry *dqrr)
{
	struct caam_drv_req *drv_req;
	const struct qm_fd *fd;
	size_t size;
	struct device *qidev = &per_cpu(pcpu_qipriv.net_dev,
					smp_processor_id()).dev;

	fd = &dqrr->fd;

	drv_req = fd_to_drv_req(fd);
	if (!drv_req) {
		dev_err(qidev,
			"Can't find original request for caam response\n");
		return qman_cb_dqrr_consume;
	}

	size = 2 * sizeof(struct qm_sg_entry);
	dma_unmap_single(drv_req->drv_ctx->qidev, fd->addr,
			 size, DMA_BIDIRECTIONAL);

	drv_req->cbk(drv_req, -EIO);

	return qman_cb_dqrr_consume;
}

static struct qman_fq *create_caam_req_fq(struct device *qidev,
					  struct qman_fq *rsp_fq,
					  dma_addr_t hwdesc,
					  int fq_sched_flag)
{
	int ret, flags;
	struct qman_fq *req_fq;
	struct qm_mcc_initfq opts;

	req_fq = kzalloc(sizeof(*req_fq), GFP_ATOMIC);
	if (!req_fq) {
		dev_err(qidev, "Mem alloc for CAAM req FQ failed\n");
		return ERR_PTR(-ENOMEM);
	}

	req_fq->cb.dqrr = caam_req_fq_dqrr_cb;
	req_fq->cb.ern = caam_fq_ern_cb;
	req_fq->cb.fqs = NULL;

	flags = QMAN_FQ_FLAG_DYNAMIC_FQID |
		QMAN_FQ_FLAG_TO_DCPORTAL |
		QMAN_FQ_FLAG_LOCKED;

	ret = qman_create_fq(0, flags, req_fq);
	if (ret) {
		dev_err(qidev, "Failed to create session REQ FQ\n");
		goto create_req_fq_fail;
	}

	flags = fq_sched_flag;
	opts.we_mask = QM_INITFQ_WE_FQCTRL | QM_INITFQ_WE_DESTWQ |
			QM_INITFQ_WE_CONTEXTB | QM_INITFQ_WE_CONTEXTA |
			QM_INITFQ_WE_CGID;

	opts.fqd.fq_ctrl = QM_FQCTRL_CPCSTASH | QM_FQCTRL_CGE;
	opts.fqd.dest.channel = qm_channel_caam;
	opts.fqd.dest.wq = 3;
	opts.fqd.cgid = qipriv.req_cgr.cgrid;
	opts.fqd.context_b = qman_fq_fqid(rsp_fq);
	opts.fqd.context_a.hi = upper_32_bits(hwdesc);
	opts.fqd.context_a.lo = lower_32_bits(hwdesc);

	ret = qman_init_fq(req_fq, flags, &opts);
	if (ret) {
		dev_err(qidev, "Failed to init session req FQ\n");
		goto init_req_fq_fail;
	}

	return req_fq;

init_req_fq_fail:
	qman_destroy_fq(req_fq, 0);

create_req_fq_fail:
	kfree(req_fq);
	return ERR_PTR(ret);
}

static int empty_retired_fq(struct device *qidev, struct qman_fq *fq)
{
	int ret;
	enum qman_fq_state state;

	u32 flags = QMAN_VOLATILE_FLAG_WAIT_INT | QMAN_VOLATILE_FLAG_FINISH;
	u32 vdqcr = QM_VDQCR_PRECEDENCE_VDQCR | QM_VDQCR_NUMFRAMES_TILLEMPTY;

	ret = qman_volatile_dequeue(fq, flags, vdqcr);
	if (ret) {
		dev_err(qidev, "Volatile dequeue fail for FQ: %u\n", fq->fqid);
		return ret;
	}

	do {
		qman_poll_dqrr(16);
		qman_fq_state(fq, &state, &flags);
	} while (flags & QMAN_FQ_STATE_NE);

	return 0;
}

static int kill_fq(struct device *qidev, struct qman_fq *fq)
{
	enum qman_fq_state state;
	u32 flags;
	int ret;

	ret = qman_retire_fq(fq, &flags);
	if (ret < 0) {
		dev_err(qidev, "qman_retire_fq failed\n");
		return ret;
	}

	if (!ret)
		goto empty_fq;

	/* Async FQ retirement condition */
	if (1 == ret) {
		/* Retry till FQ gets in retired state */
		do {
			msleep(20);
			qman_fq_state(fq, &state, &flags);
		} while (qman_fq_state_retired != state);

		WARN_ON(flags & QMAN_FQ_STATE_BLOCKOOS);
		WARN_ON(flags & QMAN_FQ_STATE_ORL);
	}

empty_fq:
	if (flags & QMAN_FQ_STATE_NE) {
		ret = empty_retired_fq(qidev, fq);
		if (ret) {
			dev_err(qidev, "empty_retired_fq fail for FQ: %u\n",
				fq->fqid);
			return ret;
		}
	}

	ret = qman_oos_fq(fq);
	if (ret)
		dev_err(qidev, "OOS of FQID: %u failed\n", fq->fqid);

	return ret;
}

/*
 * TODO: This CAAM FQ empty logic can be improved. We can enqueue a NULL
 * job descriptor to the FQ. This must be the last enqueue request to the
 * FQ. When the response of this job comes back, the FQ is empty. Also
 * holding tanks are guaranteed to be not holding any jobs from this FQ.
 */
static int empty_caam_fq(struct qman_fq *fq)
{
	int ret;
	struct qm_mcr_queryfq_np np;

	/* Wait till the older CAAM FQ get empty */
	do {
		ret = qman_query_fq_np(fq, &np);
		if (ret)
			return ret;

		if (!np.frm_cnt)
			break;

		msleep(20);
	} while (1);

	/*
	 * Give extra time for pending jobs from this FQ in holding tanks
	 * to get processed
	 */
	msleep(20);
	return 0;
}

int caam_drv_ctx_update(struct caam_drv_ctx *drv_ctx, u32 *sh_desc)
{
	size_t size;
	u32 num_words;
	int ret;
	struct qman_fq *new_fq, *old_fq;
	struct device *qidev = drv_ctx->qidev;

	/* Check the size of new shared descriptor */
	num_words = desc_len(sh_desc);
	if (num_words > MAX_SDLEN) {
		dev_err(qidev, "Invalid descriptor len: %d words\n",
			num_words);
		return -EINVAL;
	}

	/* Note down older req FQ */
	old_fq = drv_ctx->req_fq;

	/* Create a new req FQ in parked state */
	new_fq = create_caam_req_fq(drv_ctx->qidev, drv_ctx->rsp_fq,
				    drv_ctx->context_a, 0);
	if (!new_fq) {
		dev_err(qidev, "FQ allocation for shdesc update failed\n");
		return PTR_ERR(new_fq);
	}

	/* Hook up new FQ to context so that new requests keep queueing */
	drv_ctx->req_fq = new_fq;

	/* Empty and remove the older FQ */
	ret = empty_caam_fq(old_fq);
	if (ret) {
		dev_err(qidev, "Old SEC FQ empty failed\n");

		/* We can revert to older FQ */
		drv_ctx->req_fq = old_fq;

		if (kill_fq(qidev, new_fq)) {
			dev_warn(qidev, "New SEC FQ: %u kill failed\n",
				 new_fq->fqid);
		}

		return ret;
	}

	/*
	 * Now update the shared descriptor for driver context.
	 * Re-initialise pre-header. Set RSLS and SDLEN
	 */
	drv_ctx->prehdr[0] = (1 << PREHDR_RSLS_SHIFT) | num_words;

	/* Copy the new shared descriptor now */
	memcpy(drv_ctx->sh_desc, sh_desc, desc_bytes(sh_desc));

	size = sizeof(drv_ctx->sh_desc) + sizeof(drv_ctx->prehdr);
	dma_sync_single_for_device(qidev, drv_ctx->context_a,
				   size, DMA_BIDIRECTIONAL);

	/* Put the new FQ in scheduled state */
	ret = qman_schedule_fq(new_fq);
	if (ret) {
		dev_err(qidev, "Fail to sched new SEC FQ, ecode = %d\n", ret);

		/*
		 * We can kill new FQ and revert to old FQ.
		 * Since the desc is already modified, it is success case
		 */

		drv_ctx->req_fq = old_fq;

		if (kill_fq(qidev, new_fq)) {
			dev_warn(qidev, "New SEC FQ: %u kill failed\n",
				 new_fq->fqid);
		}
	} else {
		/* Remove older FQ */
		if (kill_fq(qidev, old_fq)) {
			dev_warn(qidev, "Old SEC FQ: %u kill failed\n",
				 old_fq->fqid);
		}
	}

	return 0;
}
EXPORT_SYMBOL(caam_drv_ctx_update);



struct caam_drv_ctx *caam_drv_ctx_init(struct device *qidev,
				       int *cpu,
				       u32 *sh_desc)
{
	size_t size;
	u32 num_words;
	dma_addr_t hwdesc;
	struct qman_fq *rsp_fq;
	struct caam_drv_ctx *drv_ctx;
	const cpumask_t *cpus = qman_affine_cpus();
	static DEFINE_PER_CPU(int, last_cpu);

	num_words = desc_len(sh_desc);
	if (num_words > MAX_SDLEN) {
		dev_err(qidev, "Invalid descriptor len: %d words\n",
			num_words);
		return ERR_PTR(-EINVAL);
	}

	drv_ctx = kzalloc(sizeof(*drv_ctx), GFP_ATOMIC);
	if (!drv_ctx) {
		dev_err(qidev, "Mem alloc for driver context failed\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Initialise pre-header. Set RSLS and SDLEN */
	drv_ctx->prehdr[0] = (1 << PREHDR_RSLS_SHIFT) | num_words;

	/* Copy the shared descriptor now */
	memcpy(drv_ctx->sh_desc, sh_desc, desc_bytes(sh_desc));

	/* Map address for pre-header + descriptor */
	size = sizeof(drv_ctx->prehdr) + sizeof(drv_ctx->sh_desc);
	hwdesc = dma_map_single(qidev, drv_ctx->prehdr,
				size, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(qidev, hwdesc)) {
		dev_err(qidev, "DMA map error for preheader+shdesc\n");
		kfree(drv_ctx);
		return ERR_PTR(-ENOMEM);
	}

	drv_ctx->context_a = hwdesc;

	/*
	 * If the given CPU does not own the portal, choose another
	 * one with a portal.
	 */
	if (!cpumask_test_cpu(*cpu, cpus)) {
		last_cpu = cpumask_next(last_cpu, cpus);
		if (last_cpu >= nr_cpu_ids)
			last_cpu = cpumask_first(cpus);
		 *cpu = last_cpu;
	}

	drv_ctx->cpu = *cpu;

	/* Find response FQ hooked with this CPU*/
	rsp_fq = &per_cpu(pcpu_qipriv.rsp_fq, drv_ctx->cpu);
	drv_ctx->rsp_fq = rsp_fq;

	/*Attach request FQ*/
	drv_ctx->req_fq = create_caam_req_fq(qidev, rsp_fq,
					     hwdesc, QMAN_INITFQ_FLAG_SCHED);
	if (!drv_ctx->req_fq) {
		dev_err(qidev, "create_caam_req_fq failed\n");
		dma_unmap_single(qidev, hwdesc, size, DMA_BIDIRECTIONAL);
		kfree(drv_ctx);
		return ERR_PTR(-ENOMEM);
	}

	drv_ctx->qidev = qidev;
	return drv_ctx;
}
EXPORT_SYMBOL(caam_drv_ctx_init);

static int caam_qi_poll(struct napi_struct *napi, int budget)
{
	int cleaned = qman_poll_dqrr(budget);

	if (cleaned < budget) {
		napi_complete(napi);
		qman_irqsource_add(QM_PIRQ_DQRI);
	}

	return cleaned;
}


void caam_drv_ctx_rel(struct caam_drv_ctx *drv_ctx)
{
	size_t size;

	if (!drv_ctx)
		return;

	size = sizeof(drv_ctx->sh_desc) + sizeof(drv_ctx->prehdr);

	/* Remove request FQ*/
	if (kill_fq(drv_ctx->qidev, drv_ctx->req_fq))
		dev_err(drv_ctx->qidev, "Crypto session Req FQ kill failed\n");

	dma_unmap_single(drv_ctx->qidev, drv_ctx->context_a,
			 size, DMA_BIDIRECTIONAL);

	kfree(drv_ctx);
}
EXPORT_SYMBOL(caam_drv_ctx_rel);

int caam_qi_shutdown(struct device *qidev)
{
	struct caam_qi_priv *priv = dev_get_drvdata(qidev);
	int i, ret;

	const cpumask_t *cpus = qman_affine_cpus();
	struct cpumask old_cpumask = *tsk_cpus_allowed(current);

	for_each_cpu(i, cpus) {
		napi_disable(&per_cpu(pcpu_qipriv.irqtask, i));
		netif_napi_del(&per_cpu(pcpu_qipriv.irqtask, i));
		if (kill_fq(qidev, &per_cpu(pcpu_qipriv.rsp_fq, i)))
			dev_err(qidev, "Rsp FQ kill failed, cpu: %d\n", i);
	}

	/*
	 * QMAN driver requires CGRs to be deleted from same CPU from where
	 * they were instantiated. Hence we get the module removal execute
	 * from the same CPU from where it was originally inserted.
	 */
	set_cpus_allowed_ptr(current, get_cpu_mask(mod_init_cpu));

	ret = qman_delete_cgr(&priv->req_cgr);
	if (ret)
		dev_err(qidev, "Delete request CGR failed: %d\n", ret);
	else
		qman_release_cgrid(priv->req_cgr.cgrid);

	ret = qman_delete_cgr(&priv->rsp_cgr);
	if (ret)
		dev_err(qidev, "Delete response CGR failed: %d\n", ret);
	else
		qman_release_cgrid(priv->rsp_cgr.cgrid);

	/* Now that we're done with the CGRs, restore the cpus allowed mask */
	set_cpus_allowed_ptr(current, &old_cpumask);

	platform_device_unregister(priv->qi_pdev);
	return ret;
}

static void rsp_cgr_cb(struct qman_portal *qm, struct qman_cgr *cgr,
			int congested)
{
	struct device *qidev = &per_cpu(pcpu_qipriv.net_dev,
					smp_processor_id()).dev;

	qipriv.cpu_congested = congested;

	if (congested)
		dev_warn(qidev, "CAAM rsp path congested\n");
	else
		dev_info(qidev, "CAAM rsp path congestion state exit\n");
}

static void req_cgr_cb(struct qman_portal *qm, struct qman_cgr *cgr,
			int congested)
{
	struct device *qidev = &per_cpu(pcpu_qipriv.net_dev,
					smp_processor_id()).dev;

	qipriv.sec_congested = congested;

	if (congested)
		dev_warn(qidev, "CAAM req path congested\n");
	else
		dev_info(qidev, "CAAM req path congestion state exit\n");
}

static int caam_qi_napi_schedule(struct napi_struct *napi)
{
	/*
	 * In case of threaded ISR for RT enable kernel,
	 * in_irq() does not return appropriate value, so use
	 * in_serving_softirq to distinguish softirq or irq context.
	 */
	if (unlikely(in_irq() || !in_serving_softirq())) {
		/* Disable QMan IRQ and invoke NAPI */
		int ret = qman_irqsource_remove(QM_PIRQ_DQRI);
		if (likely(!ret)) {
			napi_schedule(napi);
			return 1;
		}
	}
	return 0;
}

static enum qman_cb_dqrr_result caam_rsp_fq_dqrr_cb(struct qman_portal *p,
					struct qman_fq *rsp_fq,
					const struct qm_dqrr_entry *dqrr)
{
	struct napi_struct *napi;
	struct caam_drv_req *drv_req;
	const struct qm_fd *fd;
	size_t size;
	struct device *qidev = &per_cpu(pcpu_qipriv.net_dev,
					smp_processor_id()).dev;

	napi = &per_cpu(pcpu_qipriv.irqtask, smp_processor_id());
	if (caam_qi_napi_schedule(napi))
		return qman_cb_dqrr_stop;

	fd = &dqrr->fd;
	if (unlikely(fd->status))
		dev_err(qidev, "Error: %#x in CAAM response FD\n", fd->status);

	if (qm_fd_compound != fd->format) {
		dev_err(qidev, "Non compound FD from CAAM\n");
		return qman_cb_dqrr_consume;
	}

	drv_req = fd_to_drv_req(fd);
	if (!drv_req) {
		dev_err(qidev,
			"Can't find original request for caam response\n");
		return qman_cb_dqrr_consume;
	}

	size = 2 * sizeof(struct qm_sg_entry);
	dma_unmap_single(drv_req->drv_ctx->qidev, fd->addr,
			 size, DMA_BIDIRECTIONAL);

	drv_req->cbk(drv_req, fd->status);

	return qman_cb_dqrr_consume;
}

static int alloc_rsp_fq_cpu(struct device *qidev, unsigned int cpu)
{
	struct qm_mcc_initfq opts;
	struct qman_fq *fq;
	int ret;
	u32 flags;

	fq = &per_cpu(pcpu_qipriv.rsp_fq, cpu);

	fq->cb.dqrr = caam_rsp_fq_dqrr_cb;

	flags = QMAN_FQ_FLAG_NO_ENQUEUE |
		QMAN_FQ_FLAG_DYNAMIC_FQID;

	ret = qman_create_fq(0, flags, fq);
	if (ret) {
		dev_err(qidev, "Rsp FQ create failed\n");
		return -ENODEV;
	}

	flags = QMAN_INITFQ_FLAG_SCHED;

	opts.we_mask = QM_INITFQ_WE_FQCTRL | QM_INITFQ_WE_DESTWQ |
		QM_INITFQ_WE_CONTEXTB | QM_INITFQ_WE_CONTEXTA |
		QM_INITFQ_WE_CGID;

	opts.fqd.fq_ctrl = QM_FQCTRL_CTXASTASHING |
			   QM_FQCTRL_CPCSTASH |
			   QM_FQCTRL_CGE;

	opts.fqd.dest.channel = qman_affine_channel(cpu);
	opts.fqd.cgid = qipriv.rsp_cgr.cgrid;
	opts.fqd.dest.wq = 1;
	opts.fqd.context_a.stashing.exclusive =
					QM_STASHING_EXCL_CTX |
					QM_STASHING_EXCL_DATA;

	opts.fqd.context_a.stashing.data_cl = 1;
	opts.fqd.context_a.stashing.context_cl = 1;

	ret = qman_init_fq(fq, flags, &opts);
	if (ret) {
		dev_err(qidev, "Rsp FQ init failed\n");
		return -ENODEV;
	}

	return 0;
}

static int alloc_cgrs(struct device *qidev)
{
	struct qm_mcc_initcgr opts;
	int ret;

	/*Allocate response CGR*/
	ret = qman_alloc_cgrid(&qipriv.rsp_cgr.cgrid);
	if (ret) {
		dev_err(qidev, "CGR alloc failed for rsp FQs");
		return ret;
	}

	qipriv.rsp_cgr.cb = rsp_cgr_cb;
	memset(&opts, 0, sizeof(opts));
	opts.we_mask = QM_CGR_WE_CSCN_EN | QM_CGR_WE_CS_THRES |
			QM_CGR_WE_MODE;
	opts.cgr.cscn_en = QM_CGR_EN;
	opts.cgr.mode = QMAN_CGR_MODE_FRAME;
	qm_cgr_cs_thres_set64(&opts.cgr.cs_thres, 0x400 , 1);

	ret = qman_create_cgr(&qipriv.rsp_cgr,
				QMAN_CGR_FLAG_USE_INIT, &opts);
	if (ret) {
		dev_err(qidev, "Error %d creating CAAM rsp CGRID: %u\n",
			ret, qipriv.rsp_cgr.cgrid);
		goto create_rsp_cgr_fail;
	}

	/*Allocate request CGR*/
	ret = qman_alloc_cgrid(&qipriv.req_cgr.cgrid);
	if (ret) {
		dev_err(qidev, "CGR alloc failed for req FQs");
		goto alloc_req_cgrid_fail;
	}

	qipriv.req_cgr.cb = req_cgr_cb;
	memset(&opts, 0, sizeof(opts));
	opts.we_mask = QM_CGR_WE_CSCN_EN | QM_CGR_WE_CS_THRES;
	opts.cgr.cscn_en = QM_CGR_EN;
	qm_cgr_cs_thres_set64(&opts.cgr.cs_thres, CAAM_REQ_CGR_THRESHOLD , 1);

	ret = qman_create_cgr(&qipriv.req_cgr,
				QMAN_CGR_FLAG_USE_INIT, &opts);
	if (ret) {
		dev_err(qidev, "Error %d creating CAAM req CGRID: %u\n",
			ret, qipriv.req_cgr.cgrid);
		goto create_req_cgr_fail;
	}
	return 0;

create_req_cgr_fail:
	qman_release_cgrid(qipriv.req_cgr.cgrid);

alloc_req_cgrid_fail:
	qman_delete_cgr(&qipriv.rsp_cgr);

create_rsp_cgr_fail:
	qman_release_cgrid(qipriv.rsp_cgr.cgrid);

	return ret;
}

static int alloc_rsp_fqs(struct device *qidev)
{
	const cpumask_t *cpus = qman_affine_cpus();
	int ret, i;

	/*Now create response FQs*/
	for_each_cpu(i, cpus) {
		ret = alloc_rsp_fq_cpu(qidev, i);
		if (ret) {
			dev_err(qidev, "CAAM rsp FQ alloc failed, cpu: %u", i);
			return ret;
		}
	}

	return 0;
}

int caam_qi_init(struct platform_device *caam_pdev, struct device_node *np)
{
	struct platform_device *qi_pdev;
	struct device *ctrldev, *qidev;
	struct caam_drv_private *ctrlpriv;
	int err, i;
	const cpumask_t *cpus = qman_affine_cpus();
	struct cpumask old_cpumask = *tsk_cpus_allowed(current);

	/*
	 * QMAN requires that CGR must be removed from same CPU+portal from
	 * where it was originally allocated. Hence we need to note down
	 * the initialisation CPU and use the same CPU for module exit.
	 * We select the first CPU to from the list of portal owning
	 * CPUs. Then we pin module init to this CPU.
	 */
	mod_init_cpu = cpumask_first(cpus);
	set_cpus_allowed_ptr(current, get_cpu_mask(mod_init_cpu));

	qi_pdev = platform_device_register_simple("caam_qi", 0, NULL, 0);
	if (IS_ERR(qi_pdev))
		return PTR_ERR(qi_pdev);

	ctrldev = &caam_pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	qidev = &qi_pdev->dev;

	qipriv.qi_pdev = qi_pdev;
	dev_set_drvdata(qidev, &qipriv);

	/* Copy dma mask from controlling device */
	err = dma_set_mask(qidev, dma_get_mask(ctrldev));
	if (err) {
		platform_device_unregister(qi_pdev);
		return -ENODEV;
	}

	/* Initialise the CGRs congestion detection */
	err = alloc_cgrs(qidev);
	if (err) {
		dev_err(qidev, "Can't allocate CGRs\n");
		platform_device_unregister(qi_pdev);
		return err;
	}

	/* Initialise response FQs */
	err = alloc_rsp_fqs(qidev);
	if (err) {
		dev_err(qidev, "Can't allocate SEC response FQs\n");
		platform_device_unregister(qi_pdev);
		return err;
	}

	/*
	 * Enable the NAPI contexts on each of the core which has a affine
	 * portal.
	 */
	for_each_cpu(i, cpus) {
		per_cpu(pcpu_qipriv.net_dev, i).dev = *qidev;

		spin_lock_init(&per_cpu(pcpu_qipriv.listlock, i));
		INIT_LIST_HEAD(&per_cpu(pcpu_qipriv.bklog_list, i));

		INIT_LIST_HEAD(&per_cpu(pcpu_qipriv.net_dev, i).napi_list);

		netif_napi_add(&per_cpu(pcpu_qipriv.net_dev, i),
			       &per_cpu(pcpu_qipriv.irqtask, i),
			       caam_qi_poll, CAAM_NAPI_WEIGHT);

		napi_enable(&per_cpu(pcpu_qipriv.irqtask, i));
	}

	/* Hook up QI device to parent controlling caam device */
	ctrlpriv->qidev = qidev;

	/* Done with the CGRs; restore the cpus allowed mask */
	set_cpus_allowed_ptr(current, &old_cpumask);

	dev_info(qidev, "Linux CAAM Queue I/F driver initialised\n");

	return 0;
}
