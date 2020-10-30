#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/lockdep.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/wake_q.h>

/*
 * mmap_lock_dequeue wakes up waiters and passes them the mmap lock.
 * The waiters are woken in order, up to the first waiter that conflicts
 * with the locks already held.
 * setting w->task to NULL signals to the waiter that it's been woken.
 * This must be the last access to the mmap_lock_waiter struct, as the
 * struct is not guaranteed to exist after the waiter notices it's been woken.
 */
void mmap_lock_dequeue(struct mm_struct *mm, struct wake_q_head *wake_q)
{
	struct list_head *e = mm->mmap_lock.head.next;

	while (true) {
		struct mmap_lock_waiter *w;

		VM_BUG_ON(e == &mm->mmap_lock.head);
		w = list_entry(e, struct mmap_lock_waiter, list);
		if (!w->f(mm, w))
			break;
		e = w->list.next;
		wake_q_add(wake_q, w->task);
		smp_store_release(&w->task, NULL);
		if (e == &mm->mmap_lock.head)
			goto unlink;
	}
	if (e != mm->mmap_lock.head.next) {
	unlink:
		mm->mmap_lock.head.next = e;
		e->prev = &mm->mmap_lock.head;
	}
}
EXPORT_SYMBOL(mmap_lock_dequeue);

void mmap_f_lock_slow(struct mm_struct *mm,
		      struct mmap_lock_waiter *w, mmap_lock_f f)
{
	u64 before = ktime_get_ns();

	w->f = f;
	list_add_tail(&w->list, &mm->mmap_lock.head);
	w->task = current;

	mmap_vma_unlock(mm);

	lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
	while (true) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (!w->task)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	count_vm_events(MMAP_LOCK_BLOCKED_NS, ktime_get_ns() - before);
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_f_lock_slow);

int mmap_f_lock_killable_slow(struct mm_struct *mm, struct mmap_lock_waiter *w,
			      mmap_lock_f f)
{
	u64 before = ktime_get_ns();

	w->f = f;
	list_add_tail(&w->list, &mm->mmap_lock.head);
	w->task = current;

	mmap_vma_unlock(mm);

	lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
	while (true) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!w->task)
			break;
		if (signal_pending(current)) {
			DEFINE_WAKE_Q(wake_q);

			__set_current_state(TASK_RUNNING);
			count_vm_events(MMAP_LOCK_BLOCKED_NS,
					ktime_get_ns() - before);

			mmap_vma_lock(mm);
			if (!w->task) {
				mmap_vma_unlock(mm);
				goto acquired;
			}
			__list_del_entry(&w->list);
			if (list_is_first(&w->list, &mm->mmap_lock.head) &&
			    !list_is_last(&w->list, &mm->mmap_lock.head))
				mmap_lock_dequeue(mm, &wake_q);
			mmap_vma_unlock(mm);

			wake_up_q(&wake_q);
			lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
			return -EINTR;
		}
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	count_vm_events(MMAP_LOCK_BLOCKED_NS, ktime_get_ns() - before);
acquired:
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}
EXPORT_SYMBOL(mmap_f_lock_killable_slow);

static inline bool writer_f(struct mm_struct *mm, struct mmap_lock_waiter *w)
{
	if (mm->mmap_lock.coarse_count)
		return false;
	mm->mmap_lock.coarse_count = -1;
	return true;
}

static inline bool reader_f(struct mm_struct *mm, struct mmap_lock_waiter *w)
{
	if (mm->mmap_lock.coarse_count < 0)
		return false;
	mm->mmap_lock.coarse_count++;
	return true;
}

void mmap_write_lock(struct mm_struct *mm)
{
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!writer_f(mm, NULL))) {
		struct mmap_lock_waiter w;
		mmap_f_lock_slow(mm, &w, writer_f);
		return;
	}
	VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_lock);

#ifdef CONFIG_LOCKDEP
void mmap_write_lock_nested(struct mm_struct *mm, int subclass)
{
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, subclass, 0, NULL,
			       _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!writer_f(mm, NULL))) {
		struct mmap_lock_waiter w;
		mmap_f_lock_slow(mm, &w, writer_f);
		return;
	}
	VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_lock_nested);
#endif /* CONFIG_LOCKDEP */

int mmap_write_lock_killable(struct mm_struct *mm)
{
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!writer_f(mm, NULL))) {
		struct mmap_lock_waiter w;
		return mmap_f_lock_killable_slow(mm, &w, writer_f);
	}
	VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}
EXPORT_SYMBOL(mmap_write_lock_killable);

bool mmap_write_trylock(struct mm_struct *mm)
{
	if (!mmap_vma_trylock(mm))
		return false;	/* can not lock without blocking */
	if (!writer_f(mm, NULL)) {
		mmap_vma_unlock(mm);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
	mmap_vma_unlock(mm);
	return true;	/* acquired writer lock */
}
EXPORT_SYMBOL(mmap_write_trylock);

void mmap_write_unlock(struct mm_struct *mm)
{
	mmap_vma_lock(mm);
	VM_BUG_ON_MM(mm->mmap_lock.coarse_count != -1, mm);
	mm->mmap_lock.coarse_count = 0;
	mmap_vma_f_unlock(mm, true);
}
EXPORT_SYMBOL(mmap_write_unlock);

void mmap_write_downgrade(struct mm_struct *mm)
{
	DEFINE_WAKE_Q(wake_q);

	mmap_vma_lock(mm);
	VM_BUG_ON_MM(mm->mmap_lock.coarse_count != -1, mm);
	mm->mmap_lock.coarse_count = 1;
	if (!list_empty(&mm->mmap_lock.head))
		mmap_lock_dequeue(mm, &wake_q);
	mmap_vma_unlock(mm);

	if (unlikely(!wake_q_empty(&wake_q)))
		wake_up_q(&wake_q);
	lock_downgrade(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_downgrade);

void mmap_read_lock(struct mm_struct *mm)
{
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) ||
		     !reader_f(mm, NULL))) {
		struct mmap_lock_waiter w;
		mmap_f_lock_slow(mm, &w, reader_f);
		return;
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_read_lock);

int mmap_read_lock_killable(struct mm_struct *mm)
{
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) ||
		     !reader_f(mm, NULL))) {
		struct mmap_lock_waiter w;
		return mmap_f_lock_killable_slow(mm, &w, reader_f);
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}
EXPORT_SYMBOL(mmap_read_lock_killable);

bool mmap_read_trylock(struct mm_struct *mm)
{
	if (!mmap_vma_trylock(mm))
		return false;	/* can not lock without blocking */
	if (!list_empty(&mm->mmap_lock.head) || !reader_f(mm, NULL)) {
		mmap_vma_unlock(mm);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	mmap_vma_unlock(mm);
	return true;	/* acquired reader lock */
}
EXPORT_SYMBOL(mmap_read_trylock);

void mmap_read_unlock(struct mm_struct *mm)
{
	mmap_vma_lock(mm);
	VM_BUG_ON_MM(mm->mmap_lock.coarse_count <= 0, mm);
	mm->mmap_lock.coarse_count--;
	mmap_vma_f_unlock(mm, !mm->mmap_lock.coarse_count);
}
EXPORT_SYMBOL(mmap_read_unlock);
