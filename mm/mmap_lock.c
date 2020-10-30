#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/lockdep.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/wake_q.h>

struct mmap_lock_waiter {
	bool writer;
	struct list_head list;
	struct task_struct *task;	/* blocked locker task */
};

static inline void enqueue(struct mm_struct *mm, struct mmap_lock_waiter *w,
			   bool writer)
{
	w->writer = writer;
	list_add_tail(&w->list, &mm->mmap_lock.head);
	w->task = current;
}

static inline bool writer(struct mm_struct *mm, struct mmap_lock_waiter *w)
{
	mutex_lock(&mm->mmap_lock.mutex);
	if (!mm->mmap_lock.count) {
		VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
		mm->mmap_lock.count = -1;
		mutex_unlock(&mm->mmap_lock.mutex);
		return true;	/* acquired writer lock */
	}
	enqueue(mm, w, true);
	mutex_unlock(&mm->mmap_lock.mutex);
	return false;	/* must wait for the lock */
}

static inline bool reader(struct mm_struct *mm, struct mmap_lock_waiter *w)
{
	mutex_lock(&mm->mmap_lock.mutex);
	if (list_empty(&mm->mmap_lock.head) && mm->mmap_lock.count >= 0) {
		mm->mmap_lock.count++;
		mutex_unlock(&mm->mmap_lock.mutex);
		return true;	/* acquired reader lock */
	}
	enqueue(mm, w, false);
	mutex_unlock(&mm->mmap_lock.mutex);
	return false;	/* must wait for the lock */
}

/*
 * dequeue wakes up waiters and passes them the mmap lock.
 * The waiters are woken in order, up to the first waiter that conflicts
 * with the locks already held.
 * setting w->task to NULL signals to the waiter that it's been woken.
 * This must be the last access to the mmap_lock_waiter struct, as the
 * struct is not guaranteed to exist after the waiter notices it's been woken.
 */
static void dequeue(struct mm_struct *mm, struct wake_q_head *wake_q)
{
	struct list_head *e = mm->mmap_lock.head.next;
	long count = mm->mmap_lock.count;

	while (true) {
		struct mmap_lock_waiter *w;

		VM_BUG_ON(e == &mm->mmap_lock.head);
		w = list_entry(e, struct mmap_lock_waiter, list);
		VM_BUG_ON(count < 0);
		if (w->writer) {
			if (!count) {
				count = -1;
				e = w->list.next;
				wake_q_add(wake_q, w->task);
				smp_store_release(&w->task, NULL);
			}
			break;
		} else {
			count++;
			e = w->list.next;
			wake_q_add(wake_q, w->task);
			smp_store_release(&w->task, NULL);
		}
		if (e == &mm->mmap_lock.head)
			goto unlink;
	}
	if (e != mm->mmap_lock.head.next) {
	unlink:
		mm->mmap_lock.count = count;
		mm->mmap_lock.head.next = e;
		e->prev = &mm->mmap_lock.head;
	}
}

static void wait(struct mmap_lock_waiter *w)
{
	while (true) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (!w->task)
			break;
		schedule();
	}
	__set_current_state(TASK_RUNNING);
}

static bool wait_killable(struct mm_struct *mm, struct mmap_lock_waiter *w)
{
	while (true) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!w->task)
			break;
		if (signal_pending(current)) {
			DEFINE_WAKE_Q(wake_q);

			__set_current_state(TASK_RUNNING);

			mutex_lock(&mm->mmap_lock.mutex);
			if (!w->task) {
				mutex_unlock(&mm->mmap_lock.mutex);
				return true;
			}
			__list_del_entry(&w->list);
			if (!w->writer &&
			    list_is_first(&w->list, &mm->mmap_lock.head))
				VM_BUG_ON_MM(mm->mmap_lock.count != -1, mm);
			if (w->writer &&
			    list_is_first(&w->list, &mm->mmap_lock.head) &&
			    !list_is_last(&w->list, &mm->mmap_lock.head) &&
			    mm->mmap_lock.count > 0)
				dequeue(mm, &wake_q);
			mutex_unlock(&mm->mmap_lock.mutex);

			wake_up_q(&wake_q);
			return false;
		}
		schedule();
	}
	__set_current_state(TASK_RUNNING);
	return true;
}

void mmap_write_lock(struct mm_struct *mm)
{
	struct mmap_lock_waiter w;

	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);
	if (!writer(mm, &w)) {
		lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
		wait(&w);
	}
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_lock);

#ifdef CONFIG_LOCKDEP
void mmap_write_lock_nested(struct mm_struct *mm, int subclass)
{
	struct mmap_lock_waiter w;

	lock_acquire_exclusive(&mm->mmap_lock.dep_map, subclass, 0, NULL,
			       _RET_IP_);
	if (!writer(mm, &w)) {
		lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
		wait(&w);
	}
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_lock_nested);
#endif /* CONFIG_LOCKDEP */

int mmap_write_lock_killable(struct mm_struct *mm)
{
	struct mmap_lock_waiter w;

	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);
	if (!writer(mm, &w)) {
		lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
		if (!wait_killable(mm, &w)) {
			lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
			return -EINTR;
		}
	}
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}
EXPORT_SYMBOL(mmap_write_lock_killable);

bool mmap_write_trylock(struct mm_struct *mm)
{
	if (!mutex_trylock(&mm->mmap_lock.mutex))
		return false;	/* can not lock without blocking */
	if (mm->mmap_lock.count) {
		mutex_unlock(&mm->mmap_lock.mutex);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	VM_BUG_ON_MM(!list_empty(&mm->mmap_lock.head), mm);
	mm->mmap_lock.count = -1;
	mutex_unlock(&mm->mmap_lock.mutex);
	return true;	/* acquired writer lock */
}
EXPORT_SYMBOL(mmap_write_trylock);

void mmap_write_unlock(struct mm_struct *mm)
{
	DEFINE_WAKE_Q(wake_q);

	mutex_lock(&mm->mmap_lock.mutex);
	VM_BUG_ON_MM(mm->mmap_lock.count != -1, mm);
	mm->mmap_lock.count = 0;
	if (!list_empty(&mm->mmap_lock.head))
		dequeue(mm, &wake_q);
	mutex_unlock(&mm->mmap_lock.mutex);

	wake_up_q(&wake_q);
	lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_unlock);

void mmap_write_downgrade(struct mm_struct *mm)
{
	DEFINE_WAKE_Q(wake_q);

	mutex_lock(&mm->mmap_lock.mutex);
	VM_BUG_ON_MM(mm->mmap_lock.count != -1, mm);
	mm->mmap_lock.count = 1;
	if (!list_empty(&mm->mmap_lock.head))
		dequeue(mm, &wake_q);
	mutex_unlock(&mm->mmap_lock.mutex);

	wake_up_q(&wake_q);
	lock_downgrade(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_write_downgrade);

void mmap_read_lock(struct mm_struct *mm)
{
	struct mmap_lock_waiter w;

	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);
	if (!reader(mm, &w)) {
		lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
		wait(&w);
	}
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_read_lock);

int mmap_read_lock_killable(struct mm_struct *mm)
{
	struct mmap_lock_waiter w;

	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);
	if (!reader(mm, &w)) {
		lock_contended(&mm->mmap_lock.dep_map, _RET_IP_);
		if (!wait_killable(mm, &w)) {
			lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
			return -EINTR;
		}
	}
	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}
EXPORT_SYMBOL(mmap_read_lock_killable);

bool mmap_read_trylock(struct mm_struct *mm)
{
	if (!mutex_trylock(&mm->mmap_lock.mutex))
		return false;	/* can not lock without blocking */
	if (!list_empty(&mm->mmap_lock.head) || mm->mmap_lock.count < 0) {
		mutex_unlock(&mm->mmap_lock.mutex);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	mm->mmap_lock.count++;
	mutex_unlock(&mm->mmap_lock.mutex);
	return true;	/* acquired reader lock */
}
EXPORT_SYMBOL(mmap_read_trylock);

void mmap_read_unlock(struct mm_struct *mm)
{
	DEFINE_WAKE_Q(wake_q);

	mutex_lock(&mm->mmap_lock.mutex);
	VM_BUG_ON_MM(mm->mmap_lock.count <= 0, mm);
	mm->mmap_lock.count--;
	if (!mm->mmap_lock.count && !list_empty(&mm->mmap_lock.head))
		dequeue(mm, &wake_q);
	mutex_unlock(&mm->mmap_lock.mutex);

	wake_up_q(&wake_q);
	lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
}
EXPORT_SYMBOL(mmap_read_unlock);
