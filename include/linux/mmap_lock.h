#ifndef _LINUX_MMAP_LOCK_H
#define _LINUX_MMAP_LOCK_H

#include <linux/mmdebug.h>
#include <linux/sched/wake_q.h>

#ifdef CONFIG_MMAP_LOCK_QUEUED

#ifdef CONFIG_DEBUG_LOCK_ALLOC
#define MMAP_LOCK_DEP_MAP_INITIALIZER(lockname)	\
	.dep_map = { .name = #lockname },
#else
#define MMAP_LOCK_DEP_MAP_INITIALIZER(lockname)
#endif

#define MMAP_LOCK_INITIALIZER(name) .mmap_lock = {		\
	.mutex = __MUTEX_INITIALIZER((name).mmap_lock.mutex),	\
	.head = LIST_HEAD_INIT((name).mmap_lock.head),		\
	.coarse_count = 0,					\
	.fine_writers = 0,					\
	MMAP_LOCK_DEP_MAP_INITIALIZER((name).mmap_lock)		\
},

static inline void mmap_init_lock(struct mm_struct *mm)
{
	static struct lock_class_key __key;

	mutex_init(&mm->mmap_lock.mutex);
	INIT_LIST_HEAD(&mm->mmap_lock.head);
	mm->mmap_lock.coarse_count = 0;
	mm->mmap_lock.fine_writers = 0;
	lockdep_init_map(&mm->mmap_lock.dep_map, "&mm->mmap_lock", &__key, 0);
}

extern void mmap_write_lock(struct mm_struct *mm);
#ifdef CONFIG_LOCKDEP
extern void mmap_write_lock_nested(struct mm_struct *mm, int subclass);
#else
static inline void mmap_write_lock_nested(struct mm_struct *mm, int subclass)
{
	mmap_write_lock(mm);
}
#endif
extern int mmap_write_lock_killable(struct mm_struct *mm);
extern bool mmap_write_trylock(struct mm_struct *mm);
extern void mmap_write_unlock(struct mm_struct *mm);
extern void mmap_write_downgrade(struct mm_struct *mm);
extern void mmap_read_lock(struct mm_struct *mm);
extern int mmap_read_lock_killable(struct mm_struct *mm);
extern bool mmap_read_trylock(struct mm_struct *mm);
extern void mmap_read_unlock(struct mm_struct *mm);

static inline void mmap_assert_locked(struct mm_struct *mm)
{
	lockdep_assert_held(&mm->mmap_lock);
	VM_BUG_ON_MM(!mm->mmap_lock.coarse_count, mm);
	VM_BUG_ON_MM(mm->mmap_lock.fine_writers, mm);
}

static inline void mmap_assert_write_locked(struct mm_struct *mm)
{
	lockdep_assert_held_write(&mm->mmap_lock);
	VM_BUG_ON_MM(mm->mmap_lock.coarse_count != -1, mm);
	VM_BUG_ON_MM(mm->mmap_lock.fine_writers, mm);
}


static inline void mmap_vma_lock(struct mm_struct *mm)
{
	mutex_lock(&mm->mmap_lock.mutex);
}

static inline bool mmap_vma_trylock(struct mm_struct *mm)
{
	return mutex_trylock(&mm->mmap_lock.mutex) != 0;
}

static inline void mmap_vma_unlock(struct mm_struct *mm)
{
	mutex_unlock(&mm->mmap_lock.mutex);
}

struct mmap_lock_waiter {
	bool (*f)(struct mm_struct *mm, struct mmap_lock_waiter *w);
	struct list_head list;
	struct task_struct *task;	/* blocked locker task */
};

typedef bool (*mmap_lock_f)(struct mm_struct *mm,
			    struct mmap_lock_waiter *w);

extern void mmap_lock_dequeue(struct mm_struct *mm,
		struct wake_q_head *wake_q);
extern void mmap_f_lock_slow(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f);
extern int mmap_f_lock_killable_slow(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f);

static inline bool mmap_write_f_trylock(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	if (!mmap_vma_trylock(mm))
		return false;	/* can not lock without blocking */
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		mmap_vma_unlock(mm);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	mmap_vma_unlock(mm);
	return true;	/* acquired writer lock */
}

static inline void mmap_write_f_lock(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		mmap_f_lock_slow(mm, w, f);
		return;
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}

static inline int mmap_write_f_lock_killable (struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	lock_acquire_exclusive(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		return mmap_f_lock_killable_slow(mm, w, f);
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}

static inline bool mmap_read_f_trylock(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	if (!mmap_vma_trylock(mm))
		return false;	/* can not lock without blocking */
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		mmap_vma_unlock(mm);
		return false;	/* can not lock without blocking */
	}
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 1, NULL, _RET_IP_);
	mmap_vma_unlock(mm);
	return true;	/* acquired reader lock */
}

static inline void mmap_read_f_lock(struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		mmap_f_lock_slow(mm, w, f);
		return;
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
}

static inline int mmap_read_f_lock_killable (struct mm_struct *mm,
		struct mmap_lock_waiter *w, mmap_lock_f f)
{
	lock_acquire_shared(&mm->mmap_lock.dep_map, 0, 0, NULL, _RET_IP_);

	mmap_vma_lock(mm);
	if (unlikely(!list_empty(&mm->mmap_lock.head) || !f(mm, w))) {
		return mmap_f_lock_killable_slow(mm, w, f);
	}
	mmap_vma_unlock(mm);

	lock_acquired(&mm->mmap_lock.dep_map, _RET_IP_);
	return 0;
}

static inline void mmap_vma_f_unlock(struct mm_struct *mm, bool dequeue)
{
	DEFINE_WAKE_Q(wake_q);

	if (dequeue && !list_empty(&mm->mmap_lock.head))
		mmap_lock_dequeue(mm, &wake_q);
	mmap_vma_unlock(mm);

	if (unlikely(!wake_q_empty(&wake_q)))
		wake_up_q(&wake_q);
	lock_release(&mm->mmap_lock.dep_map, _RET_IP_);
}

#else /* !CONFIG_MMAP_LOCK_queued */

#define MMAP_LOCK_INITIALIZER(name) \
	.mmap_lock = __RWSEM_INITIALIZER((name).mmap_lock),

static inline void mmap_init_lock(struct mm_struct *mm)
{
	init_rwsem(&mm->mmap_lock);
}

static inline void mmap_write_lock(struct mm_struct *mm)
{
	down_write(&mm->mmap_lock);
}

static inline void mmap_write_lock_nested(struct mm_struct *mm, int subclass)
{
	down_write_nested(&mm->mmap_lock, subclass);
}

static inline int mmap_write_lock_killable(struct mm_struct *mm)
{
	return down_write_killable(&mm->mmap_lock);
}

static inline bool mmap_write_trylock(struct mm_struct *mm)
{
	return down_write_trylock(&mm->mmap_lock) != 0;
}

static inline void mmap_write_unlock(struct mm_struct *mm)
{
	might_sleep();
	up_write(&mm->mmap_lock);
}

static inline void mmap_write_downgrade(struct mm_struct *mm)
{
	downgrade_write(&mm->mmap_lock);
}

static inline void mmap_read_lock(struct mm_struct *mm)
{
	down_read(&mm->mmap_lock);
}

static inline int mmap_read_lock_killable(struct mm_struct *mm)
{
	return down_read_killable(&mm->mmap_lock);
}

static inline bool mmap_read_trylock(struct mm_struct *mm)
{
	return down_read_trylock(&mm->mmap_lock) != 0;
}

static inline void mmap_read_unlock(struct mm_struct *mm)
{
	might_sleep();
	up_read(&mm->mmap_lock);
}

static inline void mmap_assert_locked(struct mm_struct *mm)
{
	lockdep_assert_held(&mm->mmap_lock);
	VM_BUG_ON_MM(!rwsem_is_locked(&mm->mmap_lock), mm);
}

static inline void mmap_assert_write_locked(struct mm_struct *mm)
{
	lockdep_assert_held_write(&mm->mmap_lock);
	VM_BUG_ON_MM(!rwsem_is_locked(&mm->mmap_lock), mm);
}

#endif /* CONFIG_MMAP_LOCK_QUEUED */

#endif /* _LINUX_MMAP_LOCK_H */
