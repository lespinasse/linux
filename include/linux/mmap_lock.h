#ifndef _LINUX_MMAP_LOCK_H
#define _LINUX_MMAP_LOCK_H

#include <linux/mmdebug.h>

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
	.count = 0,						\
	MMAP_LOCK_DEP_MAP_INITIALIZER((name).mmap_lock)		\
},

static inline void mmap_init_lock(struct mm_struct *mm)
{
	static struct lock_class_key __key;

	mutex_init(&mm->mmap_lock.mutex);
	INIT_LIST_HEAD(&mm->mmap_lock.head);
	mm->mmap_lock.count = 0;
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
	VM_BUG_ON_MM(!mm->mmap_lock.count, mm);
}

static inline void mmap_assert_write_locked(struct mm_struct *mm)
{
	lockdep_assert_held_write(&mm->mmap_lock);
	VM_BUG_ON_MM(mm->mmap_lock.count != -1, mm);
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
