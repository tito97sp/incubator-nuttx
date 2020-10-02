/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CDev.cpp
 *
 * Character device base class.
 */

#include "nuttx/cdev/CDev.hpp"

#include <nuttx/semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>
//#include <stdio.h>

/* Now include architecture-specific types */

#include <stdio.h>
#include <arch/irq.h>
#include <cstring>

//#include <px4_platform_common/posix.h>
//#include <drivers/drv_device.h>

namespace cdev
{

CDev::CDev(const char *devname) :
	_devname(devname)
{
	//PX4_DEBUG("CDev::CDev");

	int ret = sem_init(&_lock, 0, 1);

	if (ret != 0) {
		//PX4_DEBUG("SEM INIT FAIL: ret %d", ret);
	}
}

CDev::~CDev()
{
	//PX4_DEBUG("CDev::~CDev");

	if (_registered) {
		unregister_driver(_devname);
	}

	if (_pollset) {
		delete[](_pollset);
	}

	sem_destroy(&_lock);
}

int
CDev::register_class_devname(const char *class_devname)
{
	//PX4_DEBUG("CDev::register_class_devname %s", class_devname);

	if (class_devname == nullptr) {
		return -EINVAL;
	}

	int class_instance = 0;
	int ret = -ENOSPC;

	while (class_instance < 4) {
		char name[32];
		snprintf(name, sizeof(name), "%s%d", class_devname, class_instance);
		ret = register_driver(name, &fops, 0666, (void *)this);

		if (ret == 0) {
			break;
		}

		class_instance++;
	}

	if (class_instance == 4) {
		return ret;
	}

	return class_instance;
}

int
CDev::unregister_class_devname(const char *class_devname, unsigned class_instance)
{
	//PX4_DEBUG("CDev::unregister_class_devname");

	char name[32];
	snprintf(name, sizeof(name), "%s%u", class_devname, class_instance);
	return unregister_driver(name);
}

int
CDev::init()
{
	//PX4_DEBUG("CDev::init");

	int ret = 0;

	// now register the driver
	if (_devname != nullptr) {
		ret = register_driver(_devname, &fops, 0666, (void *)this);

		if (ret == 0) {
			_registered = true;
		}
	}

	return ret;
}

/*
 * Default implementations of the character device interface
 */
int
CDev::open(file *filep)
{
	//PX4_DEBUG("CDev::open");
	int ret = 0;

	lock();
	/* increment the open count */
	_open_count++;

	if (_open_count == 1) {

		/* first-open callback may decline the open */
		ret = open_first(filep);

		if (ret != 0) {
			_open_count--;
		}
	}

	unlock();

	return ret;
}

int
CDev::close(file *filep)
{
	//PX4_DEBUG("CDev::close");
	int ret = 0;

	lock();

	if (_open_count > 0) {
		/* decrement the open count */
		_open_count--;

		/* callback cannot decline the close */
		if (_open_count == 0) {
			ret = close_last(filep);
		}

	} else {
		ret = -EBADF;
	}

	unlock();

	return ret;
}

int
CDev::poll(file *filep, struct pollfd *fds, bool setup)
{
	//PX4_DEBUG("CDev::Poll %s", setup ? "setup" : "teardown");
	int ret = 0;

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *)filep;
		//PX4_DEBUG("CDev::poll: fds->priv = %p", filep);

		/*
		 * Lock against poll_notify() and possibly other callers (protect _pollset).
		 */
		
		ATOMIC_ENTER;

		/*
		 * Try to store the fds for later use and handle array resizing.
		 */
		while ((ret = store_poll_waiter(fds)) == -ENFILE) {

			// No free slot found. Resize the pollset. This is expensive, but it's only needed initially.

			if (_max_pollwaiters >= 256 / 2) { //_max_pollwaiters is uint8_t
				ret = -ENOMEM;
				break;
			}

			const uint8_t new_count = _max_pollwaiters > 0 ? _max_pollwaiters * 2 : 1;
			pollfd **prev_pollset = _pollset;


			// malloc uses a semaphore, we need to call it enabled IRQ's
			leave_critical_section(flags);

			pollfd **new_pollset = new pollfd *[new_count];


			flags = enter_critical_section();


			if (prev_pollset == _pollset) {
				// no one else updated the _pollset meanwhile, so we're good to go
				if (!new_pollset) {
					ret = -ENOMEM;
					break;
				}

				if (_max_pollwaiters > 0) {
					memset(new_pollset + _max_pollwaiters, 0, sizeof(pollfd *) * (new_count - _max_pollwaiters));
					memcpy(new_pollset, _pollset, sizeof(pollfd *) * _max_pollwaiters);
				}

				_pollset = new_pollset;
				_pollset[_max_pollwaiters] = fds;
				_max_pollwaiters = new_count;

				// free the previous _pollset (we need to unlock here which is fine because we don't access _pollset anymore)

				leave_critical_section(flags);


				if (prev_pollset) {
					delete[](prev_pollset);
				}


				flags = enter_critical_section();


				// Success
				ret = 0;
				break;
			}


			leave_critical_section(flags);

			// We have to retry
			delete[] new_pollset;

			flags = enter_critical_section();

		}

		if (ret == 0) {

			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			fds->revents |= fds->events & poll_state(filep);

			/* yes? post the notification */
			if (fds->revents != 0) {
				sem_post(fds->sem);
			}

		}

		ATOMIC_LEAVE;

	} else {
		ATOMIC_ENTER;
		/*
		 * Handle a teardown request.
		 */
		ret = remove_poll_waiter(fds);
		ATOMIC_LEAVE;
	}

	return ret;
}

void
CDev::poll_notify(pollevent_t events)
{
	//PX4_DEBUG("CDev::poll_notify events = %0x", events);

	/* lock against poll() as well as other wakeups */
	ATOMIC_ENTER;

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr != _pollset[i]) {
			poll_notify_one(_pollset[i], events);
		}
	}

	ATOMIC_LEAVE;
}

void
CDev::poll_notify_one(struct pollfd *fds, pollevent_t events)
{
	//PX4_DEBUG("CDev::poll_notify_one");

	/* update the reported event set */
	fds->revents |= fds->events & events;

	//PX4_DEBUG(" Events fds=%p %0x %0x %0x", fds, fds->revents, fds->events, events);

	if (fds->revents != 0) {
		sem_post(fds->sem);
	}
}

int
CDev::store_poll_waiter(struct pollfd *fds)
{
	// Look for a free slot.
	//PX4_DEBUG("CDev::store_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr == _pollset[i]) {

			/* save the pollfd */
			_pollset[i] = fds;

			return 0;
		}
	}

	return -ENFILE;
}

int
CDev::remove_poll_waiter(struct pollfd *fds)
{
	//PX4_DEBUG("CDev::remove_poll_waiter");

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (fds == _pollset[i]) {

			_pollset[i] = nullptr;
			return 0;

		}
	}

	//PX4_DEBUG("poll: bad fd state");
	return -EINVAL;
}

int CDev::unregister_driver_and_memory()
{
	int retval = 0;

	if (_registered) {
		unregister_driver(_devname);
		_registered = false;

	} else {
		retval = -ENODEV;
	}

	if (_devname != nullptr) {
		free((void *)_devname);
		_devname = nullptr;

	} else {
		retval = -ENODEV;
	}

	return retval;
}

} // namespace cdev
