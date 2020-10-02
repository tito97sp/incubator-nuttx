#ifndef __APPS_INCLUDE_UORB_H
#define __APPS_INCLUDE_UORB_H

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/ioctl.h>
#include <atomic>

#include "uORB.h"
//#include "SubscriptionInterval.hpp"
//#include "SubscriptionCallback.hpp"


namespace uORB
{
class DeviceNode;
class DeviceMaster;
class Manager;
class SubscriptionCallback;
}

typedef uint64_t hrt_abstime;

//static uORB::SubscriptionInterval *filp_to_subscription(file *filp) { return static_cast<uORB::SubscriptionInterval *>(filp->f_priv); }

class uORB::DeviceNode
{
  public:
	DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path, uint8_t queue_size = 1);
	virtual ~DeviceNode();

	// no copy, assignment, move, move assignment
	DeviceNode(const uORB::DeviceNode &) = delete;
	DeviceNode &operator=(const uORB::DeviceNode &) = delete;
	DeviceNode(uORB::DeviceNode &&) = delete;
	DeviceNode &operator=(uORB::DeviceNode &&) = delete;

	/**
	 * Method to publish a data to this node.
	 */
	static ssize_t    publish(const orb_metadata *meta, orb_advert_t handle, const void *data);

	static int        unadvertise(orb_advert_t handle);

#ifdef ORB_COMMUNICATOR
	static int16_t topic_advertised(const orb_metadata *meta, int priority);
	//static int16_t topic_unadvertised(const orb_metadata *meta, int priority);

	/**
	 * processes a request for add subscription from remote
	 * @param rateInHz
	 *   Specifies the desired rate for the message.
	 * @return
	 *   0 = success
	 *   otherwise failure.
	 */
	int16_t process_add_subscription(int32_t rateInHz);

	/**
	 * processes a request to remove a subscription from remote.
	 */
	int16_t process_remove_subscription();

	/**
	 * processed the received data message from remote.
	 */
	int16_t process_received_message(int32_t length, uint8_t *data);
#endif /* ORB_COMMUNICATOR */

	/**
	  * Add the subscriber to the node's list of subscriber.  If there is
	  * remote proxy to which this subscription needs to be sent, it will
	  * done via uORBCommunicator::IChannel interface.
	  * @param sd
	  *   the subscriber to be added.
	  */
	void add_internal_subscriber();

	/**
	 * Removes the subscriber from the list.  Also notifies the remote
	 * if there a uORBCommunicator::IChannel instance.
	 * @param sd
	 *   the Subscriber to be removed.
	 */
	void remove_internal_subscriber();

	/**
	 * Return true if this topic has been advertised.
	 *
	 * This is used in the case of multi_pub/sub to check if it's valid to advertise
	 * and publish to this node or if another node should be tried. */
	bool is_advertised() const { return _advertised; }

	void mark_as_advertised() { _advertised = true; }

	/**
	 * Try to change the size of the queue. This can only be done as long as nobody published yet.
	 * This is the case, for example when orb_subscribe was called before an orb_advertise.
	 * The queue size can only be increased.
	 * @param queue_size new size of the queue
	 * @return PX4_OK if queue size successfully set
	 */
	int update_queue_size(unsigned int queue_size);

	/**
	 * Print statistics (nr of lost messages)
	 * @param reset if true, reset statistics afterwards
	 * @return true if printed something, false otherwise (if no lost messages)
	 */
	bool print_statistics(int max_topic_length);

	uint8_t get_queue_size() const { return _queue_size; }

	int8_t subscriber_count() const { return _subscriber_count; }

	uint32_t lost_message_count() const { return _lost_messages; }

	unsigned published_message_count() const { return _generation.load(); }

	const orb_metadata *get_meta() const { return _meta; }

	const char *get_name() const { return _meta->o_name; }

	uint8_t get_instance() const { return _instance; }

	int get_priority() const { return _priority; }
	void set_priority(uint8_t priority) { _priority = priority; }

	/**
	 * Copies data and the corresponding generation
	 * from a node to the buffer provided.
	 *
	 * @param dst
	 *   The buffer into which the data is copied.
	 * @param generation
	 *   The generation that was copied.
	 * @return bool
	 *   Returns true if the data was copied.
	 */
	bool copy(void *dst, unsigned &generation);

	/*TODO::Make it a IOCTL operation*/
	// add item to list of work items to schedule on node update
	bool register_callback(SubscriptionCallback *callback_sub);
	// remove item from list of work items
	void unregister_callback(SubscriptionCallback *callback_sub);


	/**
	 * Method to create a subscriber instance and return the struct
	 * pointing to the subscriber as a file pointer.
	 */
	int open(FAR struct file *filep);

	/**
	 * Method to close a subscriber for this topic.
	 */
	int close(FAR struct file *filep);

	/**
	 * reads data from a subscriber node to the buffer provided.
	 * @param filp
	 *   The subscriber from which the data needs to be read from.
	 * @param buffer
	 *   The buffer into which the data is read into.
	 * @param buflen
	 *   the length of the buffer
	 * @return
	 *   ssize_t the number of bytes read.
	 */
	ssize_t read(FAR struct file *filep, FAR char *buffer, size_t buflen);

	/**
	 * writes the published data to the internal buffer to be read by
	 * subscribers later.
	 * @param filp
	 *   the subscriber; this is not used.
	 * @param buffer
	 *   The buffer for the input data
	 * @param buflen
	 *   the length of the buffer.
	 * @return ssize_t
	 *   The number of bytes that are written
	 */
	ssize_t write(FAR struct file *filep, FAR const char *buffer, size_t buflen);

	/**
	 * IOCTL control for the subscriber.
	 */
	int ioctl(FAR struct file *filep, int cmd, unsigned long arg);

	int poll(FAR struct file *filep, struct pollfd *fds, bool setup);

private:

	struct UpdateIntervalData {
		uint64_t last_update{0}; /**< time at which the last update was provided, used when update_interval is nonzero */
		unsigned interval{0}; /**< if nonzero minimum interval between updates */
	};

	struct SubscriberData {
		~SubscriberData() { if (update_interval) { delete (update_interval); } }

		unsigned generation{0}; /**< last generation the subscriber has seen */
		UpdateIntervalData *update_interval{nullptr}; /**< if null, no update interval */
	};

	const orb_metadata *_meta; /**< object metadata information */
	const uint8_t _instance; /**< orb multi instance identifier */
	uint8_t     *_data{nullptr};   /**< allocated object buffer */
	hrt_abstime   _last_update{0}; /**< time the object was last updated */
	std::atomic<unsigned>  _generation{0};  /**< object generation count */
	//List<uORB::SubscriptionCallback *>	_callbacks;
	uint8_t   _priority;  /**< priority of the topic */
	bool _advertised{false};  /**< has ever been advertised (not necessarily published data yet) */
	uint8_t _queue_size; /**< maximum number of elements in the queue */
	int8_t _subscriber_count{0};

	// statistics
	uint32_t _lost_messages = 0; /**< nr of lost messages for all subscribers. If two subscribers lose the same
					message, it is counted as two. */

	//inline static SubscriberData    *filp_to_sd(cdev::file_t *filp);

	/**
	 * Check whether a topic appears updated to a subscriber.
	 *
	 * Lock must already be held when calling this.
	 *
	 * @param sd    The subscriber for whom to check.
	 * @return    True if the topic should appear updated to the subscriber
	 */
	bool      appears_updated(SubscriberData *sd);

protected:
	sem_t _lock;
	void lock(){do {} while (sem_wait(&_lock) != 0);}
	void unlock(){sem_post(&_lock);}
	/* data */
};

int uORBNodeDevice_register(FAR const char *path);


#endif /*__APPS_INCLUDE_UORB_H */