/* Copyright 2020 The MathWorks, Inc. */

/**
 * @file mavlinkcodegen_CircularQueue.hpp
 * @brief Header for Circular Queue class.
 */

#ifndef MAVLINKCODEGEN_CIRCULARQUEUE_HPP
#define MAVLINKCODEGEN_CIRCULARQUEUE_HPP

#include "mavlink/mavlink_types.h" // for mavlink_message_t
#include <stddef.h>                // for size_t

namespace uavlib {

///
/**
 * CircularQueue implementation of Circular Queue to store MAVLink messages
 */

class CircularQueue {
  private:
    /// Pointer to Queue Front
    int front;

    /// Pointer to Queue Rear
    int rear;

    /// Maximum size of Queue
    int maxSize;

    /// Placeholder for Circular queue
    mavlink_message_t* queue;

  public:
    /// Create the Circular Queue with no memory allocated
    CircularQueue();

    /// Create the Circular Queue with requested size
    CircularQueue(uint16_t queueSize);

    ~CircularQueue();

    /// check if Queue is empty
    /**
     * @return		boolean value indicating whether Queue is empty
     */
    bool isQueueEmpty() const;

    /// check if Queue is full
    /**
     * @return		boolean value indicating whether Queue is full
     */
    bool isQueueFull() const;

    /// Remove the Front element from Queue
    /**
     * @return		The MAVLink message stored at Queue Front
     */
    mavlink_message_t deQueue();

    /// Add element to Rear of Queue
    /**
     * @param[in] msg   MAVLink message to be added to Queue
     */
    void enQueue(mavlink_message_t msg);
};
} // namespace uavlib
#endif // !MAVLINKCODEGEN_CIRCULARQUEUE_HPP
