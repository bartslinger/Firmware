/****************************************************************************
 *
 * Copyright (C) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <uORB/uORB.h>
#include <px4_workqueue.h>

typedef void 	*work_queue_t;

enum queues_enum {
	HPWORK,
	LPWORK,
	ATT_WQ,
	RATES_WQ
};

/**
 * @brief The WorkQueueClient class
 *
 * Class to schedule work into a work queue
 */
class WorkQueueClient
{
public:
    WorkQueueClient() = default;

    int start(const char *queue_name);
    int scheduleWork(struct work_s *work, uint32_t delay);
    int registerTopic(const struct orb_metadata *meta, struct work_s *work);
    int unregisterTopic(const struct orb_metadata *meta);

private:
    work_queue_t _qid;
};


/**
 * @brief The WorkQueueClient class
 *
 * The work queue itself
 */
class WorkQueue{
public:
    WorkQueue() = default;

    int init();

private:
    const char *name = nullptr;
    // Keep a linked list of all existing work queues
    WorkQueue *_prev = nullptr;
    WorkQueue *_next = nullptr;

    struct wqueue_s _work;
};


/**
 * @brief The WorkQueueManager class
 *
 * This class keeps track of all existing work queues and their locations
 */
class WorkQueueManager
{
public:
    WorkQueueManager() = default;

private:

    // Keep a linked list of all existing work queues
    WorkQueue *_first = nullptr;
    WorkQueue *_last = nullptr;
};
