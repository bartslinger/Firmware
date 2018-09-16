#pragma once
#include <lib/cdev/CDev.hpp>
#include "SequentialModule.hpp"

namespace sequential_workflow {
	struct registration {
		registration* next;
		int wakeup_fd;
		unsigned int stack_size;
		SequentialModule* module;
	};
}

class SequentialWorkFlow : public cdev::CDev {
public:
	SequentialWorkFlow();
	~SequentialWorkFlow();

	static int	task_main_trampoline(int argc, char *argv[]);

	void increment();
	int getValue();

	sequential_workflow::registration* _first_registration{nullptr};
	sequential_workflow::registration* _last_registration{nullptr};
protected:
	ssize_t write(cdev::file_t *filep, const char *buffer, size_t buflen);
private:
	void task_main();
	int _test_value{0};
};
