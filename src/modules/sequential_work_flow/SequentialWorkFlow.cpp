#include "SequentialWorkFlow.hpp"

extern "C" __EXPORT int swf_main(int argc, char *argv[]);

#define SWF0_DEVICE_PATH	"/dev/swf0"

namespace sequential_workflow
{
SequentialWorkFlow	*g_swf;
}

int swf_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "init")) {

		if (sequential_workflow::g_swf != nullptr) {
			warnx("already running");
			return 1;
		}

		sequential_workflow::g_swf = new SequentialWorkFlow;

		if (sequential_workflow::g_swf == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		PX4_INFO("SUCCES, leuk man");

		return 0;
	}

	if (!strcmp(argv[1], "register")) {
		if (sequential_workflow::g_swf == nullptr) {
			warnx("not running");
			return 1;
		}
		sequential_workflow::g_swf->increment();
	}

	if (!strcmp(argv[1], "status")) {
		if (sequential_workflow::g_swf == nullptr) {
			warnx("not running");
			return 1;
		}
		PX4_INFO("value = %u", sequential_workflow::g_swf->getValue());
	}

	if (!strcmp(argv[1], "loop")) {
		if (sequential_workflow::g_swf == nullptr) {
			warnx("not running");
			return 1;
		}

		sequential_workflow::registration* reg = sequential_workflow::g_swf->_first_registration;
		while (reg != nullptr) {
			warnx("HALLO");
			int value = reg->module->swf_init_function();
			PX4_INFO("got %d", value);
			reg = reg->next;
		};
	}

	if (!strcmp(argv[1], "start")) {
		px4_task_spawn_cmd("sequential_workflow",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_POSITION_CONTROL,
				   1900,
				   (px4_main_t)&SequentialWorkFlow::task_main_trampoline,
				   nullptr);
	}
	/*

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}
	*/
	return 0;
}

int
SequentialWorkFlow::task_main_trampoline(int argc, char *argv[])
{
	sequential_workflow::g_swf->task_main();
	return 0;
}

SequentialWorkFlow::SequentialWorkFlow() : CDev(SWF0_DEVICE_PATH)
{
	CDev::init();
}

SequentialWorkFlow::~SequentialWorkFlow()
{

}

void SequentialWorkFlow::increment()
{
	_test_value++;
}

int SequentialWorkFlow::getValue()
{
	return _test_value;
}

ssize_t
SequentialWorkFlow::write(cdev::file_t *filep, const char *buffer, size_t buflen)
{
	// basically take a new registration here
	sequential_workflow::registration* new_reg = new sequential_workflow::registration;
	memcpy(new_reg, buffer, sizeof(sequential_workflow::registration));
	new_reg->next = nullptr;

	if (_first_registration == nullptr) {
		_first_registration = new_reg;
	} else {
		_last_registration->next = new_reg;
	}

	_last_registration = new_reg;

	PX4_INFO("stacksize from reg: %u", new_reg->stack_size);
	int testvalue = new_reg->module->swf_init_function();

	return testvalue;
}

void SequentialWorkFlow::task_main()
{
	// Setup all modules
	sequential_workflow::registration* reg = sequential_workflow::g_swf->_first_registration;
	while (reg != nullptr) {
		warnx("HALLO");
		int value = reg->module->swf_init_function();
		PX4_INFO("got %d", value);
		reg = reg->next;
	};

	// Loop through all modules
	while (1) {
		reg = sequential_workflow::g_swf->_first_registration;
		while (reg != nullptr) {
			warnx("HALLO");
			int value = reg->module->swf_loop_function();
			PX4_INFO("got %d", value);
			reg = reg->next;
		};
		usleep(1000 * 1000);
	}
}
