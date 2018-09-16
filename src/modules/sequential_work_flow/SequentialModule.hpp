#pragma once

class SequentialModule {
public:
        SequentialModule() = default;
        virtual ~SequentialModule() = default;

	virtual int swf_init_function(void);
	virtual int swf_loop_function(void);

protected:

private:
	int _test_value{0};
};
