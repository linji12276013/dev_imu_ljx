#include "Jscope.h"
#include "SEGGER_RTT.h"

char JS_RTT_UPBUFFER[4096];
int JS_RTT_CHANNEL = 1;

void jscopetest(void)
{
	Val_t t_test;
	
	SEGGER_RTT_ConfigUpBuffer(  1,
								"Jscope_I4I4",
								&JS_RTT_UPBUFFER[0],
								sizeof(JS_RTT_UPBUFFER),
								SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	while (1)
	{
		t_test.Val1 += 1;
		t_test.Val2 -= 1;
		SEGGER_RTT_Write(1, &t_test, sizeof(t_test));
	}
}
