#include "device.h"


#if 0
typedef struct {
	TIMER_STATE state;
	uint32_t count;
	uint32_t time;
	void *param;
	void (*func)(void *param);
}TIMER;


#define TIMER_MAX	3
static TIMER Timers[TIMER_MAX] = { 0 };

void Timer_Init(void)
{
	int i = 0;

	for (i=0; i<TIMER_MAX; i++) {
		Timers[i].state = TIMER_IDLE;
	}
}

void Timer_Tick(void)
{
	int i = 0;
	TIMER *t;

	for (i=0; i<TIMER_MAX; i++) {
		t = &Timers[i];
		if (TIMER_START != t->state) {
			continue;
		}
		if (t->count < t->time) {
			t->count++;
		}
	}
}

TIMER_HANDLE Timer_Register(uint32_t timeout, void(*func)(void*), void *param)
{
	int i = 0;
	TIMER *t = NULL;

	for (i=0; i<TIMER_MAX; i++) {
		t = &Timers[i];
		if (TIMER_IDLE == t->state) {
			t->state = TIMER_START;
			t->count = 0;
			t->time = timeout;
			t->func = func;
			t->param = param;
			break;
		}
	}
	
	return (TIMER_HANDLE)t;
}

uint8_t Timer_State(TIMER_HANDLE htimer, TIMER_STATE state)
{
	TIMER *t = (TIMER *)htimer;

	if (NULL == t)
		return 0;

	if (TIMER_IDLE == t->state)
		return 0;

	if (TIMER_RESET == t->state) {
		t->count = 0;
	}
	else {
		t->state = state;
	}
	
	return 1;
}

void Timer_Process(void)
{
	int i = 0;
	TIMER *t;

	for (i=0; i<TIMER_MAX; i++) {
		t = &Timers[i];
		if (TIMER_START != t->state) {
			continue;
		}
		if (t->count >= t->time) {
			if (t->func)
				t->func(t->param);
			t->count = 0;
		}
	}
}
#endif

