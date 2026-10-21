#ifdef __SWITCH__

#include <string.h>
#include <math.h>
#include <switch.h>
#include "gyro_nswitch.h"

#define NUM_HANDLES 4
#define GRAVITY_FILTER 0.05f

enum {
	HANDLE_HANDHELD,
	HANDLE_FULLKEY, // Pro Controller and friends
	HANDLE_JOY_LEFT,
	HANDLE_JOY_RIGHT,
};

static HidSixAxisSensorHandle handles[NUM_HANDLES];
static int started = 0;

// low-pass filtered direction of "up" in the device's own coordinates, taken from the accelerometer
static float up[3] = { 0.f, 0.f, 0.f };
static int upValid = 0;

int nxGyroInit(void)
{
	if (started) {
		return 1;
	}

	// handheld mode, a single controller and a pair of Joy-Cons all use different handles
	hidGetSixAxisSensorHandles(&handles[HANDLE_HANDHELD], 1, HidNpadIdType_Handheld, HidNpadStyleTag_NpadHandheld);
	hidGetSixAxisSensorHandles(&handles[HANDLE_FULLKEY], 1, HidNpadIdType_No1, HidNpadStyleTag_NpadFullKey);
	hidGetSixAxisSensorHandles(&handles[HANDLE_JOY_LEFT], 2, HidNpadIdType_No1, HidNpadStyleTag_NpadJoyDual);

	for (int i = 0; i < NUM_HANDLES; ++i) {
		hidStartSixAxisSensor(handles[i]);
	}

	started = 1;
	return 1;
}

static void updateUp(const HidVector *accel)
{
	const float mag = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
	if (mag < 0.001f) {
		return;
	}

	// at rest the accelerometer points away from the ground
	const float n[3] = { accel->x / mag, accel->y / mag, accel->z / mag };

	if (!upValid) {
		memcpy(up, n, sizeof(up));
		upValid = 1;
		return;
	}

	for (int i = 0; i < 3; ++i) {
		up[i] += (n[i] - up[i]) * GRAVITY_FILTER;
	}

	const float len = sqrtf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
	if (len > 0.001f) {
		for (int i = 0; i < 3; ++i) {
			up[i] /= len;
		}
	}
}

int nxGyroRead(float *pitch, float *yaw, float *roll)
{
	static const int order[NUM_HANDLES] = {
		HANDLE_HANDHELD, HANDLE_FULLKEY, HANDLE_JOY_RIGHT, HANDLE_JOY_LEFT
	};

	if (!started && !nxGyroInit()) {
		return 0;
	}

	for (int i = 0; i < NUM_HANDLES; ++i) {
		HidSixAxisSensorState state;
		memset(&state, 0, sizeof(state));

		if (hidGetSixAxisSensorStates(handles[order[i]], &state, 1) < 1) {
			continue;
		}
		if (!(state.attributes & HidSixAxisSensorAttribute_IsConnected)) {
			continue;
		}

		const HidVector *w = &state.angular_velocity;

		updateUp(&state.acceleration);

		*pitch = w->x;
		*roll = w->z;
		if (upValid) {
			// turning left/right = rotating around the gravity direction, however the device is held
			*yaw = w->x * up[0] + w->y * up[1] + w->z * up[2];
		} else {
			*yaw = w->y;
		}

		return 1;
	}

	return 0;
}

#endif // __SWITCH__
