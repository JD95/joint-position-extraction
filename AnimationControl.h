//-----------------------------------------------------------------------------
// HW02 - Builds with SKA Version 4.0
//-----------------------------------------------------------------------------
// AnimationControl.h
//    Animation controller for multiple characters.
//    This is intended to be used for HW2 - COMP 259 fall 2017.
//-----------------------------------------------------------------------------
#ifndef ANIMATIONCONTROL_DOT_H
#define ANIMATIONCONTROL_DOT_H
// SKA configuration
#include <Core/SystemConfiguration.h>
// C/C++ libraries
#include <list>
#include <vector>
using namespace std;
// SKA modules
#include <Objects/Object.h>

#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

class Skeleton;
struct MotionData {
	long frame;
	float time;
	Vector3D position;
	Vector3D distance;
	Vector3D velocity;

	MotionData(Vector3D p, long f, float t)
		: position(p)
		, frame(f)
		, time(t)
	{}
};

struct FootData {
	vector<MotionData> motion;
	vector<float> sync_frames;
	int prev_frame;
	int cycles;
	float time;
	float end_time;

	FootData() : cycles(0), prev_frame(0) {
		motion.reserve(1000);
	}
};

struct AnimationControl
{
private:
	// state for basic functionality
	bool ready;
	float run_time;
	vector<Skeleton*> characters;

	FootData foot_data[3];

	// state for enhanced functionality
	float global_timewarp;
	float next_marker_time;
	float marker_time_interval;
	float max_marker_time;

public:

	string joint_names[26] = { "root", "Hips__0", "Hips__1", "Hips__2", "Spine1__0", "Spine1__1", "Spine1__2",  "Spine", "Neck", "Head", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
		"RightShoulder", "RightArm", "RightForeArm", "RightHand", "LeftUpLeg", "LeftLeg", "LeftFoot",
		"LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase" };
	void readDataToFile(string fileName, string fileContent);
	string Convert(float number);
	AnimationControl();
	virtual ~AnimationControl();

	// loadCharacters() sets up the characters and their motion control.
	// It places all the bone objects for each character into the render list,
	// so that they can be drawn by the graphics subsystem.
	void loadCharacters(string filename);

	// updateAnimation() should be called every frame to update all characters.
	// _elapsed_time should be the time (in seconds) since the last frame/update.
	bool updateAnimation(float _elapsed_time, string filename);
	bool  getData(float _elapsed_time);
	bool AnimationControl::warpTime(float _elapsed_time);
	bool isReady() { return ready; }

	float getRunTime() { return run_time; }

	// restart resets everything to time = 0
	void restart();

	void increaseGlobalTimeWarp() { global_timewarp *= 2.0f; }
	void decreaseGlobalTimeWarp() { global_timewarp /= 2.0f; }
	float getGlobalTimeWarp() { return global_timewarp; }
};

// global single instance of the animation controller
extern AnimationControl anim_ctrl;

#endif // ANIMATIONCONTROL_DOT_H