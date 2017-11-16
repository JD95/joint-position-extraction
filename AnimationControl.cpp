//-----------------------------------------------------------------------------
// HW02 - Builds with SKA Version 4.0
//-----------------------------------------------------------------------------
// AnimationControl.cpp
//    Animation controller for multiple characters.
//    This is intended to be used for HW2 - COMP 259 fall 2017.
//-----------------------------------------------------------------------------
// SKA configuration
#include <Core/SystemConfiguration.h>
// C/C++ libraries
#include <cstdio>
#include <complex>
// SKA modules
#include <Core/Utilities.h>
//#include <Animation/RawMotionController.h>
#include <Animation/AnimationException.h>
#include <Animation/Skeleton.h>
#include <DataManagement/DataManager.h>
#include <DataManagement/DataManagementException.h>
#include <DataManagement/BVH_Reader.h>
// local application
#include "AppConfig.h"
#include "AnimationControl.h"
#include "RenderLists.h"
#include "OpenMotionSequenceController.h"
#include <fstream>

// global single instance of the animation controller
AnimationControl anim_ctrl;

enum MOCAP_TYPE { BVH, AMC };
string fileText = "";
string joint_names[20] = { "Spine", "Spine1", "Neck", "Head", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
"RightShoulder", "RightArm", "RightForeArm", "RightHand", "LeftUpLeg", "LeftLeg", "LeftFoot",
"LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase" };

struct LoadSpec {
	MOCAP_TYPE mocap_type;
	float scale;
	Color color;
	string motion_file;
	string skeleton_file;
	LoadSpec(MOCAP_TYPE _mocap_type, float _scale, Color& _color, string& _motion_file, string& _skeleton_file = string(""))
		: mocap_type(_mocap_type), scale(_scale), color(_color), motion_file(_motion_file), skeleton_file(_skeleton_file) { }
};

//const short NUM_CHARACTERS = 1;
//LoadSpec load_specs[NUM_CHARACTERS] = {};

Object* createMarkerBox(Vector3D position, Color _color)
{
	ModelSpecification markerspec("Box", _color);
	markerspec.addSpec("length", "0.5");
	markerspec.addSpec("width", "0.5");
	markerspec.addSpec("height", "0.5");
	Object* marker = new Object(markerspec, position, Vector3D(0.0f, 0.0f, 0.0f));
	return marker;
}

AnimationControl::AnimationControl()
	: ready(false), run_time(0.0f),
	global_timewarp(1.0f),
	next_marker_time(0.1f), marker_time_interval(0.1f), max_marker_time(20.0f)
{ }

AnimationControl::~AnimationControl()
{
	for (unsigned short c = 0; c<characters.size(); c++)
		if (characters[c] != NULL) delete characters[c];
}

void AnimationControl::restart()
{
	render_lists.eraseErasables();
	run_time = 0;
	updateAnimation(0.0f);
	next_marker_time = marker_time_interval;
}

bool AnimationControl::updateAnimation(float _elapsed_time)
{
	//std::cout << "Elapsed time" << _elapsed_time << endl;
	if (!ready) return false;
	run_time += _elapsed_time;
	if (characters[0] != NULL) characters[0]->update(run_time);
	Vector3D start, end;
	// drop box at left toes of 1st character
	// CAREFUL - bones names are different in different skeletons
	fileText += "Frame# ";
	for (int i = 0; i < 21; i++) {
		fileText += joint_names[i] + "X " + joint_names[i] + "Y " + joint_names[i] + "Z ";
	}
	fileText += "\n";
	/*for (int i = 0; i < 21 ; i++) {
	characters[0]->getBonePositions(joint_names[i], start, end);
	fileText += start.getX + " " + start.getY + " " + start.getZ + " ";
	}*/
	readoutJointData("test.txt");
	characters[0]->getBonePositions("LeftToeBase", start, end);
	std::cout << "Getting x of ltoes: " << start.getX() << endl;

	//	std::cout << "Getting y of ltoes: " << start.getY() << endl;
	//	std::cout << "Getting z of ltoes: " << start.getZ() << endl;
	return true;
}

void readoutJointData(string fileName) {
	std::ofstream outfile(fileName);
	outfile << fileText << std::endl;
	outfile.close();
	exit(0);
}

static Skeleton* buildCharacter(
	Skeleton* _skel,
	MotionSequence* _ms,
	Color _bone_color,
	const string& _description1,
	const string& _description2,
	vector<Object*>& _render_list)
{
	if ((_skel == NULL) || (_ms == NULL)) return NULL;

	OpenMotionSequenceController* controller = new OpenMotionSequenceController(_ms);

	//! Hack. The skeleton expects a list<Object*>, we're using a vector<Object*>
	list<Object*> tmp;
	_skel->constructRenderObject(tmp, _bone_color);
	list<Object*>::iterator iter = tmp.begin();
	while (iter != tmp.end()) { _render_list.push_back(*iter); iter++; }
	//! EndOfHack.

	_skel->attachMotionController(controller);
	_skel->setDescription1(_description1.c_str());
	_skel->setDescription2(_description2.c_str());
	return _skel;
}

void AnimationControl::loadCharacters(string filename)
{
	data_manager.addFileSearchPath(AMC_MOTION_FILE_PATH);
	data_manager.addFileSearchPath(BVH_MOTION_FILE_PATH);

	Skeleton* skel = NULL;
	MotionSequence* ms = NULL;
	string descr1, descr2;
	char* filename1 = NULL;
	char* filename2 = NULL;
	Skeleton* character = NULL;
	pair<Skeleton*, MotionSequence*> read_result;
	LoadSpec load_specs[1] = { LoadSpec(BVH, 0.2f, Color(0.0f,1.0f,0.0f), string(filename)) };
	for (short c = 0; c < 1; c++)
	{
		if (load_specs[c].mocap_type == AMC)
		{
			try
			{
				filename1 = data_manager.findFile(load_specs[c].skeleton_file.c_str());
				if (filename1 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character ASF file <" << load_specs[c].skeleton_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 1A");
				}
				filename2 = data_manager.findFile(load_specs[c].motion_file.c_str());
				if (filename2 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character AMC file <" << load_specs[c].motion_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 1B");
				}
				try {
					read_result = data_manager.readASFAMC(filename1, filename2);
				}
				catch (const DataManagementException& dme)
				{
					logout << "AnimationControl::loadCharacters: Unable to load character data files. Aborting load." << endl;
					logout << "   Failure due to " << dme.msg << endl;
					throw BasicException("ABORT 1C");
				}
			}
			catch (BasicException&) {}
		}
		else if (load_specs[c].mocap_type == BVH)
		{
			try
			{
				filename1 = data_manager.findFile(load_specs[c].motion_file.c_str());
				if (filename1 == NULL)
				{
					logout << "AnimationControl::loadCharacters: Unable to find character BVH file <" << load_specs[c].motion_file << ">. Aborting load." << endl;
					throw BasicException("ABORT 2A");
				}
				try
				{
					read_result = data_manager.readBVH(filename1);
				}
				catch (const DataManagementException& dme)
				{
					logout << "AnimationControl::loadCharacters: Unable to load character data files. Aborting load." << endl;
					logout << "   Failure due to " << dme.msg << endl;
					throw BasicException("ABORT 2C");
				}
			}
			catch (BasicException&) {}
		}

		try
		{
			skel = read_result.first;
			ms = read_result.second;

			skel->scaleBoneLengths(load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TX), load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TY), load_specs[c].scale);
			ms->scaleChannel(CHANNEL_ID(0, CT_TZ), load_specs[c].scale);

			// create a character to link all the pieces together.
			descr1 = string("skeleton: ") + load_specs[c].skeleton_file;
			descr2 = string("motion: ") + load_specs[c].motion_file;

			character = buildCharacter(skel, ms, load_specs[c].color, descr1, descr2, render_lists.bones);
			if (character != NULL) characters.push_back(character);
		}
		catch (BasicException&) {}

		strDelete(filename1); filename1 = NULL;
		strDelete(filename2); filename2 = NULL;
	}

	display_data.num_characters = (short)characters.size();
	display_data.sequence_time.resize(characters.size());
	display_data.sequence_frame.resize(characters.size());

	if (characters.size() > 0) ready = true;
}


