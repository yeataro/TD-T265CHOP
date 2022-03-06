/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "T265CHOP.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>

//#include <sstream>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <exception> 

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo *info)
{
	// Always set this to CHOPCPlusPlusAPIVersion.
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	// The opType is the unique name for this CHOP. It must start with a 
	// capital A-Z character, and all the following characters must lower case
	// or numbers (a-z, 0-9)
	info->customOPInfo.opType->setString("T265");

	// The opLabel is the text that will show up in the OP Create Dialog
	info->customOPInfo.opLabel->setString("T265");

	// Information about the author of this OP
	info->customOPInfo.authorName->setString("yeataro");
	info->customOPInfo.authorEmail->setString("yeataro@email.com");

	// This CHOP can work with 0 inputs
	info->customOPInfo.minInputs = 0;

	// It can accept up to 1 input though, which changes it's behavior
	info->customOPInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new T265CHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (T265CHOP*)instance;
}

};

void
T265CHOP::setupDevice() {
}


T265CHOP::T265CHOP(const OP_NodeInfo* info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	//myOffset = 0.0;

	try{
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	pipe.start(cfg);
	myError = nullptr;
	}
	catch (const rs2::error& e)
	{
		std::stringstream err;
		err << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what();
		//std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		myError = err.str().c_str();
	}
	catch (const std::exception& e)
	{
		//std::cerr << e.what() << std::endl;
		myError = e.what();
	}
}

T265CHOP::~T265CHOP()
{

}

void
T265CHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = bool(inputs->getParInt("Alwayscook"));

	// Note: To disable timeslicing you'll need to turn this off, as well as ensure that
	// getOutputInfo() returns true, and likely also set the info->numSamples to how many
	// samples you want to generate for this CHOP. Otherwise it'll take on length of the
	// input CHOP, which may be timesliced.
	ginfo->timeslice = bool(inputs->getParInt("Timeslice"));

	ginfo->inputMatchIndex = 0;
}

bool
T265CHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
{
	// If there is an input connected, we are going to match it's channel names etc
	// otherwise we'll specify our own.

	info->numChannels = 3+4+4+2;
	double	 Rate = inputs->getParDouble("Samplerate");
	info->sampleRate = Rate;

	return true;
	
}

void
T265CHOP::getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
{
	switch (index) {
	case 0:
		name->setString("tx");
		break;
	case 1:
		name->setString("ty");
		break;
	case 2:
		name->setString("tz");
		break;
	case 3:
		name->setString("qx");
		break;
	case 4:
		name->setString("qy");
		break;
	case 5:
		name->setString("qz");
		break;
	case 6:
		name->setString("qw");
		break;
	case 7:
		name->setString("timestamp_ms");
		break;
	case 8:
		name->setString("timestamp_sec");
		break;
	case 9:
		name->setString("timestamp_min");
		break;
	case 10:
		name->setString("timestamp_hour");
		break;
	case 11:
		name->setString("mapper_confidence");
		break;
	case 12:
		name->setString("tracker_confidence");
		break;
	}
}

void
T265CHOP::execute(CHOP_Output* output,
							  const OP_Inputs* inputs,
							  void* reserved)
{
	myExecuteCount++;
	
	//double	 scale = inputs->getParDouble("Scale");

	// Wait for the next set of frames from the camera
	auto frames = pipe.wait_for_frames();

	auto timestamp = frames.get_timestamp();
	auto timestamp_ms = fmod(timestamp,1000);
	auto timestamp_sec = int(fmod(timestamp/1000, 60));
	auto timestamp_min = int(fmod(timestamp/1000/60, 60));
	auto timestamp_hour= int(fmod(timestamp /1000 /60/60, 24));

	// Get a frame from the pose stream
	auto f = frames.first_or_default(RS2_STREAM_POSE);
	// Cast the frame to pose_frame and get its data
	auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	//auto TDtime = std::chrono::system_clock::now();
	auto mapper_confidence = pose_data.mapper_confidence;
	auto tracker_confidence = pose_data.tracker_confidence;


		// We know the first CHOP has the same number of channels
		// because we returned false from getOutputInfo. 

		//inputs->enablePar("Speed", 0);	// not used
		//inputs->enablePar("Reset", 0);	// not used
		//inputs->enablePar("Shape", 0);	// not used

		//const OP_CHOPInput	*cinput = inputs->getInputCHOP(0);

	
		for (int j = 0; j < output->numSamples; j++)
		{
			output->channels[0][j] = pose_data.translation.x;
			output->channels[1][j] = pose_data.translation.y;
			output->channels[2][j] = pose_data.translation.z;
			output->channels[3][j] = pose_data.rotation.x;
			output->channels[4][j] = pose_data.rotation.y;
			output->channels[5][j] = pose_data.rotation.z;
			output->channels[6][j] = pose_data.rotation.w;
			output->channels[7][j] = timestamp_ms;
			output->channels[8][j] = timestamp_sec;
			output->channels[9][j] = timestamp_min;
			output->channels[10][j] = timestamp_hour;
			output->channels[11][j] = mapper_confidence;
			output->channels[12][j] = tracker_confidence;
			
	}
}

int32_t
T265CHOP::getNumInfoCHOPChans(void * reserved1)
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 2;
}

void
T265CHOP::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan,
										void* reserved1)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name->setString("executeCount");
		chan->value = (float)myExecuteCount;
	}

	if (index == 1)
	{
		chan->name->setString("offset");
		chan->value = (float)myOffset;
	}
}

bool		
T265CHOP::getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
T265CHOP::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries, 
										void* reserved1)
{
	char tempBuffer[4096];

	if (index == 0)
	{
		// Set the value for the first column
		entries->values[0]->setString("executeCount");

		// Set the value for the second column
#ifdef _WIN32
		sprintf_s(tempBuffer, "%d", myExecuteCount);
#else // macOS
		snprintf(tempBuffer, sizeof(tempBuffer), "%d", myExecuteCount);
#endif
		entries->values[1]->setString(tempBuffer);
	}

	if (index == 1)
	{
		// Set the value for the first column
		entries->values[0]->setString("offset");

		// Set the value for the second column
#ifdef _WIN32
		sprintf_s(tempBuffer, "%g", myOffset);
#else // macOS
		snprintf(tempBuffer, sizeof(tempBuffer), "%g", myOffset);
#endif
		entries->values[1]->setString( tempBuffer);
	}
}

void
T265CHOP::setupParameters(OP_ParameterManager* manager, void *reserved1)
{
	{
		OP_NumericParameter	np;

		np.name = "Active";
		np.label = "Active";
		np.defaultValues[0] = 0;

		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	/* scale
	{
		OP_NumericParameter	np;

		np.name = "Scale";
		np.label = "Scale";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] =  10.0;
		
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}
	*/
	{
		OP_NumericParameter	np;

		np.name = "Alwayscook";
		np.label = "Always cook";
		np.defaultValues[0] = 1;

		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter	np;

		np.name = "Timeslice";
		np.label = "Timeslice";
		np.defaultValues[0] = 1;

		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	/*shape
	{
		OP_StringParameter	sp;

		sp.name = "Shape";
		sp.label = "Shape";

		sp.defaultValue = "Sine";

		const char *names[] = { "Sine", "Square", "Ramp" };
		const char *labels[] = { "Sine", "Square", "Ramp" };

		OP_ParAppendResult res = manager->appendMenu(sp, 3, names, labels);
		assert(res == OP_ParAppendResult::Success);
	}
		*/
	{
		OP_StringParameter	sp;

		sp.name = "Samplerate";
		sp.label = "Sample Rate";

		sp.defaultValue = "me.time.rate";

		OP_ParAppendResult res = manager->appendPython(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		
		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

}

void 
T265CHOP::pulsePressed(const char* name, void* reserved1)
{
	if (!strcmp(name, "Reset"))
	{
		myOffset = 0.0;
	}
}

void
T265CHOP::getWarningString(OP_String* warning, void* reserved1)
{
	warning->setString(myWarning);
}

void
T265CHOP::getErrorString(OP_String* error, void* reserved1)
{
	error->setString(myError);
}

