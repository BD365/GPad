//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
//#include <cstdio>
#include "driverlog.h"

//#include <vector>
#include <thread>
#include <algorithm>


//#include <chrono>
//#include <iostream>
//#include <atlbase.h>

//#include <map>
//#include <process.h>
//#include <tchar.h>

#include "Gamepad.h"

//#if defined( _WINDOWS )
#include <windows.h>
//#endif

using namespace vr;
using namespace std;
using namespace std::chrono;

typedef struct _HMDData
{
	double	X;
	double	Y;
	double	Z;
	double	Yaw;
	double	Pitch;
	double	Roll;
} THMD, * PHMD;

typedef struct _Controller
{
	double	X;
	double	Y;
	double	Z;
	double	Yaw;
	double	Pitch;
	double	Roll;
} TController, * PController;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif


//bool ctrl = true;
Gamepad gamepad; // Gamepad instance
THMD MyHMD;
TController MyCtrl[2];

int32_t gpActiveModus = 0; //(0 left or 1 right)
int gpModus = 2; //(0 (single), 1 (dual), 2 (move))
int checkL_Thumbstick = 0; // inactive/active
int checkR_Thumbstick = 0; // inactive/active
int checkRightTrigger = 0; // inactive/active

int checkL_Shoulder = 0; // inactive/active
int checkR_Shoulder = 0; // inactive/active

int checkBack = 0; // inactive/active
int gpDualModus = 0; //(0, 1, 2: ACTIVE rotate/touchpad, lstick / rstick: rotate, lstick / rstick : touchpad)
int gpMoveModus = 0; //(0, 1: lstick:ACTIVE rotate, lstick:ACTIVE touchpad

int turn = 0;
float speed = 0.01f;

//Velocity
double FirstCtrlLastPos[3] = { 0, 0, 0 }, SecondCtrlLastPos[3] = { 0, 0, 0 };
milliseconds deltaTime;

double DegToRad(double f) {
	return f * (3.14159265358979323846 / 180);
}
int ReduceDeg(int f)
{
	if (f < -180)
		f += 360;
	else if (f > 180)
		f -= 360;

	return f;
}
inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}
inline void HmdMatrix_SetIdentity(HmdMatrix34_t* pMatrix)
{
	pMatrix->m[0][0] = 1.f;
	pMatrix->m[0][1] = 0.f;
	pMatrix->m[0][2] = 0.f;
	pMatrix->m[0][3] = 0.f;
	pMatrix->m[1][0] = 0.f;
	pMatrix->m[1][1] = 1.f;
	pMatrix->m[1][2] = 0.f;
	pMatrix->m[1][3] = 0.f;
	pMatrix->m[2][0] = 0.f;
	pMatrix->m[2][1] = 0.f;
	pMatrix->m[2][2] = 1.f;
	pMatrix->m[2][3] = 0.f;
}
// DEVICE DRIVER
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CGPadControllerDriver : public vr::ITrackedDeviceServerDriver
{
	int32_t ControllerIndex;
public:
	CGPadControllerDriver()
	{
		Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
		Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
	}
	virtual ~CGPadControllerDriver()
	{
	}
	// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
	// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
	//-----------------------------------------------------------------------------
	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
		vr::HmdQuaternion_t q;

		q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
		q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
		q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
		//if (q.x < -0, 01 || (q.x < 0, 01 && q.x > 0)) q.x = 0;
		return q;
	}
	virtual void SetControllerIndex(int32_t CtrlIndex)
	{
		ControllerIndex = CtrlIndex;
	}
	/** This is called before an HMD is returned to the application. It will always be
	* called before any display or tracking methods. Memory and processor use by the
	* ITrackedDeviceServerDriver object should be kept to a minimum until it is activated.
	* The pose listener is guaranteed to be valid until Deactivate is called, but
	* should not be used after that point. */
	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		switch (ControllerIndex)
		{
		case 0:
			Ctrl1Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t);
			break;
		case 1:
			Ctrl2Index_t = unObjectId;
			m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t);
			break;
		}

		switch (ControllerIndex)
		{
		case 0:
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL1Serial");
			break;
		case 1:
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL2Serial");
			break;
		}

		switch (ControllerIndex)
		{
		case 0:
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand);
			break;
		case 1:
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
			break;
		}

		uint64_t supportedButtons = 0xA4;
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);


		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
		//vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_LegacyInputProfile_String, "vive_controller");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");

		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2218716);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{gpad}/input/gpad_profile.json");

		bool bSetupIconUsingExternalResourceFile = false;
		if (!bSetupIconUsingExternalResourceFile)
		{
			// Setup properties directly in code.
			// Path values are of the form {drivername}\icons\some_icon_filename.png
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{gpad}/icons/gpad_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{gpad}/icons/gpad_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{gpad}/icons/gpad_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{gpad}/icons/gpad_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{gpad}/icons/gpad_status_standby.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{gpad}/icons/gpad_status_ready_low.png");
		}


		switch (ControllerIndex)
		{
		case 0:
			vr::VRDriverInput()->CreateScalarComponent(
				m_ulPropertyContainer, "/input/trackpad/x", &leftJoystickXInputHandle,
				vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
			);
			vr::VRDriverInput()->CreateScalarComponent(
				m_ulPropertyContainer, "/input/trackpad/y", &leftJoystickYInputHandle,
				vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
			);
			break;
		case 1:
			vr::VRDriverInput()->CreateScalarComponent(
				m_ulPropertyContainer, "/input/trackpad/x", &rightJoystickXInputHandle,
				vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
			);
			vr::VRDriverInput()->CreateScalarComponent(
				m_ulPropertyContainer, "/input/trackpad/y", &rightJoystickYInputHandle,
				vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided
			);
			break;
		}


		vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger/value", &TriggerInputHandle[ControllerIndex],
			vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided
		);

		//  Buttons handles
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &m_start);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_back);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compA);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &m_compB);

		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &m_compX);
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &m_compY);

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

		return VRInitError_None;

	}
	/** This is called when The VR system is switching from this Hmd being the active display
	* to another Hmd being the active display. The driver should clean whatever memory
	* and thread use it can when it is deactivated */
	virtual void Deactivate()
	{
		Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid;
		Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid;
	}
	/** Handles a request from the system to put this device into standby mode. What that means is defined per-device. */
	virtual void EnterStandby()
	{
		DriverLog("gpad: Entering Standby Mode\n");
	}
	/** Requests a component interface of the driver for device-specific functionality. The driver should return NULL
	* if the requested interface or version is not supported. */
	virtual void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}
	virtual void PowerOff()
	{
	}
	/** A VR Watchdog has made this debug request of the driver. The set of valid requests is entirely
	* up to the driver and the Watchdog to figure out, as is the format of the response. Responses that
	* exceed the length of the supplied buffer should be truncated and null terminated */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	void Center() {
		double offset = 0.0;

		if (center[0] == true) {
			MyCtrl[0].X = -0.2;
			MyCtrl[0].Y = offset;
			MyCtrl[0].Z = 0;
			MyCtrl[0].Yaw = 0;
			MyCtrl[0].Pitch = 0;
			MyCtrl[0].Roll = 0;
			center[0] = false;
		}
		if (center[1] == true) {
			MyCtrl[1].X = +0.2;
			MyCtrl[1].Y = offset;
			MyCtrl[1].Z = 0;
			MyCtrl[1].Yaw = 0;
			MyCtrl[1].Pitch = 0;
			MyCtrl[1].Roll = 0;
			center[1] = false;
		}
	}

	// ------------------------------------
	// Tracking Methods
	// ------------------------------------
	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };
		//TrackedDevicePose_t trackedDevicePose;
		//VRControllerState_t controllerState;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);

		vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
		vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, devicePoses, vr::k_unMaxTrackedDeviceCount);
		vr::TrackedDevicePose_t tracker = devicePoses[0];
		HmdQuaternion_t rot = GetRotation(tracker.mDeviceToAbsoluteTracking);

		HmdQuaternion_t r;

		float a = (tracker.mDeviceToAbsoluteTracking.m[0][0]);
		float b = tracker.mDeviceToAbsoluteTracking.m[1][0];
		float c = (tracker.mDeviceToAbsoluteTracking.m[2][0]);

		if (cnt == 100) {
			
			DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, a, b, c);
		}

		if (center[0] == true || center[1] == true) {
			Center();
			pose.qRotation = rot;
			pose.vecDriverFromHeadTranslation[2] = 3.5;
			pose.vecPosition[0] = c / 4;
			pose.vecPosition[2] = a * -1 / 4;
		}

		if (ControllerIndex == 0) {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[0].X;
			pose.vecDriverFromHeadTranslation[2] = MyCtrl[0].Z;

			r.w = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.x = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.y = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.z = sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);

			pose.qRotation = rot;


			pose.qDriverFromHeadRotation = r;

			pose.vecPosition[0] = c / 4 + MyHMD.X;
			pose.vecPosition[1] = MyCtrl[0].Y;
			pose.vecPosition[2] = a * -1 / 4 + MyHMD.Y;// +0.25;

			//Velocity, right?
			//pose.vecVelocity[0] = (pose.vecPosition[0] - FirstCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; // div 3 - ghosting fix, there are right ways to remove ghosting?
			//pose.vecVelocity[1] = (pose.vecPosition[1] - FirstCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			//pose.vecVelocity[2] = (pose.vecPosition[2] - FirstCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			//FirstCtrlLastPos[0] = pose.vecPosition[0];
			//FirstCtrlLastPos[1] = pose.vecPosition[1];
			//FirstCtrlLastPos[2] = pose.vecPosition[2];

			if (cnt == 100) {
				//DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			}
		}
		else {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[1].X;
			pose.vecDriverFromHeadTranslation[2] = MyCtrl[1].Z;


			r.w = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) + sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.x = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) - sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.y = cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5) + sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5);
			r.z = sin(DegToRad(MyCtrl[1].Yaw) * 0.5) * cos(DegToRad(MyCtrl[1].Roll) * 0.5) * cos(DegToRad(MyCtrl[1].Pitch) * 0.5) - cos(DegToRad(MyCtrl[1].Yaw) * 0.5) * sin(DegToRad(MyCtrl[1].Roll) * 0.5) * sin(DegToRad(MyCtrl[1].Pitch) * 0.5);

			pose.qRotation = rot;

			pose.qDriverFromHeadRotation = r;

			//Velocity
			pose.vecVelocity[0] = (pose.vecPosition[0] - SecondCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1);
			pose.vecVelocity[1] = (pose.vecPosition[1] - SecondCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1);
			pose.vecVelocity[2] = (pose.vecPosition[2] - SecondCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1);
			SecondCtrlLastPos[0] = pose.vecPosition[0];
			SecondCtrlLastPos[1] = pose.vecPosition[1];
			SecondCtrlLastPos[2] = pose.vecPosition[2];

			pose.vecPosition[0] = (c ) / 4 + MyHMD.X / 4;
			pose.vecPosition[1] = MyCtrl[1].Y + MyHMD.Z;
			pose.vecPosition[2] = (a ) * -1 / 4 + MyHMD.Y;// +0.25;

			if (cnt == 100) {
				//DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			}
		}

		cnt++;
		if (cnt == 101) {
			cnt = 0;
		}

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;


		//pose.willDriftInYaw = false;
		//pose.shouldApplyHeadModel = false;

		if (doDebug) {
			DriverLog("%d - HMD: x:%f y:%f z:%f / rot.x:%f rot.y:%f rot.z:%f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			doDebug = false;
		}
		return pose;
	}
	void ProcessHandles(int32_t index) {

		// Sticks
		if (float LftStX = gamepad.LeftStick_X())
		{
			if (LftStX > 0.3 || LftStX < -0.3) {

				switch (gpModus) {
				case 0:
					MyCtrl[gpActiveModus].Pitch = MyCtrl[gpActiveModus].Pitch - LftStX * 0.5;
					break;
				case 1:
					switch (gpDualModus) {
					case 0:
						MyCtrl[gpActiveModus].Pitch = MyCtrl[gpActiveModus].Pitch - LftStX * 0.5;
						break;
					case 1:
						MyCtrl[0].Pitch = MyCtrl[0].Pitch - LftStX * 0.5;
						break;
					case 2:
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
						break;
					}
					break;
				case 2:
					if (gpMoveModus == 0) {
						MyCtrl[gpActiveModus].Pitch = MyCtrl[gpActiveModus].Pitch - LftStX * 0.5;
					}
					else if (gpMoveModus == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
					}
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 1)) {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
			}
			//DriverLog("%d %d - LftStX: %f\n", ControllerIndex, index, LftStX);
			//vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
		}
		else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 1)) {
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
		}


		if (float LftStY = gamepad.LeftStick_Y())
		{
			if (LftStY > 0.3 || LftStY < -0.3) {
				switch (gpModus) {
				case 0:
					MyCtrl[gpActiveModus].Roll = MyCtrl[gpActiveModus].Roll + LftStY * 0.5;
					break;
				case 1:
					switch (gpDualModus) {
					case 0:
						MyCtrl[gpActiveModus].Roll = MyCtrl[gpActiveModus].Roll + LftStY * 0.5;
						break;
					case 1:
						MyCtrl[0].Roll = MyCtrl[0].Roll + LftStY * 0.5;
						break;
					case 2:
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
						break;
					}
					break;
				case 2:
					if (gpMoveModus == 0) {
						MyCtrl[gpActiveModus].Roll = MyCtrl[gpActiveModus].Roll + LftStY * 0.5;
					}
					else if (gpMoveModus == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
					}
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 1)) {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
			}
			//DriverLog("%d %f - LftStY: %f\n", ControllerIndex, index, LftStY);
			//vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
		}
		else if ((gpModus==1 && gpDualModus ==2)|| (gpModus==2 && gpMoveModus==1)){
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
		}

		if (float RghStX = gamepad.RightStick_X())
		{
			if (RghStX > 0.3 || RghStX < -0.3) {
				switch (gpModus) {
				case 0:
					if (gpActiveModus == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
					}
					else {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, RghStX, 0);
					}
					break;
				case 1:
					switch (gpDualModus) {
					case 0:
						if (gpActiveModus == 1) {
							vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
						}
						else {
							vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, RghStX, 0);
						}
						break;
					case 1:
						MyCtrl[1].Pitch = MyCtrl[1].Pitch - RghStX * 0.5;
						break;
					case 2:
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
						break;
					}
					break;
				case 2:
					// TODO MOVE HMD
					DriverLog("%d %d - RghStX: %d\n", ControllerIndex, index, turn);
					
					switch (turn) {
					case 0:
						MyHMD.X = MyHMD.X + RghStX * speed;
						break;
					case 90:
						MyHMD.Y = MyHMD.Y - RghStX * speed;
						break;
					case -180:
					case 180:
						MyHMD.X = MyHMD.X - RghStX * speed;
						break;
					case -90:
						MyHMD.Y = MyHMD.Y + RghStX * speed;
						break;
					}
					//MyCtrl[0].X = MyCtrl[0].X + RghStX * 0.005;
					//MyCtrl[1].X = MyCtrl[1].X + RghStX * 0.005;
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 0) || (gpModus == 1 && gpDualModus == 0)) {
				if (gpActiveModus == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
				}
			}
			else if (gpModus == 1 && gpDualModus == 2) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
			}
			else if (gpModus == 2) {
				// TODO MOVE HMD - STOP
				//MyHMD.X = 0;
			}

			//DriverLog("%d %f - RghStX: %f\n", ControllerIndex, index, RghStX);
			//vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
		}
		else if ((gpModus == 0) || (gpModus == 1 && gpDualModus == 0)) {
			if (gpActiveModus == 1) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
			}
		}
		else if (gpModus == 1 && gpDualModus == 2) {
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
		}
		else if (gpModus == 2) {
			// TODO MOVE HMD - STOP
			//MyHMD.X = 0;
		}

		if (float RghStY = gamepad.RightStick_Y())
		{
			if (RghStY > 0.3 || RghStY < -0.3) {
				switch (gpModus) {
				case 0:
					if (gpActiveModus == 1) {
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
					}
					else {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, RghStY, 0);
					}
					break;
				case 1:
					switch (gpDualModus) {
					case 0:
						if (gpActiveModus == 1) {
							vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
						}
						else {
							vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, RghStY, 0);
						}
						break;
					case 1:
						MyCtrl[1].Roll = MyCtrl[1].Roll + RghStY * 0.5;
						break;
					case 2:
						vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
						break;
					}
					break;
				case 2:
					// TODO MOVE HMD
					switch (turn) {
					case 0:
						MyHMD.Y = MyHMD.Y - RghStY * speed;
						break;
					case 90:
						MyHMD.X = MyHMD.X - RghStY * speed;
						break;
					case -180:
					case 180:
						MyHMD.Y = MyHMD.Y + RghStY * speed;
						break;
					case -90:
						MyHMD.X = MyHMD.X + RghStY * speed;
						break;

					}
					//MyCtrl[0].Z = MyCtrl[0].Z - RghStY * 0.005;
					//MyCtrl[1].Z = MyCtrl[1].Z - RghStY * 0.005;
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 0) || (gpModus == 1 && gpDualModus == 0)) {
				if (gpActiveModus == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
				}
			}
			else if (gpModus == 1 && gpDualModus == 2) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
			}
			else if (gpModus == 2) {
				// TODO MOVE HMD - STOP
				//MyHMD.Y = 0;

			}
			//DriverLog("%d %f - RghStY: %f\n", ControllerIndex, index, RghStY);
			//vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
		}
		else if ((gpModus == 0) || (gpModus == 1 && gpDualModus == 0)) {
			if (gpActiveModus == 1) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
			}
		}
		else if (gpModus == 1 && gpDualModus == 2) {
			vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
		}
		else if (gpModus == 2) {
			// TODO MOVE HMD - STOP
			//MyHMD.Y = 0;
		}

		//DriverLog("%d %d - INDEX: %d\n", ControllerIndex, index, index);
		if (gpActiveModus == index) {
			ProcesUnchangedButtons(index);
		}
	}

	void ProcesUnchangedButtons(int32_t index) {
		// Dpad
		if (gamepad.GetButtonPressed(xButtons.DPad_Left))
		{
			switch (gpModus) {
			case 0:
				MyCtrl[gpActiveModus].X = MyCtrl[gpActiveModus].X - 0.005;
				break;
			case 1:
				MyCtrl[0].X = MyCtrl[0].X - 0.005;
				MyCtrl[1].X = MyCtrl[1].X - 0.005;
				break;
			case 2:
				MyCtrl[gpActiveModus].X = MyCtrl[gpActiveModus].X - 0.005;
				break;
			default:
				break;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Right))
		{
			switch (gpModus) {
			case 0:
				MyCtrl[gpActiveModus].X = MyCtrl[gpActiveModus].X + 0.005;
				break;
			case 1:
				MyCtrl[0].X = MyCtrl[0].X + 0.005;
				MyCtrl[1].X = MyCtrl[1].X + 0.005;
				break;
			case 2:
				MyCtrl[gpActiveModus].X = MyCtrl[gpActiveModus].X + 0.005;
				break;
			default:
				break;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Up))
		{
			switch (gpModus) {
			case 0:
				MyCtrl[gpActiveModus].Z = MyCtrl[gpActiveModus].Z - 0.005;
				break;
			case 1:
				MyCtrl[0].Z = MyCtrl[0].Z - 0.005;
				MyCtrl[1].Z = MyCtrl[1].Z - 0.005;
				break;
			case 2:
				MyCtrl[gpActiveModus].Z = MyCtrl[gpActiveModus].Z - 0.005;
				break;
			default:
				break;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.DPad_Down))
		{
			switch (gpModus) {
			case 0:
				MyCtrl[gpActiveModus].Z = MyCtrl[gpActiveModus].Z + 0.005;
				break;
			case 1:
				MyCtrl[0].Z = MyCtrl[0].Z + 0.005;
				MyCtrl[1].Z = MyCtrl[1].Z + 0.005;
				break;
			case 2:
				MyCtrl[gpActiveModus].Z = MyCtrl[gpActiveModus].Z + 0.005;
				break;
			default:
				break;
			}
		}

		// OK FROM HERE
		// Bummers
		if (gamepad.GetButtonPressed(xButtons.L_Shoulder))
		{
			if (gpModus == 0) {
				MyCtrl[gpActiveModus].Y = MyCtrl[gpActiveModus].Y + 0.002;
			}
			else if (gpModus == 1) {
				MyCtrl[0].Y = MyCtrl[0].Y + 0.002;
				MyCtrl[1].Y = MyCtrl[1].Y + 0.002;

			}
			else if (gpModus == 2 && checkL_Shoulder==0) {
				turn = ReduceDeg(turn + 90);
				checkL_Shoulder = 1;
			}

			//DriverLog("%d %f - L_Shoulder: %f\n", ControllerIndex, turn, "L_Shoulder");
			//DriverLog("%d %f - L_Shoulder: %f\n", ControllerIndex, turn, "L_Shoulder");
		}
		else {
			checkL_Shoulder = 0;
		}

		if (gamepad.GetButtonPressed(xButtons.R_Shoulder))
		{
			if (gpModus == 0) {
				MyCtrl[gpActiveModus].Y = MyCtrl[gpActiveModus].Y - 0.002;
			}
			else if (gpModus == 1) {
				MyCtrl[0].Y = MyCtrl[0].Y - 0.002;
				MyCtrl[1].Y = MyCtrl[1].Y - 0.002;

			}
			else if (gpModus == 2 && checkR_Shoulder==0) {
				turn = ReduceDeg(turn - 90);
				checkR_Shoulder = 1;
			}
			//DriverLog("%d %f - R_Shoulder: %f\n", ControllerIndex, index, "R_Shoulder");
		}
		else {
			checkR_Shoulder = 0;
		}

		// RIGHT STICK BUTTON
		if (gamepad.GetButtonDown(xButtons.R_Thumbstick))
		{
			if (checkR_Thumbstick == 0) {
				switch (gpModus) {
				case 0:
					center[gpActiveModus] = true;
					break;
				case 1:
					center[0] = true;
					center[1] = true;
					break;
				case 2:
					DriverLog("%d %d - xButtons.R_Thumbstick: speed (before)=%f\n", ControllerIndex, index, speed);
					if (speed == 0.01f) {
						speed = 0.05f;
					}
					else {
						speed = 0.01f;
					}
					DriverLog("%d %d - xButtons.R_Thumbstick: speed (after)=%f\n", ControllerIndex, index, speed);
					break;
				default:
					center[gpActiveModus] = true;
					break;
				}
				checkR_Thumbstick = 1;
			}
			//DriverLog("%d %d - xButtons.R_Thumbstick: center[0]=%d, center[1]=%d \n", ControllerIndex, index, center[0], center[1]);
		}
		else {
			checkR_Thumbstick = 0;
		}


		if (gamepad.GetButtonPressed(xButtons.Back))
		{
			if (checkBack == 0) {
			
				//DriverLog("%d %f - Back: %f\n", ControllerIndex, index, "Back");
				switch (gpModus) {
					case 0: // single
						vr::VRDriverInput()->UpdateBooleanComponent(m_back, 1, 0);
						break;
					case 1: // dual
						if (gpDualModus == 2) gpDualModus = 0; else gpDualModus++;
						break;
					case 2: // move
						if (gpMoveModus == 0) gpMoveModus = 1; else gpMoveModus = 0;
						break;
					default:
						break;
				}
				//DriverLog("%d %d - Back: gpModus = %d, gpDualModus = %d gpMoveModus = %d\n", ControllerIndex, index, gpModus, gpDualModus, gpMoveModus);
				checkBack = 1;
			}
		}
		else {
			checkBack = 0;
			vr::VRDriverInput()->UpdateBooleanComponent(m_back, 0, 0);
		}
		// TRIGGERS
		// LEFT STICK BUTTON
		if (gamepad.GetButtonPressed(xButtons.L_Thumbstick))
		{
			//DriverLog("%d %d - xButtons.L_Thumbstick (pressed) gpActiveModus, checkL_Thumbstick: %d %d\n", ControllerIndex, index, gpActiveModus, checkL_Thumbstick);
			if (checkL_Thumbstick == 0) {
				if (gpActiveModus == 0) gpActiveModus = 1; else gpActiveModus = 0;
				checkL_Thumbstick = 1;
				//DriverLog("%d %d - xButtons.L_Thumbstick (changed) gpActiveModus, checkL_Thumbstick: %d %d\n", ControllerIndex, index, gpActiveModus, checkL_Thumbstick);
			}
			//DriverLog("%d %d - xButtons.L_Thumbstick (definitive) gpActiveModus: %d\n", ControllerIndex, index, gpActiveModus);
		}
		else {
			checkL_Thumbstick = 0;
		}

		if (float RghT = gamepad.RightTrigger())
		{
			if (RghT > 0.6) {
				
				if (checkRightTrigger == 0) {
					//DriverLog("%d %f - RghT: %f\n", ControllerIndex, index, RghT);
					if (gpModus == 2) gpModus = 0; else gpModus++;
					//DriverLog("%d %f - RghT: Modus: %d\n", ControllerIndex, index, gpModus);
					checkRightTrigger = 1;
				} 
			}
			else {
				//DriverLog("%d %f - RghT (0.0): %f\n", ControllerIndex, index, RghT);
				//DriverLog("%d %f - RghT: Modus(4): %d\n", ControllerIndex, index, gpModus);
				checkRightTrigger = 0;
			}
		} else {
			//DriverLog("%d %f - RghT (0.0): %f\n", ControllerIndex, index, RghT);
			//DriverLog("%d %f - RghT: Modus(5): %f\n", ControllerIndex, index, gpModus);
			checkRightTrigger = 0;
		}

		if (float LftT = gamepad.LeftTrigger())
		{
			if (LftT > 0.6) {
				//DriverLog("%d %f - LftT: %f\n", ControllerIndex, index, LftT);

				vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[index], LftT, 0);
			}
			else {
				//DriverLog("%d %f - LftT (0.0): %f\n", ControllerIndex, index, LftT);

				vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[index], 0.0, 0);
			}
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[index], 0.0, 0);
		}

		// BUTTONS
		if (gamepad.GetButtonPressed(xButtons.A))
		{
			//DriverLog("%d %f - xButtons.A: %f\n", ControllerIndex, index, "xButtons.A");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 1, 0);
		}
		else {
			//DriverLog("%d %f - xButtons.A (0.0): %f\n", ControllerIndex, index, "xButtons.A");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 0, 0);
		}
		if (gamepad.GetButtonPressed(xButtons.B))
		{
			//DriverLog("%d %f - xButtons.B: %f\n", ControllerIndex, index, "xButtons.B");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 1, 0);
		}
		else {
			//DriverLog("%d %f - xButtons.B (0.0): %f\n", ControllerIndex, index, "xButtons.B");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 0, 0);
		}

		if (gamepad.GetButtonDown(xButtons.Y))
		{
			//DriverLog("%d %f - xButtons.Y: %f\n", ControllerIndex, index, "xButtons.Y");
			doDebug = true;
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
		}
		else {
			//DriverLog("%d %f - xButtons.Y (0.0): %f\n", ControllerIndex, index, "xButtons.Y");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
		}

		if (gamepad.GetButtonPressed(xButtons.X))
		{
			//DriverLog("%d %f - xButtons.X: %f\n", ControllerIndex, index, "xButtons.X");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 1, 0);
		}
		else {
			//DriverLog("%d %f - xButtons.X (0.0): %f\n", ControllerIndex, index, "xButtons.X");
			vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 0, 0);
		}
		// Start/select
		if (gamepad.GetButtonPressed(xButtons.Start))
		{
			//DriverLog("%d %f - xButtons.Start: %f\n", ControllerIndex, index, "xButtons.Start");
			vr::VRDriverInput()->UpdateBooleanComponent(m_start, 1, 0);
		}
		else {
			//DriverLog("%d %f - xButtons.Start (0.0): %f\n", ControllerIndex, index, "xButtons.Start");
			vr::VRDriverInput()->UpdateBooleanComponent(m_start, 0, 0);
		}
	}

	// DEFAULT //
	void RunFrame()
	{
		switch (ControllerIndex)
		{
		case 0:
			ProcessHandles(0);
			if (Ctrl1Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl1Index_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		case 1:
			ProcessHandles(1);
			if (Ctrl2Index_t != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl2Index_t, GetPose(), sizeof(DriverPose_t));
			}
			break;
		}
	}
	// DEFAULT //
	std::string GetSerialNumber() const {

		switch (ControllerIndex)
		{
		case 0:
			return "CTRL1Serial";
			break;
		case 1:
			return "CTRL2Serial";
			break;
		default:
			return "CTRL1Serial";
			break;
		}
	}

	// DEFAULT //
	void ProcessEvent(const vr::VREvent_t& vrEvent)
	{
		switch (vrEvent.eventType)
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
			{
				// This is where you would send a signal to your hardware to trigger actual haptic feedback
				DriverLog("BUZZ!\n");
			}
		}
		break;
		}
	}

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t;
	vr::TrackedDeviceIndex_t Ctrl2Index_t;

	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;

	vr::VRInputComponentHandle_t TriggerInputHandle[2];

	vr::VRInputComponentHandle_t rightJoystickXInputHandle;
	vr::VRInputComponentHandle_t rightJoystickYInputHandle;
	vr::VRInputComponentHandle_t leftJoystickXInputHandle;
	vr::VRInputComponentHandle_t leftJoystickYInputHandle;

	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t m_compA;
	vr::VRInputComponentHandle_t m_compB;
	vr::VRInputComponentHandle_t m_compX;
	vr::VRInputComponentHandle_t m_compY;

	vr::VRInputComponentHandle_t m_start;
	vr::VRInputComponentHandle_t m_back;

	//int swap = 0;
	//int act[2] = { 0, 0 };
	//int check = 0;
	//int chck[2] = { 0,0 };

	bool center[2] = { true, true };

	bool doDebug = false;
	int cnt = 0;
};
//-----------------------------------------------------------------------------
// Purpose: HMD driver (freetrack!)
//-----------------------------------------------------------------------------

// keys for use with the settings API
static const char* const k_pch_OpenTrack_Section = "opentrack";
static const char* const k_pch_OpenTrack_SerialNumber_String = "serialNumber";
static const char* const k_pch_OpenTrack_ModelNumber_String = "modelNumber";
static const char* const k_pch_OpenTrack_WindowX_Int32 = "windowX";
static const char* const k_pch_OpenTrack_WindowY_Int32 = "windowY";
static const char* const k_pch_OpenTrack_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_OpenTrack_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_OpenTrack_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_OpenTrack_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_OpenTrack_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_OpenTrack_DisplayFrequency_Float = "displayFrequency";

static const char* const k_pch_OpenTrack_DistortionK1_Float = "DistortionK1";
static const char* const k_pch_OpenTrack_DistortionK2_Float = "DistortionK2";
static const char* const k_pch_OpenTrack_ZoomWidth_Float = "ZoomWidth";
static const char* const k_pch_OpenTrack_ZoomHeight_Float = "ZoomHeight";
static const char* const k_pch_OpenTrack_FOV_Float = "FOV";
static const char* const k_pch_OpenTrack_DistanceBetweenEyes_Int32 = "DistanceBetweenEyes";
static const char* const k_pch_OpenTrack_ScreenOffsetX_Int32 = "ScreenOffsetX";
static const char* const k_pch_OpenTrack_DebugMode_Bool = "DebugMode";
static const char* const k_pch_OpenTrack_CrouchPressKey_String = "CrouchPressKey";
static const char* const k_pch_OpenTrack_CrouchOffset_Float = "CrouchOffset";


#define FREETRACK_HEAP "FT_SharedMem"
#define FREETRACK_MUTEX "FT_Mutext"

/* only 6 headpose floats and the data id are filled -sh */
typedef struct FTData__ {
	uint32_t DataID;
	int32_t CamWidth;
	int32_t CamHeight;
	/* virtual pose */
	float  Yaw;   /* positive yaw to the left */
	float  Pitch; /* positive pitch up */
	float  Roll;  /* positive roll to the left */
	float  X;
	float  Y;
	float  Z;
	/* raw pose with no smoothing, sensitivity, response curve etc. */
	float  RawYaw;
	float  RawPitch;
	float  RawRoll;
	float  RawX;
	float  RawY;
	float  RawZ;
	/* raw points, sorted by Y, origin top left corner */
	float  X1;
	float  Y1;
	float  X2;
	float  Y2;
	float  X3;
	float  Y3;
	float  X4;
	float  Y4;
} volatile FTData;

typedef struct FTHeap__ {
	FTData data;
	int32_t GameID;
	union
	{
		unsigned char table[8];
		int32_t table_ints[2];
	};
	int32_t GameID2;
} volatile FTHeap;

static HANDLE hFTMemMap = 0;
static FTHeap* ipc_heap = 0;
static HANDLE ipc_mutex = 0;

FTData* FreeTrack;
bool HMDConnected = false;
std::thread* pFTthread = NULL;

inline vr::HmdQuaternion_t EulerAngleToQuaternion(double Yaw, double Pitch, double Roll)
{
	vr::HmdQuaternion_t q;
	// Abbreviations for the various angular functions
	double cy = cos(Yaw * 0.5);
	double sy = sin(Yaw * 0.5);
	double cp = cos(Pitch * 0.5);
	double sp = sin(Pitch * 0.5);
	double cr = cos(Roll * 0.5);
	double sr = sin(Roll * 0.5);

	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

//FreeTrack implementation from OpenTrack (https://github.com/opentrack/opentrack/tree/unstable/freetrackclient)
static BOOL impl_create_mapping(void)
{
	if (ipc_heap != NULL)
		return TRUE;

	hFTMemMap = CreateFileMappingA(INVALID_HANDLE_VALUE,
		NULL,
		PAGE_READWRITE,
		0,
		sizeof(FTHeap),
		(LPCSTR)FREETRACK_HEAP);

	if (hFTMemMap == NULL)
		return (ipc_heap = NULL), FALSE;

	ipc_heap = (FTHeap*)MapViewOfFile(hFTMemMap, FILE_MAP_WRITE, 0, 0, sizeof(FTHeap));
	ipc_mutex = CreateMutexA(NULL, FALSE, FREETRACK_MUTEX);

	return TRUE;
}

void FTRead()
{
	while (HMDConnected) {
		if (ipc_mutex && WaitForSingleObject(ipc_mutex, 16) == WAIT_OBJECT_0) {
			memcpy(&FreeTrack, &ipc_heap, sizeof(FreeTrack));
			if (ipc_heap->data.DataID > (1 << 29))
				ipc_heap->data.DataID = 0;
			ReleaseMutex(ipc_mutex);
		}
	}
}

int KeyNameToKeyCode(std::string KeyName) {
	std::transform(KeyName.begin(), KeyName.end(), KeyName.begin(), ::toupper);

	if (KeyName == "NONE") return 0;

	else if (KeyName == "MOUSE-LEFT-BTN") return VK_LBUTTON;
	else if (KeyName == "MOUSE-RIGHT-BTN") return VK_RBUTTON;
	else if (KeyName == "MOUSE-MIDDLE-BTN") return VK_MBUTTON;
	else if (KeyName == "MOUSE-SIDE1-BTN") return VK_XBUTTON1;
	else if (KeyName == "MOUSE-SIDE2-BTN") return VK_XBUTTON2;

	else if (KeyName == "ESCAPE") return VK_ESCAPE;
	else if (KeyName == "F1") return VK_F1;
	else if (KeyName == "F2") return VK_F2;
	else if (KeyName == "F3") return VK_F3;
	else if (KeyName == "F4") return VK_F4;
	else if (KeyName == "F5") return VK_F5;
	else if (KeyName == "F6") return VK_F6;
	else if (KeyName == "F7") return VK_F7;
	else if (KeyName == "F8") return VK_F8;
	else if (KeyName == "F9") return VK_F9;
	else if (KeyName == "F10") return VK_F10;
	else if (KeyName == "F11") return VK_F11;
	else if (KeyName == "F12") return VK_F12;

	else if (KeyName == "~") return 192;
	else if (KeyName == "1") return '1';
	else if (KeyName == "2") return '2';
	else if (KeyName == "3") return '3';
	else if (KeyName == "4") return '4';
	else if (KeyName == "5") return '5';
	else if (KeyName == "6") return '6';
	else if (KeyName == "7") return '7';
	else if (KeyName == "8") return '8';
	else if (KeyName == "9") return '9';
	else if (KeyName == "0") return '0';
	else if (KeyName == "-") return 189;
	else if (KeyName == "=") return 187;

	else if (KeyName == "TAB") return VK_TAB;
	else if (KeyName == "CAPS-LOCK") return VK_CAPITAL;
	else if (KeyName == "SHIFT") return VK_SHIFT;
	else if (KeyName == "CTRL") return VK_CONTROL;
	else if (KeyName == "WIN") return VK_LWIN;
	else if (KeyName == "ALT") return VK_MENU;
	else if (KeyName == "SPACE") return VK_SPACE;
	else if (KeyName == "ENTER") return VK_RETURN;
	else if (KeyName == "BACKSPACE") return VK_BACK;

	else if (KeyName == "Q") return 'Q';
	else if (KeyName == "W") return 'W';
	else if (KeyName == "E") return 'E';
	else if (KeyName == "R") return 'R';
	else if (KeyName == "T") return 'T';
	else if (KeyName == "Y") return 'Y';
	else if (KeyName == "U") return 'U';
	else if (KeyName == "I") return 'I';
	else if (KeyName == "O") return 'O';
	else if (KeyName == "P") return 'P';
	else if (KeyName == "[") return '[';
	else if (KeyName == "]") return ']';
	else if (KeyName == "A") return 'A';
	else if (KeyName == "S") return 'S';
	else if (KeyName == "D") return 'D';
	else if (KeyName == "F") return 'F';
	else if (KeyName == "G") return 'G';
	else if (KeyName == "H") return 'H';
	else if (KeyName == "J") return 'J';
	else if (KeyName == "K") return 'K';
	else if (KeyName == "L") return 'L';
	else if (KeyName == ";") return 186;
	else if (KeyName == "'") return 222;
	else if (KeyName == "\\") return 220;
	else if (KeyName == "Z") return 'Z';
	else if (KeyName == "X") return 'X';
	else if (KeyName == "C") return 'C';
	else if (KeyName == "V") return 'V';
	else if (KeyName == "B") return 'B';
	else if (KeyName == "N") return 'N';
	else if (KeyName == "M") return 'M';
	else if (KeyName == "<") return 188;
	else if (KeyName == ">") return 190;
	else if (KeyName == "?") return 191;

	else if (KeyName == "PRINTSCREEN") return VK_SNAPSHOT;
	else if (KeyName == "SCROLL-LOCK") return VK_SCROLL;
	else if (KeyName == "PAUSE") return VK_PAUSE;
	else if (KeyName == "INSERT") return VK_INSERT;
	else if (KeyName == "HOME") return VK_HOME;
	else if (KeyName == "PAGE-UP") return VK_NEXT;
	else if (KeyName == "DELETE") return VK_DELETE;
	else if (KeyName == "END") return VK_END;
	else if (KeyName == "PAGE-DOWN") return VK_PRIOR;

	else if (KeyName == "UP") return VK_UP;
	else if (KeyName == "DOWN") return VK_DOWN;
	else if (KeyName == "LEFT") return VK_LEFT;
	else if (KeyName == "RIGHT") return VK_RIGHT;

	else if (KeyName == "NUM-LOCK") return VK_NUMLOCK;
	else if (KeyName == "NUMPAD0") return VK_NUMPAD0;
	else if (KeyName == "NUMPAD1") return VK_NUMPAD1;
	else if (KeyName == "NUMPAD2") return VK_NUMPAD2;
	else if (KeyName == "NUMPAD3") return VK_NUMPAD3;
	else if (KeyName == "NUMPAD4") return VK_NUMPAD4;
	else if (KeyName == "NUMPAD5") return VK_NUMPAD5;
	else if (KeyName == "NUMPAD6") return VK_NUMPAD6;
	else if (KeyName == "NUMPAD7") return VK_NUMPAD7;
	else if (KeyName == "NUMPAD8") return VK_NUMPAD8;
	else if (KeyName == "NUMPAD9") return VK_NUMPAD9;

	else if (KeyName == "NUMPAD-DIVIDE") return VK_DIVIDE;
	else if (KeyName == "NUMPAD-MULTIPLY") return VK_MULTIPLY;
	else if (KeyName == "NUMPAD-MINUS") return VK_SUBTRACT;
	else if (KeyName == "NUMPAD-PLUS") return VK_ADD;
	else if (KeyName == "NUMPAD-DEL") return VK_DECIMAL;

	else return 0;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CGPadDeviceDriver : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	CGPadDeviceDriver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		//DriverLog( "Using settings values\n" );
		m_flIPD = vr::VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);

		char buf[1024];
		vr::VRSettings()->GetString(k_pch_OpenTrack_Section, k_pch_OpenTrack_SerialNumber_String, buf, sizeof(buf));
		m_sSerialNumber = buf;

		vr::VRSettings()->GetString(k_pch_OpenTrack_Section, k_pch_OpenTrack_ModelNumber_String, buf, sizeof(buf));
		m_sModelNumber = buf;

		m_nWindowX = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_WindowX_Int32);
		m_nWindowY = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_WindowY_Int32);
		m_nWindowWidth = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_WindowWidth_Int32);
		m_nWindowHeight = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_WindowHeight_Int32);
		m_nRenderWidth = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_RenderWidth_Int32);
		m_nRenderHeight = vr::VRSettings()->GetInt32(k_pch_OpenTrack_Section, k_pch_OpenTrack_RenderHeight_Int32);
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_SecondsFromVsyncToPhotons_Float);
		m_flDisplayFrequency = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_DisplayFrequency_Float);

		m_fDistortionK1 = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_DistortionK1_Float);
		m_fDistortionK2 = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_DistortionK2_Float);
		m_fZoomWidth = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_ZoomWidth_Float);
		m_fZoomHeight = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_ZoomHeight_Float);
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_FOV_Float) * 3.14159265358979323846 / 180); //radians
		m_nDistanceBetweenEyes = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_DistanceBetweenEyes_Int32);
		m_nScreenOffsetX = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_ScreenOffsetX_Int32);
		m_bDebugMode = vr::VRSettings()->GetBool(k_pch_OpenTrack_Section, k_pch_OpenTrack_DebugMode_Bool);

		vr::VRSettings()->GetString(k_pch_OpenTrack_Section, k_pch_OpenTrack_CrouchPressKey_String, buf, sizeof(buf));
		m_crouchPressKey = KeyNameToKeyCode(buf);

		m_crouchOffset = vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_CrouchOffset_Float);

		/*DriverLog( "driver_null: Serial Number: %s\n", m_sSerialNumber.c_str() );
		DriverLog( "driver_null: Model Number: %s\n", m_sModelNumber.c_str() );
		DriverLog( "driver_null: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
		DriverLog( "driver_null: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
		DriverLog( "driver_null: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
		DriverLog( "driver_null: Display Frequency: %f\n", m_flDisplayFrequency );
		DriverLog( "driver_null: IPD: %f\n", m_flIPD );*/
	}

	virtual ~CGPadDeviceDriver()
	{
	}


	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);


		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		//Debug mode activate Windowed Mode (borderless fullscreen), lock to 30 FPS 
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode);

		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
		{
			return (vr::IVRDisplayComponent*)this;
		}

		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnX = m_nWindowX;
		*pnY = m_nWindowY;
		*pnWidth = m_nWindowWidth;
		*pnHeight = m_nWindowHeight;
	}

	virtual bool IsDisplayOnDesktop()
	{
		return true;
	}

	virtual bool IsDisplayRealDisplay()
	{
		if (m_nWindowX == 0 && m_nWindowY == 0)
			return false;
		else
			return true; //Support working on extended display
	}

	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnWidth = m_nRenderWidth;
		*pnHeight = m_nRenderHeight;
	}

	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
	{
		*pnY = m_nScreenOffsetX;
		*pnWidth = m_nWindowWidth / 2;
		*pnHeight = m_nWindowHeight;

		if (eEye == Eye_Left)
		{
			*pnX = m_nDistanceBetweenEyes;
		}
		else
		{
			*pnX = (m_nWindowWidth / 2) - m_nDistanceBetweenEyes;
		}
	}

	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
	{
		*pfLeft = -m_fFOV;
		*pfRight = m_fFOV;
		*pfTop = -m_fFOV;
		*pfBottom = m_fFOV;
	}

	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV)
	{
		DistortionCoordinates_t coordinates;

		//Distortion for lens implementation from https://github.com/HelenXR/openvr_survivor/blob/master/src/head_mount_display_device.cc
		float hX;
		float hY;
		double rr;
		double r2;
		double theta;

		rr = sqrt((fU - 0.5f) * (fU - 0.5f) + (fV - 0.5f) * (fV - 0.5f));
		r2 = rr * (1 + m_fDistortionK1 * (rr * rr) + m_fDistortionK2 * (rr * rr * rr * rr));
		theta = atan2(fU - 0.5f, fV - 0.5f);
		hX = sin(theta) * r2 * m_fZoomWidth;
		hY = cos(theta) * r2 * m_fZoomHeight;

		coordinates.rfBlue[0] = hX + 0.5f;
		coordinates.rfBlue[1] = hY + 0.5f;
		coordinates.rfGreen[0] = hX + 0.5f;
		coordinates.rfGreen[1] = hY + 0.5f;
		coordinates.rfRed[0] = hX + 0.5f;
		coordinates.rfRed[1] = hY + 0.5f;

		return coordinates;
	}

	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };
		if (HMDConnected) {
			pose.poseIsValid = true;
			pose.result = TrackingResult_Running_OK;
			pose.deviceIsConnected = true;
		}
		else {
			pose.poseIsValid = false;
			pose.result = TrackingResult_Uninitialized;
			pose.deviceIsConnected = false;
		}

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		if (HMDConnected) {
			//Set head tracking rotation

			//if (cnt == 100) {
				//DriverLog("%d - HMD: %f %f %f / %f %f %f\n", "GPAD", pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], FreeTrack->Roll, -FreeTrack->Yaw, FreeTrack->Pitch);
			//}
			pose.qRotation = EulerAngleToQuaternion(FreeTrack->Roll, -FreeTrack->Yaw+DegToRad(turn), FreeTrack->Pitch);
			//if (cnt == 100) {
				//DriverLog("%d - HMD: %f %f %f\n", "TURN", turn, DegToRad(turn));
			//}
			//turn = 0;

			//Set position tracking
			pose.vecPosition[0] = FreeTrack->X * 0.001 + MyHMD.X; //millimeters to meters
			pose.vecPosition[1] = FreeTrack->Z * 0.001 + MyHMD.Z; //millimeters to meters 
			if ((GetAsyncKeyState(m_crouchPressKey) & 0x8000) != 0)
				pose.vecPosition[1] -= m_crouchOffset;
			pose.vecPosition[2] = FreeTrack->Y * 0.001 + MyHMD.Y; //millimeters to meters 
		}

		cnt++;
		if (cnt == 101) {
			cnt = 0;
		}

		return pose;
	}

	void RunFrame()
	{
		// In a real driver, this should happen from some pose tracking thread.
		// The RunFrame interval is unspecified and can be very irregular if some other
		// driver blocks it for some periodic task.
		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
		{
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
		}
	}

	std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;
	std::string m_sModelNumber;

	int32_t m_nWindowX;
	int32_t m_nWindowY;
	int32_t m_nWindowWidth;
	int32_t m_nWindowHeight;
	int32_t m_nRenderWidth;
	int32_t m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons;
	float m_flDisplayFrequency;
	float m_flIPD;

	float m_fDistortionK1;
	float m_fDistortionK2;
	float m_fZoomWidth;
	float m_fZoomHeight;
	float m_fFOV;
	int32_t m_nDistanceBetweenEyes;
	int32_t m_nScreenOffsetX;
	bool m_bDebugMode;

	int32_t m_crouchPressKey;
	float m_crouchOffset;

	int cnt=0;
};


//-----------------------------------------------------------------------------
// Purpose: PROVIDER
//-----------------------------------------------------------------------------
class GPadProvider : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
	virtual void RunFrame();
	virtual void Cleanup();

private:
	CGPadDeviceDriver* m_pNullHmdLatest = nullptr;
	CGPadControllerDriver* m_pController = nullptr;
	CGPadControllerDriver* m_pController2 = nullptr;
};

static GPadProvider g_serverDriverNull;

EVRInitError GPadProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	DriverLog("GPad: Creating Controller Driver\n");
	DriverLog("GPad: Post- Creating Controller Driver\n");

	if (impl_create_mapping() == false) {
		HMDConnected = false;
	}
	else {
		HMDConnected = true;
		pFTthread = new std::thread(FTRead);
	}

	m_pNullHmdLatest = new CGPadDeviceDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest);

	m_pController = new CGPadControllerDriver();
	m_pController->SetControllerIndex(0);
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

	m_pController2 = new CGPadControllerDriver();
	m_pController2->SetControllerIndex(1);
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController2->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController2);

	return VRInitError_None;
}

void GPadProvider::Cleanup()
{
	CleanupDriverLog();
	delete m_pController;
	m_pController = NULL;
	delete m_pController2;
	m_pController2 = NULL;

	if (HMDConnected) {
		HMDConnected = false;
		if (pFTthread) {
			pFTthread->join();
			delete pFTthread;
			pFTthread = nullptr;
		}
	}
	//CleanupDriverLog();
	delete m_pNullHmdLatest;
	m_pNullHmdLatest = NULL;
}

void GPadProvider::RunFrame()
{
	if (m_pNullHmdLatest)
	{
		m_pNullHmdLatest->RunFrame();
	}

	if (gamepad.Connected()) {

		gamepad.Update();
		if (m_pController)
		{
			m_pController->RunFrame();
		}
		if (m_pController2)
		{
			m_pController2->RunFrame();
		}

		vr::VREvent_t vrEvent;
		while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
		{
			if (m_pController)
			{
				m_pController->ProcessEvent(vrEvent);
			}
			if (m_pController2)
			{
				m_pController2->ProcessEvent(vrEvent);
			}
		}
		gamepad.Refresh();
	}

}

//-----------------------------------------------------------------------------
// Purpose: FACTORY
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}
	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}

/*
void ProcessHandlesOld(int32_t index) {

	// Sticks

	if (float LftStX = gamepad.LeftStick_X())
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Pitch = MyCtrl[Active].Pitch - LftStX *0.5;
			}
		}
		else {
			if (act[0] == 0) {
				MyCtrl[0].Pitch = MyCtrl[0].Pitch - LftStX *0.5;
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
			}
		}
	}
	else {
		//vr::VRDriverInput()->UpdateScalarComponent:mmµ                                                                                                                                               µ(leftJoystickXInputHandle, 0.0, 0);

	}

	if (float LftStY = gamepad.LeftStick_Y())
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Roll = MyCtrl[Active].Roll + LftStY * 0.5;
			}
		}
		else {
			if (act[0] == 0) {
				MyCtrl[0].Roll = MyCtrl[0].Roll + LftStY * 0.5;
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
			}
		}
	}
	else {
		//vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0.0, 0);
	}

	if (float RghStX = gamepad.RightStick_X())
	{
		if (swap == 0) {
			if (index == Active) {
				if (Active == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, RghStX, 0);
				}
			}
		}
		else {
			if (act[1] == 0) {
				MyCtrl[1].Pitch = MyCtrl[1].Pitch - RghStX * 0.5;
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, RghStX, 0);
			}
		}
	}
	else {
		if (index == Active) {
			if (Active == 1) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickXInputHandle, 0, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
			}
		}
	}

	if (float RghStY = gamepad.RightStick_Y())
	{
		if (swap == 0) {
			if (index == Active) {
				if (Active == 1) {
					vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
				}
				else {
					vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, RghStY, 0);
				}
			}
		}
		else {
			if (act[1] == 0) {
				MyCtrl[1].Roll = MyCtrl[1].Roll + RghStY * 0.5;
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, RghStY, 0);
			}
		}
	}
	else {
		if (index == Active) {
			if (Active == 1) {
				vr::VRDriverInput()->UpdateScalarComponent(rightJoystickYInputHandle, 0, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
			}
		}
	}

	// Dpad

	if (gamepad.GetButtonPressed(xButtons.DPad_Left))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].X = MyCtrl[Active].X - 0.005;
			}
		}
		else {
			MyCtrl[0].X = MyCtrl[0].X - 0.005;
			MyCtrl[1].X = MyCtrl[1].X - 0.005;
		}

	}

	if (gamepad.GetButtonPressed(xButtons.DPad_Right))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].X = MyCtrl[Active].X + 0.005;
			}
		}
		else {
			MyCtrl[0].X = MyCtrl[0].X + 0.005;
			MyCtrl[1].X = MyCtrl[1].X + 0.005;
		}
	}

	if (gamepad.GetButtonPressed(xButtons.DPad_Up))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Z = MyCtrl[Active].Z - 0.005;
			}
		}
		else {
			MyCtrl[0].Z = MyCtrl[0].Z - 0.005;
			MyCtrl[1].Z = MyCtrl[1].Z - 0.005;
		}
	}

	if (gamepad.GetButtonPressed(xButtons.DPad_Down))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Z = MyCtrl[Active].Z + 0.005;
			}
		}
		else {
			MyCtrl[0].Z = MyCtrl[0].Z + 0.005;
			MyCtrl[1].Z = MyCtrl[1].Z + 0.005;
		}

	}

	// Bummers

	if (gamepad.GetButtonPressed(xButtons.L_Shoulder))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Y = MyCtrl[Active].Y + 0.002;
			}
		}
		else {
			MyCtrl[0].Y = MyCtrl[0].Y + 0.002;
			MyCtrl[1].Y = MyCtrl[1].Y + 0.002;
		}

	}

	if (gamepad.GetButtonPressed(xButtons.R_Shoulder))
	{
		if (swap == 0) {
			if (index == Active) {
				MyCtrl[Active].Y = MyCtrl[Active].Y - 0.002;
			}
		}
		else {
			MyCtrl[0].Y = MyCtrl[0].Y - 0.002;
			MyCtrl[1].Y = MyCtrl[1].Y - 0.002;
		}

	}

	if (gamepad.GetButtonPressed(xButtons.Back))
	{
		if (index == Active) {
			if (swap == 0) {
				center[Active] = true;
			}
			else {
				center[0] = true;
				center[1] = true;
			}
		}
	}

	// Singles

	if (index == Active) {

		// Triggers

		if (float LftT = gamepad.LeftTrigger())
		{
			if (LftT > 0.6) {
				vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[Active], LftT, 0);
			}
			else {
				vr::VRDriverInput()->UpdateScalarComponent(TriggerInputHandle[Active], 0.0, 0);
			}
		}

		if (float RghT = gamepad.RightTrigger())
		{
			if (RghT > 0.6) {
				if (swap == 0 && check == 0) {
					DriverLog("swap true");
					swap = 1;
					check = 1;
				}
				if (swap == 1 && check == 0) {
					DriverLog("swap false");
					swap = 0;
					check = 1;
				}
			}
			else {
				check = 0;
			}
		}
		else {
			if (check == 1) {
				swap = 0;
			}
		}

		// Buttons

		if (gamepad.GetButtonPressed(xButtons.A))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateBooleanComponent(m_compA, 0, 0);
		}

		if (gamepad.GetButtonPressed(xButtons.B))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateBooleanComponent(m_compB, 0, 0);
		}

		if (gamepad.GetButtonPressed(xButtons.Y))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateBooleanComponent(m_compY, 0, 0);
		}

		if (gamepad.GetButtonPressed(xButtons.X))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateBooleanComponent(m_compX, 0, 0);
		}

		// Start/select

		if (gamepad.GetButtonPressed(xButtons.Start))
		{
			vr::VRDriverInput()->UpdateBooleanComponent(m_start, 1, 0);
		}
		else {
			vr::VRDriverInput()->UpdateBooleanComponent(m_start, 0, 0);
		}

		// Sticks click

		if (gamepad.GetButtonPressed(xButtons.L_Thumbstick))
		{
			if (ctrl == true && swap == 0) {
				Active = 0;
				DriverLog("%d - ctrl\n", ControllerIndex);
				ctrl = false;
			}
			if (chck[0] == 0 && swap == 1) {
				switch (act[0])
				{
				case 0:
					act[0] = 1;
					break;
				case 1:
					act[0] = 0;
					break;
				}
				chck[0] = 1;
			}
			else {
				chck[0] = 1;
			}

		}
		else {
			if (chck[0] == 1 && swap == 1) {
				chck[0] = 0;
			}
		}

		if (gamepad.GetButtonPressed(xButtons.R_Thumbstick))
		{
			if (ctrl == false && swap == 0) {
				Active = 1;
				DriverLog("%d - ctrl\n", ControllerIndex);
				ctrl = true;
			}
			if (chck[1] == 0 && swap == 1) {
				switch (act[1])
				{
				case 0:
					act[1] = 1;
					break;
				case 1:
					act[1] = 0;
					break;
				}
				chck[1] = 1;
			}
			else {
				chck[1] = 1;
			}

		}
		else {
			if (chck[1] == 1 && swap == 1) {
				chck[1] = 0;
			}
		}
	}
}
*/

/*
void GetHMDData(__out THMD* HMD)
{
	HMD->X = 0;
	HMD->Y = 0;
	HMD->Z = 0;

	HMD->Yaw = 0;
	HMD->Pitch = 0;
	HMD->Roll = 0;
}

inline vr::HmdQuaternion_t EulerAngleToQuaternion(double Yaw, double Pitch, double Roll)
{
	vr::HmdQuaternion_t q;
	// Abbreviations for the various angular functions
	double cy = cos(Yaw * 0.5);
	double sy = sin(Yaw * 0.5);
	double cp = cos(Pitch * 0.5);
	double sp = sin(Pitch * 0.5);
	double cr = cos(Roll * 0.5);
	double sr = sin(Roll * 0.5);

	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}


// keys for use with the settings API
static const char* const k_pch_Sample_Section = "driver_gpad";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

class CGPadDeviceDriver : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	CGPadDeviceDriver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		DriverLog( "Using settings values\n" );
		m_flIPD = vr::VRSettings()->GetFloat( k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float );

		char buf[1024];
		//vr::VRSettings()->GetString( k_pch_Sample_Section, k_pch_Sample_SerialNumber_String, buf, sizeof( buf ) );
		//m_sSerialNumber = buf;

		vr::VRSettings()->GetString( k_pch_Sample_Section, k_pch_Sample_ModelNumber_String, buf, sizeof( buf ) );
		m_sModelNumber = buf;

		m_nWindowX = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_WindowX_Int32 );
		m_nWindowY = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_WindowY_Int32 );
		m_nWindowWidth = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_WindowWidth_Int32 );
		m_nWindowHeight = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_WindowHeight_Int32 );
		m_nRenderWidth = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_RenderWidth_Int32 );
		m_nRenderHeight = vr::VRSettings()->GetInt32( k_pch_Sample_Section, k_pch_Sample_RenderHeight_Int32 );
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float );
		m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float );

		//DriverLog( "driver_null: Serial Number: %s\n", m_sSerialNumber.c_str() );
		DriverLog( "driver_null: Model Number: %s\n", m_sModelNumber.c_str() );
		DriverLog( "driver_null: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight );
		DriverLog( "driver_null: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight );
		DriverLog( "driver_null: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons );
		DriverLog( "driver_null: Display Frequency: %f\n", m_flDisplayFrequency );
		DriverLog( "driver_null: IPD: %f\n", m_flIPD );

	}

	virtual ~CGPadDeviceDriver()
	{
	}


	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);


		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

		// avoid "not fullscreen" warnings from vrmonitor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		// Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
		// Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
		// In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
		//
		// The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
		// Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
		// An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
		// since the driver may have specialized icons based on those device class names.
		//
		// An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
		//
		// Please refer to "bin\drivers\sample\resources\driver.vrresources" which contains this sample configuration.
		//
		// "Alias" is a reserved key and specifies chaining to another json block.
		//
		// In this sample configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
		//
		// "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
		//
		// Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
		// Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
		// Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".
		//
		bool bSetupIconUsingExternalResourceFile = true;
		if (!bSetupIconUsingExternalResourceFile)
		{
			// Setup properties directly in code.
			// Path values are of the form {drivername}\icons\some_icon_filename.png
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{sample}/icons/headset_sample_status_off.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{sample}/icons/headset_sample_status_searching.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{sample}/icons/headset_sample_status_searching_alert.gif");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{sample}/icons/headset_sample_status_ready.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{sample}/icons/headset_sample_status_ready_alert.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{sample}/icons/headset_sample_status_error.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{sample}/icons/headset_sample_status_standby.png");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{sample}/icons/headset_sample_status_ready_low.png");
		}

		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
		{
			return (vr::IVRDisplayComponent*)this;
		}

		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	// debug request from a client
virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnX = m_nWindowX;
	*pnY = m_nWindowY;
	*pnWidth = m_nWindowWidth;
	*pnHeight = m_nWindowHeight;
}

virtual bool IsDisplayOnDesktop()
{
	return true;
}

virtual bool IsDisplayRealDisplay()
{
	return false;
}

virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnWidth = m_nRenderWidth;
	*pnHeight = m_nRenderHeight;
}

virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight)
{
	*pnY = 0;
	*pnWidth = m_nWindowWidth / 2;
	*pnHeight = m_nWindowHeight;

	if (eEye == Eye_Left)
	{
		*pnX = 0;
	}
	else
	{
		*pnX = m_nWindowWidth / 2;
	}
}

virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom)
{
	*pfLeft = -1.0;
	*pfRight = 1.0;
	*pfTop = -1.0;
	*pfBottom = 1.0;
}

virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV)
{
	DistortionCoordinates_t coordinates;
	coordinates.rfBlue[0] = fU;
	coordinates.rfBlue[1] = fV;
	coordinates.rfGreen[0] = fU;
	coordinates.rfGreen[1] = fV;
	coordinates.rfRed[0] = fU;
	coordinates.rfRed[1] = fV;
	return coordinates;
}

virtual DriverPose_t GetPose()
{

	DriverPose_t pose = { 0 };
	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
	pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

	//GetHMDData(&MyHMD);
	//Set head tracking rotation
	pose.qRotation = EulerAngleToQuaternion(DegToRad(0), DegToRad(-0), DegToRad(-90));

	//Set head position tracking
	pose.vecPosition[0] = MyHMD.X;
	pose.vecPosition[1] = MyHMD.Z;
	pose.vecPosition[2] = MyHMD.Y;

	if (cnt == 100) {
		DriverLog("%d - HMD: %f %f %f / %f %f %f\n", "HMD-device", pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], DegToRad(0), DegToRad(-0), DegToRad(-90));
	}

	cnt++;
	if (cnt == 101) {
		cnt = 0;
	}

	return pose;
}


void RunFrame()
{
	// In a real driver, this should happen from some pose tracking thread.
	// The RunFrame interval is unspecified and can be very irregular if some other
	// driver blocks it for some periodic task.
	if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
	{
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
	}
}

std::string GetSerialNumber() const { return "HMDSerial";; }

private:
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	//std::string m_sSerialNumber;
	std::string m_sModelNumber;

	int32_t m_nWindowX;
	int32_t m_nWindowY;
	int32_t m_nWindowWidth;
	int32_t m_nWindowHeight;
	int32_t m_nRenderWidth;
	int32_t m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons;
	float m_flDisplayFrequency;
	float m_flIPD;

	int cnt;
};
*/