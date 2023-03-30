//============ Copyright (c) Valve Corporation, All rights reserved. ============
#include <linalg.h>
#include <openvr_driver.h>
#include "driverlog.h"
#include <thread>
#include <algorithm>
#include "Gamepad.h"

//#if defined( _WINDOWS )
#include <windows.h>
//#endif
using namespace linalg::aliases;
using namespace vr;
using namespace std;
using namespace std::chrono;

typedef struct _HMDController
{
	float	rotX;
	float	rotY;
	float	moveX;
	float	moveY;
	float	moveZup;
	float	moveZdown;
	int speed = 1;
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


int checkBack = 0; // inactive/active
int gpDualModus = 0; //(0, 1, 2: ACTIVE rotate/touchpad, lstick / rstick: rotate, lstick / rstick : touchpad)
int gpMoveModus = 0; //(0, 1, 2: lstick:HMD move, ACTIVE rotate, lstick:ACTIVE touchpad

//Velocity
double FirstCtrlLastPos[3] = { 0, 0, 0 }, SecondCtrlLastPos[3] = { 0, 0, 0 };
milliseconds deltaTime;

double DegToRad(double f) {
	return f * (3.14159265358979323846 / 180);
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
static inline vr::DriverPose_t MakeDefaultPose(bool connected = true, bool tracking = true) {
	vr::DriverPose_t out_pose = { 0 };

	out_pose.deviceIsConnected = connected;
	out_pose.poseIsValid = tracking;
	out_pose.result = tracking ? vr::ETrackingResult::TrackingResult_Running_OK : vr::ETrackingResult::TrackingResult_Running_OutOfRange;
	out_pose.willDriftInYaw = false;
	out_pose.shouldApplyHeadModel = false;
	out_pose.qDriverFromHeadRotation.w = out_pose.qWorldFromDriverRotation.w = out_pose.qRotation.w = 1.0;

	return out_pose;
}

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
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_FOV_Float) * (float)3.14159265358979323846 / 180); //radians
		m_nDistanceBetweenEyes = (int32_t)vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_DistanceBetweenEyes_Int32);
		m_nScreenOffsetX = (int32_t)vr::VRSettings()->GetFloat(k_pch_OpenTrack_Section, k_pch_OpenTrack_ScreenOffsetX_Int32);
		m_bDebugMode = vr::VRSettings()->GetBool(k_pch_OpenTrack_Section, k_pch_OpenTrack_DebugMode_Bool);
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
		hX = (float)(sin(theta) * r2 * m_fZoomWidth);
		hY = (float)(cos(theta) * r2 * m_fZoomHeight);

		coordinates.rfBlue[0] = hX + 0.5f;
		coordinates.rfBlue[1] = hY + 0.5f;
		coordinates.rfGreen[0] = hX + 0.5f;
		coordinates.rfGreen[1] = hY + 0.5f;
		coordinates.rfRed[0] = hX + 0.5f;
		coordinates.rfRed[1] = hY + 0.5f;

		return coordinates;
	}

	void Update()
	{
		auto pose = MakeDefaultPose();
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

			//Set position tracking
			this->pos_x_ += FreeTrack->X * 0.001f;// + (float)MyHMD.X;//millimeters to meters
			this->pos_y_ += FreeTrack->Z * 0.001f;// + (float)MyHMD.Z;//millimeters to meters
			this->pos_z_ += FreeTrack->Y * 0.001f;// + (float)MyHMD.Y;//millimeters to meters

			float delta_seconds = deltaTime.count() / 1000.0f;

			// Get orientation
			rot_y_ += MyHMD.rotX * delta_seconds;
			rot_x_ += MyHMD.rotY * delta_seconds;
			rot_x_ = std::fmax(rot_x_, -3.14159f / 2);
			rot_x_ = std::fmin(rot_x_, 3.14159f / 2);
			//vr::HmdQuaternion_t freetrackQuaternion = EulerAngleToQuaternion(FreeTrack->Roll, rot_y_-FreeTrack->Yaw + DegToRad(turn), rot_x_+FreeTrack->Pitch);

			linalg::vec<float, 4> y_quat{ 0, std::sinf((rot_y_ - FreeTrack->Yaw  /* + (float)DegToRad(turn)*/) / 2), 0, std::cosf((rot_y_ - FreeTrack->Yaw /*+ (float)DegToRad(turn)*/) / 2) };
			linalg::vec<float, 4> x_quat{ std::sinf((rot_x_ + FreeTrack->Pitch) / 2), 0, 0, std::cosf((rot_x_ + FreeTrack->Pitch) / 2) };
			linalg::vec<float, 4> pose_rot = linalg::qmul(y_quat, x_quat);

			pose.qRotation.w = (float)pose_rot.w;
			pose.qRotation.x = (float)pose_rot.x;
			pose.qRotation.y = (float)pose_rot.y;
			pose.qRotation.z = (float)pose_rot.z;

			// Update position based on rotation
			linalg::vec<float, 3> forward_vec{ MyHMD.moveY , 0, 0};
			linalg::vec<float, 3> right_vec{ 0, 0, MyHMD.moveX};
			linalg::vec<float, 3> up_vec{ 0, MyHMD.moveZup + MyHMD.moveZdown, 0 }; //ok
			linalg::vec<float, 3> final_dir = forward_vec + right_vec + up_vec; //ok
			if (linalg::length(final_dir) > 0.01) {
				final_dir = linalg::normalize(final_dir) * (float)delta_seconds * MyHMD.speed * 2;
				final_dir = linalg::qrot(pose_rot, final_dir);
				this->pos_x_ += final_dir.x;
				this->pos_y_ += final_dir.y;
				this->pos_z_ += final_dir.z;
			}

			//// TODO 
			pose.vecPosition[0] = (float)this->pos_x_;
			pose.vecPosition[1] = (float)this->pos_y_;
			pose.vecPosition[2] = (float)this->pos_z_;

			// Post pose
			//GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, pose, sizeof(DriverPose_t));
		}

		last_pose_ = pose;

		//cnt++;
		//if (cnt == 101) {
		//	cnt = 0;
		//}
	}

	virtual DriverPose_t GetPose()
	{
		return last_pose_;

	}

	void RunFrame()
	{
		// In a real driver, this should happen from some pose tracking thread.
		// The RunFrame interval is unspecified and can be very irregular if some other
		// driver blocks it for some periodic task.
		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
		{
			Update();
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

	DriverPose_t last_pose_;
	float pos_x_ = 0, pos_y_ = 0, pos_z_ = 0;
	float rot_y_ = 0, rot_x_ = 0;
	vr::HmdQuaternion_t orientationQuaternion;

	int cnt = 0;


};


// DEVICE DRIVER
//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CGPadControllerDriver : public vr::ITrackedDeviceServerDriver
{
	int32_t ControllerIndex;
	CGPadDeviceDriver* HMDDriver;
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

	void SetHMDDriver(CGPadDeviceDriver* m_pNullHmdLatest) {
		HMDDriver = m_pNullHmdLatest;
	}
	/** This is called before an HMD is returned to the application. It will always be
	* called before any display or tracking methods. Memory and processor use by the
	* ITrackedDeviceServerDriver object should be kept to a minimum until it is activated.
	* The pose listener is guaranteed to be valid until Deactivate is called, but
	* should not be used after that point. */
	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{

		// Setup inputs and outputs
			//"/input/a": "A Button",
			//"/input/b" : "B Button",
			//"/input/joystick" : "Joystick",
			//"/input/skeleton" : "Skeleton",
			//"/output/haptic" : "Haptic"
		// NOT USED
		/*
		"/input/a/click"
		"/input/a/touch"
		"/input/b/click"
		"/input/b/touch"
		"/input/trigger/touch"

		"/input/grip/touch"
		"/input/grip/value"
		"/input/grip/force"
		"/input/system/touch"

		"/input/joystick/click"
		"/input/joystick/touch"
		"/input/joystick/x"
		"/input/joystick/y"
		*/

		// Setup inputs and outputs

		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/click", &this->a_button_click_component_);
		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/touch", &this->a_button_touch_component_);

		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/click", &this->b_button_click_component_);
		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/touch", &this->b_button_touch_component_);

		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/touch", &this->trigger_touch_component_);
		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/touch", &this->grip_touch_component_);
		//GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
		//GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/force", &this->grip_force_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/touch", &this->system_touch_component_);


		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/click", &this->joystick_click_component_);
		//GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/touch", &this->joystick_touch_component_);
		//GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
		//GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

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

		// Setup properties directly in code.
		// Path values are of the form {drivername}\icons\some_icon_filename.png
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{gpad}/icons/gpad_status_off.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{gpad}/icons/gpad_status_ready.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{gpad}/icons/gpad_status_ready_alert.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{gpad}/icons/gpad_status_error.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{gpad}/icons/gpad_status_standby.png");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{gpad}/icons/gpad_status_ready_low.png");
		//vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{gpad}/icons/gpad_status_standby.png");
		//vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{gpad}/icons/gpad_status_standby.png");

		/*
	"/input/trackpad": "Joystick",
	"/input/trigger": "Trigger",
	"/input/application_menu/click": "Start Button",
	"/input/system/click": "Back Button",
	"/input/grip/click": "A Button",
	"/input/trigger/click": "B Button",
	"/input/trackpad/click": "X Button",
	"/input/trackpad/touch": "Y Button"
		*/

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
			MyCtrl[0].Z = -0.25;
			MyCtrl[0].Yaw = 0;
			MyCtrl[0].Pitch = 0;
			MyCtrl[0].Roll = 0;
			center[0] = false;
		}
		if (center[1] == true) {
			MyCtrl[1].X = +0.2;
			MyCtrl[1].Y = offset;
			MyCtrl[1].Z = -0.25;
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

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);

		vr::DriverPose_t hmd_pose = { 0 };
		if (HMDDriver != nullptr) {
			if (cnt == 100) {
				DriverLog("%d - HMD: hmd_pose\n", ControllerIndex);
			}
			hmd_pose = HMDDriver->GetPose();
		}
		else {
			hmd_pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
			hmd_pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
			hmd_pose.qRotation = HmdQuaternion_Init(0, 0, 0, 0);
		}
		HmdQuaternion_t rot = hmd_pose.qRotation;

		HmdQuaternion_t r;

		if (center[0] == true || center[1] == true) {
			Center();
			pose.qRotation = rot;
			pose.vecDriverFromHeadTranslation[2] = 3.5;
			pose.vecPosition[0] = hmd_pose.vecPosition[0];
			pose.vecPosition[1] = hmd_pose.vecPosition[1];
			pose.vecPosition[2] = hmd_pose.vecPosition[2];
		}

		if (ControllerIndex == 0) {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[0].X;
			pose.vecDriverFromHeadTranslation[1] = MyCtrl[0].Y;
			pose.vecDriverFromHeadTranslation[2] = MyCtrl[0].Z;

			r.w = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.x = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.y = cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5) + sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5);
			r.z = sin(DegToRad(MyCtrl[0].Yaw) * 0.5) * cos(DegToRad(MyCtrl[0].Roll) * 0.5) * cos(DegToRad(MyCtrl[0].Pitch) * 0.5) - cos(DegToRad(MyCtrl[0].Yaw) * 0.5) * sin(DegToRad(MyCtrl[0].Roll) * 0.5) * sin(DegToRad(MyCtrl[0].Pitch) * 0.5);

			pose.qRotation = rot;
			pose.qDriverFromHeadRotation = r;

			pose.vecPosition[0] = hmd_pose.vecPosition[0];
			pose.vecPosition[1] = hmd_pose.vecPosition[1];;
			pose.vecPosition[2] = hmd_pose.vecPosition[2];

			//Velocity, right?
			pose.vecVelocity[0] = (pose.vecPosition[0] - FirstCtrlLastPos[0]) * 1000 / max((int)deltaTime.count(), 1) / 3; // div 3 - ghosting fix, there are right ways to remove ghosting?
			pose.vecVelocity[1] = (pose.vecPosition[1] - FirstCtrlLastPos[1]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			pose.vecVelocity[2] = (pose.vecPosition[2] - FirstCtrlLastPos[2]) * 1000 / max((int)deltaTime.count(), 1) / 3;
			FirstCtrlLastPos[0] = pose.vecPosition[0];
			FirstCtrlLastPos[1] = pose.vecPosition[1];
			FirstCtrlLastPos[2] = pose.vecPosition[2];

			//if (cnt == 100) {
				//DriverLog("%d - HMD: %f %f %f / %f %f %f\n", ControllerIndex, pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2], rot.x, rot.y, rot.z);
			//}
		}
		else {

			pose.vecDriverFromHeadTranslation[0] = MyCtrl[1].X;
			pose.vecDriverFromHeadTranslation[1] = MyCtrl[1].Y;
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

			pose.vecPosition[0] = hmd_pose.vecPosition[0];
			pose.vecPosition[1] = hmd_pose.vecPosition[1];
			pose.vecPosition[2] = hmd_pose.vecPosition[2];
		}

		cnt++;
		if (cnt == 101) {
			cnt = 0;
		}

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;
		pose.willDriftInYaw = false;
		pose.shouldApplyHeadModel = false;

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
						if (LftStX > 0) {
							MyHMD.moveY = 1;
						}
						else if (LftStX < 0) {
							MyHMD.moveY = -1;
						}
						else {
							MyHMD.moveY = 0;
						}
					}
					else if (gpMoveModus == 1) {
						MyCtrl[gpActiveModus].Pitch = MyCtrl[gpActiveModus].Pitch - LftStX * 0.5;
					}
					else if (gpMoveModus == 2) {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, LftStX, 0);
					}
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 2)) {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
			}
			else if (gpModus == 2 && gpMoveModus == 0) {
				// TODO MOVE HMD - STOP
				MyHMD.moveY = 0;
			}

			//DriverLog("%d %d - LftStX: %f\n", ControllerIndex, index, LftStX);
		}
		else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 2)) {
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickXInputHandle, 0, 0);
		}
		else if (gpModus == 2 && gpMoveModus == 0) {
			// TODO MOVE HMD - STOP
			MyHMD.moveY = 0;
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
						//TODO MOVR
						if (LftStY > 0) {
							MyHMD.rotY = 1;
						}
						else if (LftStY < 0) {
							MyHMD.rotY = -1;
						}
						else {
							MyHMD.rotY = 0;
						}
					}
					else if (gpMoveModus == 1) {
						MyCtrl[gpActiveModus].Roll = MyCtrl[gpActiveModus].Roll + LftStY * 0.5;
					}
					else if (gpMoveModus == 2) {
						vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, LftStY, 0);
					}
					break;
				default:
					break;
				}
			}
			else if ((gpModus == 1 && gpDualModus == 2) || (gpModus == 2 && gpMoveModus == 2)) {
				vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
			}
			else if (gpModus == 2 && gpMoveModus == 0) {
				// TODO MOVE HMD - STOP
				MyHMD.rotY = 0;
			}
			//DriverLog("%d %f - LftStY: %f\n", ControllerIndex, index, LftStY);
		}
		else if ((gpModus==1 && gpDualModus ==2)|| (gpModus==2 && gpMoveModus==2)){
			vr::VRDriverInput()->UpdateScalarComponent(leftJoystickYInputHandle, 0, 0);
		}
		else if (gpModus == 2 && gpMoveModus == 0) {
			// TODO MOVE HMD - STOP
			MyHMD.rotY = 0;
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
					// TODO MOVE
					//DriverLog("%d %d - RghStX: %d\n", ControllerIndex, index, turn);
					if (RghStX > 0) {
						MyHMD.rotX = -1;
					}
					else if (RghStX < 0) {
						MyHMD.rotX = 1;
					}
					else {
						MyHMD.rotX = 0;
					}

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
				MyHMD.rotX = 0;
			}
			//DriverLog("%d %f - RghStX: %f\n", ControllerIndex, index, RghStX);
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
			MyHMD.rotX = 0;
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
					if (RghStY > 0) {
						MyHMD.moveX = -1;
					}
					else if (RghStY < 0) {
						MyHMD.moveX = 1;
					}
					else {
						MyHMD.moveX = 0;
					}
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
				MyHMD.moveX = 0;
			}
			//DriverLog("%d %f - RghStY: %f\n", ControllerIndex, index, RghStY);
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
			MyHMD.moveX = 0;
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
			else if (gpModus == 2) {
				MyHMD.moveZup = 1;
			}

			//DriverLog("%d %f - L_Shoulder: %f\n", ControllerIndex, turn, "L_Shoulder");
		}
		else {
			MyHMD.moveZup = 0;
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
			else if (gpModus == 2){
				MyHMD.moveZdown = -1;
			}
			//DriverLog("%d %f - R_Shoulder: %f\n", ControllerIndex, index, "R_Shoulder");
		}
		else {
			MyHMD.moveZdown = 0;
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
					DriverLog("%d %d - xButtons.R_Thumbstick: speed (before)=%d\n", ControllerIndex, index, MyHMD.speed);
					if (MyHMD.speed == 1) {
						MyHMD.speed = 3;
					}
					else {
						MyHMD.speed = 1;
					}
					DriverLog("%d %d - xButtons.R_Thumbstick: speed (after)=%d\n", ControllerIndex, index, MyHMD.speed);
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
						if (gpMoveModus == 2) gpMoveModus = 0; else gpMoveModus++;
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
				//gamepad.SetRumble(1.0f, 1.0f);
				// This is where you would send a signal to your hardware to trigger actual haptic feedback
				//DriverLog("BUZZ!\n");
			}
			else {
				//gamepad.SetRumble(0.0f, 0.0f);
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

	//EXTRA's
	vr::VRInputComponentHandle_t a_button_click_component_ = 0;
	//vr::VRInputComponentHandle_t a_button_touch_component_ = 0;
	vr::VRInputComponentHandle_t b_button_click_component_ = 0;
	//vr::VRInputComponentHandle_t b_button_touch_component_ = 0;

	//vr::VRInputComponentHandle_t trigger_touch_component_ = 0;

	//vr::VRInputComponentHandle_t grip_touch_component_ = 0;
	//vr::VRInputComponentHandle_t grip_value_component_ = 0;
	//vr::VRInputComponentHandle_t grip_force_component_ = 0;

	//vr::VRInputComponentHandle_t system_touch_component_ = 0;

	vr::VRInputComponentHandle_t joystick_click_component_ = 0;
	//vr::VRInputComponentHandle_t joystick_touch_component_ = 0;
	vr::VRInputComponentHandle_t joystick_x_component_ = 0;
	vr::VRInputComponentHandle_t joystick_y_component_ = 0;

	bool center[2] = { true, true };

	bool doDebug = false;
	int cnt = 0;
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

EVRInitError GPadProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	DriverLog("GPad: Creating Controller Driver\n");

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
	m_pController->SetHMDDriver(m_pNullHmdLatest);
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

	m_pController2 = new CGPadControllerDriver();
	m_pController2->SetControllerIndex(1);
	m_pController2->SetHMDDriver(m_pNullHmdLatest);
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
	//Velocity
	static milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
	lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());

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

std::shared_ptr < GPadProvider> g_serverDriverNull;

//-----------------------------------------------------------------------------
// Purpose: FACTORY
//-----------------------------------------------------------------------------

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		if (!g_serverDriverNull) {
			// Instantiate concrete impl
			g_serverDriverNull = std::make_shared<GPadProvider>();
		}
		// We always have at least 1 ref to the shared ptr in "driver" so passing out raw pointer is ok
		return g_serverDriverNull.get();
	}
	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}