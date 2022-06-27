/*!
 * \name     SCANeRII_API
 * \file     scanerAPI_DLL_Enums.h
 * \brief    Definition of data types for SCANeR II API
 */

#ifndef __SCANER_API_DLL_ENUMS_H__
#define __SCANER_API_DLL_ENUMS_H__

#define MAX_PROCNAME_SIZE 255
#define MAX_HOSTNAME_SIZE 255

#ifdef __cplusplus
extern "C" {
# endif

	/*! \enum APIOutputLevel **/
	/*! Enum for output severity
	*/
	typedef enum
	{
		OL_Notify = 1,
		OL_Debug = 2,
		OL_Warning = 3,
		OL_Err = 4
	} APIOutputLevel;

	/*! \enum APIProcessState */
	/*! Enum for current SCANeR II process state:
	 * - PS_DEAD: SCANeR II Process is dead (need to exit the program).
	 * - PS_DAEMON: SCANeR II Process is just launched (nothing loaded).
	 * - PS_LOADED: SCANeR II Process is loaded with a scenario.
	 * - PS_PAUSED: SCANeR II Process is paused.
	 * - PS_READY: SCANeR II Process is ready and waiting for other processes to go running.
	 * - PS_RUNNING: SCANeR II Process is running.
	 */
	typedef enum
	{
		PS_DEAD = 0,
		PS_DAEMON = 1,
		PS_LOADED = 2,
		PS_PAUSED = 3,
		PS_READY = 4,
		PS_RUNNING = 5
	} APIProcessState;

	/*! \enum APIShutdownType **/
	/*! \brief Type of shutdown */
	typedef enum {
		ST_POWEROFF = 0,
		ST_REBOOT = 1,
		ST_SHUTDOWN = 2,
		ST_STANDBY = 3
	} APIShutdownType;


	typedef enum {
		RQ_SOL = 0,		// use the sol data : fast picking on the road and terrain triangles
		RQ_OSG = 1,		// use the OpenSceneGraph picking: slow picking on the road and terrain triangles
		RQ_RND = 2,		// use the RoadNetworkDescription file: fast access to the road description
		RQ_RS = 3,		// use the RoadSurface file: fast access to the road surface
		RQ_FLAT = 4		// use Z=0 picking
	} APIRoadQueryType;


	/*! \brief Structure of process informations with frequency*/
	struct APIProcessInfo
	{
		/*! State of the process */
		APIProcessState state;
		/*! Name of the process */
		char name[MAX_PROCNAME_SIZE];
		/*! module frequency (available in run time only) */
		float frequency;
		/*! module desired frequency (available in run time only) */
		float desiredFrequency;
		/*! Hostname*/
		char hostname[MAX_HOSTNAME_SIZE];
	};

	/*! \enum UpdateType */
	/*! Enumerant for choosing update type:
	 * - UT_NetworkData: Update only Network Data (messages).
	 * - UT_ShmData: Update only data from shared memory.
	 * - UT_AllData: Update both kind of data.
	 * - UT_Unknown: Undefined.
	 */
	typedef enum { UT_Unknown = 0, UT_AllData, UT_NetworkData, UT_ShmData } UpdateType;

	/*! \enum EventType */
	/*! Enumerant for event type:
	 * - ET_message: Event is a message reception.
	 * - ET_state: Event is a state change request.
	 * - ET_unknown: Undefined.
	 */
	typedef enum { ET_message = 0, ET_state = 1, ET_unknown = 2 } EventType;

	/*! \enum StateEventType */
	/*! Enumerant for state change event */
	typedef enum { ST_Unknown, ST_Load, ST_Start, ST_Init, ST_Go, ST_Pause, ST_Stop, ST_Unload, ST_Kill } StateEventType;

	/*! \enum VType */
	/*! Enumerant for vehicle type in SCANeR(c)II software*/
	enum VType
	{
		UNKNOWN_TYPE = 0,
		RIGID = 1,
		TRACTOR = 2,
		SEMI_TRAILER = 3,
		TRAILER = 4,
		CAR = 5,
		BUS = 6,
		MOTORBIKE = 7,
		BICYCLE = 8,
		PEDESTRIAN = 9,
		STATIC_OBJECT = 10,
		TRAM = 11,
		TRAIN = 12,
		ALIVEBEING = 13,
		AIRPLANE = 14,
		CNT_VEHICLETYPE
	};

	/*! \enum VBehaviour */
	/*! Enumerant for vehicle behaviour in SCANeR(c)II software*/
	enum VBehaviour { EXTERNAL = 0, AUTONOMOUS = 1, INTERACTIVE = 2, ANIMATED = 3, VB_UNKNONW = 4 };

	/*! \enum VState */
	/*! Enumerant for vehicle state in SCANeR(c)II software*/
	enum VState { V_GHOST, V_VISIBLE, V_ALIVE, V_UNKNOWN };

	/*! \enum OState */
	/*! Enumerant for object state in SCANeR(c) software*/
	enum OState { O_GHOST, O_VISIBLE, O_ALIVE, O_UNKNOWN };

	typedef  struct {
		double x, y, z;
	}InitPositionStruct;

	typedef  struct {
		double x, y, z;
	}Vector3;

	/*! \struct VehicleStruct **/
	/*! \brief Structure of vehicle informations */
	typedef struct
	{
		int id;
		char name[256];
		char modelName[256];
		VType type;
		VBehaviour behaviour;
		VState state;
		InitPositionStruct initPosition;
		double heading;
	} VStruct;

	/*! \struct ObjectStruct **/
	/*! \brief Structure of object informations */
	typedef struct
	{
		int id;
		char name[256];
		double mass;
		char staticObject; // bool value
		OState state;
		InitPositionStruct initPosition;
		double heading;
		char heighmap[256];
		char hasHeighmap; // bool value
		Vector3 scale;
		double friction;
		double rollingFrition;
	} OStruct;


	/*! \struct InitialCondition **/
	/*! \brief Structure of Initial Conditions*/
	/*! \Structure valid from Load events only */
	typedef struct
	{
		char scenarioName[256];
		char terrainName[256];
		int vehiclesCount;
		int objectCount;
	} InitialConditions;


	typedef struct 
	{
		int nbWheels;
		double myWheelWidth;
		double myWheelRadius;
		double axleLength;
	} TechAxle ;

	/*! \struct VehicleStruct **/
	/*! \brief Structure of vehicle informations */
	typedef struct
	{
		VType type;
		char model[256]; //Simple, Intermetiadte, Advanced, Tridym etc...
		char modelName[256]; //name of the *.mdl tech file
		char visualModel[256]; //the name of the v3d file
		char soundModel[256]; //!>configuration file for sound
		double length;
		double width;
		double height;
		double wheelBase;
		double rideHeight;
		double rearOverhang;
		Vector3 frontArtic; //front articulation point in SCANeR referential (X,Y,Z)
		Vector3 rearArtic; //rear articulation point in SCANeR referential (X,Y,Z)
		double minWeight;
		double maxWeight;
		double maxSpeed; /*m/sec*/
		double maxWheelsAngle;
		double maxSteeringWheelAngle; /*rad*/ //! lock to lock
		double maxDeceleration;		//! <0
		Vector3 eyePosition;
		//! engine position in scaner vehicle referential (not tec referential!)
		Vector3 enginePosition;
		double maxEngineSpeed;
		int myGearCount;

		int nbAxle; // Default size is 2, the index 0 correspond to the rear axle, 1 for the front axle
		TechAxle axles[10];

	} VehicleInfoStruct;

	//------------------------------------------------------------------------------------

#ifdef __cplusplus 
}
#endif
#endif /*! __SCANER_API_DLL_ENUMS_H__ */
