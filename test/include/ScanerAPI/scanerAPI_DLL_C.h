/*!****************************************************************************
* \name SCANeRII_API                                                          *
* \file scanerAPI_DLL_C.h                                                     *
* \brief SCANeR API Header                                                    *
* Copyright: OKTAL S.A. all rights reserved.                                  *
* C language Functions called by client Application                           *
* Use it to do implicit linking with the all SCANeR II APIs.                  *
******************************************************************************/

#ifndef __SCANER_API_CBINDING_h__
#define __SCANER_API_CBINDING_h__

//! use this compilation option to create scanerAPI.dll ///
#if defined(SCANERAPI_STATIC) || defined(__linux__)
#define declScanerAPI
#else
#ifdef SCANERAPI_EXPORTS
#define declScanerAPI __declspec(dllexport)
#else 
#define declScanerAPI __declspec(dllimport)
#endif
#endif

#include "scanerAPI_DLL_Enums.h"

#define MAX_DATA_ID_STRING_SIZE 256

#ifndef SCANERAPI_STATIC
#ifdef __cplusplus 
extern "C" {
# endif
#endif

	/**************************************************************************/
	/*! \name SCANeR II process management                                   **/
	/**************************************************************************/

	/*@{*/
	/*! \brief Initialize the process with parameters */
	/*! \param ProcName	Name of process (as in the Process.xfg file)
	*  \param ConfigName Name of the configuration (as in the configurations.cfg file)
	*  \param Frequency Frequency of the module (in Hz) if negative ==> Scheduled Mode, ABS(frequency) used for refreshing
	*  \return 0 if success, -1 if error, -2 if no licence for SCANeR, -3 if the specified configuration is invalid or does not exist,
	*		   -4 if the process is not in the configuration file, -5 if network error
	*             Warning: if the return value is lower or equal to -1, all other SCANeR API functions will not work and may crash your software.
	*/
	declScanerAPI int	Process_InitParams(const char* ProcName, const char* ConfigName, float Frequency);

	/*! \brief Initialize the process using given parameters from command line. */
	/*! 
	* - Required argument is:
	*  -p ProcName:		Name of process (as in the Process.xfg file).
	* - Optional arguments are:
	*  -c ConfigName:	Name of the configuration (as in the configurations.cfg file).
	*											If not specified the first configuration is used.
	*  -f Frequency:	Frequency of the module (in Hz).
	*											If not specified 100 Hz.
	*  \return -1 if error, -2 if no licence for SCANeR, 0 if success
	*             Warning: if -1 is returned, all other SCANeR API functions will not work and may crash your software.
	*/
#ifndef SWIG
	declScanerAPI	int	Process_Init(int argc, char *argv[]);
#endif

	/*! \brief Run the process (Mandatory)*/
	/*! This method need to be call at each loop to execute operations (network communication, state management) */
	/*! \return current process state */
	declScanerAPI	int	Process_Run();

	/*! \brief Set the verbose level for process (Mandatory)*/
	/*! Call this method in order to switch from debug (verbose) to release (non verbose mode) for process */
	declScanerAPI	void Process_Verbose(bool bVerbose);

	/*! \brief Frequency synchronization */
	/*! This method synchronizes the loop with the frequency given in initialization.
	*  This method also updates the time returned by Process_GetTime.
	*  If you don't call this method the process will run as fast as possible and use all the CPU.
	*  \return -1 if error, 0 if success
	*/
	declScanerAPI	int	Process_Wait();

	/*! \brief Clean way to stop the SCANeR II process */
	/*! This method stop the process and inform the simulation. */
	/*!	\return -1 if error, 0 if success */
	declScanerAPI	int	Process_Close();

	/*! \brief Get SCANeR II process state */
	/*! This method return the current process state. */
	declScanerAPI	APIProcessState	 Process_GetState();

	/*! \brief Output a message on the SCANeR Message Log */
	/*! \param msg Message to send  */
	declScanerAPI	void	 Process_Output(const char* msg);

	/*! \brief Output a message on the SCANeR Message Log with a warning level */
	/*! \param msg Message to send  */
	declScanerAPI	void	 Process_OutputLevel(const char* msg, APIOutputLevel level);

	/*! \brief Get the time since the process is in running state
				0 in deamon and loded state.
				The time since the first running state. The time is not incremented in pause or ready state.
				The time may be different from the computer time, it depends on how SCANeR manages the time.
				The time is updated only when Process_Wait is called.
		\return Current simulation time
	*/
	declScanerAPI	double	Process_GetTime();

	/*! \brief Get the log directory (works if the ProcessManager exists)
	this is the folder wqhere the daemon log files are ie. "data/log"
	*/
	declScanerAPI	const char*   Process_GetLogDir();


	/*! \brief get the folder where all the record files/log files are to be stored for this simulation
	(eg. eg files gz files etc.)
	This folder can be used to write module specific record/log files
	*/
	declScanerAPI	const char*   Process_GetRecordDir();

	/*! \brief Get the id of the vehicle handled by the process
	 *  \return
	 *   - The id of the vehicle handled by the process.
	 *   - -1 if there is no vehicle or if the scenario is not loaded
	 */
	declScanerAPI	int   Process_GetDrivenVehicleId();

	/*! \brief Get the corresponding id of the export channel according to the parameters
	*	\param category		category of the export channel, the parameter can be empty
	*	\param name			name of the export channel, the parameter is necessary
	*	\param vehicleId	SCANeR ID of the vehicle in the current scenario, if the parameter value is -1 then we search for an export channel extern to the vehicles
	*	\return	The id of the export channel or -1 if there is not found
	*/
	declScanerAPI	short   Process_GetExportChannelIndex(const char* category, const char* name, int vehicleId);

	/*! \brief Get the process id
	 *  \return
	 *   - The id of the process.
	 *   - -1 if Process_Init or Process_InitParams was not called or if the initialization function return false
	 */
	declScanerAPI	int   Process_GetId();


	/*@}*/

	/**************************************************************************/
	/*! \name Simulation supervisor management                               **/
	/**************************************************************************/

	/*@{*/
	/*! \brief Initialize the process with parameters to do a supervisor (ie capability to manage processes) */
	/*! This function is only needed to build a "supervisor". In this case use this function instead 
	*  of normal Process_InitParams(...) function.
	*  \param ConfigName Name of the configuration (as in the configurations.cfg file)
	*  \param Frequency Frequency of the module (in Hz)
	*  \return -1 if error, -2 if no licence for SCANeR, 0 if success
	*             Warning: if an error is returned, all other SCANeR API functions will not work and may crash your software.
	*/
	declScanerAPI int		Simulation_InitParams(const char* ConfigName, float Frequency);

	/*! \brief Initialize the process using given parameters from command line to do a supervisor (ie capability to manage processes) */
	/*! This function is only needed to build a "supervisor". In this case use this function instead 
	*  of normal Process_Init(...) function.
	* - Optional arguments are:
	*  - -c ConfigName:	Name of the configuration (as in the configurations.cfg file).
	*											If not specified the first configuration is used.
	*  - -f Frequency:	Frequency of the module (in Hz).
	*											If not specified 100 Hz.
	*  \return -1 if error, -2 if no licence for SCANeR, 0 if success
	*             Warning: if an error is returned, all other SCANeR API functions will not work and may crash your software.
	*/
#ifndef SWIG
	declScanerAPI	int		Simulation_Init(int argc, char *argv[]);
#endif

	/*! \brief Start processes */
	/*! All processes will go to DAEMON state */
	/*! This function start all processes of the Process.xfg marked as auto start (* at the end) */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_Launch(int timeoutms);

	/*! \brief Scenario Loading */
	/*! Send a Load message to all processes. */
	/*! Processes will load the given scenario and go to PAUSED state. */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_LoadScenario(int timeoutms,  const char* scenarioName);	

	/*! \brief Send a Play message to all processes */
	/*! All processes will go to RUNNING state */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_Play(int timeoutms);

	/*! \brief Send a Pause message to all processes */
	/*! All processes will go to PAUSED state */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_Pause(int timeoutms);

	/*! \brief Send a UnLoad message to all processes */
	/*! All processes will go to DAEMON state */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_UnLoad(int timeoutms);

	/*! \brief Send a Stop message to all processes */
	/*! All processes will go to STOP state */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_Stop(int timeoutms);

	/*! \brief Kill all started processes */
	/*! \param timoutms: <=0 default timout (100s) */
	declScanerAPI	bool	Simulation_KillAllProcesses(int timeoutms);

	/*! \brief Check if all started processes are alive */
	declScanerAPI	bool	Simulation_AllProcessesOk();

	/*! \brief wait until the list of modules is  in a specified state */
	/*! \param timoutms: <=0 default timout (100s) */
	declScanerAPI	bool	Simulation_WaitForState (const char* modules, APIProcessState state, int timeoutms);

	/*! \brief Start list of processes */
	/*! The process will go to DAEMON state */
	/*! \param moduleName list of processes separated by spaces */
	/*! \param timoutms: -1/0= default timout; <-1= no waiting state  */
	declScanerAPI	bool	Simulation_StartProcess(const char* moduleName, int timeoutms);

	/*! \brief Kill a specific process */
	/*! \param procId id of the process */
	declScanerAPI	void	Simulation_KillProcess(int procId);

	/*! \brief kill a  list of processes */
	/*! All processes will go to DEAD state */
	/*! \param moduleName list of processes separated by spaces */
	/*! \param timoutms: <=0 default timout (100s) */
	declScanerAPI	void	Simulation_KillProcesses(const char* moduleName, int timeoutms=0);	

	/*! \brief Request a update of all process informations */
	declScanerAPI	void	Simulation_UpdateProcessInfo();

	/*! \brief Get the number of process informations available*/
	declScanerAPI	unsigned int Simulation_getProcessNumber();

	/*! \brief Get all processes informations */
	/*! \param array Pre-allocated array of APIProcessInfo object with the size returned by Simulation_getProcessNumber().*/
	declScanerAPI	void	Simulation_getAllProcessInfo(APIProcessInfo *array);

	/*! \brief Get one process information */
	/*! \param id id of the process
	*  \param inf pointer on a APIProcessInfo struture
	*/
	declScanerAPI bool	Simulation_GetProcessInfo(int id, APIProcessInfo *inf);

	/*! \brief Get process id from name */
	/*! \param ProcName name of the process */
	declScanerAPI	int		Simulation_GetIdFromName(const char* ProcName);

	/*! \brief is it a process to be launch on SCANeR Start */
	/*! \param procId id of the process */
	declScanerAPI	bool	Simulation_IsProcessAutoLaunched(const int ProcId);

	/*! \brief Shutdown simulation */
	/*! \param allComputers 
	*  \param option configuration  */
	declScanerAPI	bool	Simulation_Shutdown(bool allComputers, APIShutdownType shutdownType);

	/*! \brief Get the current configuration */
	declScanerAPI	const char* Simulation_GetCurrentConfig();

	/*! \brief Get the total number of SCANeR configurations available*/
	declScanerAPI	int Simulation_GetConfigNumber();

	/*! \brief Get the name of the index th SCANeR configuration*/
	/*! \return the string pointed by the returned char* must be copied because it might be overwritten by the next call to Simulation_GetConfigName() */
	declScanerAPI	const char* Simulation_GetConfigName(unsigned int index);

	/*! \brief Change the configuration */
	/*! \param ConfigName new configuration name */
	declScanerAPI	void	Simulation_ChangeConfig(const char* ConfigName);

	/*! set the environnement variable varEnv **/
	declScanerAPI	bool	Simulation_setVarEnv(const char *varEnv);

	/*@}*/

	/**************************************************************************/
	/*! \name Data management (Network and Shared Memory)                    **/
	/**************************************************************************/
	struct DataInterface;
	struct Event;

	/*@{*/
	/*! \brief Declare data for input */
	/*! \param dataIdString Name of the data to listen to.
	*  \param index Index of vehicle to listen to or -1 for all.
	*/
	declScanerAPI	DataInterface * Com_declareInputData(const char *dataIdString, int index = -1);

	/*! \brief Declare data for output */
	/*! \param dataIdString Name of the data to send.
	*  \param index Index of vehicle to listen to or -1 for all
	*/
	declScanerAPI	DataInterface * Com_declareOutputData(const char *dataIdString, int index = - 1);

	/*! \brief Delete data interface */
	/*! \param dataInterface Interface to release.
	*/
	declScanerAPI	void Com_releaseInterface(DataInterface * dataInterface);

	/*! updating of Input or Output data (Shm or Network) **/
	declScanerAPI	bool	Com_updateInputs (UpdateType updateType = UT_AllData);
	declScanerAPI	bool	Com_updateOutputs(UpdateType updateType = UT_AllData);

	/*! send a one-shot data **/
	declScanerAPI	DataInterface * Com_createOutputDataInterface(const char * dataIdString);
	declScanerAPI	bool            Com_updateOutputDataInterface(DataInterface * dataInterface);
	declScanerAPI	bool            Com_deleteOutputDataInterface(DataInterface * dataInterface);

	/*!\name utilities **/
	/*@{*/
	/*! \brief  Return the field number of the data interface
	*/
	declScanerAPI int	Com_getFieldNumber(const DataInterface * dataInterf);
	/*! Memory field information */
	struct FieldInfo
	{
		char name[MAX_DATA_ID_STRING_SIZE]; /*! field name */
		char typeName[MAX_DATA_ID_STRING_SIZE]; /*! field data type */
	};
	/*! \brief filled an array with the field information of the giving DataInterface
	FieldArray should be allocated with enough room to store all the DataInterface fields
	\see getFieldNumber
	*/
	declScanerAPI bool	Com_getFieldInfoArray(const DataInterface * dataInterf, FieldInfo* infoArray);

	/*@}*/
	/*! getters .. **/
	declScanerAPI	const char* Com_getStringData(DataInterface * dataInterf,const char* fieldName);
	declScanerAPI	short		Com_getShortData (DataInterface * dataInterf,const char* fieldName);
	declScanerAPI	long		Com_getLongData  (DataInterface * dataInterf,const char* fieldName);
	declScanerAPI   unsigned long long Com_getUInt64Data(DataInterface* dataInterf, const char*fieldName);
	declScanerAPI	float		Com_getFloatData (DataInterface * dataInterf,const char* fieldName);
	declScanerAPI	double		Com_getDoubleData(DataInterface * dataInterf,const char* fieldName);
	declScanerAPI	char		Com_getCharData  (DataInterface * dataInterf,const char* fieldName);

	/*
	Get an array of data in a single call. These functions currently work only for Network data, not for Shm.

	Parameters
		dataInterf: the data interface on which to retrieve data
		fieldName: the data name
		buffer: a pointer to a caller allocated buffer which will receive the data
		bufferNum: the number of elements to retrieve
	Return value
		-1 if arguments are incorrect or if the field doesn't exist
		0 if the message has not been received yet or is empty
		>0 on success. In this case the return value is the number of element actually copied

	For variable size arrays, you can use the following pattern:
		1) Call this function with buffer=NULL and bufferNum=0 to retrieve the size of the last received message.
		2) Allocate a buffer with the correct size.
		3) Call this function with buffer!=NULL and bufferNum!=0 to fill the buffer.
	*/

	declScanerAPI	int			Com_getShortDataArray(DataInterface* dataInterf, const char* fieldName, short* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getLongDataArray(DataInterface* dataInterf, const char* fieldName, long* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getUInt64DataArray(DataInterface* dataInterf, const char* fieldName, unsigned long long* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getFloatDataArray(DataInterface* dataInterf, const char* fieldName, float* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getDoubleDataArray(DataInterface* dataInterf, const char* fieldName, double* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getCharDataArray(DataInterface* dataInterf, const char* fieldName, char* buffer, unsigned int bufferNum);

	declScanerAPI	const char* Com_getStringDataByName(const char * dataIdString,const char* fieldName);
	declScanerAPI	short		Com_getShortDataByName (const char * dataIdString,const char* fieldName);
	declScanerAPI	long		Com_getLongDataByName  (const char * dataIdString,const char* fieldName);
	declScanerAPI   unsigned long long Com_getUInt64DataByName(const char * dataIdString,const char* fieldName);
	declScanerAPI	float		Com_getFloatDataByName (const char * dataIdString,const char* fieldName);
	declScanerAPI	double		Com_getDoubleDataByName(const char * dataIdString,const char* fieldName);
	declScanerAPI	char		Com_getCharDataByName  (const char * dataIdString,const char* fieldName);

	/*
	Get an array of data in a single call. These functions currently work only for Network data, not for Shm.

	Parameters
		dataIdString: the data path of the interface on which to retrieve data
		fieldName: the data name
		buffer: a pointer to a caller allocated buffer which will receive the data
		bufferNum: the number of elements to retrieve
	Return value
		-1 if arguments are incorrect or if the field doesn't exist
		0 if the message has not been received yet or is empty
		>0 on success. In this case the return value is the number of element actually copied

	For variable size arrays, you can use the following pattern:
		1) Call this function with buffer=NULL and bufferNum=0 to retrieve the size of the last received message.
		2) Allocate a buffer with the correct size.
		3) Call this function with buffer!=NULL and bufferNum!=0 to fill the buffer.
	*/

	declScanerAPI	int			Com_getShortDataArrayByName(const char* dataIdString, const char* fieldName, short* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getLongDataArrayByName(const char* dataIdString, const char* fieldName, long* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getUInt64DataArrayByName(const char* dataIdString, const char* fieldName, unsigned long long* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getFloatDataArrayByName(const char* dataIdString, const char* fieldName, float* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getDoubleDataArrayByName(const char* dataIdString, const char* fieldName, double* buffer, unsigned int bufferNum);
	declScanerAPI	int			Com_getCharDataArrayByName(const char* dataIdString, const char* fieldName, signed char* buffer, unsigned int bufferNum);

	declScanerAPI	bool		Com_setStringData(DataInterface * dataInterf, const char*fieldName, const char*val);
	declScanerAPI	bool		Com_setShortData (DataInterface * dataInterf, const char*fieldName, short val);
	declScanerAPI	bool		Com_setLongData  (DataInterface * dataInterf, const char*fieldName, long  val);
	declScanerAPI   bool		Com_setUInt64Data(DataInterface* dataInterf, const char*fieldName, unsigned long long val);
	declScanerAPI	bool		Com_setFloatData (DataInterface * dataInterf, const char*fieldName, float val);
	declScanerAPI	bool		Com_setDoubleData(DataInterface * dataInterf, const char*fieldName, double val);
	declScanerAPI	bool		Com_setCharData  (DataInterface * dataInterf, const char*fieldName, char val);

	declScanerAPI	bool		Com_setStringDataByName(const char * dataIdString, const char*fieldName, const char*val);
	declScanerAPI	bool		Com_setShortDataByName (const char * dataIdString, const char*fieldName, short val);
	declScanerAPI	bool		Com_setLongDataByName  (const char * dataIdString, const char*fieldName, long  val);
	declScanerAPI   bool		Com_setUInt64DataByName(const char * dataIdString, const char*fieldName, unsigned long long val);
	declScanerAPI	bool		Com_setFloatDataByName (const char * dataIdString, const char*fieldName, float val);
	declScanerAPI	bool		Com_setDoubleDataByName(const char * dataIdString, const char*fieldName, double val);
	declScanerAPI	bool		Com_setCharDataByName  (const char * dataIdString, const char*fieldName, char val);

	/*! Events management **/
	declScanerAPI	bool		Com_registerEvent  (const char* eventId, int index = - 1);
	declScanerAPI   bool		Com_unregisterEvent(const char* eventId, int index = - 1);
	declScanerAPI	Event *		Com_getNextEvent();
	declScanerAPI   int			Com_getNbPendingEvent();
	declScanerAPI	EventType   Com_getTypeEvent(Event * event);

	/*! Message Events : get the data interface **/
	declScanerAPI	DataInterface* Com_getMessageEventDataInterface(Event* event);

	/*! Message Events : get the data stringId **/
	declScanerAPI	const char* Com_getMessageEventDataStringId(Event* event);

	/*! State Events : get the state change type **/
	declScanerAPI	StateEventType Com_getStateEventType(Event* event);

	/*! State Events : validate the state change **/
	declScanerAPI	bool		Com_validateStateEvent(Event* event);

	/*! State Events : Get scenario name from Load events only **/
	declScanerAPI	const char *	Com_getScenarioName(Event* event);

	/*! State Events : Get terrain name from Load events only **/
	declScanerAPI	const char *	Com_getTerrainName(Event* event);

	/*! State Events : Get Initial Conditions from Load events only
	\return the inital conditiond, it must not to be changed or deleted. It's life duration is the same as the event
	**/
	declScanerAPI	const InitialConditions* Com_getInitConditions(Event* event);

	/*! Utility functions to get objects and vehicles from InitialConditions
	    Usefull in C# and graphical language where array mapping is difficult.
	\return the corresponding object.
	**/
	declScanerAPI	const OStruct* Com_getInitConditionsObject(Event* event, int index);
	declScanerAPI	const VStruct* Com_getInitConditionsVehicle(Event* event, int index);


	/*! State Events : Get User Data count from Load events only **/
	declScanerAPI	int Com_getUserDataCount(Event* event);

	/*! State Events : Get User Data key by index from Load events only.
		If the index is invalid, a NULL pointer is returned.
		The returned string is a pointer to an internal buffer, the caller must not try to free it.
		It is valid until the next call to getNextEvent. **/
	declScanerAPI	const char* Com_getUserDataKeyByIndex(Event* event, int index);

	/*! State Events : Get User Data value by index from Load events only.
		If the index is invalid, a NULL pointer is returned.
		The returned string is a pointer to an internal buffer, the caller must not try to free it.
		It is valid until the next call to getNextEvent. **/
	declScanerAPI	const char* Com_getUserDataValueByIndex(Event* event, int index);

	/*! State Events : Get User Data value by key from Load events only.
		If the key doesn't exist, a NULL pointer is returned.
		The returned string is a pointer to an internal buffer, the caller must not try to free it.
		It is valid until the next call to getNextEvent. **/
	declScanerAPI	const char* Com_getUserDataValue(Event* event, const char* key);

	/*! Events management for labVIEW**/
	declScanerAPI	EventType		Com_getNextEventType();

	/*! Message Events : get the data stringId **/
	declScanerAPI	const char* Com_getCurrentMessageEventDataStringId();

	/*! State Events : get the state change type **/
	declScanerAPI	StateEventType Com_getCurrentStateEventType();

	/*! State Events : validate the state change **/
	declScanerAPI	bool		Com_validateCurrentStateEvent();

	/*! State Events : Get scenario name from Load events only **/
	declScanerAPI	const char *	Com_getScenario();


	/*@}*/

	/**************************************************************************/
	/*! \name SCANeR Utils management                                        **/
	/**************************************************************************/

	/*@{*/
	/*! \brief Return the full path of a SCANeR Studio file */
	/*! \param fileScanerPath	Path of the file in the SCANeR Studio format, (eg: "data/scenario/my_scenario.sce" or "config/observer.cfg")
	*! There are 2 behaviours :
	*!		- If the path starts with "data", it assume a file/directory in one of your data directories.
	*!		  in this case it returns the first file/directory found in your data directories, in using the order specify in the GUI (first is USER, last in DEFAULT directory).
	*!		- If the path starts with "config", it assume a file/directory in your coniguration directory
	*! 
	*! \return
	*! 	- The returned char* is never NULL
	*!		- When an error occured the returned string is "" and an error is displayed
	*!     - The caller has to delete the char* unsing Utils_releaseChar
	*/
	declScanerAPI	const char *	Utils_getPath(const char* fileScanerPath);

	/*! \brief Return all the paths corresponding with a scaner path. */
	/*! For example if you have multiple data path configure for your loaded configuration, 
	*! "data/scenario" will return all the existing directory in yours data paths.
	*! if you look for a file in configuration directory, it return the same result as Utils_getPath.
	*! \param fileScanerPath	Path of the file in the SCANeR Studio format, (eg: "data/scenario/my_scenario.sce" or "data/terrain" or "config/observer.cfg")
	*! \return an array of the founded absolute path. The last element of the array is a NULL char pointer. delete the array using Utils_releaseCharArray
	*/
	declScanerAPI	const char** Utils_getMultiplePath(const char* fileScanerPath);

	/*! \brief Return the token value from a file */
	/*! The returned pointer points to a dynamically allocated string. After use, release its memory by calling Utils_releaseChar*/
	/*! \param file	full path of the configuration file.
	/*! The file have to be at SCANeR configuration format.
	/*! \param token	token that you want read the value.
	*/
	declScanerAPI	const char *	Utils_getTokenValue(const char* file, const char* token);

	/*! \brief Return the token value from a section of a file */
	/*! The returned pointer points to a dynamically allocated string. After use, release its memory by calling Utils_releaseChar*/
	/*! \param file	full path of the configuration file.
	/*! The file have to be at SCANeR configuration format.
	/*! \param token	token that you want read the value.
	/*! \param section	section to which the token belongs.
	*/
	declScanerAPI	const char *	Utils_getTokenValueFromSection(const char* file, const char* token, const char* section);

	/*! \brief delete a string allocated by the scaner API */
	/*! \param data	allocated memory to delete.
	*/
	declScanerAPI void Utils_releaseChar(const char* data);

	/*! \brief delete a string array allocated by the scaner API */
	/*! \param data	allocated memory to delete.
	*/
	declScanerAPI void Utils_releaseCharArray(const char** data);

	/*! Get Vehicle information. Only valid after scenario loading
	/*! \param id vehicle id to get information
	/*! \param vehinfo struct to be filled with vehicle informations
	\return 1 if success, 0 if the scenario is not loaded or unknow vehicle id (vehinfo not modified)
	**/
	declScanerAPI	int Utils_getVehicleInformations(short id, VehicleInfoStruct* vehinfo);


	/*@}*/

#ifndef SCANERAPI_STATIC
#ifdef __cplusplus 
}
#endif
#endif
#endif //! __SCANER_API_CBINDING_h__ */
