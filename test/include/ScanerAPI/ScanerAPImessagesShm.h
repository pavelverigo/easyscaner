/*
 * File : ScanerAPImessagesShm.h
 * Copyright (C) 2001, Renault all rights reserved
 * SCANeR II is distributed by OKTAL S.A.
 */


/**
 * Enum Telma
 *  **/
typedef enum 
{
	SC_TELMA_OFF,
	SC_TELMA_ONE,
	SC_TELMA_TWO,
	SC_TELMA_THREE,
	SC_TELMA_FOUR
} Telma;

/**
 * Enum GearBoxPosition
 *  **/
typedef enum 
{
	SC_GEAR_SLOW,
	SC_GEAR_REVERSE,
	SC_GEAR_NEUTRAL,
	SC_GEAR_ONE,
	SC_GEAR_TWO,
	SC_GEAR_THREE,
	SC_GEAR_FOUR,
	SC_GEAR_FIVE,
	SC_GEAR_SIX,
	SC_GEAR_SEVEN,
	SC_GEAR_EIGHT,
	SC_GEAR_D,
	SC_GEAR_P,
	SC_GEAR_SEQ_UP,
	SC_GEAR_SEQ_NEUTRAL,
	SC_GEAR_SEQ_DOWN,
	SC_GEAR_SPORT
} GearBoxPosition;

/**
 * Method CabToModel
 * From Cabin to model
 **/
#define SHM_MODELCABIN_CABTOMODEL "Shm/ModelCabin/CabToModel"

/**
 * Method VehicleOutput
 * Output of the vehicle
 **/
#define SHM_MODELCABIN_VEHICLEOUTPUT "Shm/ModelCabin/VehicleOutput"

/**
 * Method GearBoxSupToModel
 * From Cabin extended gear box command to model
 **/
#define SHM_MODELCABIN_GEARBOXSUPTOMODEL "Shm/ModelCabin/GearBoxSupToModel"

/**
 * Method SuspensionHeightToModel
 * From Cabin suspension command to model
 **/
#define SHM_MODELCABIN_SUSPENSIONHEIGHTTOMODEL "Shm/ModelCabin/SuspensionHeightToModel"

/**
 * Method CabToModelAutonomousCorrective
 * Provides coefficients and additives values to change CabToModel data
 **/
#define SHM_MODELCABIN_CABTOMODELAUTONOMOUSCORRECTIVE "Shm/ModelCabin/CabToModelAutonomousCorrective"

/**
 * Method CabToModelCorrective
 * Provides coefficients and additives values to change CabToModel data
 **/
#define SHM_MODELCABIN_CABTOMODELCORRECTIVE "Shm/ModelCabin/CabToModelCorrective"

/**
 * Method CabToSteering
 * From Cabin to model
 **/
#define SHM_MODELCABIN_CABTOSTEERING "Shm/ModelCabin/CabToSteering"

/**
 * Method CabToSteeringAutonomousCorrective
 * From Cabin to model : corrective commands
 **/
#define SHM_MODELCABIN_CABTOSTEERINGAUTONOMOUSCORRECTIVE "Shm/ModelCabin/CabToSteeringAutonomousCorrective"

/**
 * Method CabToSteeringCorrective
 * From Cabin to model : corrective commands
 **/
#define SHM_MODELCABIN_CABTOSTEERINGCORRECTIVE "Shm/ModelCabin/CabToSteeringCorrective"

/**
 * Method SteeringToCabCorrective
 * From Model to Cabin : corrective commands
 **/
#define SHM_MODELCABIN_STEERINGTOCABCORRECTIVE "Shm/ModelCabin/SteeringToCabCorrective"

/**
 * Method CabToModelClient
 * From Cabin to model
 **/
#define SHM_MODELCABIN_CABTOMODELCLIENT "Shm/ModelCabin/CabToModelClient"

/**
 * Method ModelToCabClient
 * From Model to cabin
 **/
#define SHM_MODELCABIN_MODELTOCABCLIENT "Shm/ModelCabin/ModelToCabClient"

/**
 * Method TurretToModel0
 * From Turret to Model
 **/
#define SHM_MODELCABIN_TURRETTOMODEL0 "Shm/ModelCabin/TurretToModel0"

/**
 * Method TurretToModel1
 * From Turret to Model
 **/
#define SHM_MODELCABIN_TURRETTOMODEL1 "Shm/ModelCabin/TurretToModel1"

/**
 * Method TurretToModel2
 * From Turret to Model
 **/
#define SHM_MODELCABIN_TURRETTOMODEL2 "Shm/ModelCabin/TurretToModel2"

/**
 * Method TurretToModel3
 * From Turret to Model
 **/
#define SHM_MODELCABIN_TURRETTOMODEL3 "Shm/ModelCabin/TurretToModel3"

/**
 * Method TurretToModel4
 * From Turret to Model
 **/
#define SHM_MODELCABIN_TURRETTOMODEL4 "Shm/ModelCabin/TurretToModel4"

/**
 * Method ToggleElectronicAssistanceToModel
 * From Toggle Electronic Assistance to Model
 **/
#define SHM_MODELCABIN_TOGGLEELECTRONICASSISTANCETOMODEL "Shm/ModelCabin/ToggleElectronicAssistanceToModel"

/**
 * Method PlowToModel
 * From Plow to Model
 **/
#define SHM_MODELCABIN_PLOWTOMODEL "Shm/ModelCabin/PlowToModel"

/**
 * Method AutonomousModeToModel
 * Autonomous Mode buttons to Model
 **/
#define SHM_MODELCABIN_AUTONOMOUSMODETOMODEL "Shm/ModelCabin/AutonomousModeToModel"

/**
 * Method PedalsFeedbackCommand
 * Pedals Feedback Command
 **/
#define SHM_MODELCABIN_PEDALSFEEDBACKCOMMAND "Shm/ModelCabin/PedalsFeedbackCommand"

/**
 * Method CabPhysicalState
 * Cabin Physical State
 **/
#define SHM_MODELCABIN_CABPHYSICALSTATE "Shm/ModelCabin/CabPhysicalState"

/**
 * Method ModelToPlatform
 * From model to platform Strategy
 **/
#define SHM_MODELPLATFORM_MODELTOPLATFORM "Shm/ModelPlatform/ModelToPlatform"

/**
 * Method StrategyToModel
 * Platform strategy outputs
 **/
#define SHM_MODELPLATFORM_STRATEGYTOMODEL "Shm/ModelPlatform/StrategyToModel"

/**
 * Method PlatformToModel
 * Platform outputs
 **/
#define SHM_MODELPLATFORM_PLATFORMTOMODEL "Shm/ModelPlatform/PlatformToModel"
