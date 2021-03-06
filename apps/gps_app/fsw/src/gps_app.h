/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
*******************************************************************************/

/**
 * @file
 *
 * Main header file for the GPS application
 */

#ifndef GPS_APP_H
#define GPS_APP_H

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "gpsnodemcu_lib.h"

#include "gps_app_perfids.h"
#include "gps_app_msgids.h"
#include "gps_app_msg.h"

/***********************************************************************/
#define GPS_APP_PIPE_DEPTH 50 /* Depth of the Command Pipe for Application */

#define GPS_APP_NUMBER_OF_TABLES 1 /* Number of Table(s) */

/* Define filenames of default data images for tables */
#define GPS_APP_TABLE_FILE "/cf/gps_app_tbl.tbl"

#define GPS_APP_TABLE_OUT_OF_RANGE_ERR_CODE -1

#define GPS_APP_TBL_ELEMENT_1_MAX 10
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/
typedef struct
{
    /*
    ** Command interface counters...
    */
    uint16 CmdCounter;
    uint16 ErrCounter;
	uint8  I2CBlockedMutFlag;
	uint16 I2CErrorCounter;
    
    double Time;
    double Lat;
    double Long;
    double Alt;
    
    /*
    ** Housekeeping telemetry packet...
    */
    GPS_APP_HkTlm_t HkTlm;

    /*
    ** Run Status variable used in the main processing loop
    */
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[GPS_APP_EVENT_COUNTS];
    CFE_TBL_Handle_t    TblHandles[GPS_APP_NUMBER_OF_TABLES];

} GPS_APP_Data_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (GPS_APP_Main), these
**       functions are not called from any other source module.
*/
void  GPS_APP_Main(void);
int32 GPS_APP_Init(void);
void  GPS_APP_ProcessCommandPacket(CFE_SB_MsgPtr_t MessagePtr);
void  GPS_APP_ProcessGroundCommand(CFE_SB_MsgPtr_t MessagePtr);
void  GPS_APP_Execute(CFE_SB_MsgPtr_t MessagePtr);
int32 GPS_APP_ReportHousekeeping(CFE_SB_MsgPtr_t MessagePtr);
int32 GPS_APP_ResetCounters(CFE_SB_MsgPtr_t MessagePtr);
int32 GPS_APP_Process(CFE_SB_MsgPtr_t MessagePtr);
int32 GPS_APP_Noop(CFE_SB_MsgPtr_t MessagePtr);
void  GPS_APP_GetCrc(const char *TableName);

int32 GPS_APP_TblValidationFunc(void *TblData);

//bool GPS_APP_VerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);

#endif /* GPS_APP_H */
