/************************************************************************
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
*************************************************************************/

/**
 * @file
 *
 * Define GPS App Events IDs
 */

#ifndef GPS_APP_EVENTS_H
#define GPS_APP_EVENTS_H

#define GPS_APP_RESERVED_EID          0
#define GPS_APP_STARTUP_INF_EID       1
#define GPS_APP_COMMAND_ERR_EID       2
#define GPS_APP_COMMANDNOP_INF_EID    3
#define GPS_APP_COMMANDRST_INF_EID    4
#define GPS_APP_INVALID_MSGID_ERR_EID 5
#define GPS_APP_LEN_ERR_EID           6
#define GPS_APP_PIPE_ERR_EID          7

#define GPS_APP_EVENT_COUNTS 7

#endif /* GPS_APP_EVENTS_H */
