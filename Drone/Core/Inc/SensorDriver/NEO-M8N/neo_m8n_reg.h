#ifndef INC_SENSORDRIVER_NEO_M8N_NEO_M8N_REG_H_
#define INC_SENSORDRIVER_NEO_M8N_NEO_M8N_REG_H_

/* THIS MODULE DOES NOT USE REGISTER CONCEPTION
 * File name is like that because I wan it has 
 * similar naming convention with other modules */

/* The NEO-M8N has no registers: it is set up and read with UBX messages on
 * the UART (u-blox M8 receiver protocol). These are their numbers.
 *
 * UBX frame: B5 62 | class | id | length (2 bytes, little endian) | payload | CK_A CK_B
 * CK_A/CK_B: 8-bit Fletcher sum over class, id, length and payload */

#define UBX_SYNC_CHAR_1             0xB5
#define UBX_SYNC_CHAR_2             0x62
#define UBX_HEADER_LEN              6U      /* sync 1, sync 2, class, id, length (2) */
#define UBX_CHECKSUM_LEN            2U

/* message classes */
#define UBX_CLASS_NAV               0x01
#define UBX_CLASS_CFG               0x06

/* NAV messages */
#define UBX_NAV_PVT                 0x07    /* position, velocity, time */
#define UBX_NAV_PVT_LEN             84U

/* CFG messages */
#define UBX_CFG_PRT                 0x00    /* port: baud, protocols */
#define UBX_CFG_PRT_LEN             20U
#define UBX_CFG_MSG                 0x01    /* how often a message is sent */
#define UBX_CFG_MSG_LEN             3U      /* short form: for the port it arrives on */
#define UBX_CFG_RATE                0x08    /* measurement rate */
#define UBX_CFG_RATE_LEN            6U
#define UBX_CFG_NAV5                0x24    /* navigation engine: dynamic model */
#define UBX_CFG_NAV5_LEN            36U

/* CFG-PRT fields */
#define UBX_PRT_PORT_UART1          1U
#define UBX_PRT_MODE_8N1            0x000008D0UL    /* 8 data bits, no parity, 1 stop bit */
#define UBX_PRT_PROTO_UBX           0x0001U
#define UBX_PRT_PROTO_NMEA          0x0002U

/* CFG-RATE fields */
#define UBX_RATE_TIME_REF_UTC       0U
#define UBX_RATE_TIME_REF_GPS       1U

/* CFG-NAV5 fields */
#define UBX_NAV5_MASK_DYN           0x0001U         /* apply the dynamic model only */
#define UBX_NAV5_DYN_PORTABLE       0U              /* factory default */
#define UBX_NAV5_DYN_AIRBORNE_1G    6U              /* airborne, < 1 g */
#define UBX_NAV5_DYN_AIRBORNE_2G    7U

/* NAV-PVT payload offsets */
#define UBX_PVT_ITOW                0       /* U4 ms, GPS time of week */
#define UBX_PVT_FIX_TYPE            20      /* U1 */
#define UBX_PVT_FLAGS               21      /* X1, bit 0 gnssFixOK */
#define UBX_PVT_NUM_SV              23      /* U1 */
#define UBX_PVT_LON                 24      /* I4 1e-7 deg */
#define UBX_PVT_LAT                 28      /* I4 1e-7 deg */
#define UBX_PVT_HMSL                36      /* I4 mm */
#define UBX_PVT_HACC                40      /* U4 mm */
#define UBX_PVT_VACC                44      /* U4 mm */
#define UBX_PVT_VELN                48      /* I4 mm/s */
#define UBX_PVT_VELE                52      /* I4 mm/s */
#define UBX_PVT_VELD                56      /* I4 mm/s */
#define UBX_PVT_SACC                68      /* U4 mm/s */
#define UBX_PVT_PDOP                76      /* U2 0.01 */

#define UBX_PVT_FLAG_FIX_OK         0x01U

/* fix types */
#define UBX_FIX_NONE                0U
#define UBX_FIX_DEAD_RECKONING      1U
#define UBX_FIX_2D                  2U
#define UBX_FIX_3D                  3U
#define UBX_FIX_GNSS_DR             4U
#define UBX_FIX_TIME_ONLY           5U

#endif /* INC_SENSORDRIVER_NEO_M8N_NEO_M8N_REG_H_ */
