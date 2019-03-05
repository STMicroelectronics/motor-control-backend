/*
 * motor_control_backend.c
 * Implements the motor control backend to intereact with the GUI.
 *
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 * Author: Vincent Abriou <vincent.abriou@st.com> for STMicroelectronics.
 *
 * License type: GPLv2
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h> // for usleep
#include <errno.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <gre/greio.h>

#include <microhttpd.h>

#define MAX_BUF 80

#define MC_SEND_CHANNEL "mc_frontend"
#define MC_RECEIVE_CHANNEL "mc_backend"

#define PORT 8888
#define GET             0
#define POST            1
#define POSTBUFFERSIZE  512

//#define REF_SPEED_MIN 1000
//#define REF_SPEED_MAX 4000
#define SPEED_KP_VAL_MIN 1
#define SPEED_KP_VAL_MAX 100000
#define SPEED_KP_POS_MIN 0
#define SPEED_KP_POS_MAX 100
#define SPEED_KI_VAL_MIN 1
#define SPEED_KI_VAL_MAX 100000
#define SPEED_KI_POS_MIN 0
#define SPEED_KI_POS_MAX 100
#define RAMP_DUR_MIN 100
#define RAMP_DUR_MAX 2000

struct connection_info_struct
{
  int connectiontype;
  char *answerstring;
  struct MHD_PostProcessor *postprocessor;
};

pthread_mutex_t ttyMutex;
//pthread_cond_t ttyCondvar;

struct MHD_Daemon *mHttpDaemon;

//char FIRM_NAME[] = "rprocttymotwc0106.elf";
//char FIRM_NAME[] = "rprocttymotwc0100";
char FIRM_NAME[30];
char prtmsg[200];
struct timeval tval_before, tval_after, tval_result;

typedef enum {
	MC_ACTION_NOTHING,              /*  0 */
	MC_ACTION_GET_STATUS,           /*  1 */
	MC_ACTION_GET_CONTROL_MODE,     /*  2 */
	MC_ACTION_START_MOTOR,          /*  3 */
	MC_ACTION_STOP_MOTOR,           /*  4 */
	MC_ACTION_SET_SPEED_VALUE,      /*  5 */
	MC_ACTION_SET_SPEED_REFERENCE,  /*  6 */
	MC_ACTION_SET_SPEED_KP,         /*  7 */
	MC_ACTION_SET_SPEED_KI,         /*  8 */
	MC_ACTION_GET_SPEED_VALUE,      /*  9 */
	MC_ACTION_GET_SPEED_REFERENCE,  /* 10 */
	MC_ACTION_GET_BUS_VOLTAGE,      /* 11 */
	MC_ACTION_GET_TORQUE_VALUE,     /* 12 */
	MC_ACTION_GET_TORQUE_REFERENCE, /* 13 */
	MC_ACTION_GET_HEAT_SINK,        /* 14 */
	MC_ACTION_SET_SPEED_RAMP,		/* 15 */
	MC_ACTION_SET_STROBOSCOPE_SPEED,/* 16 */
	MC_ACTION_SET_TORQUE_REFERENCE, /* 17 */
	MC_ACTION_SET_CONTROL_MODE,     /* 18 */
	MC_ACTION_SET_TORQUE_KP,        /* 19 */
	MC_ACTION_SET_TORQUE_KI,        /* 20 */
	MC_ACTION_SET_WDOG_REFILL,      /* 21 */
	MC_ACTION_GET_SPEED_KP,         /* 22 */
	MC_ACTION_GET_SPEED_KI,         /* 23 */
	MC_ACTION_GET_TORQUE_KP,        /* 24 */
	MC_ACTION_GET_TORQUE_KI,        /* 25 */
	MC_ACTION_FAULT_ACK,            /* 26 */
    MC_ACTION_GET_FLAGS,            /* 27 */
    MC_ACTION_TOGGLE_STROBOSCOPE,   /* 28 */
} mc_action_t;

typedef enum {
	ACTION_ENABLE,
	ACTION_SET,
	ACTION_SET2,
	ACTION_GET
} mc_action_type_t;

typedef struct {
	char *name;
	mc_action_t action;
	mc_action_type_t type;
} motor_action_t;

typedef struct {
	int v1;
	int v2;
} action_data2_t;

typedef enum {
	MC_STATE_IDLE,                  /*  0 */
	MC_STATE_IDLE_ALIGNMENT,        /*  1 */
	MC_STATE_ALIGNMENT,             /*  2 */
	MC_STATE_IDLE_START,            /*  3 */
	MC_STATE_START,                 /*  4 */
	MC_STATE_START_RUN,             /*  5 */
	MC_STATE_RUN,                   /*  6 */
	MC_STATE_ANY_STOP,              /*  7 */
	MC_STATE_STOP,                  /*  8 */
	MC_STATE_STOP_IDLE,             /*  9 */
	MC_STATE_FAULT_NOW,             /* 10 */
	MC_STATE_FAULT_OVER,            /* 11 */
	MC_STATE_ICLWAIT,               /* 12 */
	MC_STATE_ALIGN_CHARGE_BOOT_CAP, /* 13 */
	MC_STATE_ALIGN_OFFSET_CALIB,    /* 14 */
	MC_STATE_ALIGN_CLEAR,		    /* 15 */
	MC_STATE_CHARGE_BOOT_CAP,	    /* 16 */
	MC_STATE_OFFSET_CALIB,	        /* 17 */
	MC_STATE_CLEAR,	                /* 18 */
} mc_status_t;

typedef enum {
	MC_DIR_FORWARD,                 /*  0 */
	MC_DIR_BACKWARD,                /*  1 */
} mc_dir_t;

typedef enum {
	MC_MODE_TORQUE,                 /*  0 */
	MC_MODE_SPEED,                  /*  1 */
} mc_ctrl_mode_t;

typedef enum {
	MC_REG_IDLE,                    /*  0 */
	MC_REG_STATUS_READING,          /*  1 */
    MC_REG_FLAGS_READING,           /*  2 */
	MC_REG_FAULT_ACKING,            /*  3 */
	MC_REG_SPEEDM_READING,          /*  4 */
	MC_REG_SPEEDR_READING,          /*  5 */
	MC_REG_BUSVOLT_READING,         /*  6 */
	MC_REG_TORQUEM_READING,         /*  7 */
	MC_REG_TORQUER_READING,         /*  8 */
	MC_REG_HEATSINK_READING,        /*  9 */
	MC_REG_SPEEDKP_READING,         /*  10 */
	MC_REG_SPEEDKI_READING,         /*  11 */
	MC_REG_TORQUEKP_READING,        /*  12 */
	MC_REG_TORQUEKI_READING,        /*  13 */
	MC_REG_CTRLMOD_READING,         /*  14 */
} mc_reg_poll_t;

/* The file descriptor used to manage our TTY over RPMSG */
static int mFdRpmsg = -1;

static int REF_SPEED_MIN = 1000;
static int REF_SPEED_MAX = 4000;
static mc_status_t mMachineStatus = MC_STATE_IDLE;
static int mSwButStart = 0;
static int mSwButRamp = 0;
static int mSwSpeedRef = 0;
static int mSwSpeedRefMem = 0;
static int mSwSpeedRefChanged = 0;
static int mSwTorqueRef = 0;
static int mSwTorqueRefChanged = 0;
static int mSwCtrlChanged = 0;
static mc_ctrl_mode_t mSwCtrlMode = MC_MODE_SPEED;
static int mSwSpeedKpChanged = 0;
static int mSwSpeedKp = 0;
static int mSwSpeedKiChanged = 0;
static int mSwSpeedKi = 0;
static int mSwTorqueKpChanged = 0;
static int mSwTorqueKp = 0;
static int mSwTorqueKiChanged = 0;
static int mSwTorqueKi = 0;
static int mSwRampRef = 1500;
static int mSwRampDur = 200;
static mc_dir_t mSwDirection = MC_DIR_FORWARD;
static int mSwStroboChanged = 0;
static int mSwStroboEnabled = 0;
static int mSwStroboSpeed;
static int mSwStroboSpeedSaved;
static int mSwSWdogChanged = 0;
static int mMachineState = MC_STATE_CLEAR;
static mc_reg_poll_t mFocGetReg = MC_REG_IDLE;
static int mTargetSpeedValue = -20000;
static int mTargetSpeedReference = -20000;
static int mTargetBusVoltage = 0;
static int mTargetTorqueValue = -20000;
static int mTargetTorqueReference = -20000;
static int mTargetHeatSink = 0;
static int mTargetSpeedKp=0;
static int mTargetSpeedKi=0;
static int mTargetTorqueKp=0;
static int mTargetTorqueKi=0;
static int mTargetCtrlMode=-1;
static int mTargetAck;
static int mTargetFlags;
static int mTargetTmpVal;
char ctrlFormBuff[400];
char dirFormBuff[400];
char speed1FormBuff[400];
char speed2FormBuff[400];
char kp1FormBuff[400];
char kp2FormBuff[400];
char ki1FormBuff[400];
char ki2FormBuff[400];
char ramp1FormBuff[600];
char ramp2FormBuff[540];
char stroboFormBuff[540];

static int virtual_tty_send_command(mc_action_t action, int d1, int d2);

// This list matches the order of the above action enums
static motor_action_t actionEvents[] = {
	{"", 0, ACTION_ENABLE},
	{"motorGetStatus", MC_ACTION_GET_STATUS, ACTION_GET},
	{"motorGetControlMode", MC_ACTION_GET_CONTROL_MODE, ACTION_GET},
	{"motorStart", MC_ACTION_START_MOTOR, ACTION_ENABLE},
	{"motorStop", MC_ACTION_STOP_MOTOR, ACTION_ENABLE},
	{"motorSetSpeedValue", MC_ACTION_SET_SPEED_VALUE, ACTION_SET},
	{"motorSetSpeedRef", MC_ACTION_SET_SPEED_REFERENCE, ACTION_SET},
	{"motorSetSpeedKp", MC_ACTION_SET_SPEED_KP, ACTION_SET},
	{"motorSetSpeedKi", MC_ACTION_SET_SPEED_KI, ACTION_SET},
	{"motorGetSpeedValue", MC_ACTION_GET_SPEED_VALUE, ACTION_GET},
	{"motorGetSpeedRef", MC_ACTION_GET_SPEED_REFERENCE, ACTION_GET},
	{"motorGetBusVoltage", MC_ACTION_GET_BUS_VOLTAGE, ACTION_GET},
	{"motorGetTorqueValue", MC_ACTION_GET_TORQUE_VALUE, ACTION_GET},
	{"motorGetTorqueRef", MC_ACTION_GET_TORQUE_REFERENCE, ACTION_GET},
	{"motorGetHeatSink", MC_ACTION_GET_HEAT_SINK, ACTION_GET},
	{"motorSetSpeedRamp", MC_ACTION_SET_SPEED_RAMP, ACTION_SET2},
	{"motorSetStroboscopeSpeed", MC_ACTION_SET_STROBOSCOPE_SPEED, ACTION_SET},
	{"motorToggleStroboscope", MC_ACTION_TOGGLE_STROBOSCOPE, ACTION_SET},
	{NULL, 0,0},
};

static char MC_state[19][20] = {"IDLE", "IDLE_ALIGN", "ALIGN", "IDLE_START", "START", "START_RUN", "RUN", "ANY_STOP",
    "STOP", "STOP_IDLE", "FAULT_NOW", "FAULT_OVER", "ICLWAIT", "AL_CHARGE_BOOT_CAP", "AL_OFFSET_CALIB",
    "AL_CLEAR", "CHARGE_BOOT_CAP", "OFFSET_CALIB", "CLEAR"};
    
static char MC_ctrlmod[2][20] = {"CTRL_TORQUE", "CTRL_SPEED"};

#define SB_CHANNEL_NAME "motorControl"

#define SB_EVENT_FIRMWARE_SUCCESS	"motorRespFirmwareLoaded"
#define SB_EVENT_FIRMWARE_FAILURE	"motorRespFirmwareFailure"
#define SB_EVENT_STATUS				"motorRespStatus"
#define SB_EVENT_BUS_VOLTAGE		"motorRespBusVoltage"
#define SB_EVENT_TORQUE_VALUE		"motorRespTorqueValue"
#define SB_EVENT_TORQUE_REFERENCE	"motorRespTorqueRef"
#define SB_EVENT_HEAT_SINK			"motorRespHeatSink"
#define SB_EVENT_SPEED_VALUE		"motorRespSpeedValue"
#define SB_EVENT_SPEED_REFERENCE	"motorRespSpeedRef"

#define SB_EVENT_FORMAT				"4s1 value"

typedef struct {
	int status;
	int speed;
	int speedRef;
	int torque;
	int voltage;
	int heat;
    int ctrlmod;
    int speedkp;
    int speedki;
    int torqueRef;
    int stroboscopeState;
    int stroboscopeSpeed;
} status_message_t;     //TODO add strobo_enabled and strobo speed values
#define STATUS_FMT "4s1 status 4s1 speed 4s1 speedRef 4s1 torque 4s1 voltage 4s1 heat 4s1 ctrlmod 4s1 speedkp 4s1 speedki 4s1 torqueRef 4s1 stroboscopeState 4s1 stroboscopeSpeed"

char *sendChannelName = NULL;
gre_io_t *shandle = NULL;
gre_io_serialized_data_t *sendBuffer = NULL;

static char mByteBuffer[512];
static mc_action_t mc_action = MC_ACTION_NOTHING;

static pthread_t thread;

static char FOCuartRx[] = {0,0,0,0,0,0,0,0,0,0};
static int FOCuartRxCnt = 0;


/********************************************************************************
Copro functions allowing to manage a virtual TTY over RPMSG
*********************************************************************************/
int copro_isFwRunning(void)
{
	int fd;
	size_t byte_read;
	int result = 0;
	unsigned char bufRead[MAX_BUF];
	fd = open("/sys/class/remoteproc/remoteproc0/state", O_RDWR);
	if (fd < 0) {
		printf("Error opening remoteproc0/state, err=-%d\n", errno);
		return (errno * -1);
	}
	byte_read = (size_t) read (fd, bufRead, MAX_BUF);
	if (byte_read >= strlen("running")) {
		char* pos = strstr((char*)bufRead, "running");
		if(pos) {
			result = 1;
		}
	}
	close(fd);
	return result;
}

int copro_stopFw(void)
{
	int fd;
	fd = open("/sys/class/remoteproc/remoteproc0/state", O_RDWR);
	if (fd < 0) {
		printf("Error opening remoteproc0/state, err=-%d\n", errno);
		return (errno * -1);
	}
	write(fd, "stop", strlen("stop"));
	close(fd);
	return 0;
}

int copro_startFw(void)
{
	int fd;
	fd = open("/sys/class/remoteproc/remoteproc0/state", O_RDWR);
	if (fd < 0) {
		printf("Error opening remoteproc0/state, err=-%d\n", errno);
		return (errno * -1);
	}
	write(fd, "start", strlen("start"));
	close(fd);
	return 0;
}

int copro_getFwPath(char* pathStr)
{
	int fd;
	int byte_read;
	fd = open("/sys/module/firmware_class/parameters/path", O_RDWR);
	if (fd < 0) {
		printf("Error opening firmware_class/parameters/path, err=-%d\n", errno);
		return (errno * -1);
	}
	byte_read = read (fd, pathStr, MAX_BUF);
	close(fd);
	return byte_read;
}

int copro_setFwPath(char* pathStr)
{
	int fd;
	int result = 0;
	fd = open("/sys/module/firmware_class/parameters/path", O_RDWR);
	if (fd < 0) {
		printf("Error opening firmware_class/parameters/path, err=-%d\n", errno);
		return (errno * -1);
	}
	result = write(fd, pathStr, strlen(pathStr));
	close(fd);
	return result;
}

int copro_getFwName(char* pathStr)
{
	int fd;
	int byte_read;
	fd = open("/sys/class/remoteproc/remoteproc0/firmware", O_RDWR);
	if (fd < 0) {
		printf("Error opening remoteproc0/firmware, err=-%d\n", errno);
		return (errno * -1);
	}
	byte_read = read (fd, pathStr, MAX_BUF);
	close(fd);
	return byte_read;
}

int copro_setFwName(char* nameStr)
{
	int fd;
	int result = 0;
	fd = open("/sys/class/remoteproc/remoteproc0/firmware", O_RDWR);
	if (fd < 0) {
		printf("Error opening remoteproc0/firmware, err=-%d\n", errno);
		return (errno * -1);
	}
	result = write(fd, nameStr, strlen(nameStr));
	close(fd);
	return result;
}

int copro_openTtyRpmsg(int modeRaw)
{
	struct termios tiorpmsg;
	mFdRpmsg = open("/dev/ttyRPMSG0", O_RDWR |  O_NOCTTY | O_NONBLOCK);
	if (mFdRpmsg < 0) {
		printf("Error opening ttyRPMSG0, err=-%d\n", errno);
		return (errno * -1);
	}
	/* get current port settings */
	tcgetattr(mFdRpmsg,&tiorpmsg);
	if (modeRaw) {
		tiorpmsg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
				      | INLCR | IGNCR | ICRNL | IXON);
		tiorpmsg.c_oflag &= ~OPOST;
		tiorpmsg.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		tiorpmsg.c_cflag &= ~(CSIZE | PARENB);
		tiorpmsg.c_cflag |= CS8;
	} else {
		/* ECHO off, other bits unchanged */
		tiorpmsg.c_lflag &= ~ECHO;
		/*do not convert LF to CR LF */
		tiorpmsg.c_oflag &= ~ONLCR;
	}
	tcsetattr(mFdRpmsg, TCSANOW, &tiorpmsg);
	return 0;
}

int copro_closeTtyRpmsg(void)
{
	close(mFdRpmsg);
	mFdRpmsg = -1;
	return 0;
}

int copro_writeTtyRpmsg(int len, char* pData)
{
	int result = 0;
	if (mFdRpmsg < 0) {
		printf("Error writing ttyRPMSG0, fileDescriptor is not set\n");
		return mFdRpmsg;
	}
	
    /* print the received frame
    gettimeofday(&tval_after, NULL);
    timersub(&tval_after, &tval_before, &tval_result);
    int n = sprintf(prtmsg, "[%ld.%06ld] tty send %d bytes: ", (long int)tval_result.tv_sec, (long int)tval_result.tv_usec, len);
    for (int i=0; i<len; i++) {
      n += sprintf(prtmsg+n, "%x ", (unsigned int)*(pData+i));
    }
    printf("%s\n", prtmsg); */
	
	result = write(mFdRpmsg, pData, len);
	return result;
}

int copro_readTtyRpmsg(int len, char* pData)
{
	int byte_rd, byte_avail;
	int result = 0;
	if (mFdRpmsg < 0) {
		printf("Error reading ttyRPMSG0, fileDescriptor is not set\n");
		return mFdRpmsg;
	}
	ioctl(mFdRpmsg, FIONREAD, &byte_avail);
	if (byte_avail > 0) {
		if (byte_avail >= len) {
			byte_rd = read (mFdRpmsg, pData, len);
		} else {
			byte_rd = read (mFdRpmsg, pData, byte_avail);
		}
		/*printf("copro_readTtyRpmsg, read successfully %d bytes to %p, [0]=0x%x\n", byte_rd, pData, pData[0]);*/
		result = byte_rd;
	} else {
		result = 0;
	}
	return result;
}
/********************************************************************************
End of Copro functions 
*********************************************************************************/


int64_t
print_time() {
	struct timespec ts;
    clock_gettime(1, &ts);
    printf("Time: %lld\n", (ts.tv_sec * 1000LL) + (ts.tv_nsec / 1000000LL));
}

int acount = 0;

/********************************************************************************
Remote UI functions called thanks microhttpd library (Web server)
*********************************************************************************/
static int KpPosition2Value(int pos) {
    int minVal = 1;
    int maxVal = 100000;
    int minPos = 0;
    int maxPos = 100;
    double minV_f = log((double)minVal);
    double maxV_f = log((double)maxVal);
    double scale_f = (maxV_f - minV_f) / (maxPos - minPos);
    double v_f = exp(minV_f + scale_f * (pos - minPos));
    return round(v_f);
}

static int KpValue2Position(int val) {
    int minVal = 1;
    int maxVal = 100000;
    int minPos = 0;
    int maxPos = 100;
    double minV_f = log((double)minVal);
    double maxV_f = log((double)maxVal);
    double scale_f = (maxV_f - minV_f) / (maxPos - minPos);
    double p_f = minPos + (log(val) - minV_f) / scale_f;
    return round(p_f);
}

static int KiPosition2Value(int pos) {
    int minVal = 1;
    int maxVal = 100000;
    int minPos = 0;
    int maxPos = 100;
    double minV_f = log((double)minVal);
    double maxV_f = log((double)maxVal);
    double scale_f = (maxV_f - minV_f) / (maxPos - minPos);
    double v_f = exp(minV_f + scale_f * (pos - minPos));
    return round(v_f);
}

static int KiValue2Position(int val) {
    int minVal = 1;
    int maxVal = 100000;
    int minPos = 0;
    int maxPos = 100;
    double minV_f = log((double)minVal);
    double maxV_f = log((double)maxVal);
    double scale_f = (maxV_f - minV_f) / (maxPos - minPos);
    double p_f = minPos + (log(val) - minV_f) / scale_f;
    return round(p_f);
}

static ssize_t
file_reader (void *cls, uint64_t pos, char *buf, size_t max)
{
  FILE *file = cls;

  (void) fseek (file, pos, SEEK_SET);
  return fread (buf, 1, max, file);
}

static void
file_free_callback (void *cls)
{
  FILE *file = cls;
  fclose (file);
}

char html_answer_buff[10000];

static int
iterate_post (void *coninfo_cls, enum MHD_ValueKind kind, const char *key,
              const char *filename, const char *content_type,
              const char *transfer_encoding, const char *data, uint64_t off,
              size_t size)
{
  struct connection_info_struct *con_info = coninfo_cls;
  int ret = MHD_YES;

  pthread_mutex_lock(&ttyMutex);
  if (0 == strcmp (key, "refform")) {
    if (0 == strcmp(data, "torque")) {
        mSwCtrlMode = MC_MODE_TORQUE;
        mSwCtrlChanged = 1;
    } else if (0 == strcmp(data, "speed")) {
        mSwCtrlMode = MC_MODE_SPEED;
        mSwCtrlChanged = 1;
    }
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "dirform")) {
    if (0 == strcmp(data, "dirf")) {
        mSwDirection = MC_DIR_FORWARD;
        mSwSpeedRefChanged = 1;
    } else if (0 == strcmp(data, "dirb")) {
        mSwDirection = MC_DIR_BACKWARD;
        mSwSpeedRefChanged = 1;
    }
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "onoff")) {
    mSwButStart = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "m100")) {
    mSwSpeedRef -= 100;
    if (mSwSpeedRef < REF_SPEED_MIN) mSwSpeedRef = REF_SPEED_MIN;
    mSwSpeedRefChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "p100")) {
    mSwSpeedRef += 100;
    if (mSwSpeedRef > REF_SPEED_MAX) mSwSpeedRef = REF_SPEED_MAX;
    mSwSpeedRefChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "speedrefval")) {
    mSwSpeedRef = atoi(data);
    mSwSpeedRefChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "speedkpval")) {
    int pos = atoi(data);
    mSwSpeedKp = KpPosition2Value(pos);
    mSwSpeedKpChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "speedkival")) {
    int pos = atoi(data);
    mSwSpeedKi = KiPosition2Value(pos);
    mSwSpeedKiChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "rampdval")) {
    mSwRampDur = atoi(data);
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "ramprval")) {
    mSwRampRef = atoi(data);
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "rampst")) {
    mSwButRamp = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "togstrob")) {
    if (mSwStroboEnabled == 0) {
        mSwStroboEnabled = 1;
        //mSwStroboSpeed = mSwSpeedRef;
        mSwStroboSpeed = mSwStroboSpeedSaved;
    } else {
        mSwStroboEnabled = 0;
        mSwStroboSpeed = 0;
    }
    mSwStroboChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "mstrob")) {
    mSwStroboSpeed--;
    mSwStroboChanged = 1;
    ret = MHD_NO;
  }
  if (0 == strcmp (key, "pstrob")) {
    mSwStroboSpeed++;
    mSwStroboChanged = 1;
    ret = MHD_NO;
  }
  pthread_mutex_unlock(&ttyMutex);

  return ret;
}

static void
request_completed (void *cls, struct MHD_Connection *connection,
                   void **con_cls, enum MHD_RequestTerminationCode toe)
{
  struct connection_info_struct *con_info = *con_cls;

  if (NULL == con_info)
    return;

  if (con_info->connectiontype == POST)
    {
      MHD_destroy_post_processor (con_info->postprocessor);
      if (con_info->answerstring)
        free (con_info->answerstring);
    }

  free (con_info);
  *con_cls = NULL;
}

static int HtmlMotorPage(struct MHD_Connection *connection) {
  pthread_mutex_lock(&ttyMutex);
  if (mTargetCtrlMode == MC_MODE_TORQUE) {
      sprintf(ctrlFormBuff, "<form method='post' name='refform'><td class='head'> Control mode : </td><td class='head'><input type='radio' name='ctrlform' value='speed' onclick='this.form.submit();' class='radio1' >Speed</td><td class='head'><input type='radio' name='ctrlform' value='torque' checked='checked' onclick='this.form.submit();' class='radio1' >Torque</td></form>");
  } else {
      sprintf(ctrlFormBuff, "<form method='post' name='refform'><td class='head'> Control mode : </td><td class='head'><input type='radio' name='ctrlform' value='speed' checked='checked' onclick='this.form.submit();' class='radio1' >Speed</td><td class='head'><input type='radio' name='ctrlform' value='torque' onclick='this.form.submit();' class='radio1' >Torque</td></form>");
  }
  if (mSwDirection == MC_DIR_FORWARD) {
      if (mMachineState == MC_STATE_IDLE) {
          sprintf(dirFormBuff, "<form method='post' name='dirform'><td class='head'> Direction :</td><td class='head'><input type='radio' name='dirform' value='dirf' id='dirf' checked='checked' onclick='this.form.submit();' class='radio1' />Forward</td><td class='head'><input type='radio' name='dirform' value='dirb' id='dirb' onclick='this.form.submit();' class='radio1' />Backward</td></form>");
      } else {
          sprintf(dirFormBuff, "<form method='post' name='dirform'><td class='head'> Direction :</td><td class='head'><input type='radio' name='dirform' value='dirf' id='dirf' checked='checked' disabled class='radio1' />Forward</td><td class='head'><input type='radio' name='dirform' value='dirb' id='dirb' disabled class='radio1' />Backward</td></form>");
      }
  } else {
      if (mMachineState == MC_STATE_IDLE) {
          sprintf(dirFormBuff, "<form method='post' name='dirform'><td class='head'> Direction :</td><td class='head'><input type='radio' name='dirform' value='dirf' id='dirf' onclick='this.form.submit();' class='radio1' />Forward</td><td class='head'><input type='radio' name='dirform' value='dirb' id='dirb' checked='checked' onclick='this.form.submit();' class='radio1' />Backward</td></form>");
      } else {
          sprintf(dirFormBuff, "<form method='post' name='dirform'><td class='head'> Direction :</td><td class='head'><input type='radio' name='dirform' value='dirf' id='dirf' disabled class='radio1' />Forward</td><td class='head'><input type='radio' name='dirform' value='dirb' id='dirb' checked='checked' disabled class='radio1' />Backward</td></form>");
      }
  }
  sprintf(kp1FormBuff, "<td class='head'> Speed Kp : </td><td class='minkslider'></td><td class='valslider'><span id='kpval'>%d</span></td><td class='maxkslider'></td>",
        mSwSpeedKp
  );
  sprintf(kp2FormBuff, "<form method='post' name='kpform'><td class='head'> </td><td class='minkslider'>%d</td><td class='kslider' ><input style=\"width:98%\" type='range' name='speedkpval' value='%d' min='%d' max='%d' oninput='skpSliderChange(this.value)' onchange='this.form.submit();'/></td><td class='maxkslider'>%d</td></form>",
        SPEED_KP_VAL_MIN, KpValue2Position(mSwSpeedKp), SPEED_KP_POS_MIN, SPEED_KP_POS_MAX, SPEED_KP_VAL_MAX
  );
  sprintf(ki1FormBuff, "<td class='head'> Speed Ki :</td><td class='minkslider'></td><td class='valslider'><span id='kival'>%d</span></td><td class='maxkslider'></td>",
        mSwSpeedKi
  );
  sprintf(ki2FormBuff, "<form method='post' name='kiform'><td class='head'> </td><td class='minkslider'>%d</td><td class='kslider' ><input style=\"width:98%\" type='range' name='speedkival' value='%d' min='%d' max='%d' oninput='skiSliderChange(this.value)' onchange='this.form.submit();'/></td><td class='maxkslider'>%d</td></form>",
        SPEED_KI_VAL_MIN, KiValue2Position(mSwSpeedKi), SPEED_KI_POS_MIN, SPEED_KI_POS_MAX, SPEED_KI_VAL_MAX
  );
  if (mMachineState == MC_STATE_RUN) {
      sprintf(ramp1FormBuff, "<td class='head'> Ramp duration (ms):</td><form method='post' name='rampdurform'><td class='rampslider'><input style=\"width:98%\" name='rampdval' type='range' min='%d' max='%d' value='%d' oninput='rampdSliderChange(this.value)' onchange='this.form.submit();'/></td></form><td class='ramplabel'> <span id='rampDVal'>%d</span></td><form method='post' name='rampstartform'><td class='rampbut'> <input style=\"width:100%\" type='submit' name='rampst' class='onoff' value='Exec ramp' /></td></form>",
            RAMP_DUR_MIN, RAMP_DUR_MAX, mSwRampDur, mSwRampDur
      );
  } else {
      sprintf(ramp1FormBuff, "<td class='head'> Ramp duration (ms):</td><form method='post' name='rampdurform'><td class='rampslider'><input style=\"width:98%\" name='rampdval' type='range' min='%d' max='%d' value='%d' oninput='rampdSliderChange(this.value)' onchange='this.form.submit();'/></td></form><td class='ramplabel'> <span id='rampDVal'>%d</span></td><td class='rampbut'> </td>",
            RAMP_DUR_MIN, RAMP_DUR_MAX, mSwRampDur, mSwRampDur
      );
  }
  sprintf(ramp2FormBuff, "<td class='head'> Ramp reference (rpm):</td><form method='post' name='ramprefform'><td class='rampslider'><input style=\"width:98%\" name='ramprval' type='range' min='%d' max='%d' value='%d' oninput='ramprSliderChange(this.value)' onchange='this.form.submit();'/></td></form><td class='ramplabel'> <span id='rampRVal'>%d</span></td><td class='rampbut'></td>",
        REF_SPEED_MIN, REF_SPEED_MAX, mSwRampRef, mSwRampRef
  );
  sprintf(speed1FormBuff, "<td class='head'> Speed Reference : </td><td class='minkslider'></td><td class='valslider'><span id='speedRefVal'>%d</span></td><td class='maxkslider'></td>",
        mSwSpeedRef
  );
  sprintf(speed2FormBuff, "<form method='post' name='speedform'><td class='head'> </td><td class='minkslider'>%d</td><td class='kslider' ><input style=\"width:98%\" type='range' name='speedrefval' value='%d' min='%d' max='%d' step='100' oninput='speedSliderChange(this.value)' onchange='this.form.submit();'/></td><td class='maxkslider'>%d</td></form>",
        REF_SPEED_MIN, mSwSpeedRef, REF_SPEED_MIN, REF_SPEED_MAX, REF_SPEED_MAX
  );
  if (mMachineState == MC_STATE_RUN) {
      sprintf(stroboFormBuff, "<form method='post' name='strobSform'><td class='strob' > <input style=\"width:80%\" type='submit' name='togstrob' class='butstrob' value='Enable / Disable' /></td></form><form method='post' name='strobMform'><td class='strob' > <input style=\"width:50%\" type='submit' name='mstrob' class='butstrob' value='-1 rpm' /></td></form><form method='post' name='strobPform'><td class='strob' > <input style=\"width:50%\" type='submit' name='pstrob' class='butstrob' value='+1 rpm' /></td></form>");
  } else {
      sprintf(stroboFormBuff, "<td class='strob' ></td><td class='strob' ></td><td class='strob' ></td>");
  }
  
  memset(html_answer_buff, 0, sizeof html_answer_buff);
  sprintf(html_answer_buff,
    "<!DOCTYPE html>"
    "<html>"
    " <head>"
    "  <meta charset=\"utf-8\" />"
    "  <meta http-equiv=\"refresh\" content=\"5\">"
    "  <style type='text/css'>"
    "   section { margin-bottom: 15px; font-size: 20px; }"
    "   table { margin-left:10px; border: 1px solid black; border-collapse: collapse; }"
    "   caption { border: 1px solid black; font-size: 24px; font-weight: bold; background-color: #87ceeb; }"
    "   td { border: none; font-size: 20px; }"
    "   input { padding:3px; font-size: 20px; }"
    "   .radio1 { width: 33%; height: 1.2em; }"
    "   .onoff {text-align: center; color: white; background-color: #1020f4;}"
    "   .head { width: 33%; color: black;} "
    "   .valmeas { width: 25%; color: blue;}"
    "   .headmeas { width: 25%;}"
    "   .kslider { width: 62%; }"
    "   .minkslider { width: 5%;  text-align: right; }"
    "   .maxkslider { width: 10%; }"
    "   .valslider { width: 62%; text-align: center; color: blue;}"
    "   .rampslider { width: 39%; }"
    "   .rampbut { width: 14%; }"
    "   .ramplabel { width: 14%; color: blue; }"
    "   .capt {margin-top: 15px; margin-bottom: 15px; font-size: 44px; font-weight: italic; text-align: center; background-color: #cccccc; }"
    "   .strob { width: 20%; } "
    "   .valstrob { border: 1px solid black; width: 7%; text-align: center; color: blue;} "
    "   .butstrob { width: 20%; margin-left:10px; text-align: center; color: white; background-color: #1020f4;} "
    "  </style>"
    "  <script type='text/javascript' >"
    "   function speedSliderChange(val){ document.getElementById('speedRefVal').innerHTML = val; }"
    "   function rampdSliderChange(val){ document.getElementById('rampDVal').innerHTML = val; }"
    "   function ramprSliderChange(val){ document.getElementById('rampRVal').innerHTML = val; }"
    "   function skpSliderChange(val){ document.getElementById('kpval').innerHTML = val; }"
    "   function skiSliderChange(val){ document.getElementById('kival').innerHTML = val; }"
    "  </script>"
    " </head>"
    " <body bgcolor='#FFFFFF'>"
    "  <header>"
    "   <table width='67%'><caption class='capt'>STFocMotorControl<img src=\"ST_icon2.jpg\" align='right'/></caption></table>"
    "  </header>"
    "  <section>"
    // ctrl mode
    "   <table width='67%'><caption>Control</caption><tbody><tr >%s</tr ></tbody></table>"
    // dir
    "   <table width='67%'><tbody><tr >%s</tr ></tbody></table>"
    // Kp
    "   <table width='67%'><tbody><tr >%s</tr ><tr >%s</tr ></tbody></table>"
    // Ki
    "   <table width='67%'><tbody><tr >%s</tr ><tr >%s</tr ></tbody></table>"
    // Ramp
    "   <table width='67%'><tbody><tr >%s</tr ><tr >%s</tr ></tbody></table>"
    // Ref
    "   <table width='67%'><tbody><tr >%s</tr ><tr >%s</tr ></tbody></table>"
    // buttons
    "   <table width='67%'><tbody><tr >"
    "       <form method='post' name='m100form'><td class='head' align='center'> <input type='submit' name='m100' class='onoff' value='-100' /></td></form>"
    "       <form method='post' name='onoffform'><td class='head' align='center'> <input style=\"width:100%\" type='submit' name='onoff' class='onoff' value='Start / Stop' /></td></form>"
    "       <form method='post' name='p100form'><td class='head' align='center'> <input type='submit' name='p100' class='onoff' value='+100' /></td></form>"
    "   </tr ></tbody></table>"
    // strobo
    "   <table width='67%'><tbody><tr >"
    "     <td class='head'> Stroboscope speed(rpm) : </td><td class='valstrob'>%d</td>%s"
    "   </tr ></tbody></table>"
    "  </section>"
    "   <table width=\"67%\">"
    "    <caption>Measurements</caption>"
    "    <tbody>"
    "     <tr ><td class='headmeas'>Current state:</td><td class='valmeas'>%s</td><td class='headmeas'>Control mode:</td><td class='valmeas'>%s</td></tr >"
    "     <tr ><td class='headmeas'>Speed reference (rpm) :</td><td class='valmeas'>%d</td><td class='headmeas'>Speed value (rpm) :</td><td class='valmeas'>%d</td></tr >"
    "     <tr ><td class='headmeas'>Speed Kp:</td><td class='valmeas'>%d</td><td class='headmeas'>Speed Ki:</td><td class='valmeas'>%d</td></tr >"
    "     <tr ><td class='headmeas'>Torque reference:</td><td class='valmeas'>%d</td><td class='headmeas'>Torque value:</td><td class='valmeas'>%d</td></tr >"
    "     <tr ><td class='headmeas'>Bus voltage (V) :</td><td class='valmeas'>%d</td><td class='headmeas'>Heatsink temperature (°C) :</td><td class='valmeas'>%d</td></tr >"
//    "     <tr ><td class='headmeas'>Stroboscope speed (rpm) :</td><td class='valmeas'>%d</td><td class='headmeas'></td><td class='valmeas'></td></tr >"
    "     <tr ><td class='headmeas'>FW name :</td><td class='valmeas'>%s</td><td class='headmeas'></td><td class='valmeas'></td></tr >"
    "    </tbody>"
    "   </table>"
    "  </body>"
    "</html>", 
    ctrlFormBuff, dirFormBuff, kp1FormBuff, kp2FormBuff, ki1FormBuff, ki2FormBuff, ramp1FormBuff, ramp2FormBuff, speed1FormBuff, speed2FormBuff, 
    mSwStroboSpeed, stroboFormBuff, MC_state[mMachineState], MC_ctrlmod[mTargetCtrlMode], mTargetSpeedReference, 
    mTargetSpeedValue, mTargetSpeedKp, mTargetSpeedKi, mTargetTorqueReference, mTargetTorqueValue, 
    mTargetBusVoltage, mTargetHeatSink
//    ,mSwStroboSpeed
    ,FIRM_NAME
    );
    //printf("Html (len=%d) page=%s\n FWname=%s\n", strlen (html_answer_buff), html_answer_buff, FIRM_NAME);
  pthread_mutex_unlock(&ttyMutex);
  
  struct MHD_Response *response;
  int ret;
  
  response =
    MHD_create_response_from_buffer (strlen (html_answer_buff), (void *) html_answer_buff, 
				     MHD_RESPMEM_PERSISTENT);
  ret = MHD_queue_response (connection, MHD_HTTP_OK, response);
  MHD_destroy_response (response);
  return ret;
}

static int
answer_to_connection (void *cls, struct MHD_Connection *connection,
                      const char *url, const char *method,
                      const char *version, const char *upload_data,
                      size_t *upload_data_size, void **con_cls)
{
  struct MHD_Response *response;
  int ret;
  FILE *file;
  struct stat buf;
  if (NULL == *con_cls) {
    struct connection_info_struct *con_info;
    con_info = malloc (sizeof (struct connection_info_struct));
    if (NULL == con_info) {
        return MHD_NO;
    }
    con_info->answerstring = NULL;
    if (0 == strcmp (method, "POST")) {
        con_info->postprocessor = MHD_create_post_processor (connection, POSTBUFFERSIZE,
            iterate_post, (void *) con_info);
        if (NULL == con_info->postprocessor) {
              free (con_info);
              return MHD_NO;
        }
        con_info->connectiontype = POST;
    } else {
        con_info->connectiontype = GET;
    }
    *con_cls = (void *) con_info;
    return MHD_YES;
  }
  if (0 == strcmp (method, "POST")) {
    struct connection_info_struct *con_info = *con_cls;
    if (*upload_data_size != 0) {
        MHD_post_process (con_info->postprocessor, upload_data,	
                *upload_data_size);
        *upload_data_size = 0;
        return MHD_YES;
    }
  }
  if (0 == strcmp (method, "GET")) {
    if ( (0 == stat (&url[1], &buf)) && (S_ISREG (buf.st_mode)) ) {
        file = fopen (&url[1], "rb");
        if (file != NULL) {
            response = MHD_create_response_from_callback (buf.st_size, 32 * 1024,     /* 32k PAGE_NOT_FOUND size */
                    &file_reader, file, &file_free_callback);
            if (response == NULL) {
                fclose (file);
                return MHD_NO;
            }
            ret = MHD_queue_response (connection, MHD_HTTP_OK, response);
            MHD_destroy_response (response);    
            return ret;            
        }
    }
  }
  
  ret = HtmlMotorPage(connection);

  return ret;
}
/*************************************************************************************
End of Remote UI functions
*************************************************************************************/

static int
send_message(char *message, int value)
{
	if (shandle == NULL) {
		shandle = gre_io_open(sendChannelName, GRE_IO_TYPE_WRONLY);
		if (shandle == NULL) {
			printf("Unable to open send channel: %s\n", sendChannelName);
			return -1;
		}
	}

	sendBuffer = gre_io_serialize(NULL, NULL, message, "4s1 value", &value, sizeof(value));
	gre_io_send(shandle, sendBuffer);
}

static int
send_status(status_message_t *msg)
{
	if (shandle == NULL) {
		shandle = gre_io_open(sendChannelName, GRE_IO_TYPE_WRONLY);
		if (shandle == NULL) {
			printf("Unable to open send channel: %s\n", sendChannelName);
			return -1;
		}
	}

	sendBuffer = gre_io_serialize(NULL, NULL, SB_EVENT_STATUS, STATUS_FMT, msg, sizeof(*msg));
	gre_io_send(shandle, sendBuffer);
}

char computeCRC(char *bb, uint8_t len)
{
	short sum = 0;
	for (int i = 0; i < len - 1; i++)
		sum += (bb[i] & 0xFF);

	char res = (char) (sum & 0xFF);
	res += (char) (sum >> 8);
	return res;
}


static void sleep_ms(int milliseconds)
{
	usleep(milliseconds * 1000);
}

#define READ_ACK	1
#define READ_DATA	2

static int virtual_tty_process_read_data(mc_action_t action, char *data, int size, int *retVal)
{
	int ret = 0;
    // print the received frame
    gettimeofday(&tval_after, NULL);
    timersub(&tval_after, &tval_before, &tval_result);
    int n = sprintf(prtmsg, "[%ld.%06ld] tty got %d bytes: ", (long int)tval_result.tv_sec, (long int)tval_result.tv_usec, size);
    for (int i=0; i<size; i++) {
      n += sprintf(prtmsg+n, "%x ", (unsigned int) *(data+i));
    }
    //printf("%s\n", prtmsg);
	
	if (FOCuartRxCnt == 0) {
		char stFound = 0;
		for (int i = 0; i < size; i++) {
			if (!stFound) {
				if ((data[i] & 0xFF) == 0xF0) {/*good ack*/
					stFound = 1;
					FOCuartRx[FOCuartRxCnt++] = data[i];
				}
			} else {
				FOCuartRx[FOCuartRxCnt++] = data[i];
			}
		}
	}
	// check if frame is complete
	if (FOCuartRxCnt >= (FOCuartRx[1]+3)) {
		// job is done
		if (FOCuartRx[1] == 0x00) { /*length is 0 means ack*/
			*retVal = READ_ACK;
			ret = 1;
		} else if (FOCuartRx[1] == 0x01) { /*length is 1 means status*/
			short val = (short)(FOCuartRx[2]  & 0xFF);
			ret = 1;
            *retVal = val;
		} else if (FOCuartRx[1] == 0x02) { /*length is 2*/
			short val = (short)((FOCuartRx[2]  & 0xFF) | (FOCuartRx[3]  & 0xFF) << 8);
			ret = 1;
			*retVal = val;
		} else if (FOCuartRx[1] == 0x04) { /*length is 4 means speed*/
			int speed = (FOCuartRx[2]  & 0xFF) | (FOCuartRx[3]  & 0xFF) << 8 | (FOCuartRx[4]  & 0xFF) << 16 | (FOCuartRx[5]  & 0xFF) << 24;
			ret = 1;
			*retVal = speed;
		} else {
		    printf("other Ln not managed!!!)\n");
		}
		FOCuartRxCnt = 0;
	}

	return ret;
}

// read and return the data in *value
static int virtual_tty_wait_answer(mc_action_t action, int *value)
{
	int read = 0;
	int ret = 0;
    
    do {
        read = copro_readTtyRpmsg(512, mByteBuffer);
    } while (read <= 0);
	if (read > 0) {
		ret = virtual_tty_process_read_data(action, mByteBuffer, read, value);
	}
	return ret;
}

static int virtual_tty_send_command(mc_action_t action, int actionData, int actionData2) {
	int ret;
	int retValue = 1;

    switch (action)
    {
    case MC_ACTION_START_MOTOR:
        {
            char TXbuffer5[] = {0x23, 0x01, 0x01, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("start motor command fails\n");
        }
        break;
    case MC_ACTION_STOP_MOTOR:
        {
            char TXbuffer5[] = {0x23, 0x01, 0x02, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("stop motor command fails\n");
        }
        break;
    case MC_ACTION_SET_SPEED_KP:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x05, 0x00, 0x00, 0x00};
            int len = sizeof(TXbuffer5);
            TXbuffer5[4] = (int8_t)(actionData >> 8);
            TXbuffer5[3] = (int8_t)actionData;
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("Set speed value command fails\n");
        }
        break;
    case MC_ACTION_SET_SPEED_KI:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x06, 0x00, 0x00, 0x00};
            int len = sizeof(TXbuffer5);
            TXbuffer5[4] = (int8_t)(actionData >> 8);
            TXbuffer5[3] = (int8_t)actionData;
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("Set speed value command fails\n");
        }
        break;
    case MC_ACTION_SET_SPEED_REFERENCE:
        {
            char TXbuffer5[] = {0x21, 0x05, 0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
            int len = sizeof(TXbuffer5);
            int d = (int)actionData;
            TXbuffer5[6] = (char)(d >> 24);
            TXbuffer5[5] = (char)(d >> 16);
            TXbuffer5[4] = (char)(d >> 8);
            TXbuffer5[3] = (char)d;
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[7] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("Set speed ref command fails\n");
        }
        break;
    case MC_ACTION_SET_SPEED_RAMP:
        {
            char TXbuffer5[] = {0x27, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            int len = sizeof(TXbuffer5);
            TXbuffer5[5] = (char)(actionData2 >> 24);
            TXbuffer5[4] = (char)(actionData2 >> 16);
            TXbuffer5[3] = (char)(actionData2 >> 8);
            TXbuffer5[2] = (char)(actionData2);
            TXbuffer5[7] = (char)(actionData >> 8);
            TXbuffer5[6] = (char)actionData;
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[8] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("Set speed ramp command fails\n");
        }
        break;
    case MC_ACTION_SET_STROBOSCOPE_SPEED:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x83, 0x00, 0x00, 0x00};
            int len = sizeof(TXbuffer5);
            TXbuffer5[4] = (char)(actionData >> 8);
            TXbuffer5[3] = (char)actionData;
            //printf("SET S: %d\n", actionData);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("Set speed ramp command fails\n");
        }
        break;
    case MC_ACTION_GET_BUS_VOLTAGE:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x19, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get bus voltage command fails\n");
        }
        break;
	case MC_ACTION_GET_STATUS:
		{
			char TXbuffer5[] = {0x22, 0x01, 0x02, 0x00};
			int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
			TXbuffer5[3] = crc;
			ret = copro_writeTtyRpmsg(len, TXbuffer5);
			if (ret != len)
				printf("get status command fails\n");
		}
		break;
    case MC_ACTION_GET_SPEED_VALUE:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x1E, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get speed value command fails\n");
        }
        break;
    case MC_ACTION_GET_TORQUE_VALUE:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x1F, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque value command fails\n");
        }
        break;
    case MC_ACTION_GET_HEAT_SINK:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x1A, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_SPEED_REFERENCE:
        {
            //char TXbuffer5[] = {0x22, 0x01, 0x04, 0x00};
            char TXbuffer5[] = {0x22, 0x01, 0x5B, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get speed reference command fails\n");
        }
        break;
    case MC_ACTION_SET_TORQUE_REFERENCE:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x08, 0x00, 0x00, 0x00};
            TXbuffer5[4] = (char)(actionData >> 8);
            TXbuffer5[3] = (char)actionData;
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("set torque reference command fails\n");
        }
        break;
    case MC_ACTION_SET_CONTROL_MODE:
        {
            char TXbuffer5[] = {0x21, 0x02, 0x03, 0x00, 0x00};
            TXbuffer5[3] = (char)actionData;
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[4] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("set torque Ki command fails\n");
        }
        break;
    case MC_ACTION_SET_TORQUE_KP:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x09, 0x00, 0x00, 0x00};
            TXbuffer5[4] = (char)(actionData >> 8);
            TXbuffer5[3] = (char)actionData;
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("set torque Kp command fails\n");
        }
        break;
    case MC_ACTION_SET_TORQUE_KI:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x0A, 0x00, 0x00, 0x00};
            TXbuffer5[4] = (char)(actionData >> 8);
            TXbuffer5[3] = (char)actionData;
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("set torque Ki command fails\n");
        }
        break;
    case MC_ACTION_SET_WDOG_REFILL:
        {
            char TXbuffer5[] = {0x21, 0x03, 0x84, 0x00, 0x00, 0x00};
            TXbuffer5[4] = (char)(actionData >> 8);
            TXbuffer5[3] = (char)actionData;
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[5] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("refill WDOG command fails\n");
        }
        break;
    case MC_ACTION_GET_TORQUE_REFERENCE:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x08, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_SPEED_KP:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x05, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_SPEED_KI:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x06, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_TORQUE_KP:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x09, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_TORQUE_KI:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x0A, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get torque reference command fails\n");
        }
        break;
    case MC_ACTION_GET_CONTROL_MODE:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x03, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("get control mode command fails\n");
        }
        break;
    case MC_ACTION_FAULT_ACK:
        {
            char TXbuffer5[] = {0x23, 0x01, 0x7, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("fault ack command fails\n");
        }
        break;
    case MC_ACTION_GET_FLAGS:
        {
            char TXbuffer5[] = {0x22, 0x01, 0x1, 0x00};
            int len = sizeof(TXbuffer5);
            char crc = computeCRC(TXbuffer5, len);
            TXbuffer5[3] = crc;
            ret = copro_writeTtyRpmsg(len, TXbuffer5);
            if (ret != len)
                printf("fault ack command fails\n");
        }
        break;

        case MC_ACTION_NOTHING:
    default:
        /* no action */
        retValue = 0;
        break;
    }

	return retValue;
}

void treatHealth(void) {
	status_message_t msg;
    switch (mFocGetReg) {
        case MC_REG_IDLE:
            mFocGetReg = MC_REG_STATUS_READING;
            break;
        case MC_REG_STATUS_READING:
            virtual_tty_send_command(MC_ACTION_GET_STATUS, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_STATUS, &mTargetTmpVal);
            if (mTargetTmpVal != mMachineState) {
                printf("Change mMachineState from %d to %d\n", mMachineState, mTargetTmpVal);
                mMachineState = mTargetTmpVal;
            }
            if (mMachineState == MC_STATE_FAULT_OVER) {
                mFocGetReg = MC_REG_FAULT_ACKING;
            } else if (mMachineState == MC_STATE_FAULT_NOW) {
                mFocGetReg = MC_REG_FLAGS_READING;
            } else if (mMachineState == MC_STATE_RUN) {
                if (mSwSpeedRefMem > 0) {
                    int speedRef = mSwSpeedRefMem;
                    mSwSpeedRefMem = 0;
                    if (mSwDirection != MC_DIR_FORWARD) {
                        speedRef *= (-1);
                    }
                    virtual_tty_send_command(MC_ACTION_SET_SPEED_REFERENCE, speedRef, 0);
                    virtual_tty_wait_answer(MC_ACTION_SET_SPEED_REFERENCE, &mTargetAck);
                }
                mFocGetReg = MC_REG_SPEEDM_READING;
            } else {
                mFocGetReg = MC_REG_SPEEDM_READING;
            }
            break;
        case MC_REG_FLAGS_READING:
            virtual_tty_send_command(MC_ACTION_GET_FLAGS, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_FLAGS, &mTargetFlags);
            mFocGetReg = MC_REG_SPEEDM_READING;
            break;
        case MC_REG_FAULT_ACKING:
            virtual_tty_send_command(MC_ACTION_FAULT_ACK, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_FAULT_ACK, &mTargetAck);
            mFocGetReg = MC_REG_SPEEDM_READING;
            break;
        case MC_REG_SPEEDM_READING:
            virtual_tty_send_command(MC_ACTION_GET_SPEED_VALUE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_SPEED_VALUE, &mTargetTmpVal);
            if (mTargetTmpVal != mTargetSpeedValue) {
                printf("Change mTargetSpeedValue from %d to %d\n", mTargetSpeedValue, mTargetTmpVal);
                mTargetSpeedValue = mTargetTmpVal;
            }
            mFocGetReg = MC_REG_SPEEDR_READING;
            break;
        case MC_REG_SPEEDR_READING:
            virtual_tty_send_command(MC_ACTION_GET_SPEED_REFERENCE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_SPEED_REFERENCE, &mTargetTmpVal);
            if (mTargetTmpVal != mTargetSpeedReference) {
                printf("Change mTargetSpeedReference from %d to %d\n", mTargetSpeedReference, mTargetTmpVal);
                mTargetSpeedReference = mTargetTmpVal;
            }
            if (mSwSpeedRef == 0) {
                mSwSpeedRef = mTargetTmpVal;    // set the reference value after first read
            }
            mFocGetReg = MC_REG_BUSVOLT_READING;
            break;
        case MC_REG_BUSVOLT_READING:
            virtual_tty_send_command(MC_ACTION_GET_BUS_VOLTAGE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_BUS_VOLTAGE, &mTargetTmpVal);
            if (mTargetTmpVal != mTargetBusVoltage) {
                printf("Change mTargetBusVoltage from %d to %d\n", mTargetBusVoltage, mTargetTmpVal);
                mTargetBusVoltage = mTargetTmpVal;
            }
            mFocGetReg = MC_REG_TORQUEM_READING;
            break;
        case MC_REG_TORQUEM_READING:
            virtual_tty_send_command(MC_ACTION_GET_TORQUE_VALUE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_TORQUE_VALUE, &mTargetTmpVal);
            if (mTargetTmpVal != mTargetTorqueValue) {
                printf("Change mTargetTorqueValue from %d to %d\n", mTargetTorqueValue, mTargetTmpVal);
                mTargetTorqueValue = mTargetTmpVal;
            }
            mFocGetReg = MC_REG_TORQUER_READING;
            break;
        case MC_REG_TORQUER_READING:
            virtual_tty_send_command(MC_ACTION_GET_TORQUE_REFERENCE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_TORQUE_REFERENCE, &mTargetTorqueReference);
            mFocGetReg = MC_REG_HEATSINK_READING;
            break;
        case MC_REG_HEATSINK_READING:
            virtual_tty_send_command(MC_ACTION_GET_HEAT_SINK, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_HEAT_SINK, &mTargetTmpVal);
            if (mTargetTmpVal != mTargetHeatSink) {
                printf("Change mTargetHeatSink from %d to %d\n", mTargetHeatSink, mTargetTmpVal);
                mTargetHeatSink = mTargetTmpVal;
            }
            mFocGetReg = MC_REG_SPEEDKP_READING;
            break;
        case MC_REG_SPEEDKP_READING:
            virtual_tty_send_command(MC_ACTION_GET_SPEED_KP, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_SPEED_KP, &mTargetSpeedKp);
            if (mSwSpeedKp == 0) {
                mSwSpeedKp = mTargetSpeedKp;    // set the reference value after first read
            }
            mFocGetReg = MC_REG_SPEEDKI_READING;
            break;
        case MC_REG_SPEEDKI_READING:
            virtual_tty_send_command(MC_ACTION_GET_SPEED_KI, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_SPEED_KI, &mTargetSpeedKi);
            if (mSwSpeedKi == 0) {
                mSwSpeedKi = mTargetSpeedKi;    // set the reference value after first read
            }
            mFocGetReg = MC_REG_CTRLMOD_READING;
            break;
        case MC_REG_CTRLMOD_READING:
            virtual_tty_send_command(MC_ACTION_GET_CONTROL_MODE, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_GET_CONTROL_MODE, &mTargetCtrlMode);
            msg.status = mMachineState;
            msg.speed = mTargetSpeedValue;
            msg.speedRef = mTargetSpeedReference;
            msg.torque = mTargetTorqueValue;
            msg.voltage = mTargetBusVoltage;
            msg.heat = mTargetHeatSink;
            msg.ctrlmod = mTargetCtrlMode;
            msg.speedkp = mTargetSpeedKp;
            msg.speedki = mTargetSpeedKi;
            msg.torqueRef = mTargetTorqueReference;
            msg.stroboscopeState = mSwStroboEnabled;
            msg.stroboscopeSpeed = mSwStroboSpeed;
            send_status(&msg);
            mFocGetReg = MC_REG_IDLE;
            break;
    }
}

int treatCommand(void) {
    int ret = 0;
    if (mFocGetReg > MC_REG_IDLE) return 0;
    if (mSwSpeedRefChanged) {
        mSwSpeedRefChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            int speedRef = mSwSpeedRef;
            if (speedRef >= REF_SPEED_MIN & speedRef <= REF_SPEED_MAX) {
                mSwSpeedRefMem = mSwSpeedRef;
                if (mSwDirection != MC_DIR_FORWARD) {
                    speedRef *= (-1);
                }
                virtual_tty_send_command(MC_ACTION_SET_SPEED_REFERENCE, speedRef, 0);
                virtual_tty_wait_answer(MC_ACTION_SET_SPEED_REFERENCE, &mTargetAck);
            }
        } else {
            mSwSpeedRefMem = 0;
        }
        if (mSwStroboEnabled) {
            mSwStroboChanged = 1;
            mSwStroboSpeed = mSwSpeedRef;
        } else {
            mSwStroboSpeedSaved = mSwSpeedRef;
        }
        ret = 1;
    } else if (mSwButStart) {
        mSwButStart = 0;
        if (mMachineState == MC_STATE_IDLE) {
            int speedRef = mSwSpeedRef;
            // limit speed at start on Shinano to avoid FAULT_OVER
            if ((REF_SPEED_MIN == 1000)&&(speedRef > 1500)) {
                speedRef = 1500;
                if (mSwDirection != MC_DIR_FORWARD) {
                    speedRef *= (-1);
                }
                virtual_tty_send_command(MC_ACTION_SET_SPEED_REFERENCE, speedRef, 0);
                virtual_tty_wait_answer(MC_ACTION_SET_SPEED_REFERENCE, &mTargetAck);
            }
            virtual_tty_send_command(MC_ACTION_START_MOTOR, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_START_MOTOR, &mTargetAck);
        } else {
            virtual_tty_send_command(MC_ACTION_STOP_MOTOR, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_STOP_MOTOR, &mTargetAck);
        }
        ret = 1;
    } else if (mSwButRamp) {
        mSwButRamp = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            int rampRef = mSwRampRef;
            if (mSwDirection != MC_DIR_FORWARD) {
                rampRef *= (-1);
            }
            virtual_tty_send_command(MC_ACTION_SET_SPEED_RAMP, mSwRampDur, mSwRampRef);
            virtual_tty_wait_answer(MC_ACTION_SET_SPEED_RAMP, &mTargetAck);
        }
        if (mSwStroboEnabled) {
            mSwStroboChanged = 1;
            mSwStroboSpeed = mSwRampRef;
        } else {
            mSwStroboSpeedSaved = mSwRampRef;
        }
        mSwSpeedRef = mSwRampRef;
        ret = 1;
    } else if (mSwTorqueRefChanged) {
        mSwTorqueRefChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            int torqueRef = mSwTorqueRef;
            if (mSwDirection != MC_DIR_FORWARD) {
                torqueRef *= (-1);
            }
            virtual_tty_send_command(MC_ACTION_SET_TORQUE_REFERENCE, torqueRef, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_TORQUE_REFERENCE, &mTargetAck);
        }
        ret = 1;
    } else if (mSwCtrlChanged) {
        mSwCtrlChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_CONTROL_MODE, mSwCtrlMode, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_CONTROL_MODE, &mTargetAck);
        }
        ret = 1;
    } else if (mSwSpeedKpChanged) {
        mSwSpeedKpChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_SPEED_KP, mSwSpeedKp, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_SPEED_KP, &mTargetAck);
        }
        ret = 1;
    } else if (mSwSpeedKiChanged) {
        mSwSpeedKiChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_SPEED_KI, mSwSpeedKi, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_SPEED_KI, &mTargetAck);
        }
        ret = 1;
    } else if (mSwTorqueKpChanged) {
        mSwTorqueKpChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_TORQUE_KP, mSwTorqueKp, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_TORQUE_KP, &mTargetAck);
        }
        ret = 1;
    } else if (mSwTorqueKiChanged) {
        mSwTorqueKiChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_TORQUE_KI, mSwTorqueKi, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_TORQUE_KI, &mTargetAck);
        }
        ret = 1;
    } else if (mSwStroboChanged) {
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            mSwStroboChanged = 0;
            virtual_tty_send_command(MC_ACTION_SET_STROBOSCOPE_SPEED, mSwStroboSpeed, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_STROBOSCOPE_SPEED, &mTargetAck);
        }
        ret = 1;
    } else if (mSwSWdogChanged) {
        mSwSWdogChanged = 0;
        if ((mMachineState == MC_STATE_IDLE) || (mMachineState == MC_STATE_RUN)) {
            virtual_tty_send_command(MC_ACTION_SET_WDOG_REFILL, 0, 0);
            virtual_tty_wait_answer(MC_ACTION_SET_WDOG_REFILL, &mTargetAck);
        }
        ret = 1;
    }

    return ret;
}

void *vitural_tty_thread(void *arg)
{
	int read;

	// open the ttyRPMSG in raw mode
	if (copro_openTtyRpmsg(1)) {
		printf("fails to open the tty RPMESG\n");
		return NULL;
	}

	while (1) {
        pthread_mutex_lock(&ttyMutex);
        if (!treatCommand()) {
            treatHealth();
        }
        pthread_mutex_unlock(&ttyMutex);
        sleep_ms(50);      // give time to UI
	}
}

static int
parse_message(char *name, void *data)
{
	int i = 0;
	int value = (int)data;
	int index, d2; 
	int16_t d1;

//	printf("Received: %s : ", name);
//	print_time();
	while (actionEvents[i].name) {      // search the right action in the list
		if (strcmp(name, actionEvents[i].name) == 0) {
			//printf("Invoking action\n");
			pthread_mutex_lock(&ttyMutex);
			if (actionEvents[i].type == ACTION_SET || actionEvents[i].type == ACTION_SET2) {
				if (data == NULL) {
					printf("Action call for setting data but no data found\n");
				} else if (actionEvents[i].type == ACTION_SET) {
					d1 = *(int16_t *) data;
					//printf("\tAction Data: %d\n", d1);
				} else {
					action_data2_t *d = (action_data2_t *)data;
					d1 = d->v1;
					d2 = d->v2;
				}
			}
			index = i;
            switch (actionEvents[index].action) {
            case MC_ACTION_START_MOTOR:
                mSwButStart = 1;
                printf("MC_ACTION_START_MOTOR mSwButStart = 1\n");
                break;
            case MC_ACTION_STOP_MOTOR:
                mSwButStart = 1;
                printf("MC_ACTION_STOP_MOTOR mSwButStart = 1\n");
                break;
            case MC_ACTION_SET_SPEED_REFERENCE:
                mSwSpeedRef = d1;
                mSwSpeedRefChanged = 1;
                printf("MC_ACTION_SET_SPEED_REFERENCE mSwSpeedRefChanged=1 mSwSpeedRef=%d\n", mSwSpeedRef);
                break;
            case MC_ACTION_SET_SPEED_KP:
                mSwSpeedKp = d1;
                mSwSpeedKpChanged = 1;
                break;
            case MC_ACTION_SET_SPEED_KI:
                mSwSpeedKi = d1;
                mSwSpeedKiChanged = 1;
                break;
            case MC_ACTION_SET_SPEED_RAMP:
                mSwRampDur = d1;
                mSwRampRef = d2;
                mSwButRamp = 1;
                break;
            case MC_ACTION_SET_STROBOSCOPE_SPEED:
                mSwStroboSpeed = d1;
                mSwStroboChanged = 1;
                break;
            case MC_ACTION_TOGGLE_STROBOSCOPE:
                if (mSwStroboEnabled) {
                    mSwStroboEnabled = 0;
                    mSwStroboSpeed = 0;
                } else {
                    mSwStroboEnabled = 1;
                    //mSwStroboSpeed = mSwSpeedRef;
                    mSwStroboSpeed = mSwStroboSpeedSaved;
                }
                mSwStroboChanged = 1;
                break;
            }
			pthread_mutex_unlock(&ttyMutex);
			break;
		}
		i++;
	}
	if (actionEvents[i].name == NULL) {
		printf("No action found\n");
	}
}

static int
greio_receiver() {
    gre_io_t                 *rhandle;
    gre_io_serialized_data_t *buffer = NULL;
    int                       ret;

    rhandle = gre_io_open(SB_CHANNEL_NAME, GRE_IO_TYPE_RDONLY);
    if (rhandle == NULL) {
        fprintf(stderr, "Can't open IO channel [%s] error %d\n", SB_CHANNEL_NAME, errno);
        return -1;
    }
	printf("Successfully opened receive channel: %s\n", SB_CHANNEL_NAME);

    while (1) {
    	char	*revent_name;
    	char    *revent_target;
    	char    *revent_format;
    	void    *revent_vdata;
    	uint8_t	*revent_data;
    	int     offset, i, rnbytes;

        ret = gre_io_receive(rhandle, &buffer);
        if (ret < 0) {
            fprintf(stderr, "Problem receiving data on channel [%s] error %d\n", SB_CHANNEL_NAME, errno);
            break;
        }

    	rnbytes = gre_io_unserialize(buffer, &revent_target, &revent_name, &revent_format, &revent_vdata);
		if (strcmp(revent_name, "quit") == 0) {
			printf("Exit main loop\n");
			break;	
		}
        parse_message(revent_name, revent_vdata);
    }

    gre_io_close(rhandle);

    return 0;
}


int main(int argc, char **argv)
{
	int ret = 0;
	char FwName[30];
    
    gettimeofday(&tval_before, NULL);

	if (argc < 1) {
		fprintf(stderr, "No channel specified\n");
		return 0;
	}
	sendChannelName = strdup(argv[1]);
    
	if (argc > 2) {
        //printf("argc = %d, ARGS = %s %s %s\n", argc, argv[0], argv[1], argv[2]);
        strcpy(FIRM_NAME, argv[2]);
        //printf("FIRM_NAME = %s\n", FIRM_NAME);
    } else {
        strcpy(FIRM_NAME, "rprocttymotwc0100");
    }

	/* check if copro is already running */
	ret = copro_isFwRunning();
	if (ret) {
		// check FW name
		int nameLn = copro_getFwName(FwName);
		
		if (strcmp(FwName, FIRM_NAME) == 0) {
			printf("%s is already running.\n", FIRM_NAME);
			goto fwrunning;
		}else {
			printf("wrong FW running. Try to stop it...\n");
			if (copro_stopFw()) {
				printf("fails to stop firmware\n");
				goto end;
			}
		}
	}
	
setname:	
	/* set the firmware name to load */
	ret = copro_setFwName(FIRM_NAME);
	if (ret <= 0) {
		printf("fails to change the firmware name\n");
		goto end;
	}

	/* start the firmware */
	if (copro_startFw()) {
		printf("fails to start firmware\n");
		goto end;
	}
	/* wait for 1 seconds the creation of the virtual ttyRPMSGx */
	sleep_ms(1000);

fwrunning:

	// notify the backend that the firmware has loaded
	send_message(SB_EVENT_FIRMWARE_SUCCESS, 0);
    
    if(getenv("MOTOR_TYPE")) {
        printf("MOTOR_TYPE=%s", getenv("MOTOR_TYPE"));
        if (strcmp(getenv("MOTOR_TYPE"), "Shinano")==0) {
            // change MAX_SPEED
            REF_SPEED_MIN = 1000;
            REF_SPEED_MAX = 2000;
            printf(" REF_SPEED_MAX = 2000\n");
        } else {
            REF_SPEED_MIN = 1500;
            REF_SPEED_MAX = 4000;
            printf(" REF_SPEED_MAX = 4000\n");
        }
    }

	pthread_mutex_init(&ttyMutex, NULL);
    //pthread_cond_init (&ttyCondvar, NULL);

	if (pthread_create( &thread, NULL, vitural_tty_thread, NULL) != 0) {
		printf("vitural_tty_thread creation fails\n");
		goto end;
	}

	mHttpDaemon = MHD_start_daemon (MHD_USE_SELECT_INTERNALLY, PORT, NULL, NULL,
                            &answer_to_connection, NULL, 
                            MHD_OPTION_NOTIFY_COMPLETED, request_completed,
                            NULL, MHD_OPTION_END);
	if (NULL == mHttpDaemon) {
		printf("MHD_start_daemon fails!!!\n");
	}

	greio_receiver(argv[1]);

end:
	/* check if copro is already running */
	if (copro_isFwRunning()) {
		printf("stop the firmware before exit\n");
		copro_stopFw();
	}
	return ret;
}
