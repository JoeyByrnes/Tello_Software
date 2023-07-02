/* Copyright 2018-2021 The MathWorks, Inc. */

#ifndef xcpsfuncifc_hpp
#define xcpsfuncifc_hpp

#include <stdbool.h>
#include <stdint.h>

#if defined(MATLAB_MEX_FILE)
    #define XCPAPI __declspec(dllimport)
    #if defined (_WIN32)
        #define XCPAPI __declspec(dllimport)
    #elif defined (__linux)
        #define XCPAPI
    #endif
#elif defined(XCP_DLL_EXPORT)
    #if defined (_WIN32)
        #define XCPAPI __declspec(dllexport)
    #elif defined (__linux)
        #define XCPAPI __attribute__((visibility("default")))
    #endif
#else
    #define XCPAPI
#endif

#ifdef __cplusplus
#include <functional>
namespace xcp {
extern "C" {
#endif

enum SIGNALDT_E {
    UBYTE_E = 0,
    SBYTE_E,
    UWORD_E,
    SWORD_E,
    ULONG_E,
    SLONG_E,
    A_UINT64_E,
    A_INT64_E,
    FLOAT16_IEEE_E,
    FLOAT32_IEEE_E,
    FLOAT64_IEEE_E,
    MW_STRING
};

enum MAXSTRLENGTH { MAXLEN = 1024 };

enum INPUT_OUTPUT_DATATYPE { PHYS = 0, RAW, RAW_DOUBLE };

#ifdef __cplusplus
	}
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void* HXCP;
enum ETYPE {
	XCP_ERROR = 0,
	XCP_SUCCESS = 1
};

XCPAPI HXCP xcpm_getHandle(const char* tag);

XCPAPI void xcpm_releaseXCPHandle(const char* tag);

XCPAPI bool xcpm_isConnected(HXCP handle);

XCPAPI const char* xcpm_getErr(HXCP handle);

XCPAPI enum ETYPE xcpm_config_attachUDPTL(HXCP handle,
                                          uint32_t mAddr,
                                          uint16_t mPort,
                                          const double* tl_info);
XCPAPI enum ETYPE xcpm_config_attachCANTL(HXCP handle, const char* tlTag, const double* tl_info);
XCPAPI enum ETYPE xcpm_config_attachCANFDTL(HXCP handle, const char* tlTag, const double* tl_info);
XCPAPI enum ETYPE xcpm_config_registerSlave(HXCP handle,
                                            const char* seedkeylib,
                                            const double* protocol_info,
                                            const double* daq_info,
                                            const double* ev_info,
                                            uint8_t ctr,
                                            double daqTimeout,
                                            const char* measurement_info_JSON,
                                            const char* compuMethod_info_JSON,
                                            const char* compuTab_info_JSON);

XCPAPI bool xcpm_config_isFrameInit();
XCPAPI void xcpm_config_initializeFramePool();

XCPAPI enum ETYPE xcpm_config_udpExecuteRx(HXCP handle);

XCPAPI enum ETYPE xcpm_config_canReceiveCB(HXCP handle, uint8_t* data, uint8_t length);

XCPAPI enum ETYPE xcpm_config_runOnce(HXCP handle, double* daqOverload, double* stimOverload);

XCPAPI enum ETYPE xcpm_config_spawnRxTx(HXCP handle);

#ifdef __cplusplus

XCPAPI void xcpm_bypass_addDAQBypassFcn(const char* dlTag,
                                        std::function<void()> bypassTrig,
                                        uint8_t pri);
#endif

XCPAPI bool xcpm_bypass_daqReady(const char* dlTag);

XCPAPI enum ETYPE xcpm_daqstim_createDaqList(HXCP handle,
                                             uint16_t* dlNum,
                                             const char* dlTag,
                                             uint16_t event_channel_id,
                                             bool stimDirection,
                                             uint8_t priority,
                                             uint8_t rate,
                                             bool timestampEnable,
                                             bool pidOff);

XCPAPI enum ETYPE xcpm_daqstim_addMeasurement(HXCP handle,
                                              uint16_t dlNum,
                                              int32_t measurement_id,
                                              void* port,
                                              int portDT);

XCPAPI enum ETYPE xcpm_daqstim_daq(HXCP handle, uint16_t dlNum, int ioDatatypes);

XCPAPI enum ETYPE xcpm_daqstim_stim(HXCP handle, uint16_t dlNum, int ioDatatypes);

XCPAPI void xcpm_daqstim_readDAQTime(HXCP handle, uint16_t dlNum, double* out);

XCPAPI void xcpm_daqstim_setSTIMTime(HXCP handle, uint16_t dlNum, const double* timePtr);

typedef struct {
    double daqConfigType;
    double overloadIndicator;
    double maxDAQ;
    double maxEventChannel;
    double minDAQ;
    double optimizationType;
    double addressExtension;
    double identificationField;
    double granularityODTEntrySizeDAQ;
    double maxODTEntrySizeDAQ;
    double granularityODTEntrySizeSTIM;
    double maxODTEntrySizeSTIM;
    double bitwiseSTIMSupported;
    double timestampTicks;
    double timestampSize;
    double timestampResolution;
} Daq_real_T;

typedef struct {
    double channelNum;
    double type;
    double maxDAQ;
    double TimeCycle;
    double TimeUnitNr;
    double Priority;
} EventInfo_real_T;

typedef struct {
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double maxCTO;
    double maxDTO;
    double byteOrder;
    double addressGranularity;
} Protocol_real_T;

typedef struct {
    double ipAddress;
    double port;
} XCP_IP_real_T;

#ifdef __cplusplus
}
#endif

#endif
