#ifndef DI_H_
#define DI_H_

#include "ifm_types.h"


// include the newest channel interface for channel 8
#include "../Channels/DI_CH08_1_4.h"
#include "../Channels/DI_CH14_1_4.h"
#include "../Channels/DI_CH20_1_4.h"
#include "../Channels/DI_CH256_1_5.h"

typedef ifm_o3m_AlgoIFOutput_DIA1_1_4 Channel8;
typedef ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4 Channel14;
typedef ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4 Channel20;
typedef ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5 Channel256;

#define FWVARIANT "BF"
#define NUM_SENSOR_PIXELS (1024)

// We copy some received data from channel 8 into this structure.
typedef struct
{
    // the distance image from the sensor
    ifm_o3m_uint16_t distanceData[NUM_SENSOR_PIXELS];
    // the pixel flag matrix from the sensor
    ifm_o3m_uint16_t confidence[NUM_SENSOR_PIXELS];
} distanceData_t;

// We copy some received data from channel 20 into this structure.
typedef struct
{
    // the number of ROI-groups defined on the sensor.
    ifm_o3m_sint32_t numberOfGroups;
    // the X-results for each ROI-group
    ifm_o3m_float32_t groupX[64];
} basicFunctionsData_t;

typedef struct
{
    ifm_o3m_uint8_t DistImageCust_blockageSensitivity;
    ifm_o3m_uint8_t Modulation_Frequency_Mode;
} parameterData_t;

typedef struct
{
    ifm_o3m_uint8_t OpMode;
    ifm_o3m_sint8_t Variant[3];
    ifm_o3m_float32_t mainTemperature;
} statusData_t;

//A datastructure for custom data retrieval
typedef struct
{
	ifm_o3m_uint16_t distanceImageResult_sensorWidth;
	ifm_o3m_uint16_t distanceImageResult_sensorHeight;
	ifm_o3m_float32_t distanceImageResult_X[1024];
	ifm_o3m_float32_t distanceImageResult_Y[1024];
	ifm_o3m_float32_t distanceImageResult_Z[1024];
	ifm_o3m_uint16_t distanceImageResult_amplitude[1024];
	ifm_o3m_uint16_t distanceImageResult_distanceData[1024];
	ifm_o3m_uint16_t distanceImageResult_confidence[1024];
	//NOTE: To calculate a normalized amplitude value for pixel with index i, use the following formula:ampl_norm = amplitude[i] * amplitude_normalization[(confidence[i] &gt;&gt; 1) &amp; 3]
} customData_t;

int checkChannel(int argc, char* argv[], unsigned *ch_no);
int processChannel(ifm_o3m_uint32_t channel_no, void* buf, ifm_o3m_uint32_t size);
void displayChannel(ifm_o3m_uint32_t channel_no);
customData_t getCustomData();
#endif
