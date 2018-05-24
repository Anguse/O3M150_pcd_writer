#include "DI.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "example_results.h"

static distanceData_t g_distanceData;
static basicFunctionsData_t g_basicFunctionsData;
static parameterData_t g_parameterData;
static statusData_t g_statusData;
static customData_t g_customData;

int checkChannel(int argc, char* argv[], unsigned *ch_no)
{
    int channelOfInterest;
    if(argc != 2)
    {
        printf("\nPlease provide your channel of interest as argument.\n");
        printf("Available channels:\n\n");
        printf("\t8: pixel data\n");
        printf("\t14: current parameters\n");
        printf("\t20: %s results\n", FWVARIANT);
        printf("\t256: hardware/firmware diagnosis data\n");
        printf("\n");
        return 1;
    }

    channelOfInterest = atoi(argv[1]);
    switch(channelOfInterest)
    {
    case 8:
    case 20:
    case 14:
    case 256:
        break;
    default:
        printf("\nOnly channels 8, 14, 20, 256 supported\n");
        return 1;
    }
    *ch_no = (unsigned)channelOfInterest;
    return 0;
}

// copy the data for DI structure
int copyChannel8(Channel8* p)
{
    int i;

    if (p == NULL)
        return RESULT_ERROR;

    for (i = 0; i<NUM_SENSOR_PIXELS; i++)
    {
        g_distanceData.distanceData[i] = p->distanceImageResult.distanceData[i];
        g_distanceData.confidence[i] = p->distanceImageResult.confidence[i];
		
		g_customData.distanceImageResult_X[i] = p->distanceImageResult.X[i];
		g_customData.distanceImageResult_Y[i] = p->distanceImageResult.Y[i];
		g_customData.distanceImageResult_Z[i] = p->distanceImageResult.Z[i];
		g_customData.distanceImageResult_amplitude[i] = p->distanceImageResult.amplitude[i];
    }
		g_customData.distanceImageResult_sensorHeight = p->distanceImageResult.sensorHeight;
		g_customData.distanceImageResult_sensorWidth = p->distanceImageResult.sensorWidth;

    return RESULT_OK;
}

// copies the data into our own structure.
int processChannel8(void* buf, ifm_o3m_uint32_t size)
{
    Channel8* p_ch8 = NULL;

    // Is this DI structure version 1.4?
    p_ch8 = ifm_o3m_ConvertBufferToLittleEndian_DIA1_1_4(buf, size);
    if(p_ch8)
    {
        // yes it is, so copy the data from this structure
        copyChannel8(p_ch8);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}

void displayChannel8()
{
    ifm_o3m_uint32_t pixelPos;
    ifm_o3m_uint32_t pixelValid;

    // Pixel 32/7 is in the middle of the sensor
    ifm_o3m_uint32_t column = 32;
    ifm_o3m_uint32_t line = 7;

    pixelPos = 64 * line + column;

    // The lowest bit of the confidence contains the information if the pixel is valid
    pixelValid = g_distanceData.confidence[pixelPos] & 1;

    // 0=valid, 1=invalid
    if(pixelValid == 0)
    {
        // \r is carriage return without newline. This way the output is always written in the same line
        printf("Distance of pixel: %d cm                             \r",
            g_distanceData.distanceData[pixelPos]);
    }
    else
    {
        printf("Pixel is invalid                            \r");
    }

}

int copyChannel14(Channel14* p)
{
    if(p == NULL)
        return RESULT_ERROR;

    g_parameterData.DistImageCust_blockageSensitivity = p->DspCustomerParameter.DistImageCust_blockageSensitivity;
    g_parameterData.Modulation_Frequency_Mode = p->DspCustomerParameter.Modulation_Frequency_Mode;

    return RESULT_OK;
}

// copies the data into our own structure.
int processChannel14(void* buf, ifm_o3m_uint32_t size)
{
    Channel14* p_ch = NULL;

    // Is this DI structure version 1.4?
    p_ch = ifm_o3m_ConvertBufferToLittleEndian_DID1_1_4(buf, size);
    if(p_ch)
    {
        // yes it is, so copy the data from this structure
        copyChannel14(p_ch);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}

void displayChannel14()
{
    printf("Blockage sensitivity: %d\n", g_parameterData.DistImageCust_blockageSensitivity);
    printf("Modulation frequency mode: ");
    switch(g_parameterData.Modulation_Frequency_Mode)
    {
    case 0:
        printf("random");
        break;
    case 1:
        printf("triple 1");
        break;
    case 2:
        printf("triple 2");
        break;
    case 3:
        printf("triple 3");
        break;
    default:
        printf("unknown");
        break;
    }
    printf("\n\n");
}

// copy the data for BF structure
int copyChannel20(Channel20* p)
{
    int i;

    if(p == NULL)
        return RESULT_ERROR;

    memset(g_basicFunctionsData.groupX, 0, sizeof(g_basicFunctionsData.groupX));
    g_basicFunctionsData.numberOfGroups = p->basicFunctionsResult.numberOfGroups;
    for(i = 0; i < g_basicFunctionsData.numberOfGroups; i++)
    {
        g_basicFunctionsData.groupX[i] = p->basicFunctionsResult.groupX[i];
    }

    return RESULT_OK;
}

int processChannel20(void* buf, ifm_o3m_uint32_t size)
{
    Channel20* p_ch20 = NULL;

    p_ch20 = ifm_o3m_ConvertBufferToLittleEndian_DIA2_1_4(buf, size);
    if(p_ch20)
    {
        copyChannel20(p_ch20);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}

void displayChannel20()
{
    printf("Number of ROI-groups: %d.\n", g_basicFunctionsData.numberOfGroups);

    // printing the first 3 ROI-group results
    for(int i = 0; i < ((g_basicFunctionsData.numberOfGroups < 3) ? g_basicFunctionsData.numberOfGroups : 3); ++i)
    {
        printf("     ROI-group %d: X = %g", i, g_basicFunctionsData.groupX[i]);

        // if a ROI-group result is invalid it is marked as NaN. Check it here.
        if(g_basicFunctionsData.groupX[i] == g_basicFunctionsData.groupX[i])
        {
            printf("\n");
        }
        else
        {
            printf(" (invalid)\n");
        }
    }
    printf("\n\n");
}

// copy the data for LG structure version into our internal structure
int copyChannel256(Channel256* p)
{
    if(p == NULL)
        return RESULT_ERROR;

    g_statusData.OpMode = p->OpMode;
    g_statusData.Variant[0] = p->KpCustomerParameter.Variant[0];
    g_statusData.Variant[1] = p->KpCustomerParameter.Variant[1];
    g_statusData.Variant[2] = p->KpCustomerParameter.Variant[2];
    g_statusData.mainTemperature = p->CameraMainTemperature;

    return RESULT_OK;
}

// checks which version of the data it is and copies the data into our own structure.
// You need this for every channel you want to process.
int processChannel256(void* buf, ifm_o3m_uint32_t size)
{
    Channel256* p_ch256 = ifm_o3m_ConvertBufferToLittleEndian_DIK1_1_5(buf, size);
    if(p_ch256)
    {
        copyChannel256(p_ch256);
        return RESULT_OK;
    }

    printf("*** No supported version, check if you use the appropriate FW-version    *** \n\n");
    printf("*** FW-version must match with the FW-packet of this sample code             *** \n");
    printf("*** Connect with Vision Assistant and look into Device setup -> Device       *** \n\n");
    return RESULT_ERROR;
}

void displayChannel256()
{
    char mode[16];
    switch(g_statusData.OpMode)
    {
    case 0x12:
        strcpy(mode, "INIT");
        break;
    case 0x15:
        strcpy(mode, "BOOT");
        break;
    case 0x21:
        strcpy(mode, "LIMITED RUN");
        break;
    case 0x22:
        strcpy(mode, "RUN");
        break;
    case 0x31:
        strcpy(mode, "EMERGENCY");
        break;
    default:
        strcpy(mode, "UNKNOWN");
        break;
    }

    printf("Variant: %c%c%c, Working status: %s, Main temperature: %lf\ °C\n",
        g_statusData.Variant[0],
        g_statusData.Variant[1],
        g_statusData.Variant[2],
        mode,
        g_statusData.mainTemperature);
}

// this function shows how to interprete channel data depending on the channel number
int processChannel(ifm_o3m_uint32_t channel_no, void* buf, ifm_o3m_uint32_t size)
{
    switch(channel_no)
    {
    case 8:
        return processChannel8(buf, size);
    case 14:
        return processChannel14(buf, size);
    case 20:
        return processChannel20(buf, size);
    case 256:
        return processChannel256(buf, size);
    default:
        break;
    }
    return RESULT_OK;
}

// display some received data of given channel
void displayChannel(ifm_o3m_uint32_t channel_no)
{
    switch(channel_no)
    {
    case 8:
        displayChannel8();
        break;
    case 14:
        displayChannel14();
        break;
    case 20:
        displayChannel20();
        break;
    case 256:
        displayChannel256();
        break;
    default:
        break;
    }
}

// return specific data
customData_t getCustomData()
{
	return g_customData;
}
