/*needspatch@xmldoc*/ /* This file provides the definition of the struct SDspFrameCustomerImeas_t in the interface version DID1_1.4.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_SDSPFRAMECUSTOMERIMEAS_T_DID1_1_4_CONVERTER_H_INCLUDED
#define IFM_O3M_SDSPFRAMECUSTOMERIMEAS_T_DID1_1_4_CONVERTER_H_INCLUDED

#include "ifm_types.h"

/* The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    /*@xmldoc(magic_no)*/
    ifm_o3m_sint8_t magic_no[4];
    /*@xmldoc(struct_id)*/
    ifm_o3m_sint8_t struct_id[4];
    /*@xmldoc(version)*/
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
    struct
    {
        /*@xmldoc(DspCustomerParameter.BlockIdName)*/
        ifm_o3m_sint8_t BlockIdName[21];
        /*@xmldoc(DspCustomerParameter.BlockVersion)*/
        ifm_o3m_uint8_t BlockVersion[3];
        /*@xmldoc(DspCustomerParameter.Variant)*/
        ifm_o3m_sint8_t Variant[3];
        /*@xmldoc(DspCustomerParameter.Ipv4AddressCamera)*/
        ifm_o3m_uint8_t Ipv4AddressCamera[4];
        /*@xmldoc(DspCustomerParameter.SubnetMask)*/
        ifm_o3m_uint8_t SubnetMask[4];
        /*@xmldoc(DspCustomerParameter.Ipv4AddressDestination)*/
        ifm_o3m_uint8_t Ipv4AddressDestination[4];
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.destinationUDPPort)*/
        ifm_o3m_uint16_t destinationUDPPort;
        /*@xmldoc(DspCustomerParameter.EthernetOutputConfiguration)*/
        ifm_o3m_uint8_t EthernetOutputConfiguration;
        /*@xmldoc(DspCustomerParameter.EthernetLoadConfiguration)*/
        ifm_o3m_uint8_t EthernetLoadConfiguration;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transX)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transY)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transZ)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotX)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotY)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotZ)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transX)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transY)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transZ)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCalibIsRelative)*/
        ifm_o3m_uint8_t PMDExtrCalib_IlluCalibIsRelative;
        /*@xmldoc(DspCustomerParameter.DistImageCust_sprayRemovalSensitivity)*/
        ifm_o3m_uint8_t DistImageCust_sprayRemovalSensitivity;
        /*@xmldoc(DspCustomerParameter.DistImageCust_pixelPlausibilizationThresholds)*/
        ifm_o3m_uint8_t DistImageCust_pixelPlausibilizationThresholds;
        /*@xmldoc(DspCustomerParameter.DistImageCust_blockageSensitivity)*/
        ifm_o3m_uint8_t DistImageCust_blockageSensitivity;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterXMin)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterXMin;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterXMax)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterXMax;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterYMin)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterYMin;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterYMax)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterYMax;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterZMin)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterZMin;
        /*@xmldoc(DspCustomerParameter.DistImageCust_spatialFilterZMax)*/
        ifm_o3m_float32_t DistImageCust_spatialFilterZMax;
        /*@xmldoc(DspCustomerParameter.DistImageCust_reflectorThresholdValue)*/
        ifm_o3m_float32_t DistImageCust_reflectorThresholdValue;
        /*@xmldoc(DspCustomerParameter.DistImageCust_reflectorCloseRange)*/
        ifm_o3m_uint8_t DistImageCust_reflectorCloseRange;
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfActive)*/
        ifm_o3m_sint32_t DistImageCust_bfActive;
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_group)*/
        ifm_o3m_sint8_t DistImageCust_bfROIs_group[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_upperLeftX)*/
        ifm_o3m_sint8_t DistImageCust_bfROIs_upperLeftX[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_upperLeftY)*/
        ifm_o3m_sint8_t DistImageCust_bfROIs_upperLeftY[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_lowerRightX)*/
        ifm_o3m_sint8_t DistImageCust_bfROIs_lowerRightX[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_lowerRightY)*/
        ifm_o3m_sint8_t DistImageCust_bfROIs_lowerRightY[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_numberOfROIs)*/
        ifm_o3m_sint32_t DistImageCust_bfROIs_numberOfROIs;
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_minCartX)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_minCartX[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_maxCartX)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_maxCartX[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_minCartY)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_minCartY[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_maxCartY)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_maxCartY[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_minCartZ)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_minCartZ[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfROIs_maxCartZ)*/
        ifm_o3m_sint16_t DistImageCust_bfROIs_maxCartZ[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfResultTypes)*/
        ifm_o3m_sint32_t DistImageCust_bfResultTypes[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfReferenceDomains)*/
        ifm_o3m_uint8_t DistImageCust_bfReferenceDomains[64];
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfTemporalWindowSize)*/
        ifm_o3m_sint32_t DistImageCust_bfTemporalWindowSize;
        /*@xmldoc(DspCustomerParameter.DistImageCust_bfOutputMode)*/
        ifm_o3m_sint32_t DistImageCust_bfOutputMode;
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_numberOfPatterns)*/
        ifm_o3m_uint8_t AutoCalibParam_numberOfPatterns;
        ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_006; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_007; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_xPattern)*/
        ifm_o3m_float32_t AutoCalibParam_xPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_yPattern)*/
        ifm_o3m_float32_t AutoCalibParam_yPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_zPattern)*/
        ifm_o3m_float32_t AutoCalibParam_zPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_patternType)*/
        ifm_o3m_uint8_t AutoCalibParam_patternType[8];
        /*@xmldoc(DspCustomerParameter.twoD_mirrorX)*/
        ifm_o3m_uint8_t twoD_mirrorX;
        /*@xmldoc(DspCustomerParameter.twoD_mirrorY)*/
        ifm_o3m_uint8_t twoD_mirrorY;
        /*@xmldoc(DspCustomerParameter.triggeredStreetCalibration)*/
        ifm_o3m_uint8_t triggeredStreetCalibration;
        ifm_o3m_uint8_t pad_008; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.num_Frames_Averaging)*/
        ifm_o3m_sint32_t num_Frames_Averaging;
        /*@xmldoc(DspCustomerParameter.pixelPlausiReflectivityThreshold)*/
        ifm_o3m_float32_t pixelPlausiReflectivityThreshold;
        /*@xmldoc(DspCustomerParameter.Modulation_Frequency_Mode)*/
        ifm_o3m_uint8_t Modulation_Frequency_Mode;
        /*@xmldoc(DspCustomerParameter.DistanceImageOnSwitch)*/
        ifm_o3m_uint8_t DistanceImageOnSwitch;
        /*@xmldoc(DspCustomerParameter.bfResultParam)*/
        ifm_o3m_uint8_t bfResultParam[64];
        ifm_o3m_uint8_t pad_009; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_010; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.amplThresholdFactor)*/
        ifm_o3m_float32_t amplThresholdFactor;
        struct
        {
            struct
            {
                /*@xmldoc(DspCustomerParameter.Logic.logicGraph.operation)*/
                ifm_o3m_uint16_t operation;
                /*@xmldoc(DspCustomerParameter.Logic.logicGraph.inputs)*/
                ifm_o3m_uint16_t inputs[3];
            } logicGraph[500];
            struct
            {
                /*@xmldoc(DspCustomerParameter.Logic.imeasIntrospection.channelID)*/
                ifm_o3m_uint16_t channelID;
                /*@xmldoc(DspCustomerParameter.Logic.imeasIntrospection.expression)*/
                ifm_o3m_sint8_t expression[32];
            } imeasIntrospection[25];
        } Logic;
        /*@xmldoc(DspCustomerParameter.RotateImage90)*/
        ifm_o3m_uint8_t RotateImage90;
        /*@xmldoc(DspCustomerParameter.Reserved)*/
        ifm_o3m_uint8_t Reserved[3];
        ifm_o3m_uint8_t pad_011; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_012; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.RoD_Xmin)*/
        ifm_o3m_float32_t RoD_Xmin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Xmax)*/
        ifm_o3m_float32_t RoD_Xmax[2];
        /*@xmldoc(DspCustomerParameter.RoD_Ymin)*/
        ifm_o3m_float32_t RoD_Ymin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Ymax)*/
        ifm_o3m_float32_t RoD_Ymax[2];
        /*@xmldoc(DspCustomerParameter.RoD_Zmin)*/
        ifm_o3m_float32_t RoD_Zmin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Zmax)*/
        ifm_o3m_float32_t RoD_Zmax[2];
    } DspCustomerParameter;
} ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4;

        
/* Casts the buffer to ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* ifm_o3m_ConvertBufferToLittleEndian_DID1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* ifm_o3m_ConvertBufferToBigEndian_DID1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
