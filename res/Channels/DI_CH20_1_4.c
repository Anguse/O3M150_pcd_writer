#include "DI_CH20_1_4.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* ifm_o3m_ConvertBufferToLittleEndian_DIA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* res = (ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4*)buffer;
    if( (!buffer) || (bufferSize != 3024) || (sizeof(ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4) != 3024) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '2') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* ifm_o3m_ConvertBufferToBigEndian_DIA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4* res = (ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4*)buffer;
    if( (!buffer) || (bufferSize != 3024) || (sizeof(ifm_o3m_AlgoIFOutputNoDI_DIA2_1_4) != 3024) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '2') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    /* basicFunctionsResult.numberOfGroups */
    IFM_SWAP32(&buf[12]);
    /* basicFunctionsResult.regionsOfInterest.numberOfRois */
    IFM_SWAP32(&buf[336]);
    /* basicFunctionsResult.regionsOfInterest.minCartX */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+340]);
    }
    /* basicFunctionsResult.regionsOfInterest.maxCartX */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+468]);
    }
    /* basicFunctionsResult.regionsOfInterest.minCartY */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+596]);
    }
    /* basicFunctionsResult.regionsOfInterest.maxCartY */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+724]);
    }
    /* basicFunctionsResult.regionsOfInterest.minCartZ */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+852]);
    }
    /* basicFunctionsResult.regionsOfInterest.maxCartZ */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+980]);
    }
    /* basicFunctionsResult.outputMode */
    IFM_SWAP32(&buf[1108]);
    /* basicFunctionsResult.groupX */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1240]);
    }
    /* basicFunctionsResult.groupY */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1496]);
    }
    /* basicFunctionsResult.groupZ */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1752]);
    }
    /* basicFunctionsResult.groupAmpl */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2008]);
    }
    /* masterclockTimestamp */
    IFM_SWAP32(&buf[2264]);
    /* frameCounter */
    IFM_SWAP32(&buf[2268]);
    /* available */
    IFM_SWAP32(&buf[2272]);
    /* cameraCalibration.transX */
    IFM_SWAP32(&buf[2276]);
    /* cameraCalibration.transY */
    IFM_SWAP32(&buf[2280]);
    /* cameraCalibration.transZ */
    IFM_SWAP32(&buf[2284]);
    /* cameraCalibration.rotX */
    IFM_SWAP32(&buf[2288]);
    /* cameraCalibration.rotY */
    IFM_SWAP32(&buf[2292]);
    /* cameraCalibration.rotZ */
    IFM_SWAP32(&buf[2296]);
    /* fieldOfView.upperLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2300]);
    }
    /* fieldOfView.upperRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2312]);
    }
    /* fieldOfView.lowerLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2324]);
    }
    /* fieldOfView.lowerRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2336]);
    }
    /* intrExtrCalib_2d.intrCalib_2D_fx */
    IFM_SWAP32(&buf[2348]);
    /* intrExtrCalib_2d.intrCalib_2D_fy */
    IFM_SWAP32(&buf[2352]);
    /* intrExtrCalib_2d.intrCalib_2D_mx */
    IFM_SWAP32(&buf[2356]);
    /* intrExtrCalib_2d.intrCalib_2D_my */
    IFM_SWAP32(&buf[2360]);
    /* intrExtrCalib_2d.intrCalib_alpha */
    IFM_SWAP32(&buf[2364]);
    /* intrExtrCalib_2d.intrCalib_k1 */
    IFM_SWAP32(&buf[2368]);
    /* intrExtrCalib_2d.intrCalib_k2 */
    IFM_SWAP32(&buf[2372]);
    /* intrExtrCalib_2d.intrCalib_k5 */
    IFM_SWAP32(&buf[2376]);
    /* intrExtrCalib_2d.intrCalib_k3 */
    IFM_SWAP32(&buf[2380]);
    /* intrExtrCalib_2d.intrCalib_k4 */
    IFM_SWAP32(&buf[2384]);
    /* intrExtrCalib_2d.extrCalib_center_tx */
    IFM_SWAP32(&buf[2388]);
    /* intrExtrCalib_2d.extrCalib_center_ty */
    IFM_SWAP32(&buf[2392]);
    /* intrExtrCalib_2d.extrCalib_center_tz */
    IFM_SWAP32(&buf[2396]);
    /* intrExtrCalib_2d.extrCalib_delta_tx */
    IFM_SWAP32(&buf[2400]);
    /* intrExtrCalib_2d.extrCalib_delta_ty */
    IFM_SWAP32(&buf[2404]);
    /* intrExtrCalib_2d.extrCalib_delta_tz */
    IFM_SWAP32(&buf[2408]);
    /* intrExtrCalib_2d.extrCalib_rot_x */
    IFM_SWAP32(&buf[2412]);
    /* intrExtrCalib_2d.extrCalib_rot_y */
    IFM_SWAP32(&buf[2416]);
    /* intrExtrCalib_2d.extrCalib_rot_z */
    IFM_SWAP32(&buf[2420]);
    /* illuPosition */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2424]);
    }
    /* blockageRatio */
    IFM_SWAP32(&buf[2436]);
    /* calibrationResult.commonCalibrationResult.calibValid */
    IFM_SWAP32(&buf[2444]);
    /* calibrationResult.commonCalibrationResult.calibrationStableCounter */
    IFM_SWAP32(&buf[2448]);
    /* calibrationResult.commonCalibrationResult.calibResult.transX */
    IFM_SWAP32(&buf[2452]);
    /* calibrationResult.commonCalibrationResult.calibResult.transY */
    IFM_SWAP32(&buf[2456]);
    /* calibrationResult.commonCalibrationResult.calibResult.transZ */
    IFM_SWAP32(&buf[2460]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotX */
    IFM_SWAP32(&buf[2464]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotY */
    IFM_SWAP32(&buf[2468]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotZ */
    IFM_SWAP32(&buf[2472]);
    /* calibrationResult.pacCalibrationResult.triangleDetections.score */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*40)+2480]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.pos3D */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2484]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2524]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2564]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2604]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2644]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2684]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2724]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2764]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.corners */
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2496]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2536]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2576]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2616]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2656]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2696]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2736]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2776]);
    }
    /* calibrationResult.pacCalibrationResult.frameValid */
    IFM_SWAP32(&buf[2800]);
    /* calibrationResult.pacCalibrationResult.frameReprojectError */
    IFM_SWAP32(&buf[2804]);
    /* calibrationResult.streetCalibrationResult.planeValid */
    IFM_SWAP32(&buf[2808]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.pitchAngle */
    IFM_SWAP32(&buf[2812]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.rollAngle */
    IFM_SWAP32(&buf[2816]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.camHeight */
    IFM_SWAP32(&buf[2820]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalx */
    IFM_SWAP32(&buf[2824]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normaly */
    IFM_SWAP32(&buf[2828]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalz */
    IFM_SWAP32(&buf[2832]);
    /* calibrationResult.streetCalibrationResult.plausibility */
    IFM_SWAP32(&buf[2836]);
    /* calibrationResult.streetCalibrationResult.distanceDeviation */
    IFM_SWAP32(&buf[2840]);
    /* logicOutput.analogOutput */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2944]);
    }

    return res;
}
