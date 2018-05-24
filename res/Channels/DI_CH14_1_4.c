#include "DI_CH14_1_4.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* ifm_o3m_ConvertBufferToLittleEndian_DID1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* res = (ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4*)buffer;
    if( (!buffer) || (bufferSize != 6648) || (sizeof(ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4) != 6648) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'D') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* ifm_o3m_ConvertBufferToBigEndian_DID1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4* res = (ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4*)buffer;
    if( (!buffer) || (bufferSize != 6648) || (sizeof(ifm_o3m_SDspFrameCustomerImeas_t_DID1_1_4) != 6648) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'D') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    /* DspCustomerParameter.destinationUDPPort */
    IFM_SWAP16(&buf[52]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transX */
    IFM_SWAP32(&buf[56]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transY */
    IFM_SWAP32(&buf[60]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transZ */
    IFM_SWAP32(&buf[64]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotX */
    IFM_SWAP32(&buf[68]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotY */
    IFM_SWAP32(&buf[72]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotZ */
    IFM_SWAP32(&buf[76]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transX */
    IFM_SWAP32(&buf[80]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transY */
    IFM_SWAP32(&buf[84]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transZ */
    IFM_SWAP32(&buf[88]);
    /* DspCustomerParameter.DistImageCust_spatialFilterXMin */
    IFM_SWAP32(&buf[96]);
    /* DspCustomerParameter.DistImageCust_spatialFilterXMax */
    IFM_SWAP32(&buf[100]);
    /* DspCustomerParameter.DistImageCust_spatialFilterYMin */
    IFM_SWAP32(&buf[104]);
    /* DspCustomerParameter.DistImageCust_spatialFilterYMax */
    IFM_SWAP32(&buf[108]);
    /* DspCustomerParameter.DistImageCust_spatialFilterZMin */
    IFM_SWAP32(&buf[112]);
    /* DspCustomerParameter.DistImageCust_spatialFilterZMax */
    IFM_SWAP32(&buf[116]);
    /* DspCustomerParameter.DistImageCust_reflectorThresholdValue */
    IFM_SWAP32(&buf[120]);
    /* DspCustomerParameter.DistImageCust_bfActive */
    IFM_SWAP32(&buf[128]);
    /* DspCustomerParameter.DistImageCust_bfROIs_numberOfROIs */
    IFM_SWAP32(&buf[452]);
    /* DspCustomerParameter.DistImageCust_bfROIs_minCartX */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+456]);
    }
    /* DspCustomerParameter.DistImageCust_bfROIs_maxCartX */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+584]);
    }
    /* DspCustomerParameter.DistImageCust_bfROIs_minCartY */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+712]);
    }
    /* DspCustomerParameter.DistImageCust_bfROIs_maxCartY */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+840]);
    }
    /* DspCustomerParameter.DistImageCust_bfROIs_minCartZ */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+968]);
    }
    /* DspCustomerParameter.DistImageCust_bfROIs_maxCartZ */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1096]);
    }
    /* DspCustomerParameter.DistImageCust_bfResultTypes */
    for(i = 0; i < 64; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1224]);
    }
    /* DspCustomerParameter.DistImageCust_bfTemporalWindowSize */
    IFM_SWAP32(&buf[1544]);
    /* DspCustomerParameter.DistImageCust_bfOutputMode */
    IFM_SWAP32(&buf[1548]);
    /* DspCustomerParameter.AutoCalibParam_xPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1556]);
    }
    /* DspCustomerParameter.AutoCalibParam_yPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1588]);
    }
    /* DspCustomerParameter.AutoCalibParam_zPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1620]);
    }
    /* DspCustomerParameter.num_Frames_Averaging */
    IFM_SWAP32(&buf[1664]);
    /* DspCustomerParameter.pixelPlausiReflectivityThreshold */
    IFM_SWAP32(&buf[1668]);
    /* DspCustomerParameter.amplThresholdFactor */
    IFM_SWAP32(&buf[1740]);
    /* DspCustomerParameter.Logic.logicGraph.operation */
    for(i = 0; i < 500; i++)
    {
        IFM_SWAP16(&buf[(i*8)+1744]);
    }
    /* DspCustomerParameter.Logic.logicGraph.inputs */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1746]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1754]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1762]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1770]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1778]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1786]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1794]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1802]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1810]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1818]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1826]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1834]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1842]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1850]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1858]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1866]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1874]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1882]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1890]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1898]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1906]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1914]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1922]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1930]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1938]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1946]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1954]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1962]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1970]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1978]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1986]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1994]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2002]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2010]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2018]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2026]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2034]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2042]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2050]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2058]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2066]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2074]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2082]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2090]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2098]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2106]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2114]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2122]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2130]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2138]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2146]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2154]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2162]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2170]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2178]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2186]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2194]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2202]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2210]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2218]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2226]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2234]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2242]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2250]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2258]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2266]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2274]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2282]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2290]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2298]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2306]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2314]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2322]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2330]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2338]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2346]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2354]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2362]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2370]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2378]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2386]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2394]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2402]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2410]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2418]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2426]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2434]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2442]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2450]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2458]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2466]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2474]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2482]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2490]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2498]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2506]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2514]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2522]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2530]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2538]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2546]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2554]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2562]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2570]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2578]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2586]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2594]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2602]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2610]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2618]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2626]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2634]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2642]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2650]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2658]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2666]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2674]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2682]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2690]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2698]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2706]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2714]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2722]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2730]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2738]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2746]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2754]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2762]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2770]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2778]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2786]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2794]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2802]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2810]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2818]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2826]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2834]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2842]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2850]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2858]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2866]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2874]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2882]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2890]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2898]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2906]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2914]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2922]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2930]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2938]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2946]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2954]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2962]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2970]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2978]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2986]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2994]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3002]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3010]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3018]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3026]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3034]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3042]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3050]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3058]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3066]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3074]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3082]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3090]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3098]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3106]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3114]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3122]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3130]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3138]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3146]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3154]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3162]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3170]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3178]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3186]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3194]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3202]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3210]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3218]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3226]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3234]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3242]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3250]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3258]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3266]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3274]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3282]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3290]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3298]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3306]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3314]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3322]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3330]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3338]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3346]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3354]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3362]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3370]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3378]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3386]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3394]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3402]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3410]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3418]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3426]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3434]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3442]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3450]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3458]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3466]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3474]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3482]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3490]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3498]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3506]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3514]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3522]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3530]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3538]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3546]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3554]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3562]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3570]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3578]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3586]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3594]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3602]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3610]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3618]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3626]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3634]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3642]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3650]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3658]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3666]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3674]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3682]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3690]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3698]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3706]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3714]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3722]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3730]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3738]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3746]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3754]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3762]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3770]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3778]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3786]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3794]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3802]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3810]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3818]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3826]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3834]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3842]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3850]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3858]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3866]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3874]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3882]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3890]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3898]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3906]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3914]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3922]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3930]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3938]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3946]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3954]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3962]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3970]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3978]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3986]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3994]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4002]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4010]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4018]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4026]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4034]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4042]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4050]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4058]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4066]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4074]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4082]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4090]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4098]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4106]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4114]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4122]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4130]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4138]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4146]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4154]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4162]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4170]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4178]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4186]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4194]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4202]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4210]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4218]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4226]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4234]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4242]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4250]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4258]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4266]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4274]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4282]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4290]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4298]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4306]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4314]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4322]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4330]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4338]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4346]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4354]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4362]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4370]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4378]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4386]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4394]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4402]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4410]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4418]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4426]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4434]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4442]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4450]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4458]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4466]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4474]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4482]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4490]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4498]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4506]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4514]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4522]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4530]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4538]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4546]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4554]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4562]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4570]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4578]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4586]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4594]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4602]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4610]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4618]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4626]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4634]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4642]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4650]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4658]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4666]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4674]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4682]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4690]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4698]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4706]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4714]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4722]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4730]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4738]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4746]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4754]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4762]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4770]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4778]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4786]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4794]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4802]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4810]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4818]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4826]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4834]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4842]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4850]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4858]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4866]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4874]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4882]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4890]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4898]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4906]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4914]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4922]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4930]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4938]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4946]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4954]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4962]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4970]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4978]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4986]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4994]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5002]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5010]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5018]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5026]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5034]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5042]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5050]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5058]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5066]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5074]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5082]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5090]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5098]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5106]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5114]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5122]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5130]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5138]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5146]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5154]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5162]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5170]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5178]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5186]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5194]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5202]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5210]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5218]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5226]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5234]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5242]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5250]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5258]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5266]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5274]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5282]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5290]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5298]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5306]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5314]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5322]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5330]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5338]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5346]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5354]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5362]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5370]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5378]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5386]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5394]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5402]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5410]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5418]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5426]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5434]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5442]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5450]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5458]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5466]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5474]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5482]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5490]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5498]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5506]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5514]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5522]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5530]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5538]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5546]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5554]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5562]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5570]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5578]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5586]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5594]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5602]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5610]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5618]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5626]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5634]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5642]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5650]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5658]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5666]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5674]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5682]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5690]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5698]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5706]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5714]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5722]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5730]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+5738]);
    }
    /* DspCustomerParameter.Logic.imeasIntrospection.channelID */
    for(i = 0; i < 25; i++)
    {
        IFM_SWAP16(&buf[(i*34)+5744]);
    }
    /* DspCustomerParameter.RoD_Xmin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6600]);
    }
    /* DspCustomerParameter.RoD_Xmax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6608]);
    }
    /* DspCustomerParameter.RoD_Ymin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6616]);
    }
    /* DspCustomerParameter.RoD_Ymax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6624]);
    }
    /* DspCustomerParameter.RoD_Zmin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6632]);
    }
    /* DspCustomerParameter.RoD_Zmax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6640]);
    }

    return res;
}
