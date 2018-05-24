#include "DI_CH256_1_5.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5 (if possible) and returns a pointer to it.
   Use this function on big Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5* ifm_o3m_ConvertBufferToBigEndian_DIK1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5* res = (ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5*)buffer;
    if( (!buffer) || (bufferSize != 360) || (sizeof(ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5) != 360) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'K') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 5  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5. 
   Note: The original buffer is modified in place. 
   Use this function on little Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5* ifm_o3m_ConvertBufferToLittleEndian_DIK1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5* res = (ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5*)buffer;
    if( (!buffer) || (bufferSize != 360) || (sizeof(ifm_o3m_Meas_Customer_Data_Type_DIK1_1_5) != 360) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'D') && (res->struct_id[1] == 'I') && (res->struct_id[2] == 'K') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 5  ) ) )
    {
        return 0;
    }

    /* CameraMainTemperature */
    IFM_SWAP32(&buf[12]);
    /* CANBaudrate */
    IFM_SWAP32(&buf[48]);
    /* DTCList */
    for(i = 0; i < 16; i++)
    {
        IFM_SWAP32(&buf[(i*4)+56]);
    }
    /* IlluTemperature */
    IFM_SWAP32(&buf[148]);
    /* KpCustomerParameter.CANBaudrate */
    IFM_SWAP32(&buf[188]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalShortAlgoPrims */
    IFM_SWAP16(&buf[302]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalLongAlgoPrims */
    IFM_SWAP16(&buf[304]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui32_ItemsOutOfBounds */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+308]);
    }
    /* KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalShortFixedPrims */
    IFM_SWAP16(&buf[342]);
    /* KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalLongFixedPrims */
    IFM_SWAP16(&buf[344]);
    /* KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalShortIPC2DPrims */
    IFM_SWAP16(&buf[348]);
    /* KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalLongIPC2DPrims */
    IFM_SWAP16(&buf[350]);
    /* KP_Info2D.f32_AverageAlgoPrimsGenTime */
    IFM_SWAP32(&buf[352]);
    /* KP_Info2D.f32_AverageAlgoPrimsGenfps */
    IFM_SWAP32(&buf[356]);

    return res;
}
