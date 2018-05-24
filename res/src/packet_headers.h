#ifndef PACKET_HEADERS_H_
#define PACKET_HEADERS_H_

#include "ifm_types.h"

#define UDP_PACKET_BUF_LEN (1460)


// This pragma sets the alignment to 1 byte in Microsofts Visual C++. 
// This is necessary so that there are no gaps between the elements of the struct. 
// The size of the elements are chosen in a way so that it shouldn't happen on a 32bit system, 
// but we have to make sure.
// 
// With this gapless structure we are able to access in the packet header without further copying on 
// a little endian CPU (like x86). On a big endian machine this won't work.
#pragma pack(push, 1)

// This struct represents the first part of each UDP packet.
typedef struct PacketHeader
{
    ifm_o3m_uint16_t Version;
    ifm_o3m_uint16_t Device;
    ifm_o3m_uint32_t PacketCounter;
    ifm_o3m_uint32_t CycleCounter;
    ifm_o3m_uint16_t NumberOfPacketsInCycle;
    ifm_o3m_uint16_t IndexOfPacketInCycle;
    ifm_o3m_uint16_t NumberOfPacketsInChannel;
    ifm_o3m_uint16_t IndexOfPacketInChannel;
    ifm_o3m_uint32_t ChannelID;
    ifm_o3m_uint32_t TotalLengthOfChannel;
    ifm_o3m_uint32_t LengthPayload;
} PacketHeader;

typedef struct ChannelHeader
{
    ifm_o3m_uint32_t StartDelimiter;
    ifm_o3m_uint8_t reserved[24];
} ChannelHeader;

typedef struct ChannelEnd
{
    ifm_o3m_uint32_t EndDelimiter;
} ChannelEnd;

// This pragma restores the alignment to the previous value in Microsofts Visual C++
#pragma pack(pop)

#endif
