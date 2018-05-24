#ifndef CustomerExample_H_
#define CustmerExample_H_

#include <winsock2.h>
#include <memory.h>
#include <stdio.h>

#include "example_results.h"
#include "ifm_types.h"
#include "packet_headers.h"
#include "O3MVariant.h"
#include "DI.h"

// include the newest channel interface for channel 8
#include "../Channels/DI_CH08_1_4.h"
#include "../Channels/DI_CH14_1_4.h"
#include "../Channels/DI_CH20_1_4.h"
#include "../Channels/DI_CH256_1_5.h"

int startUDPConnection(ifm_o3m_uint16_t port, SOCKET* pSocket);
void stopUDPConnection(SOCKET* pSocket);
int processPacket(ifm_o3m_sint8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
    ifm_o3m_uint32_t currentPacketSize, // size of the udp packet payload
    ifm_o3m_sint8_t* channelBuffer,      // buffer for a complete channel
    ifm_o3m_uint32_t channelBufferSize, // size of the buffer for the complete channel
    ifm_o3m_uint32_t* pos);             // the current pos in the channel buffer
int Receiver(SOCKET* pSocket, unsigned channelOfInterest);
int refreshCam(int argc, char* argv[]);
customData_t getData();

#endif
