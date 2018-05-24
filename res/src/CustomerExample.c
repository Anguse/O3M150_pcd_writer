/* Example code to receive and display data with the O3M sensor.
It reads the data packets from the network, assembles them together and then displays
some data.


Written and tested with Microsoft Visual C++ 2013 on Windows 7 64bit.

Last changed: 2017-08-30

Copyright (C) 2017 ifm electronic GmbH
*/

#include <winsock2.h>
#include <memory.h>
#include <stdio.h>
#include <iostream>
#include <windows.h>

#include "example_results.h"
#include "ifm_types.h"
#include "packet_headers.h"
#include "O3MVariant.h"
#include "CustomerExample.h"

/* This code uses the winsock2 library and therefore needs the Ws2_32.lib */
#pragma message("------------> This code uses the winsock2 library. It should be found in the windows sdk. However, some versions may be missing the ws2_32.lib <------------")
#pragma comment(lib, "Ws2_32.lib")

static customData_t data;

// Initializes WinSock and starts the connection
int startUDPConnection(ifm_o3m_uint16_t port, SOCKET* pSocket)
{
    WSADATA wsa;
    SOCKADDR_IN addr;
    int ret;
    int rc;

    // Start Winsock
    ret = WSAStartup(MAKEWORD(2, 0), &wsa);
    if(ret != 0)
    {
        printf("Can't start Windows Socks API\n");
        return RESULT_ERROR;
    }

    // create socket
    *pSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if(*pSocket == INVALID_SOCKET)
    {
        printf("Coudln't create the UDP socket\n");
        return RESULT_ERROR;
    }

    // bind socket to port
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = ADDR_ANY;

    rc = bind(*pSocket, (SOCKADDR*)&addr, sizeof(SOCKADDR_IN));
    if(rc == SOCKET_ERROR)
    {
        printf("Bind didn't work, error code: %d\n", WSAGetLastError());
        return RESULT_ERROR;
    }

    return RESULT_OK;
}


// Deinitializes Network
void stopUDPConnection(SOCKET* pSocket)
{
    closesocket(*pSocket);
    WSACleanup();
}

// Extracts the data in the payload of the udp packet und puts it into the channel buffer
int processPacket(ifm_o3m_sint8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
    ifm_o3m_uint32_t currentPacketSize, // size of the udp packet payload
    ifm_o3m_sint8_t* channelBuffer,      // buffer for a complete channel
    ifm_o3m_uint32_t channelBufferSize, // size of the buffer for the complete channel
    ifm_o3m_uint32_t* pos)              // the current pos in the channel buffer
{

    // There is always a PacketHeader structure at the beginning
    PacketHeader* ph = (PacketHeader*)currentPacketData;
    int Start = sizeof(PacketHeader);
    int Length = currentPacketSize - sizeof(PacketHeader);

    // Only the first packet of a channel contains a ChannelHeader
    if(ph->IndexOfPacketInChannel == 0)
    {
        Start += sizeof(ChannelHeader);
        Length -= sizeof(ChannelHeader);
    }

    // Only the last packet of a channel contains an EndDelimiter (at the end, after the data)
    if(ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
    {
        Length -= sizeof(ChannelEnd);
    }

    // Is the buffer big enough?
    if((*pos) + Length > channelBufferSize)
    {
        // Too small means either an error in the program logic or a corrupt packet
        printf("Channel buffer is too small.\n");
        return RESULT_ERROR;
    }
    else
    {
        memcpy(channelBuffer + (*pos), currentPacketData + Start, Length);
    }

    (*pos) += Length;

    return RESULT_OK;

}


// Receives the UDP data in loop
int Receiver(SOCKET* pSocket, unsigned channelOfInterest)
{
    // Implemented channels: 8 (pixel data + objects) and 20 (objects only)
    // Only channel 8 or 20 are transmitted. The setting can be done in VisionAssistant
    // (Device setup -> Ethernet -> Output pixel data via Ethernet)
    const ifm_o3m_uint32_t customerDataChannel = channelOfInterest;

    // holds the sender information of the received packet
    struct sockaddr remoteAddr;
    ifm_o3m_sint32_t remoteAddrLen;

    // buffer for a single UDP packet
    ifm_o3m_sint8_t udpPacketBuf[UDP_PACKET_BUF_LEN];

    // As the alignment was forced to 1 we can work with the struct on the buffer.
    // This assumes the byte order is little endian which it is on a PC.
    PacketHeader* ph = (PacketHeader*)udpPacketBuf;

    // the size of the channel may change so the size will be taken from the packet
    ifm_o3m_uint32_t channel_buf_size = 0;
    ifm_o3m_sint8_t* channelBuf = NULL;

    // As there is no offset in the packet header we have to remember where the next part should go
    ifm_o3m_uint32_t pos_in_channel = 0;

    // remember the counter of the previous packet so we know when we are losing packets
    ifm_o3m_uint32_t previous_packet_counter = 0;
    ifm_o3m_sint32_t previous_packet_counter_valid = 0;

    // the receiption of the data may start at any time. So we wait til we find the beginning of our channel
    ifm_o3m_sint32_t startOfChannelFound = 0;

    remoteAddrLen = sizeof(struct sockaddr);

	LARGE_INTEGER t1, t2, frequency;
	double elapsedTime = 0.0;

	// start timer
	QueryPerformanceCounter(&t1);

    // run for 20ms as long as no error occurs
    while(elapsedTime < 20)
    {
		// get ticks per second
		QueryPerformanceFrequency(&frequency);

        // receive the data. rc contains the number of received bytes and also the error code
        // IMPORTANT: This is a blocking call. If it doesn't receive anything it will wait forever
        ifm_o3m_sint32_t rc = recvfrom(*pSocket, (char*)udpPacketBuf, UDP_PACKET_BUF_LEN, 0, &remoteAddr, &remoteAddrLen);
        if(rc == SOCKET_ERROR)
        {
            printf("Error in recvfrom, error code: %d\n", WSAGetLastError());
            return RESULT_ERROR;
        }
        else
        {
            // Check the packet counter for missing packets
            if(previous_packet_counter_valid)
            {
                // if the type of the variables is ui32, it will also work when the wrap around happens.
                if((ph->PacketCounter - previous_packet_counter) != 1)
                {
                    printf("Packet Counter jumped from %ul to %ul (missing packets; try to redirect output)\n", previous_packet_counter, ph->PacketCounter);

                    // With this it will ignore the already received parts and resynchronize at
                    // the beginning of the next cycle.
                    startOfChannelFound = 0;
                }
            }

            previous_packet_counter = ph->PacketCounter;
            previous_packet_counter_valid = 1;

            // is this the channel with our data?
            if(ph->ChannelID == customerDataChannel)
            {
                // are we at the beginning of the channel?
                if(ph->IndexOfPacketInChannel == 0)
                {
                    startOfChannelFound = 1;

                    // If we haven't allocated memory for the channel do it now.
                    if(channel_buf_size == 0)
                    {
                        channel_buf_size = ph->TotalLengthOfChannel;
                        channelBuf = (ifm_o3m_sint8_t*)malloc(channel_buf_size);
                    }

                    // as we reuse the buffer we clear it at the beginning of a transmission
                    memset(channelBuf, 0, channel_buf_size);
                    pos_in_channel = 0;

                }

                // if we have found the start of the channel at least once, we are ready to process the packet
                if(startOfChannelFound)
                {
                    processPacket(udpPacketBuf, rc, channelBuf, channel_buf_size, &pos_in_channel);

                    // Have we found the last packet in this channel? Then we are able to process it
                    // The index is zero based so a channel with n parts will have indices from 0 to n-1
                    if(ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
                    {
                        // pos_in_channel is the position where the (not existing) next packet would be
                        // placed. This is also the size of the data.
                        processChannel(customerDataChannel, channelBuf, pos_in_channel);
                        displayChannel(customerDataChannel);
                    }
                }
            }
        }
		//Stop timer
		QueryPerformanceCounter(&t2);
		//Compute time in milliseconds
		elapsedTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
		//Update data from camera
		data = getCustomData();

    }

    return RESULT_OK;
}

// Updates the data variable
int refreshCam(int argc, char* argv[]){
  unsigned channelOfInterest;
  if(checkChannel(argc, argv, &channelOfInterest))
  {
      return 0;
  }

  const ifm_o3m_uint16_t port = 42000; // Default is UDP port 42000
  SOCKET socket;

  int ret = startUDPConnection(port, &socket);
  if(ret == RESULT_OK)
  {
      Receiver(&socket, channelOfInterest);
  }

  stopUDPConnection(&socket);

  return RESULT_OK;
}

//Returns the data variable
customData_t getData(){
  return data;
}
