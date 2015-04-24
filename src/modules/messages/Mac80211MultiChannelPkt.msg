//**************************************************************************
// * file:        Mac80211MultiChannelPkt.msg
// *
// * author:      
// * copyright:   
// *              
// *              
// * email:       
// **************************************************************************
// * part of:     framework implementation developed by tkn
// *************************************************************************/

cplusplus {{
#include "MiXiMMacPkt.h"
}}


class MacPkt;

//
// Defines all fields of an 802.11 MAC frame
//
packet Mac80211MultiChannelPkt extends MacPkt
{
    int address3;
    int address4;
    int fragmentation;	//part of the Frame Control field
    int informationDS;	//part of the Frame Control field
    int sequenceControl;
    bool retry;
    simtime_t duration; //the expected remaining duration the current transaction 
						//(RTS->CTS->DATA->ACK) between two macs will occupy the channel 
	
	int pktsToSend;
	bool cub[90];		//NO_OF_CHANNELS = 3, NO_OF_SLOTS = 30, 30*3 = 90
	bool cab[90];		//NO_OF_CHANNELS = 3, NO_OF_SLOTS = 30, 30*3 = 90
	int transmissionAttempts;
}
