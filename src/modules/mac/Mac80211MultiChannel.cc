#include "mac/Mac80211MultiChannel.h"

#include "PhyToMacControlInfo.h"
#include "FWMath.h"
#include "phy/Decider80211.h"
#include "DeciderResult80211.h"
#include "BaseConnectionManager.h"
#include "MacToPhyInterface.h"
#include "ChannelSenseRequest_m.h"
#include "BasePhyLayer.h"

Define_Module(Mac80211MultiChannel);

Mac80211MultiChannel::Mac80211MultiChannel()
	: BaseMacLayer()
	, timeout()
	, nav(NULL)
    , atim_window(NULL)
    , communication_window(NULL)
    , communication_slot(NULL)
	, contention(NULL)
	, endSifs(NULL)
    , sniffer(NULL)
	, chSenseStart()
	, state()
	, defaultBitrate(0)
	, txPower(0)
	, centerFreq(0)
	, bitrate(0)
	, autoBitrate(false)
	, snrThresholds()
	, queueLength(0)
	, nextIsBroadcast(false)
	, fromUpperLayer()
    , fromUpperLayerBroadcast()
    , fromUpperLayerBroadcastBuffer()
    , fromUpperLayerHandled()
    , fromUpperLayerBroadcastHandled()
	, longRetryCounter(0)
	, shortRetryCounter(0)
	, remainingBackoff()
	, currentIFS()
	, rtsCtsThreshold(0)
	, delta()
	, neighborhoodCacheSize(0)
	, neighborhoodCacheMaxAge()
	, neighbors()
	, switching(false)
	, fsc(0)
    , slotNo(0)
    , cub()
    , slotInstructions()
{}

void Mac80211MultiChannel::initialize(int stage)
{
    BaseMacLayer::initialize(stage);

    if (stage == 0)
    {
        debugEV << "Initializing stage 0\n";

        switching = false;
        fsc = intrand(0x7FFFFFFF);
        if(fsc == 0) fsc = 1;
        debugEV << " fsc: " << fsc << "\n";

        queueLength = hasPar("queueLength") ? par("queueLength").longValue() : 30;

        // timers
        timeout = new cMessage("timeout", TIMEOUT);
        nav = new cMessage("NAV", NAV);
        atim_window = new cMessage("atim_window", ATIM_WINDOW);
        communication_window = new cMessage("communication_window", COMMUNICATION_WINDOW);
        communication_slot = new cMessage("communication_slot", COMMUNICATION_SLOT);

        contention = new ChannelSenseRequest("contention", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        contention->setSenseMode(UNTIL_BUSY);
        endSifs = new ChannelSenseRequest("end SIFS", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        endSifs->setSenseMode(UNTIL_BUSY);
        endSifs->setSenseTimeout(SIFS);
        sniffer = new ChannelSenseRequest("sniffer", MacToPhyInterface::CHANNEL_SENSE_REQUEST);
        sniffer->setSenseMode(UNTIL_BUSY);

        state = IDLE;
        longRetryCounter = 0;
        shortRetryCounter = 0;
        rtsCtsThreshold = hasPar("rtsCtsThreshold") ? par("rtsCtsThreshold").longValue() : 1;
        currentIFS = EIFS;

        autoBitrate = hasPar("autoBitrate") ? par("autoBitrate").boolValue() : false;

        txPower = hasPar("txPower") ? par("txPower").doubleValue() : 110.11;

        delta = 1E-9;

        debugEV << "SIFS: " << SIFS << " DIFS: " << DIFS << " EIFS: " << EIFS << endl;
    }
    else if(stage == 1) {
    	BaseConnectionManager* cc = getConnectionManager();

    	if(cc->hasPar("pMax") && txPower > cc->par("pMax").doubleValue())
            opp_error("TranmitterPower can't be bigger than pMax in ConnectionManager! "
            	      "Please adjust your omnetpp.ini file accordingly.");

    	int channel = phy->getCurrentRadioChannel();
    	if(!(1<=channel && channel<=14)) {
    		opp_error("MiximRadio set to invalid channel %d. Please make sure the"
    				  " phy modules parameter \"initialRadioChannel\" is set to"
    				  " a valid 802.11 channel (1 to 14)!", channel);
    	}
    	centerFreq = CENTER_FREQUENCIES[channel];

        bool found = false;
        bitrate = hasPar("bitrate") ? par("bitrate").doubleValue() : BITRATES_80211[0];
        for(int i = 0; i < 4; i++) {
            if(bitrate == BITRATES_80211[i]) {
                found = true;
                break;
            }
        }
        if(!found) bitrate = BITRATES_80211[0];
        defaultBitrate = bitrate;

        snrThresholds.push_back(hasPar("snr2Mbit") ? par("snr2Mbit").doubleValue() : 100);
        snrThresholds.push_back(hasPar("snr5Mbit") ? par("snr5Mbit").doubleValue() : 100);
        snrThresholds.push_back(hasPar("snr11Mbit") ? par("snr11Mbit").doubleValue() : 100);
        snrThresholds.push_back(111111111); // sentinel

        neighborhoodCacheSize = hasPar("neighborhoodCacheSize") ? par("neighborhoodCacheSize").longValue() : 0;
        neighborhoodCacheMaxAge = hasPar("neighborhoodCacheMaxAge") ? par("neighborhoodCacheMaxAge").longValue() : 10000;

        debugEV << " MAC Address: " << myMacAddr
           << " rtsCtsThreshold: " << rtsCtsThreshold
           << " bitrate: " << bitrate
           << " channel: " << channel
           << " autoBitrate: " << autoBitrate
           << " 2MBit: " << snrThresholds[0]
           << " 5.5MBit: " <<snrThresholds[1]
           << " 11MBit: " << snrThresholds[2]
           << " neighborhoodCacheSize " << neighborhoodCacheSize
           << " neighborhoodCacheMaxAge " << neighborhoodCacheMaxAge
           << endl;

        for(int i = 0; i < 3; i++) {
            snrThresholds[i] = FWMath::dBm2mW(snrThresholds[i]);
        }

        slotNo = 0;

        resetCub();

        resetSlotInstructions();

        remainingBackoff = backoff();
        senseChannelWhileIdle(remainingBackoff + currentIFS);

        scheduleAt(simTime() + ATIM_WINDOW_PERIOD, atim_window);
    }
}

void Mac80211MultiChannel::switchChannel(int channel) {
    if (!(1 <= channel && channel <= 14))
    {
        opp_error("Invalid channel %d. Mac tried to switch to a channel not"
                " supported by this protocoll.", channel);
    }
    phy->setCurrentRadioChannel(channel);

    centerFreq = CENTER_FREQUENCIES[channel];
}

int Mac80211MultiChannel::getChannel() const
{
    return phy->getCurrentRadioChannel();
}

void Mac80211MultiChannel::senseChannelWhileIdle(simtime_t_cref duration) {
	if(contention->isScheduled()) {
		error("Cannot start a new channel sense request because already sensing the channel!");
	}

	chSenseStart = simTime();
	contention->setSenseTimeout(duration);

	sendControlDown(contention);
}

/**
 * This implementation does not support fragmentation, so it is tested
 * if the maximum length of the MPDU is exceeded.
 */
void Mac80211MultiChannel::handleUpperMsg(cMessage *msg)
{
	cPacket* pkt = static_cast<cPacket*>(msg);

	debugEV << "Mac80211MultiChannel::handleUpperMsg " << msg->getName() << "\n";
    if (pkt->getBitLength() > 18496){
        error("packet from higher layer (%s)%s is too long for 802.11b, %d bytes (fragmentation is not supported yet)",
              pkt->getClassName(), pkt->getName(), pkt->getByteLength());
    }

    // check if packet is a broadcast
    bool isBroadcast = getUpperDestinationFromControlInfo(pkt->getControlInfo()).isBroadcast();
    if (isBroadcast)
        debugEV << "packet is a broadcast\n";

    // delete the packet if queue is full and the packet is not a broadcast
    if(fromUpperLayer.size() == queueLength && !isBroadcast) {
		//TODO: CSMAMacLayer does create a new mac packet and sends it up. Maybe settle on a consistent solution here
        msg->setName("MAC ERROR");
        msg->setKind(PACKET_DROPPED);
        sendControlUp(msg);
        debugEV << "packet " << msg << " received from higher layer but MAC queue is full, signalling error\n";
        return;
    }

    Mac80211MultiChannelPkt *mac = static_cast<Mac80211MultiChannelPkt*>(encapsMsg(pkt));
    debugEV << "packet " << pkt << " received from higher layer, dest=" << mac->getDestAddr() << ", encapsulated\n";

    // if the packet is a broadcast, push it into fromUpperLayerBroadcastBuffer
    // fromUpperLayerBroadcastBuffer is emptied into fromUpperLayerBroadcast @ the start of each ATIM window
    if (isBroadcast) {
        fromUpperLayerBroadcastBuffer.push_back(mac);
        debugEV << "packet " << mac << "pushed into fromUpperLayerBroadcastBuffer\n";
    }
    // else just push into normal queue
    else {
        fromUpperLayer.push_back(mac);
        debugEV << "packet " << mac << "pushed into fromUpperLayer\n";
    }

    // If the MAC is in the IDLE state, then start a new contention period
    if (state == IDLE && !endSifs->isScheduled()) {
        beginNewCycle();
    }
    else
    {
    	debugEV << "enqueued, will be transmitted later\n";
    }
}

/**
 * Encapsulates the received network-layer packet into a MacPkt and set all needed
 * header fields.
 */
Mac80211MultiChannel::macpkt_ptr_t Mac80211MultiChannel::encapsMsg(cPacket* netw)
{

    Mac80211MultiChannelPkt *pkt = new Mac80211MultiChannelPkt(netw->getName());
    // headerLength, including final CRC-field AND the phy header length!
    pkt->setBitLength(MAC80211_HEADER_LENGTH);
    pkt->setRetry(false);                 // this is not a retry
    pkt->setSequenceControl(fsc++);       // add a unique fsc to it
    if(fsc <= 0) fsc = 1;

    // initialize transmissionAttempt value
    pkt->setTransmissionAttempts(0);

    // copy dest address from the Control Info attached to the network
    // mesage by the network layer
    cObject *const cInfo = netw->removeControlInfo();

    debugEV << "CInfo removed, mac addr=" << getUpperDestinationFromControlInfo(cInfo) << endl;
    pkt->setDestAddr(getUpperDestinationFromControlInfo(cInfo));

    //delete the control info
    delete cInfo;

    //set the src address to own mac address (nic module id())
    pkt->setSrcAddr(myMacAddr);

    //encapsulate the network packet
    pkt->encapsulate(netw);
    debugEV <<"pkt encapsulated, length: " << pkt->getBitLength() << "\n";

    return pkt;
}

cPacket *Mac80211MultiChannel::decapsMsg(macpkt_ptr_t frame) {
    cPacket *m = frame->decapsulate();
    setUpControlInfo(m, frame->getSrcAddr());
    debugEV << " message decapsulated " << endl;
    return m;
}

/**
 *  Handle all messages from lower layer. Checks the destination MAC
 *  adress of the packet. Then calls one of the three functions :
 *  handleMsgNotForMe(), handleBroadcastMsg(), or handleMsgForMe().
 *  Called by handleMessage().
 */
void Mac80211MultiChannel::handleLowerMsg(cMessage *msg)
{
    Mac80211MultiChannelPkt *af = static_cast<Mac80211MultiChannelPkt *>(msg);
    int radioState = phy->getRadioState();
    if(radioState == MiximRadio::RX) {
        // end of the reception
        debugEV << " handleLowerMsg frame " << af << " received from " << af->getSrcAddr() << " addressed to " << af->getDestAddr() << "\n";
        addNeighbor(af);
        if (contention->isScheduled()) {
            error("Gaack! I am changing the IFS on an ongoing contention");
        }
        currentIFS = DIFS;
        if(af->getDestAddr() == myMacAddr) {
            handleMsgForMe(af);
        }
        else if(LAddress::isL2Broadcast(af->getDestAddr())) {
            if(af->getKind() == BRD)
                handleBRDframe(af);
            else
                handleBroadcastMsg(af);
        }
        else {
            handleMsgNotForMe(af, af->getDuration());
        }
    }
    else {
    	debugEV << " handleLowerMsg frame " << af << " deleted, strange race condition\n";
        delete af;
    }
}

void Mac80211MultiChannel::handleLowerControl(cMessage *msg)
{
	debugEV << simTime() << " handleLowerControl " << msg->getName() << "\n";
    switch(msg->getKind()) {
    case MacToPhyInterface::CHANNEL_SENSE_REQUEST:
    	if(msg == contention) {
    		handleEndContentionTimer();
    	} else if(msg == endSifs) {
    		handleEndSifsTimer();   // noch zu betrachten
    	} else if(msg == sniffer) {
    	    handleEndSnifferTimer();
    	} else {
    		error("Unknown ChannelSenseRequest returned!");
    	}
        break;

    case Decider80211::COLLISION:
    case BaseDecider::PACKET_DROPPED:
    {
    	int radioState = phy->getRadioState();
        if(radioState == MiximRadio::RX) {
            if (contention->isScheduled()) {
                error("Gaack! I am changing the IFS on an ongoing contention");
            }
            currentIFS = EIFS;
            handleMsgNotForMe(msg, 0);
        }
        else {
        	debugEV << " frame " << msg->getName() << " deleted, strange race condition\n";
            delete msg;
        }
        break;
    }
    case MacToPhyInterface::TX_OVER:
    	debugEV << "PHY indicated transmission over" << endl;
        phy->setRadioState(MiximRadio::RX);
        handleEndTransmission();
        delete msg;
        break;

    case MacToPhyInterface::RADIO_SWITCHING_OVER:
    	//TODO: handle this correctly (by now we assume switch times are zero)
    	delete msg;
    	break;

    default:
    	ev << "Unhandled control message from physical layer: " << msg << endl;
    	delete msg;
    	break;
    }
}

/**
 * handle timers
 */
void Mac80211MultiChannel::handleSelfMsg(cMessage * msg)
{
	debugEV << simTime() << " handleSelfMsg " << msg->getName() << "\n";
    switch (msg->getKind())
    {
        // the MAC was waiting for a CTS, a DATA, or an ACK packet but the timer has expired.
    case TIMEOUT:
        handleTimeoutTimer();   // noch zu betrachten..
        break;

        // the MAC was waiting because an other communication had won the channel. This communication is now over
    case NAV:
        handleNavTimer();       // noch zu betrachten...
        break;

    case ATIM_WINDOW:
        handleEndAtimWindowTimer();
        break;

    case COMMUNICATION_WINDOW:
        handleEndCommunicationWindowTimer();
        break;

    case COMMUNICATION_SLOT:
        handleEndCommunicationSlotTimer();
        break;

    default:
        error("unknown timer type");
        break;
    }
}

/**
 *  Handle all ACKs,RTS, CTS, or DATA not for the node. If RTS/CTS
 *  is used the node must stay quiet until the current handshake
 *  between the two communicating nodes is over.  This is done by
 *  scheduling the timer message nav (Network Allocation Vector).
 *  Without RTS/CTS a new contention is started. If an error
 *  occured the node must defer for EIFS.  Called by
 *  handleLowerMsg()
 */
void Mac80211MultiChannel::handleMsgNotForMe(cMessage *af, simtime_t_cref duration)
{
	debugEV << "handle msg not for me " << af->getName() << "\n";

    // if the duration of the packet is null, then do nothing (to avoid
    // the unuseful scheduling of a self message)
    if(duration != 0) {
        // the node is already deferring
        if (state == QUIET)
        {
            // the current value of the NAV is not sufficient
            if (nav->getArrivalTime() < simTime() + duration)
            {
                cancelEvent(nav);
                scheduleAt(simTime() + duration, nav);
                debugEV << "NAV timer started for: " << duration << " State QUIET\n";
            }
        }
        // other states
        else
        {
            // if the MAC wait for another frame, it can delete its time out
            // (exchange is aborted)
            if (timeout->isScheduled()) {
                cancelEvent(timeout);
                if(state == WFACK) {
                    fromUpperLayerHandled.front()->setRetry(true);
                }
                if(state == WFCTS) {
                    //if(rtsCts(fromUpperLayer.front())) {
                        longRetryCounter++;
                    //}
                    //else {
                    //    shortRetryCounter++;
                    //}
                }
            }
            // the node must defer for the time of the transmission
            scheduleAt(simTime() + duration, nav);
            debugEV << "NAV timer started, not QUIET: " << duration << endl;

            assert(!contention->isScheduled());
            //suspendContention();

            setState(QUIET);
        }
    }

    if((af->getKind() == BaseDecider::PACKET_DROPPED) || (af->getKind() == Decider80211::COLLISION)) {

    	assert(!contention->isScheduled());
        //suspendContention();
        //if (contention->isScheduled()) {
        //    error("Gaack! I am changing the IFS on an ongoing contention");
        //}

    	//handle broken cts and ACK frames
    	if(state == WFCTS) {
    		assert(timeout->isScheduled());
    		cancelEvent(timeout);
    		rtsTransmissionFailed();
    	}
    	else if(state == WFACK) {
    		assert(timeout->isScheduled());
			cancelEvent(timeout);
    		dataTransmissionFailed();
    	}

        currentIFS = EIFS;
    }

    // if the message is a CTS/ATIMACK or ATIMRES, update the CUB
    if (af->getKind() == CTS || af->getKind() == RES) {

        Mac80211MultiChannelPkt *msg = static_cast<Mac80211MultiChannelPkt *>(af);
        updateCubFromFrame(msg);
    }

    beginNewCycle();
    delete af;
}

/**
 *  Handle a packet for the node. The result of this reception is
 *  a function of the type of the received message (RTS,CTS,DATA,
 *  or ACK), and of the current state of the MAC (WFDATA, CONTEND,
 *  IDLE, WFCTS, or WFACK). Called by handleLowerMsg()
 */
void Mac80211MultiChannel::handleMsgForMe(Mac80211MultiChannelPkt *af)
{
	debugEV << "handle msg for me " << af->getName() << " in " <<  stateName(state) << "\n";

    switch (state)
    {
    case IDLE:     // waiting for the end of the contention period
    case CONTEND:  // or waiting for RTS

        // RTS or DATA accepted
        if (af->getKind() == RTS) {
        	assert(!contention->isScheduled());
			//suspendContention();

            handleRTSframe(af);
        }
        else if (af->getKind() == DATA) {
        	assert(!contention->isScheduled());
        	//suspendContention();

            handleDATAframe(af);
        }
        else if (af->getKind() == RES) {
            assert(!contention->isScheduled());
            //suspendContention();

            delete af;
        }
        else {
            error("in handleMsgForMe() IDLE/CONTEND, strange message %s", af->getName());
        }
        break;

    case WFDATA:  // waiting for DATA
        if (af->getKind() == DATA) {
            handleDATAframe(af);
        }
        else if (af->getKind() == RES) {
            assert(!contention->isScheduled());
            //suspendContention();

            delete af;
        }
        else {
        	EV << "unexpected message -- probably a collision of RTSs\n";
            delete af;
        }
        break;

    case WFACK:  // waiting for ACK
        if (af->getKind() == ACK) {
            handleACKframe(af);
        }
        else {
            error("in handleMsgForMe() WFACK, strange message %s", af->getName());
        }
        delete af;
        break;

    case WFCTS:  // The MAC is waiting for CTS
        if (af->getKind() == CTS) {
            handleCTSframe(af);
        }
        else {
            EV << "unexpected msg -- deleted \n";
            delete af;
        }
        break;
    case QUIET: // the node is currently deferring.

        // cannot handle any packet with its MAC adress
        delete af;
        break;

    case BUSY: // currently transmitting an ACK or a BROADCAST packet
        if(switching) {
            EV << "message received during radio state switchover\n";
            delete af;
        }
        else {
            error("logic error: node is currently transmitting, can not receive "
                  "(does the physical layer do its job correctly?)");
        }
        break;
    default:
        error("unknown state %d", state);
        break;
    }
}

/**
 *  Handle aframe wich is expected to be an RTS. Called by
 *  HandleMsgForMe()
 */
void Mac80211MultiChannel::handleRTSframe(Mac80211MultiChannelPkt * af)
{
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleRTSframe when SIFS scheduled");
    // wait a short interframe space
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle a frame which expected to be a DATA frame. Called by
 *  HandleMsgForMe()
 */
void Mac80211MultiChannel::handleDATAframe(Mac80211MultiChannelPkt * af)
{
    NeighborList::iterator it;
    //if (rtsCts(af)) cancelEvent(timeout);  // cancel time-out event
    it = findNeighbor(af->getSrcAddr());
    if(it == neighbors.end()) error("Mac80211MultiChannel::handleDATAframe: neighbor not registered");
    if(af->getRetry() && (it->fsc == af->getSequenceControl())) {
    	debugEV << "Mac80211MultiChannel::handleDATAframe suppressed duplicate message " << af
           << " fsc: " << it->fsc << "\n";
    }
    else {
        it->fsc = af->getSequenceControl();
        // pass the packet to the upper layer
        sendUp(decapsMsg(af));
    }
    // wait a short interframe space
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleDATAframe when SIFS scheduled");
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle ACK and delete corresponding packet from queue
 */
void Mac80211MultiChannel::handleACKframe(Mac80211MultiChannelPkt * /*af*/)
{
    // cancel time-out event
    cancelEvent(timeout);

    // the transmission is acknowledged : initialize long_retry_counter
    longRetryCounter = 0;
    shortRetryCounter = 0;
    // post transmit backoff
    remainingBackoff = backoff();
    // removes the acknowledged packet from the queue
    Mac80211MultiChannelPkt *temp = fromUpperLayerHandled.front();
    fromUpperLayerHandled.pop_front();
    delete temp;

    // if thre's a packet to send and if the channel is free then start a new contention period
    beginNewCycle();
}

/**
 *  Handle a CTS frame. Called by HandleMsgForMe(Mac80211MultiChannelPkt* af)
 */
void Mac80211MultiChannel::handleCTSframe(Mac80211MultiChannelPkt * af)
{
    // cancel time-out event
    cancelEvent(timeout);

    // reset counters as TX considers handshake successful
    longRetryCounter = 0;
    shortRetryCounter = 0;

    // remove x unicast packets from fromUpperLayer
    // x is the number of common slots designated
    removePktsFromQueueFor(af->getPktsToSend(), af->getSrcAddr());

    // use the CAB to update our CUB and schedule instructions
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // CAB has been marked
            if (af->getCab(i * 30 + j)) {
                // update CUB, block out time slot
                for (int k = 0; k < NO_OF_CHANNELS; k++) {
                    cub[k][j] = true;
                }
                // schedule instructions
                slotInstructions[j].changeChannel = true;
                slotInstructions[j].channel = i;
                slotInstructions[j].isSender = true;
                slotInstructions[j].recepient = af->getSrcAddr();

                debugEV << "***scheduled transmission of data to " << af->getSrcAddr()
                        << " on channel " << i + 2 << ", slot " << j << "***\n";
            }
        }
    }

    // wait a short interframe space
    if(endSifs->isScheduled()) error("Mac80211MultiChannel::handleCTSframe when SIFS scheduled");
    endSifs->setContextPointer(af);

    sendControlDown(endSifs);
    //scheduleAt(simTime() + SIFS, endSifs);
}

/**
 *  Handle a broadcast packet. This packet is simply passed to the
 *  upper layer. No acknowledgement is needed.  Called by
 *  handleLowerMsg(Mac80211MultiChannelPkt *af)
 */
void Mac80211MultiChannel::handleBroadcastMsg(Mac80211MultiChannelPkt *af)
{
	debugEV << "handle broadcast\n";
    if((state == BUSY) && (!switching)) {
        error("logic error: node is currently transmitting, can not receive "
              "(does the physical layer do its job correctly?)");
    }
    sendUp(decapsMsg(af));
    delete af;
    if (state == CONTEND) {
    	assert(!contention->isScheduled());
		//suspendContention();

        beginNewCycle();
    }
}

void Mac80211MultiChannel::updateCubFromFrame(Mac80211MultiChannelPkt *af)
{
    debugEV << "Mac80211MultiChannel::updateCubFromFrame: " << af->getName() << endl;

    // use the CAB to update our CUB
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // CAB has been marked
            if (af->getCab(i * 30 + j)) {
                // update CUB
                cub[i][j] = true;
            }
        }
    }
}

void Mac80211MultiChannel::handleBRDframe(Mac80211MultiChannelPkt *af)
{
    debugEV << "Mac80211MultiChannel::handleBRDframe" << endl;

    if((state == BUSY) && (!switching)) {
        error("logic error: node is currently transmitting, can not receive "
              "(does the physical layer do its job correctly?)");
    }

    // use the CAB to update our CUB and schedule instructions
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // CAB has been marked
            if (af->getCab(i * 30 + j)) {
                // update CUB, block out time slot
                for (int k = 0; k < NO_OF_CHANNELS; k++) {
                    cub[k][j] = true;
                }
                // schedule instructions
                slotInstructions[j].changeChannel = true;
                slotInstructions[j].channel = i;
                // if scheduled as TX, broadcast has higher priority
                if (slotInstructions[j].isSender && !LAddress::isL2Broadcast(slotInstructions[j].recepient))
                    slotInstructions[j].isSender = false;

                debugEV << "***scheduled reception of broadcast from " << af->getSrcAddr()
                        << " on channel " << i + 2 << ", slot " << j << "***\n";
            }
        }
    }

    delete af;
    if (state == CONTEND) {
        assert(!contention->isScheduled());
        //suspendContention();

        beginNewCycle();
    }
}

/**
 *  The node has won the contention, and is now allowed to send an
 *  RTS/DATA or Broadcast packet. The backoff value is deleted and
 *  will be newly computed in the next contention period
 */
void Mac80211MultiChannel::handleEndContentionTimer()
{
    // not supposed to expire in Communication window
    if(communication_window->isScheduled()) {
        error("logic error: expiration of the contention timer outside of ATIM window, should not happen");
    }

    if(!contention->getResult().isIdle()) {
        suspendContention();
        return;
    }

    if(state == IDLE) {
        remainingBackoff = 0;
        // bug: ID: 3460932
        /*
        In Mac80211MultiChannel, currentIFS is set to DIFS upon successful reception and EIFS
        upon reception of a corrupted frame. This is correct. However, currentIFS
        remains unchanged while performing post-backoff. As a result, the EIFS related
        to reception of a corrupted frame (and subsequent post-backoff) is used again in
        the minimum duration of idle time before a transmission is performed (i.e. the
        mandatory IFS during which a station decides whether to transmit directly or go
        do backoff). This results in incorrect behaviour, as this is a new opportunity.

        A fix is simple: in handleEndContentionTimer(), add "currentIFS = DIFS;" inside the
        state==IDLE if-clause. This is where post-backoff completes and not only remainingBackoff
        but also currentIFS should be reset.

        See also:
        http://www.freeminded.org/?p=811
        */
        currentIFS       = DIFS;
    }
    else if(state == CONTEND) {
        // the node has won the channel, the backoff window is deleted and
        // will be new calculated in the next contention period
        remainingBackoff = 0;
        // unicast packet
        phy->setRadioState(MiximRadio::TX);
        if (!nextIsBroadcast)
        {
            //if(rtsCts(fromUpperLayer.front())) {
                // send a RTS
                sendRTSframe();
            //}
            //else {
            //    sendDATAframe(0);
            //}
        }// broadcast packet
        else {
            //sendBROADCASTframe();
            // removes the packet from the queue without waiting for an acknowledgement
            //Mac80211MultiChannelPkt *temp = fromUpperLayer.front();
            //fromUpperLayer.pop_front();
            //delete(temp);
            sendBRDframe();
        }
    }
    else {
        error("logic error: expiration of the contention timer outside of CONTEND/IDLE state, should not happen");
    }
}

void Mac80211MultiChannel::handleEndSnifferTimer()
{
    if(!sniffer->getResult().isIdle()) {
        debugEV << "postponing broadcast" << endl;
        return;
    }

    debugEV << "sending broadcast" << endl;
    sendBROADCASTframe();
}

/**
 *  Handle the NAV timer (end of a defering period). Called by
 *  HandleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleNavTimer()
{
    if (state != QUIET)
        error("logic error: expiration of the NAV timer outside of the state QUIET, should not happen");

    // if there's a packet to send and if the channel is free, then start a new contention period

    // lmf - Potential race condition: If nav expires at same time
    // medium becomes IDLE (usual case), but the navTimeout comes
    // before the mediumIndication, then the medium is still BUSY when
    // beginNewCycle() is called, so the backoff doesn't resume.
    // Usually it doesn't matter, since there is also an arriving
    // frame and so beginNewCycle() will be called again by the
    // handleMsgXXX.  But if there is no frame (mobility, exposed
    // terminal, etc) there's a potential problem.

    beginNewCycle();
}

void Mac80211MultiChannel::handleEndAtimWindowTimer()
{
    debugEV << "ATIM window over. Starting Communication window.\n";

    if (nav->isScheduled())
        cancelEvent(nav);

    // reset slotNo
    slotNo = 0;

    scheduleAt(simTime() + NO_OF_SLOTS * SLOT_DURATION, communication_window);
    scheduleAt(simTime() + 0, communication_slot);

}

void Mac80211MultiChannel::handleEndCommunicationWindowTimer()
{
    debugEV << "Communication window over. Starting ATIM window.\n";

    // reset to prepare for new ATIM window
    setState(IDLE);
    currentIFS = EIFS;
    remainingBackoff = backoff();

    // clear the CUB and slot instructions
    resetCub();
    resetSlotInstructions();
    recyclePkts();

    // transfer broadcast messages from buffer
    transferBroadcastPktsFromBuffer();

    scheduleAt(simTime() + ATIM_WINDOW_PERIOD, atim_window);

    // switch back to channel 1 and beginNewCycle
    switchChannel(1);
    beginNewCycle();
}

void Mac80211MultiChannel::handleEndCommunicationSlotTimer()
{
    if (slotNo + 1 < NO_OF_SLOTS)
        scheduleAt(simTime() + SLOT_DURATION, communication_slot);

    setState(IDLE);

    // check slotInstructions if anything needs to be done
    if (slotInstructions[slotNo].changeChannel) {
        debugEV << "Time slot " << slotNo << " has started\n";
        // perform scheduled channel switch
        debugEV << "switching to channel " << slotInstructions[slotNo].channel + 2 << endl;
        switchChannel(slotInstructions[slotNo].channel + 2);
        if (slotInstructions[slotNo].isSender) {
            phy->setRadioState(MiximRadio::TX);
            // check if supposed to send broadcast
            if (LAddress::isL2Broadcast(slotInstructions[slotNo].recepient)) {
                debugEV << "sending broadcast" << endl;
                sendBROADCASTframe();

                /*simtime_t sparetime = SLOT_DURATION -
                        (packetDuration(fromUpperLayerBroadcastHandled.front()->getBitLength(), bitrate) + delta);
                int backoffUpper = sparetime.dbl() * 1000000;

                sniffer->setSenseTimeout((simtime_t)intrand(backoffUpper) / 1000000);
                sendControlDown(sniffer);*/
            }
            else {
                // find a packet for the recipient
                bringPktForAddressToFront(slotInstructions[slotNo].recepient);
                // send the packet and wait for ACK
                debugEV << "sending data" << endl;
                sendDATAframe(0);
            }
        }
    }

    slotNo++;
}

void Mac80211MultiChannel::dataTransmissionFailed()
{
    //bool rtscts = rtsCts(fromUpperLayer.front());
    //if(rtscts){
    //    longRetryCounter++;
    //}else{
    //    shortRetryCounter++;
    //}
    fromUpperLayerHandled.front()->setRetry(true);

    int transmissionAttempts = fromUpperLayerHandled.front()->getTransmissionAttempts();
    transmissionAttempts++;
    fromUpperLayerHandled.front()->setTransmissionAttempts(transmissionAttempts);

    if (transmissionAttempts > 2) {
        // delete the frame to transmit
        Mac80211MultiChannelPkt *temp = fromUpperLayerHandled.front();
        fromUpperLayerHandled.pop_front();
        temp->setName("MAC ERROR");
        temp->setKind(PACKET_DROPPED);
        sendControlUp(temp);
    }

    //remainingBackoff = backoff(rtscts);
}

void Mac80211MultiChannel::rtsTransmissionFailed()
{
    longRetryCounter++;
    remainingBackoff = backoff();
}

/**
 *  Handle the time out timer. Called by handleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleTimeoutTimer()
{
    debugEV << simTime() << " handleTimeoutTimer " << stateName(state) << "\n";
    if(state == WFCTS) {
    	rtsTransmissionFailed();
    }
    else if(state == WFACK) {
    	dataTransmissionFailed();
    }

    // if there's a packet to send and if the channel is free then
    // start a new contention period
    if (state != QUIET)
        beginNewCycle();
}


/**
 *  Handle the end sifs timer. Then sends a CTS, a DATA, or an ACK
 *  frame
 */
void Mac80211MultiChannel::handleEndSifsTimer()
{
    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(endSifs->getContextPointer());

    if(!endSifs->getResult().isIdle() && frame->getKind() != CTS){
        // delete the previously received frame
        //delete static_cast<Mac80211MultiChannelPkt *>(endSifs->getContextPointer());
        delete frame;

        // state in now IDLE or CONTEND
        if (fromUpperLayer.empty())
            setState(IDLE);
        else
            setState(CONTEND);

        return;
    }

    phy->setRadioState(MiximRadio::TX);
    switch (frame->getKind())
    {
    case RTS:
        sendCTSframe(frame);
        break;
    case CTS:
        //sendDATAframe(frame);
        sendRESframe(frame);
        break;
    case DATA:
        sendACKframe(frame);
        break;
    default:
        error("logic error: end sifs timer when previously received packet is not RTS/CTS/DATA");
        break;
    }

    // don't need previous frame any more
    delete frame;
}

/**
 *  Handle the end of transmission timer (end of the transmission
 *  of an ACK or a broadcast packet). Called by
 *  HandleTimer(cMessage* msg)
 */
void Mac80211MultiChannel::handleEndTransmission()
{
    debugEV << "transmission of packet is over\n";
    if(state == BUSY) {
        if(nextIsBroadcast) {
            shortRetryCounter = 0;
            longRetryCounter = 0;
            remainingBackoff = backoff();
        }
        beginNewCycle();
    }
    else if(state == WFDATA) {
    	beginNewCycle();
    }
}

/**
 *  Send a DATA frame. Called by HandleEndSifsTimer() or
 *  handleEndContentionTimer()
 */
void Mac80211MultiChannel::sendDATAframe(Mac80211MultiChannelPkt *af)
{
    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(fromUpperLayerHandled.front()->dup());
    double br;

    if(af) {
        br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();

        delete af->removeControlInfo();
    }
    else {
       br  = retrieveBitrate(frame->getDestAddr());

       //if(shortRetryCounter) frame->setRetry(true);
    }
    simtime_t duration = packetDuration(frame->getBitLength(), br);
    setDownControlInfo(frame, createSignal(simTime(), duration, txPower, br));
    // build a copy of the frame in front of the queue'
    frame->setSrcAddr(myMacAddr);
    frame->setKind(DATA);
    frame->setDuration(SIFS + packetDuration(LENGTH_ACK, br));

    // schedule time out
    scheduleAt(simTime() + timeOut(DATA, br), timeout);
    debugEV << "sending DATA  to " << frame->getDestAddr() << " with bitrate " << br << endl;
    // send DATA frame
    sendDown(frame);

    // update state and display
    setState(WFACK);
}

/**
 *  Send an ACK frame.Called by HandleEndSifsTimer()
 */
void Mac80211MultiChannel::sendACKframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-ack");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_ACK, br), txPower, br));
    frame->setKind(ACK);
    frame->setBitLength((int)LENGTH_ACK);

    // the dest address must be the src adress of the RTS or the DATA
    // packet received. The src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());
    frame->setDuration(0);

    sendDown(frame);
    debugEV << "sent ACK frame!\n";

    // update state and display
    setState(BUSY);
}

/**
 *  Send a RTS frame.Called by handleContentionTimer()
 */
void Mac80211MultiChannel::sendRTSframe()
{
    if (!CubSlotsAvailable()) {
        // no point sending any handshakes
        return;
    }

    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-rts");

    const Mac80211MultiChannelPkt* frameToSend = fromUpperLayer.front();
    double br = retrieveBitrate(frameToSend->getDestAddr());

    setDownControlInfo(frame, createSignal(simTime(),packetDuration(LENGTH_ATIM, br), txPower, br));

    frame->setKind(RTS);
    frame->setBitLength((int)LENGTH_ATIM);

    // the src adress and dest address are copied in the frame in the queue (frame to be sent)
    frame->setSrcAddr(frameToSend->getSrcAddr());
    frame->setDestAddr(frameToSend->getDestAddr());

    // copy our CUB into the message
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            frame->setCub(i * 30 + j, cub[i][j]);
        }
    }
    debugEV << "Copied the CUB into the message" << endl;

    // find the number of packets in the queue for the destination address
    int pktsToSend = countNumberOfPktsFor(frame->getDestAddr());
    if (pktsToSend < 1)
        error("logic error: RTS requesting for < 1 slot");
    frame->setPktsToSend(pktsToSend);

    // set channel clearance duration
    frame->setDuration(2 * SIFS + packetDuration(LENGTH_ATIMACK, br) +
                       packetDuration(LENGTH_ATIMRES, br));

    debugEV << " Mac80211MultiChannel::sendRTSframe duration: " <<  packetDuration(LENGTH_ATIM, br) << " br: " << br << "\n";

    // schedule time-out
    scheduleAt(simTime() + timeOut(RTS, br), timeout);

    // send RTS frame
    sendDown(frame);

    // update state and display
    setState(WFCTS);
}

/**
 *  Send a CTS frame.Called by HandleEndSifsTimer()
 */
void Mac80211MultiChannel::sendCTSframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-cts");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_ATIMACK, br), txPower, br));

    frame->setKind(CTS);
    frame->setBitLength((int)LENGTH_ATIMACK);

    // the dest adress must be the src adress of the RTS/ATIM received.
    // The src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());

    /*generate CAB*/
    //  read the CUB from the RTS/ATIM and create a list of common slots
    std::vector<int> slotsAvailable;
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // if both CUBs are not marked for the slot
            if (!cub[i][j] && !af->getCub(i * 30 + j))
                slotsAvailable.push_back(i * 30 + j);
        }
    }
    //  if no common slots available, then we pretend we never received RTS/ATIM
    if (slotsAvailable.empty()) {
        debugEV << "No common slots available. Ignoring RTS/ATIM from " << af->getSrcAddr() << "\n";
        delete frame;
        beginNewCycle();
        return;
    }

    //  shuffle slotsAvailable
    std::random_shuffle(slotsAvailable.begin(), slotsAvailable.end());

    //  select slots based on number of slots requested by TX
    std::vector<int> selectedSlots;

    while (!slotsAvailable.empty()) {

        selectedSlots.push_back(slotsAvailable.back());
        int removedSlot = slotsAvailable.back() % NO_OF_SLOTS;
        // filter away options existing in the same time slot
        for (int i = 0; i < NO_OF_CHANNELS; i++) {
            slotsAvailable.erase(remove(slotsAvailable.begin(), slotsAvailable.end(), removedSlot + i * NO_OF_SLOTS), slotsAvailable.end());
        }
    }

    debugEV << af->getSrcAddr() << " requested " << af->getPktsToSend() << " slots. "
            << selectedSlots.size() << " common time slots available\n";

    unsigned int slotsRequested = af->getPktsToSend();
    while(selectedSlots.size() > slotsRequested) {
        selectedSlots.pop_back();
    }
    frame->setPktsToSend(selectedSlots.size());

    //  initialize the message's CAB
    for (int i = 0; i < NO_OF_CHANNELS * NO_OF_SLOTS; i++) {
        frame->setCab(i, false);
    }
    //  write CAB for new message
    while(!selectedSlots.empty()) {
        frame->setCab(selectedSlots.back(), true);
        selectedSlots.pop_back();
    }
    /*end CAB generation*/

    // use the CAB to update our CUB and schedule instructions
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // CAB has been marked
            if (frame->getCab(i * 30 + j)) {
                // update CUB, block out time slot
                for (int k = 0; k < NO_OF_CHANNELS; k++) {
                    cub[k][j] = true;
                }
                // schedule instructions
                slotInstructions[j].changeChannel = true;
                slotInstructions[j].channel = i;

                debugEV << "***scheduled reception of data from " << af->getSrcAddr()
                        << " on channel " << i + 2 << ", slot " << j << "***\n";
            }
        }
    }

    // set channel clearance duration
    frame->setDuration(af->getDuration() - SIFS - packetDuration(LENGTH_ATIMACK, br));

    debugEV << " Mac80211MultiChannel::sendCTSframe duration: " <<  packetDuration(LENGTH_ATIMACK, br) << " br: " << br << "\n";
    // send CTS frame
    sendDown(frame);

    // update state and display
    setState(WFDATA);
}

/**
 *  Send a BROADCAST frame.Called by handleContentionTimer()
 */
void Mac80211MultiChannel::sendBROADCASTframe()
{
    // send a copy of the frame in front of the queue
    Mac80211MultiChannelPkt *frame = static_cast<Mac80211MultiChannelPkt *>(fromUpperLayerBroadcastHandled.front()->dup());

    double br = retrieveBitrate(frame->getDestAddr());

    simtime_t duration = packetDuration(frame->getBitLength(), br);
    setDownControlInfo(frame, createSignal(simTime(), duration, txPower, br));

    frame->setKind(BROADCAST);

    sendDown(frame);

    // removes the packet from the queue without waiting for an acknowledgement
    Mac80211MultiChannelPkt *temp = fromUpperLayerBroadcastHandled.front();
    fromUpperLayerBroadcastHandled.pop_front();
    delete(temp);

    // update state and display
    setState(BUSY);
}

void Mac80211MultiChannel::sendRESframe(Mac80211MultiChannelPkt * af)
{
    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-res");

    double br = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af))->getBitrate();
    delete af->removeControlInfo();

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_ATIMRES, br), txPower, br));
    frame->setKind(RES);
    frame->setBitLength((int)LENGTH_ATIMRES);

    // the dest address must be the src adress of the CTS/ATIMACK
    // packet received. The src adress is the adress of the node
    frame->setSrcAddr(myMacAddr);
    frame->setDestAddr(af->getSrcAddr());

    // copy the CAB from the previous message
    for (int i = 0; i < NO_OF_CHANNELS * NO_OF_SLOTS; i++) {
        frame->setCab(i, af->getCab(i));
    }

    // set channel clearance duration
    frame->setDuration(0);

    sendDown(frame);
    debugEV << "sent RES frame!\n";

    // update state and display
    setState(BUSY);
}

void Mac80211MultiChannel::sendBRDframe()
{
    if (!CubSlotsAvailable()) {
        // no point sending any handshakes
        return;
    }

    Mac80211MultiChannelPkt *frame = new Mac80211MultiChannelPkt("wlan-brd");

    const Mac80211MultiChannelPkt* frameToSend = fromUpperLayerBroadcast.front();
    double br = retrieveBitrate(frameToSend->getDestAddr());

    setDownControlInfo(frame, createSignal(simTime(), packetDuration(LENGTH_ATIMBRD, br), txPower, br));

    frame->setKind(BRD);
    frame->setBitLength((int)LENGTH_ATIMBRD);

    frame->setDestAddr(LAddress::L2BROADCAST);

    /*generate CAB*/
    //  read our CUB's free slots into a list
    std::vector<int> slotsAvailable;
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // if CUBs is not marked for the slot
            if (!cub[i][j] && timeSlotFree(j))
                slotsAvailable.push_back(i * 30 + j);
        }
    }

    //  shuffle slotsAvailable
    std::random_shuffle(slotsAvailable.begin(), slotsAvailable.end());

    //  select slots based on number of slots requested by TX
    std::vector<int> selectedSlots;

    while (!slotsAvailable.empty()) {

        selectedSlots.push_back(slotsAvailable.back());
        int removedSlot = slotsAvailable.back() % NO_OF_SLOTS;
        // filter away options existing in the same time slot
        for (int i = 0; i < NO_OF_CHANNELS; i++) {
            slotsAvailable.erase(remove(slotsAvailable.begin(), slotsAvailable.end(), removedSlot + i * NO_OF_SLOTS), slotsAvailable.end());
        }
    }

    unsigned int slotsRequested = countNumberOfBroadcastPkts();
    debugEV << slotsRequested << " broadcast packets to send. "
            << selectedSlots.size() << " time slots available\n";

    while(selectedSlots.size() > slotsRequested) {
        selectedSlots.pop_back();
    }

    // remove "handled" broadcast messages from queue
    removeBroadcastPktsFromQueue(selectedSlots.size());

    //  initialize the message's CAB
    for (int i = 0; i < NO_OF_CHANNELS * NO_OF_SLOTS; i++) {
        frame->setCab(i, false);
    }
    //  write CAB for new message
    while(!selectedSlots.empty()) {
        frame->setCab(selectedSlots.back(), true);
        selectedSlots.pop_back();
    }
    /*end CAB generation*/

    // use the CAB to update our CUB and schedule instructions
    for (int i = 0; i < NO_OF_CHANNELS; i++) {
        for (int j = 0; j < NO_OF_SLOTS; j++) {
            // CAB has been marked
            if (frame->getCab(i * 30 + j)) {
                // update CUB, block out time slot
                for (int k = 0; k < NO_OF_CHANNELS; k++) {
                    cub[k][j] = true;
                }
                // schedule instructions
                slotInstructions[j].changeChannel = true;
                slotInstructions[j].channel = i;
                slotInstructions[j].isSender = true;
                slotInstructions[j].recepient = LAddress::L2BROADCAST;

                debugEV << "***scheduled broadcast on channel "
                        << i + 2 << ", slot " << j << "***\n";
            }
        }
    }

    // set channel clearance duration
    frame->setDuration(0);

    sendDown(frame);
    debugEV << "sent BRD frame!\n";

    // update state and display
    setState(BUSY);
}

/**
 *  Start a new contention period if the channel is free and if
 *  there's a packet to send.  Called at the end of a deferring
 *  period, a busy period, or after a failure. Called by the
 *  HandleMsgForMe(), HandleTimer() HandleUpperMsg(), and, without
 *  RTS/CTS, by handleMsgNotForMe().
 */
void Mac80211MultiChannel::beginNewCycle()
{
    // before trying to send one more time a packet, test if the
    // maximum retry limit is reached. If it is the case, then
    // delete the packet and send the next packet.
    testMaxAttempts();

    // if currently in the Communication window, we cannot start a new cycle
    if (communication_window->isScheduled()) {
        debugEV << "cannot beginNewCycle until ATIM window begins at t " << communication_window->getArrivalTime() << endl;
        return;
    }

    if (nav->isScheduled()) {
    	debugEV << "cannot beginNewCycle until NAV expires at t " << nav->getArrivalTime() << endl;
        return;
    }

    /*
    if(timeout->isScheduled()) {
    	cancelEvent(timeout);
    }
    */

    // if there are packets to be sent
    if (!fromUpperLayer.empty() || !fromUpperLayerBroadcast.empty()) {

        // look if the next packet is unicast or broadcast
        // it is a broadcast if there is something in fromUpperLayerBroadcast
        nextIsBroadcast = !fromUpperLayerBroadcast.empty();

        setState(CONTEND);
        if(!contention->isScheduled()) {

            /*perform the following checks before starting/scheduling contention*/
            // total length of successful ATIM handshake
            simtime_t atim_t;
            if (!nextIsBroadcast) {
                atim_t =    packetDuration(LENGTH_ATIM, bitrate) +
                            packetDuration(LENGTH_ATIMACK, bitrate) +
                            packetDuration(LENGTH_ATIMRES, bitrate) +
                            3 * delta + 2 * SIFS + DIFS;

                // binary exponential backoff... shorten by half
                remainingBackoff /= 2;
            }
            else
                atim_t =    packetDuration(LENGTH_ATIMBRD, bitrate) + delta;

            // check if there is enough time in the ATIM window for successful handshake
            if (atim_window->getArrivalTime() < simTime() + currentIFS + remainingBackoff + atim_t) {
                debugEV << "not enough time left in ATIM window to start handshake" << endl;

                // might still have time to answer RTS/ATIM from other nodes
                setState(IDLE);
                return;
            }
            // check if there are any slots available
            if (!CubSlotsAvailable()) {
                // no point answering any handshakes
                return;
            }
            /*checks end*/

            ChannelState channel = phy->getChannelState();
            debugEV << simTime() << " do contention: medium = " << channel.info() << ", backoff = "
               <<  remainingBackoff << endl;

            if(channel.isIdle()) {
                senseChannelWhileIdle(currentIFS + remainingBackoff);
                //scheduleAt(simTime() + currentIFS + remainingBackoff, contention);
            }
            else {
                /*
                Mac80211MultiChannel in MiXiM uses the same mechanism for backoff and post-backoff, a senseChannelWhileIdle which
                schedules a timer for a duration of IFS + remainingBackoff (i.e. The inter-frame spacing and the present
                state of the backoff counter).

                If Host A were doing post-backoff when the frame from Host B arrived, the remainingBackoff would have
                been > 0 and backoff would have resumed after the frame from Host B finishes.

                However, Host A has already completed its post-backoff (remainingBackoff was 0) so it essentially was
                IDLE when the beacon was generated (actually, it was receiving Host B’s frame). So what happens now is
                that all nodes which have an arrival during Host B’s frame AND have completed their post-backoff, will
                wait one IFS and then transmit, resulting in synchronised collisions one IFS after a transmission (or
                an EIFS after a collision).

                The correct behaviour here is for Host A’s MAC to note that post-backoff has completed (remainingBackoff
                has reached 0) and the medium is busy, so the MAC must draw a new backoff from the contention window.

                http://www.freeminded.org/?p=801
                */
                //channel is busy
                if(remainingBackoff==0) {
                    remainingBackoff = backoff();
                }
            }
        }
    }
    else {
        // post-xmit backoff (minor nit: if random backoff=0, we punt)
        if(remainingBackoff > 0 && !contention->isScheduled()) {
        	ChannelState channel = phy->getChannelState();
        	debugEV << simTime() << " do contention: medium = " << channel.info() << ", backoff = "
               <<  remainingBackoff << endl;

            if(channel.isIdle()) {
                if (atim_window->getArrivalTime() > simTime() + currentIFS + remainingBackoff) {
                    senseChannelWhileIdle(currentIFS + remainingBackoff);
                }
                //scheduleAt(simTime() + currentIFS + remainingBackoff, contention);
            }
        }
        setState(IDLE);
    }
}

/**
 * Compute the backoff value.
 */
simtime_t Mac80211MultiChannel::backoff(bool rtscts) {
    unsigned rc = (rtscts) ?  longRetryCounter : shortRetryCounter;
    unsigned cw = ((CW_MIN + 1) << rc) - 1;
    if(cw > CW_MAX) cw = CW_MAX;

    simtime_t value = ((double) intrand(cw + 1)) * ST;
    debugEV << simTime() << " random backoff = " << value << endl;

    return value;
}

/**
 *  Test if the maximal retry limit is reached, and delete the
 *  frame to send in this case.
 */
void Mac80211MultiChannel::testMaxAttempts()
{
    if ((longRetryCounter >= LONG_RETRY_LIMIT) || (shortRetryCounter >= SHORT_RETRY_LIMIT)) {
    	debugEV << "retry limit reached src: "<< shortRetryCounter << " lrc: " << longRetryCounter << endl;
        // initialize counter
        longRetryCounter = 0;
        shortRetryCounter = 0;
        // delete the frame to transmit
        Mac80211MultiChannelPkt *temp = fromUpperLayer.front();
        fromUpperLayer.pop_front();
        temp->setName("MAC ERROR");
        temp->setKind(PACKET_DROPPED);
        sendControlUp(temp);
    }
}

Signal* Mac80211MultiChannel::createSignal(	simtime_t_cref start, simtime_t_cref length,
								double power, double bitrate)
{
	simtime_t end = start + length;
	//create signal with start at current simtime and passed length
	Signal* s = new Signal(start, length);

	//create and set tx power mapping
	ConstMapping* txPowerMapping
			= createSingleFrequencyMapping(	start, end,
											centerFreq, 11.0e6,
											power);
	s->setTransmissionPower(txPowerMapping);

	//create and set bitrate mapping

	//create mapping over time
	Mapping* bitrateMapping
			= MappingUtils::createMapping(DimensionSet::timeDomain,
										  Mapping::STEPS);

	Argument pos(start);
	bitrateMapping->setValue(pos, BITRATE_HEADER);

	pos.setTime(PHY_HEADER_LENGTH / BITRATE_HEADER);
	bitrateMapping->setValue(pos, bitrate);

	s->setBitrate(bitrateMapping);

	return s;
}

/**
 *  Return a time-out value for a type of frame. Called by
 *  SendRTSframe, sendCTSframe, etc.
 */
simtime_t Mac80211MultiChannel::timeOut(Mac80211MultiChannelMessageKinds type, double br)
{
    simtime_t time_out = 0;

    switch (type)
    {
    case RTS:
        time_out = SIFS + packetDuration(LENGTH_ATIM, br) + ST + packetDuration(LENGTH_ATIMACK, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut RTS " << time_out << "\n";
        break;
    case DATA:
        time_out = SIFS + packetDuration(fromUpperLayerHandled.front()->getBitLength(), br) + ST + packetDuration(LENGTH_ACK, br) + delta;
        debugEV << " Mac80211MultiChannel::timeOut DATA " << time_out << "\n";
        break;
    default:
        EV << "Unused frame type was given when calling timeOut(), this should not happen!\n";
        break;
    }
    return time_out;
}

/**
 * Computes the duration of the transmission of a frame over the
 * physical channel. 'bits' should be the total length of the
 * mac packet in bits excluding the phy header length.
 */
simtime_t Mac80211MultiChannel::packetDuration(double bits, double br)
{
    return bits / br + phyHeaderLength / BITRATE_HEADER;
}

const char *Mac80211MultiChannel::stateName(State state)
{
#define CASE(x) case x: s=#x; break
    const char *s = "???";
    switch (state)
    {
        CASE(WFDATA);
        CASE(QUIET);
        CASE(IDLE);
        CASE(CONTEND);
        CASE(WFCTS);
        CASE(WFACK);
        CASE(BUSY);
    }
    return s;
#undef CASE
}

void Mac80211MultiChannel::setState(State newState)
{
    if (state==newState)
    	debugEV << "staying in state " << stateName(state) << "\n";
    else
    	debugEV << "state " << stateName(state) << " --> " << stateName(newState) << "\n";
    state = newState;
}

void Mac80211MultiChannel::suspendContention()  {
	assert(!contention->isScheduled());
    // if there's a contention period

    //if(requestReturned || chSenseRequest->isScheduled()) {
        // update the backoff window in order to give higher priority in
        // the next battle

    	simtime_t quietTime = simTime() - chSenseStart;

    	debugEV << simTime() << " suspend contention: "
		   << "began " << chSenseStart
		   << ", ends " << chSenseStart + contention->getSenseTimeout()
		   << ", ifs " << currentIFS
		   << ", quiet time " << quietTime
		   << endl;

        if(quietTime < currentIFS) {
        	debugEV << "suspended during D/EIFS (no backoff)" << endl;
        }
        else {
            double remainingSlots;
            remainingSlots = SIMTIME_DBL(contention->getSenseTimeout() - quietTime)/ST;

            // Distinguish between (if) case where contention is
            // suspended after an integer number of slots and we
            // _round_ to integer to avoid fp error, and (else) case
            // where contention is suspended mid-slot (e.g. hidden
            // terminal xmits) and we _ceil_ to repeat the partial
            // slot.  Arbitrary value 0.0001 * ST is used to
            // distinguish the two cases, which may a problem if clock
            // skew between nic's is ever implemented.

            if (fabs(ceil(remainingSlots) - remainingSlots) < 0.0001 * ST ||
                fabs(floor(remainingSlots) - remainingSlots) < 0.0001 * ST) {
                remainingBackoff = floor(remainingSlots + 0.5) * ST;
            }
            else {
                remainingBackoff = ceil(remainingSlots) * ST;
            }

            debugEV << "backoff was " << ((contention->getSenseTimeout() - currentIFS))/ST
               << " slots, now " << remainingSlots << " slots remain" << endl;
        }

        debugEV << "suspended backoff timer, remaining backoff time: "
           << remainingBackoff << endl;

    //}
}

double Mac80211MultiChannel::retrieveBitrate(const LAddress::L2Type& destAddress) {
    double bitrate = defaultBitrate;
    NeighborList::iterator it;
    if(autoBitrate && !LAddress::isL2Broadcast(destAddress) &&
       (longRetryCounter == 0) && (shortRetryCounter == 0)) {
        it = findNeighbor(destAddress);
        if((it != neighbors.end()) && (it->age > (simTime() - neighborhoodCacheMaxAge))) {
            bitrate = it->bitrate;
        }
    }
    return bitrate;
}

void Mac80211MultiChannel::addNeighbor(Mac80211MultiChannelPkt *af) {
    const LAddress::L2Type&   srcAddress = af->getSrcAddr();
    NeighborList::iterator    it         = findNeighbor(srcAddress);
    const DeciderResult80211* result     = static_cast<const DeciderResult80211*>(PhyToMacControlInfo::getDeciderResult(af));
    double snr = result->getSnr();

    double bitrate = BITRATES_80211[0];
    NeighborEntry entry;

    if(snr > snrThresholds[0]) bitrate = BITRATES_80211[1];
    if(snr > snrThresholds[1]) bitrate = BITRATES_80211[2];
    if(snr > snrThresholds[2]) bitrate = BITRATES_80211[3];

    if(it != neighbors.end()) {
        it->age = simTime();
        it->bitrate = bitrate;
    }
    else {
        if(neighbors.size() < neighborhoodCacheSize) {
            entry.address = srcAddress;
            entry.age = simTime();
            entry.bitrate = bitrate;
            entry.fsc = 0;
            neighbors.push_back(entry);
        }
        else {
            it = findOldestNeighbor();
            if(it != neighbors.end()) {
                it->age = simTime();
                it->bitrate = bitrate;
                it->address = srcAddress;
                it->fsc = 0;
            }
        }
    }
    debugEV << "updated information for neighbor: " << srcAddress
       << " snr: " << snr << " bitrate: " << bitrate << endl;
}

Mac80211MultiChannel::~Mac80211MultiChannel() {
	cancelAndDelete(timeout);
	cancelAndDelete(nav);
	cancelAndDelete(atim_window);
	cancelAndDelete(communication_window);
	cancelAndDelete(communication_slot);
	if(contention && !contention->isScheduled())
		delete contention;
	if(endSifs && !endSifs->isScheduled())
		delete endSifs;
	if(sniffer && !sniffer->isScheduled())
        delete sniffer;

	MacPktList::iterator it;
	for(it = fromUpperLayer.begin(); it != fromUpperLayer.end(); ++it) {
        delete (*it);
    }
    fromUpperLayer.clear();
    for(it = fromUpperLayerBroadcast.begin(); it != fromUpperLayerBroadcast.end(); ++it) {
        delete (*it);
    }
    fromUpperLayerBroadcast.clear();
    for(it = fromUpperLayerBroadcastBuffer.begin(); it != fromUpperLayerBroadcastBuffer.end(); ++it) {
        delete (*it);
    }
    fromUpperLayerBroadcastBuffer.clear();
    for(it = fromUpperLayerHandled.begin(); it != fromUpperLayerHandled.end(); ++it) {
        delete (*it);
    }
    fromUpperLayerHandled.clear();
    for(it = fromUpperLayerBroadcastHandled.begin(); it != fromUpperLayerBroadcastHandled.end(); ++it) {
        delete (*it);
    }
    fromUpperLayerBroadcastHandled.clear();

}

void Mac80211MultiChannel::finish() {
    BaseMacLayer::finish();
}

