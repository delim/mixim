#ifndef __MIXIM_MULTICHANNELMAC80211_H_
#define __MIXIM_MULTICHANNELMAC80211_H_

#include <omnetpp.h>
#include <list>
#include <vector>
#include <algorithm>

#include "MiXiMDefs.h"
#include "BaseMacLayer.h"
#include "Consts80211.h"
#include "Mac80211MultiChannelPkt_m.h"

#define BEACON_PERIOD       0.1
#define NO_OF_CHANNELS      3
#define NO_OF_SLOTS         30
#define SLOT_DURATION       0.00275
#define ATIM_WINDOW_PERIOD  BEACON_PERIOD - (NO_OF_SLOTS * SLOT_DURATION)
#define LENGTH_ATIM         LENGTH_RTS + (NO_OF_SLOTS * NO_OF_CHANNELS) + 5 //+CUB +pktsToSend
#define LENGTH_ATIMACK      LENGTH_CTS + (NO_OF_SLOTS * NO_OF_CHANNELS) //+CAB
#define LENGTH_ATIMRES      LENGTH_CTS + (NO_OF_SLOTS * NO_OF_CHANNELS) //+CAB
#define LENGTH_ATIMBRD      64 + (NO_OF_SLOTS * NO_OF_CHANNELS) //+CAB -ADDR

bool compByArrivalTime(Mac80211MultiChannelPkt* a, Mac80211MultiChannelPkt* b)
{
    return a->getArrivalTime() < b->getArrivalTime();
}

class ChannelSenseRequest;

/**
 * @brief An implementation of the 802.11b MAC.
 *
 * For more info, see the NED file.
 *
 * @ingroup macLayer
 * @ingroup ieee80211
 * @author David Raguin, Karl Wessel (port for MiXiM)
 */
class MIXIM_API Mac80211MultiChannel : public BaseMacLayer
{
private:
	/** @brief Copy constructor is not allowed.
	 */
	Mac80211MultiChannel(const Mac80211MultiChannel&);
	/** @brief Assignment operator is not allowed.
	 */
	Mac80211MultiChannel& operator=(const Mac80211MultiChannel&);

public:

	/** @brief frame kinds */
	enum Mac80211MultiChannelMessageKinds {
	  //between MAC layers of two nodes
	  RTS = LAST_BASE_MAC_MESSAGE_KIND, // request to send
	  CTS,                 // clear to send
	  ACK,                 // acknowledgement
	  DATA,
	  BROADCAST,
	  RES,
	  BRD,
	  LAST_MAC_80211_MESSAGE_KIND
	};
protected:
	/** @brief Type for a queue of Mac80211MultiChannelPkts.*/
    typedef std::list<Mac80211MultiChannelPkt*> MacPktList;

    /** Definition of the timer types */
    enum timerType {
      TIMEOUT,
      NAV,
      ATIM_WINDOW,
      COMMUNICATION_WINDOW,
      COMMUNICATION_SLOT
    };

    /** Definition of the states*/
    enum State {
      WFDATA = 0, // waiting for data packet
      QUIET = 1,  // waiting for the communication between two other nodes to end
      IDLE = 2,   // no packet to send, no packet receiving
      CONTEND = 3,// contention state (battle for the channel)
      WFCTS = 4,  // RTS sent, waiting for CTS
      WFACK = 5,  // DATA packet sent, waiting for ACK
      BUSY = 6    // during transmission of an ACK or a BROADCAST packet
    };

    /** @brief Data about a neighbor host.*/
    struct NeighborEntry {
    	/** @brief The neighbors address.*/
    	LAddress::L2Type address;
        int              fsc;
        simtime_t        age;
        double           bitrate;

        NeighborEntry() : address(), fsc(0), age(), bitrate(0) {}
    };

    /** @brief Type for a list of NeighborEntries.*/
    typedef std::list<NeighborEntry> NeighborList;

  public:
    Mac80211MultiChannel();
    virtual ~Mac80211MultiChannel();

    virtual void initialize(int);
    virtual void finish();

    /**
    * @brief Tells the MAC layer to switch to the passed channel.
    *
    * This method can be used by upper layers to change the channel.
    * @param channel The channel to switch to, must be 1<=channel<=14.
    */
    void switchChannel(int channel);

    /**
    * @brief Returns the currently used channel.
    * @return The currently used channel.
    */
    int getChannel() const;

 protected:

    /** @brief Handle self messages such as timer... */
	virtual void handleSelfMsg(cMessage*);

	/** @brief Handle messages from upper layer */
	virtual void handleUpperMsg(cMessage* msg);

	/** @brief Handle messages from lower layer */
	virtual void handleLowerMsg(cMessage*);

	/** @brief Handle messages from lower layer */
	virtual void handleLowerControl(cMessage*);


    /** @brief handle end of contention */
    virtual void handleEndContentionTimer();

    /** @brief handle a message that is not for me or errornous*/
    void handleMsgNotForMe(cMessage *af, simtime_t_cref duration);
    /** @brief handle a message that was meant for me*/
    void handleMsgForMe(Mac80211MultiChannelPkt*);
    // ** @brief handle a Broadcast message*/
    void handleBroadcastMsg(Mac80211MultiChannelPkt*);

    /** @brief handle the end of a transmission...*/
    void handleEndTransmission();

    /** @brief handle end of SIFS*/
    void handleEndSifsTimer();

    void handleEndSnifferTimer();

    /** @brief handle time out*/
    void handleTimeoutTimer();
    /** @brief NAV timer expired, the exchange of messages of other
       stations is done*/
    void handleNavTimer();

    void handleEndAtimWindowTimer();
    void handleEndCommunicationWindowTimer();
    void handleEndCommunicationSlotTimer();

    void handleRTSframe(Mac80211MultiChannelPkt*);

    void handleDATAframe(Mac80211MultiChannelPkt*);

    void handleACKframe(Mac80211MultiChannelPkt*);

    void handleCTSframe(Mac80211MultiChannelPkt*);

    void updateCubFromFrame(Mac80211MultiChannelPkt*);

    void handleBRDframe(Mac80211MultiChannelPkt*);

    void dataTransmissionFailed();

    void rtsTransmissionFailed();

    /** @brief send data frame */
    virtual void sendDATAframe(Mac80211MultiChannelPkt*);

    /** @brief send Acknoledgement */
    void sendACKframe(Mac80211MultiChannelPkt*);

    /** @brief send CTS frame */
    void sendCTSframe(Mac80211MultiChannelPkt*);

    /** @brief send RTS frame */
    virtual void sendRTSframe();

    /** @brief send broadcast frame */
    void sendBROADCASTframe();

    void sendRESframe(Mac80211MultiChannelPkt*);

    void sendBRDframe();

    /** @brief encapsulate packet */
    virtual macpkt_ptr_t encapsMsg(cPacket *netw);

    /** @brief decapsulate packet */
    virtual cPacket* decapsMsg(macpkt_ptr_t frame);

    /** @brief start a new contention period */
    virtual void beginNewCycle();

    /** @brief Compute a backoff value */
    simtime_t backoff(bool rtscts = true);

    /** @brief Test if maximum number of retries to transmit is exceeded */
    void testMaxAttempts();

    /** @brief return a timeOut value for a certain type of frame*/
    simtime_t timeOut(Mac80211MultiChannelMessageKinds type, double br);

    /** @brief computes the duration of a transmission over the physical channel, given a certain bitrate */
    simtime_t packetDuration(double bits, double br);

    /** @brief Produce a readable name of the given state */
    const char *stateName(State state);

    /** @brief Sets the state, and produces a log message in between */
    void setState(State state);

    /** @brief Check whether the next packet should be send with RTS/CTS */
    bool rtsCts(Mac80211MultiChannelPkt* m) {
        return m->getBitLength() - MAC80211_HEADER_LENGTH > rtsCtsThreshold;
    }

    void resetCub() {
        for (int i = 0; i < NO_OF_CHANNELS; i++) {
            for (int j = 0; j < NO_OF_SLOTS; j++) {
                cub[i][j] = false;
            }
        }
        debugEV << "CUB reseted\n";
    }

    bool CubSlotsAvailable() {
        for (int i = 0; i < NO_OF_CHANNELS; i++) {
            for (int j = 0; j < NO_OF_SLOTS; j++) {
                if(cub[i][j] == false)
                    return true;
            }
        }
        debugEV << "CUB shows no slots available. Setting STATE to QUIET until next ATIM window\n";
        // no point answering any handshakes
        setState(QUIET);
        return false;
    }

    bool timeSlotFree(int timeslot) {
        for (int i = 0; i < NO_OF_CHANNELS; i++) {
            if(cub[i][timeslot])
                return false;
        }
        return true;
    }

    void resetSlotInstructions() {
        for (int i = 0; i < NO_OF_SLOTS; i++) {
            slotInstructions[i].changeChannel = false;
            slotInstructions[i].channel = -1;
            slotInstructions[i].isSender = false;
            slotInstructions[i].recepient = LAddress::L2NULL;
        }
        debugEV << "slot instructions reseted\n";
    }

    void transferBroadcastPktsFromBuffer() {
        int count = fromUpperLayerBroadcastBuffer.size();
        while (!fromUpperLayerBroadcastBuffer.empty()) {
            fromUpperLayerBroadcast.push_back(fromUpperLayerBroadcastBuffer.front());
            fromUpperLayerBroadcastBuffer.pop_front();
        }
        debugEV << "transferred " << count << " broadcast messages from broadcast buffer\n";
    }

    int countNumberOfPktsFor(LAddress::L2Type address) {
        int count = 0;
        MacPktList temp;

        while(!fromUpperLayer.empty()) {
            if (fromUpperLayer.front()->getDestAddr() == address)
                count++;

            temp.push_front(fromUpperLayer.front());
            fromUpperLayer.pop_front();
        }

        while(!temp.empty()) {
            fromUpperLayer.push_front(temp.front());
            temp.pop_front();
        }
        debugEV << "found " << count << " packets for " << address << "\n";

        if (count > NO_OF_SLOTS)
            return NO_OF_SLOTS;
        return count;
    }

    int countNumberOfBroadcastPkts() {
        if (fromUpperLayerBroadcast.size() > NO_OF_SLOTS)
            return NO_OF_SLOTS;
        return fromUpperLayerBroadcast.size();
    }

    void removePktsFromQueueFor(int x, LAddress::L2Type address) {
        int count = 0;
        MacPktList temp;

        while(!fromUpperLayer.empty()) {
            if (fromUpperLayer.front()->getDestAddr() == address) {
                count++;
                // we only want to transfer x packets from queue
                if (count > x) {
                    break;
                }
                fromUpperLayerHandled.push_back(fromUpperLayer.front());
            }
            else {
                temp.push_front(fromUpperLayer.front());
            }
            fromUpperLayer.pop_front();
        }

        if (count < x)
            error("logic error: unexpected number of packets");

        while(!temp.empty()) {
            fromUpperLayer.push_front(temp.front());
            temp.pop_front();
        }
        debugEV << x << " packets for " << address << " transferred to fromUpperLayerHandled\n";
    }

    void removeBroadcastPktsFromQueue(int x) {
        for (int i = 0; i < x; i++) {
            fromUpperLayerBroadcastHandled.push_back(fromUpperLayerBroadcast.front());
            fromUpperLayerBroadcast.pop_front();
        }
    }

    void bringPktForAddressToFront(LAddress::L2Type address) {
        MacPktList temp;
        bool contains = false;

        debugEV << "bringing packet for " << address << " to front of fromUpperLayerHandled\n";

        while (!fromUpperLayerHandled.empty()) {
            if (fromUpperLayerHandled.front()->getDestAddr() == address) {
                contains = true;
                break;
            }
            temp.push_front(fromUpperLayerHandled.front());
            fromUpperLayerHandled.pop_front();
        }

        if (contains) {
            temp.push_back(fromUpperLayerHandled.front());
            fromUpperLayerHandled.pop_front();
        }
        else
            error("logic error: no packet belonging that address found in fromUpperLayerHandled!");

        while (!temp.empty()) {
            fromUpperLayerHandled.push_front(temp.front());
            temp.pop_front();
        }
    }

    void recyclePkts() {

        // singlecast packets
        if (!fromUpperLayerHandled.empty()) {

            debugEV << fromUpperLayerHandled.size() << " singlecast packets leftover from Communication Window\n";
            std::vector<Mac80211MultiChannelPkt*> temp;

            // transfer contents to temp vector
            while(!fromUpperLayerHandled.empty()) {
                temp.push_back(fromUpperLayerHandled.front());
                fromUpperLayerHandled.pop_front();
            }

            // sort according to arrival time
            std::sort(temp.begin(), temp.end(), compByArrivalTime);

            if (fromUpperLayer.size() < queueLength && !temp.empty()) {
                // recycle the contents of fromUpperLayerHandled
                debugEV << "recycling packets back into the queue\n";
                longRetryCounter = 0;
                while(fromUpperLayer.size() < queueLength) {
                    if (temp.empty())
                        break;
                    fromUpperLayer.push_front(temp.back());
                    debugEV << temp.back()->getName() << " recycled!\n";
                    temp.pop_back();
                }
            }

            // discard the remainder
            debugEV << "removing leftovers\n";
            while (!temp.empty()) {
                Mac80211MultiChannelPkt *discard = temp.back();
                temp.pop_back();
                discard->setName("MAC ERROR");
                discard->setKind(PACKET_DROPPED);
                sendControlUp(discard);
            }
        }

        // broadcast packets
        if (!fromUpperLayerBroadcastHandled.empty()) {

            debugEV << fromUpperLayerBroadcastHandled.size() << " broadcast packets leftover from Communication Window\n";
            while (!fromUpperLayerBroadcastHandled.empty()) {
                fromUpperLayerBroadcast.push_front(fromUpperLayerBroadcastHandled.back());
                debugEV << fromUpperLayerBroadcastHandled.back()->getName() << " recycled!\n";
                fromUpperLayerBroadcastHandled.pop_back();
            }
        }
    }

    /** @brief suspend an ongoing contention, pick it up again when the channel becomes idle */
    void suspendContention();

    /** @brief figure out at which bitrate to send to this particular destination */
    double retrieveBitrate(const LAddress::L2Type& destAddress);

    /** @brief add a new entry to the neighbor list */
    void addNeighbor(Mac80211MultiChannelPkt *af);

    /** @brief find a neighbor based on his address */
    NeighborList::iterator findNeighbor(const LAddress::L2Type& address)  {
        NeighborList::iterator it;
        for(it = neighbors.begin(); it != neighbors.end(); ++it) {
            if(it->address == address) break;
        }
        return it;
    }

    /** @brief find the oldest neighbor -- usually in order to overwrite this entry */
    NeighborList::iterator findOldestNeighbor() {
        NeighborList::iterator it = neighbors.begin();
        NeighborList::iterator oldIt = neighbors.begin();
        simtime_t age = it->age;
        for(; it != neighbors.end(); ++it) {
            if(it->age < age) {
                age = it->age;
                oldIt = it;
            }
        }
        return oldIt;
    }


    /**
     * @brief Starts a channel sense request which sense the channel for the
     * passed duration or until the channel is busy.
     *
     * Used during contend state to check if the channel is free.
     */
    void senseChannelWhileIdle(simtime_t_cref duration);

    /**
     * @brief Creates the signal to be used for a packet to be sent.
     */
    Signal* createSignal(simtime_t_cref start, simtime_t_cref length, double power, double bitrate);

protected:

    // TIMERS:

    /** @brief Timer used for time-outs after the transmission of a RTS,
       a CTS, or a DATA packet*/
    cMessage* timeout;

    /** @brief Timer used for the defer time of a node. Also called NAV :
       networks allocation vector*/
    cMessage* nav;

    cMessage* atim_window;
    cMessage* communication_window;
    cMessage* communication_slot;

    /** @brief Used to sense if the channel is idle for contention periods*/
    ChannelSenseRequest* contention;

    /** @brief Timer used to indicate the end of a SIFS*/
    ChannelSenseRequest* endSifs;

    ChannelSenseRequest* sniffer;

    /** @brief Stores the the time a channel sensing started.
     * Used to calculate the quiet-time of the channel if the sensing was
     * aborted. */
    simtime_t chSenseStart;

    /** @brief Current state of the MAC*/
    State state;

    /** @brief Default bitrate
     *
     * The default bitrate must be set in the omnetpp.ini. It is used
     * whenever an auto bitrate is not appropriate, like broadcasts.
     */
    double defaultBitrate;

    /** @brief The power at which data is transmitted */
    double txPower;

    /** @brief Stores the center frequency the Mac uses. */
    double centerFreq;

    /** @brief Current bit rate at which data is transmitted */
    double bitrate;
    /** @brief Auto bit rate adaptation -- switch */
    bool autoBitrate;
    /** @brief Hold RSSI thresholds at which to change the bitrates */
    std::vector<double> snrThresholds;

    /** @brief Maximal number of packets in the queue; should be set in
       the omnetpp.ini*/
    unsigned queueLength;

    /** @brief Boolean used to know if the next packet is a broadcast packet.*/
    bool nextIsBroadcast;

    /** @brief Buffering of messages from upper layer*/
    MacPktList fromUpperLayer;

    MacPktList fromUpperLayerBroadcast;
    MacPktList fromUpperLayerBroadcastBuffer;
    MacPktList fromUpperLayerHandled;
    MacPktList fromUpperLayerBroadcastHandled;

    /** @brief Number of frame transmission attempt
     *
     *  Incremented when the SHORT_RETRY_LIMIT is hit, or when an ACK
     *  or CTS is missing.
     */
    unsigned longRetryCounter;

    /** @brief Number of frame transmission attempt*/
    unsigned shortRetryCounter;

    /** @brief remaining backoff time.
     * If the backoff timer is interrupted,
     * this variable holds the remaining backoff time. */
    simtime_t remainingBackoff;

    /** @brief current IFS value (DIFS or EIFS)
     * If an error has been detected, the next backoff requires EIFS,
     * once a valid frame has been received, resets to DIFS. */
    simtime_t currentIFS;

    /** @brief Number of bits in a packet before RTS/CTS is used */
    int rtsCtsThreshold;

    /** @brief Very small value used in timer scheduling in order to avoid
       multiple changements of state in the same simulation time.*/
    simtime_t delta;

    /** @brief Keep information for this many neighbors */
    unsigned neighborhoodCacheSize;
    /** @brief Consider information in cache outdate if it is older than this */
    simtime_t neighborhoodCacheMaxAge;

    /** @brief A list of this hosts neighbors.*/
    NeighborList neighbors;

    /** take care of switchover times */
    bool switching;

    /** sequence control -- to detect duplicates*/
    int fsc;

    int slotNo;

    bool cub[NO_OF_CHANNELS][NO_OF_SLOTS];

    struct instruction {
        bool changeChannel;
        int channel;
        bool isSender;
        LAddress::L2Type recepient;
    };
    instruction slotInstructions[NO_OF_SLOTS];
};

#endif
