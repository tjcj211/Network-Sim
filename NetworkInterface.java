/*************
 * NetworkInterface
 * Author: Christian Duncan
 *
 * Represents an interface for a router to interact with the network.
 * This simulates a "network interface card" simplistically.
 *************/
import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

public class NetworkInterface {
    private Network net;  // A reference to the whole network - so we can see where this interface belongs
    private int nsap;   // The ID for this NIC
    private ArrayList<Integer> outgoingLinks;   // A list of outgoing links
    private ArrayList<Integer> incomingLinks;   // A list of incoming links

    private int capacity;                       // The limit to number of packets that can be waiting for processing on Queue
    private Queue<TransmitPair> transmissionQueue;    // A list of data that needs to be transmitted starting from this NIC
    private Queue<ReceivePair> receivedQueue;        // A list of data that has been received on this NIC and needs to be processed (received or routed)

    public class TransmitPair {
        int destination;  // Destination of the data
        Object data;      // The data to transmit
        public TransmitPair(int destination, Object data) { this.destination = destination; this.data = data; }
    };

    public class ReceivePair {
        int originator;   // The link originator that this data came from (direct link not original source)
        Object data;      // The data to route
        public ReceivePair(int originator, Object data) { this.originator = originator; this.data = data; }
    }
    
    public NetworkInterface(Network net, int nsap, ArrayList<Integer> outgoingLinks, ArrayList<Integer> incomingLinks, int capacity) {
        this.net = net;
        this.nsap = nsap;
        this.outgoingLinks = outgoingLinks;
        this.incomingLinks = incomingLinks;
        this.capacity = capacity;
        this.transmissionQueue = new ConcurrentLinkedQueue<TransmitPair>();
        this.receivedQueue = new ConcurrentLinkedQueue<ReceivePair>();
    }

    /** Return the NSAP ID for this NIC **/
    public int getNSAP() { return nsap; }

    /** 
     * Get the list of outgoing and incoming links.
     * The list is an array of NSAPs (IDs)
     **/
    public ArrayList<Integer> getOutgoingLinks() { return outgoingLinks; }
    public ArrayList<Integer> getIncomingLinks() { return incomingLinks; }

    /**
     * Send a data "packet" on the given link
     * @param linkIndex The index of the link in the outgoing list
     * @param packet The data to transmit
     * @returns true if successful, false if the machine is currently down or the index was invalid.
     **/
    public boolean sendOnLink(int linkIndex, Object packet) {
        // Use the "network" to transmit between machines
        return net.sendOnLink(this.nsap, linkIndex, packet);
    }

    /**
     * Transmit a payload on this network starting at THIS NIC's Router
     * The router must grab off the queue and process
     **/
    public synchronized void transmit(int dest, Object payload) {
        if (payload == null) {
            // No transmission of NULL objects -- something must be transmitted.
            Debug.getInstance().println(0, "Transmission must include at least ONE byte of information.  Sent to Node " + nsap);
            return;
        }
        if (transmissionQueue.size() < capacity) {
            // There is room to add it
            transmissionQueue.add(new TransmitPair(dest, payload));
        } else {
            Debug.getInstance().println(4, "Dropped payload by Node " + nsap);
        }
    }

    /**
     * Store a received payload from another NIC.
     * The router must grab off the queue and process
     **/
    public synchronized void receive(int originator, Object payload) {
        if (payload == null) {
            // No transmission of NULL objects -- something must be transmitted.
            Debug.getInstance().println(0, "Received message with no data.  Must include at least ONE byte of information.  Sent to Node " + nsap);
            return;
        }
        if (receivedQueue.size() < capacity) {
            // There is room to add it
            receivedQueue.add(new ReceivePair(originator, payload));
        } else {
            Debug.getInstance().println(4, "Node " + nsap + " dropped packet sent on link from " + originator);
            Debug.getInstance().println(6, "   Payload: " + payload.toString());
        }
    }

    /**
     * Get data from transmission Queue
     * @returns Next element to transmit or null if nothing
     **/
    public synchronized TransmitPair getTransmit() {
        return transmissionQueue.poll();
    }

    /**
     * Get data from received Queue
     * @returns Next element to process or null if nothing
     **/
    public synchronized ReceivePair getReceived() {
        return receivedQueue.poll();
    }

    /**
     * Statistics tracker - used to help track packets that were sent/received
     */
    public void trackArrivals(Object payload) {
        net.receive(nsap, payload);
    }
}
