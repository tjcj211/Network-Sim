/***************
 * DistanceVectorRouter
 * Author: Christian Duncan
 * Modified by: Ryan Hayes
 * Represents a router that uses a Distance Vector Routing algorithm.
 ***************/
import java.util.ArrayList;
import java.util.HashMap;

public class DistanceVectorRouter extends Router {
    // A generator for the given DistanceVectorRouter class
    public static class Generator extends Router.Generator {
        public Router createRouter(int id, NetworkInterface nic) {
            return new DistanceVectorRouter(id, nic);
        }
    }

    public static class Packet {
        // This is how we will store our Packet Header information
        int source;
        int dest;
        int hopCount;  // Maximum hops to get there
        Object[] payload;  // The payload!
        
        public Packet(int source, int dest, int hopCount, Object[] payload) {
            this.source = source;
            this.dest = dest;
            this.hopCount = hopCount;
            this.payload = payload;
        }
    }

    Debug debug;
    private HashMap<Integer,Integer> routingTable; // Stores all connections and their costs
    
    public DistanceVectorRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance();  // For debugging!
        initializeRoutingTable(nsap, nic); 
    }

    public void run() {
        while (true) {
            // See if there is anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();
            if (toSend != null) {
                // There is something to send out
                process = true;
                // Send DV to all neighbors if some conditions are met
                if () { // Changes found or router has been dropped
                    Object[] payload = {true, /*[DISTANCE VECTOR]*/};
                    route(-1, new Packet(nsap, toSend.destination, 1, payload));

                } else { // No changes found or no routers dropped
                    Object[] payload = {false, toSend.data};
                    route(-1, new Packet(nsap, toSend.destination, 5, payload));
                }
                

                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }

            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                // There is something to route through - or it might have arrived at destination
                process = true;
                if (toRoute.data instanceof Packet) {
                    Packet p = (Packet) toRoute.data;
                    //For each recieved if has different table, update own table
                    if ((Boolean) p.payload[0]){ //Is flagged as a DV
                    	if ((int) p.payload[1] != routingTable.get(p.source)) {
                    		routingTable.put(p.source, (int) p.payload[1]);
                    	}
                    }

                    //If it isn't at the end of the line, send it through

                    if (p.dest == nsap) {
                        // It made it!  Inform the "network" for statistics tracking purposes
                        debug.println(4, "(DistanceVectorRouter.run): Packet has arrived!  Reporting to the NIC - for accounting purposes!");
                        debug.println(6, "(DistanceVectorRouter.run): Payload: " + p.payload);
                        nic.trackArrivals(p.payload);
                    } else if (p.hopCount > 0) {
                        // Still more routing to do
                        p.hopCount--;
                        Object[] payload = {false, toRoute.data};
                        route(-1, new Packet(p.source, p.dest, p.hopCount, payload));
                        
                    } else {
                        debug.println(5, "Packet has too many hops.  Dropping packet from " + p.source + " to " + p.dest + " by router " + nsap);
                    }
                } else {
                    debug.println(0, "Error.  The packet being transmitted is not a recognized Flood Packet.  Not processing");
                }
                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }

            if (!process) {
                // Didn't do anything, so sleep a bit
                try { Thread.sleep(1); } catch (InterruptedException e) { }
            }
        }
    }

    //Set distance to self to 0 and all other connections to maxInt
    private void initializeRoutingTable(int nsap, NetworkInterface nic) {
        routingTable.put(nsap, 0);
        ArrayList<Integer> out = nic.getOutgoingLinks();
        for (int i = 0; i <= out.size(); i++) {
            routingTable.put(out.get(i), Integer.MAX_VALUE); //initializes every connection distance to max_value
        }
    }

    public HashMap<Integer, Integer> getRoutingTable(){
        return routingTable;
    }

    /** Route the given packet out.
        In our case, we go to all nodes except the originator
    **/
    private void route(int linkOriginator, Packet p) {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            if (outLinks.get(i) != linkOriginator) {
                // Not the originator of this packet - so send it along!
                nic.sendOnLink(i, p);
            }
        }
    }
}

