
/***************
 * DistanceVectorRouter
 * Author: Christian Duncan
 * Modified by: Tim Carta and Ryan Hayes
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
        int hopCount; // Maximum hops to get there
        Object payload; // The payload!

        public Packet(int source, int dest, int hopCount, Object payload) {
            this.source = source;
            this.dest = dest;
            this.hopCount = hopCount;
            this.payload = payload;
        }
    }

    public static class PingPacket {
        int source;
        int dest;
        int hopCount;
        long ping;

        public PingPacket(int source, int dest, int hopCount, long ping) {
            this.source = source;
            this.dest = dest;
            this.hopCount = hopCount;
            this.ping = ping;
        }
    }

    public static class RouterPacket {
        int source;
        int dest;
        int hopCount;
        HashMap<Integer, Integer> payload;

        public RouterPacket(int source, int dest, int hopCount, HashMap<Integer, Integer> payload) {
            this.source = source;
            this.dest = dest;
            this.hopCount = hopCount;
            this.payload = payload;
        }
    }

    Debug debug;
    private HashMap<Integer, Integer> routingTable; // Stores all connections and their costs

    public DistanceVectorRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance(); // For debugging!
        initializeRoutingTable(nsap, nic);
    }

    boolean changedTable = false;

    public void run() {
        while (true) {
            // See if there is anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();
            if (toSend != null) {
                // There is something to send out
                process = true;
                // Send DV to all neighbors if some conditions are met
                if (changedTable) { // Changes found or router has been dropped
                    HashMap<Integer, Integer> payload = this.routingTable;
                    routeRouter(-1, new RouterPacket(nsap, toSend.destination, 1, payload));
                    changedTable = false;
                } else { // No changes found or no routers dropped
                    Object payload = toSend.data;
                    route(-1, new Packet(nsap, toSend.destination, 5, payload));
                }

                // Send a PingPacket (when?)
                PingPacket p = new PingPacket(nsap, toSend.destination, 2, System.currentTimeMillis());
                routePing(nsap, p);

                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data
                        + " to the destination: " + toSend.destination);
            }

            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                // There is something to route through - or it might have arrived at destination
                process = true;
                if (toRoute.data instanceof Packet) {
                    Packet p = (Packet) toRoute.data;
                    if (p.dest == nsap) {
                        debug.println(4,
                                "(DistanceVectorRouter.run): Packet has arrived!  Reporting to the NIC - for accounting purposes!");
                        debug.println(6, "(DistanceVectorRouter.run): Payload: " + p.payload);
                        nic.trackArrivals(p.payload);
                    } else if (p.hopCount > 0) {
                        p.hopCount--;
                        route(toRoute.originator, p);
                    } else {
                        debug.println(5, "Packet has too many hops.  Dropping packet from " + p.source + " to " + p.dest
                                + " by router " + nsap);
                    }
                }

                if (toRoute.data instanceof RouterPacket) {
                    RouterPacket p = (RouterPacket) toRoute.data;
                    // For each recieved if has different table, update own table
                    // Check if table is different
                    HashMap<Integer, Integer> payloadRoutingTable = p.payload;
                    for (Integer key : payloadRoutingTable.keySet()) {
                        // check to see if values are the same
                        if (routingTable.get(key) != payloadRoutingTable.get(key)) {

                            // if not, change our value to time to get to neighbor + neighbor values
                            routingTable.put(key, payloadRoutingTable.get(key) + routingTable.get(p.source));
                            changedTable = true;

                        }
                    }
                }

                if (toRoute.data instanceof PingPacket) {
                    PingPacket p = (PingPacket) toRoute.data;
                    p.hopCount--;
                    if (p.hopCount == 0) { // Packet has been returned
                        int ping = (int) ((System.currentTimeMillis() - p.ping) / 2); // The ping between two
                        // routers
                        routingTable.put(p.source, ping);
                    } else if (p.hopCount == 1) { // Send back to the original source
                        p.dest = p.source; // Change the destination to the source to be returned
                        routePing(p.source, p);
                    }
                }

            } else {
                debug.println(0, "Error.  The packet being transmitted is not a recognized Packet.  Not processing");
            }
            debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data
                    + " to the destination: " + toSend.destination);

            if (!process) {
                // Didn't do anything, so sleep a bit
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                }
            }
        }
    }

    // Set distance to self to 0 and all other connections to maxInt
    private void initializeRoutingTable(int nsap, NetworkInterface nic) {
        routingTable = new HashMap<Integer, Integer>();
        routingTable.put(nsap, 0);
        ArrayList<Integer> out = nic.getOutgoingLinks();
        for (int i = 0; i < out.size(); i++) {
            routingTable.put(out.get(i), Integer.MAX_VALUE); // initializes every connection distance to max_value
        }
        changedTable = true;
    }

    public HashMap<Integer, Integer> getRoutingTable() {
        return routingTable;
    }

    /**
     * Route the given packet out. In our case, we go to all nodes except the
     * originator
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

    /**
     * Route the PingPacket Send back to the original source or pass to the
     * destination
     **/
    private void routePing(int linkOriginator, PingPacket p) {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            if (p.hopCount == 2) { // Send to destination to check ping
                if (outLinks.get(i) == p.dest) {
                    nic.sendOnLink(p.dest, p);
                }
            } else { // Send back to source
                if (outLinks.get(i) == linkOriginator) {
                    nic.sendOnLink(linkOriginator, p);
                }
            }
        }
    }

    private void routeRouter(int linkOriginator, RouterPacket p) {
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
