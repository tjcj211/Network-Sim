
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
        int previous;
        int hopCount; // Maximum hops to get there
        Object payload; // The payload!

        public Packet(int source, int dest, int previous, int hopCount, Object payload) {
            this.source = source;
            this.dest = dest;
            this.previous = previous;
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
    // Stores all connections and their costs
    private HashMap<Integer, Integer> routingTable;
    private HashMap<Integer, HashMap<Integer, Integer>> masterRoutingTable;
    boolean changedTable = false;

    public DistanceVectorRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance(); // For debugging!

    }

    public void run() {
        initializeRoutingTable(nsap, nic);
        initialPing();
        while (true) {
            // See if there is anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();
            if (toSend != null) {
                // There is something to send out
                process = true;
                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data
                        + " to the destination: " + toSend.destination);
                // Send DV to all neighbors if some conditions are met
                if (changedTable) { // Changes found or router has been dropped
                    HashMap<Integer, Integer> payload = this.routingTable;
                    routeRouter(-1, new RouterPacket(nsap, toSend.destination, 1, payload));
                    changedTable = false;
                    // Send a PingPacket
                    PingPacket p = new PingPacket(nsap, toSend.destination, 2, System.currentTimeMillis());
                    routePing(-1, p);
                } else { // No changes found or no routers dropped
                    Object payload = toSend.data;
                    route(-1, new Packet(nsap, toSend.destination, nsap, 7, payload));
                }
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

                } else if (toRoute.data instanceof RouterPacket) {
                    RouterPacket p = (RouterPacket) toRoute.data;
                    // For each recieved if has different table, update own table
                    // Check if table is different
                    HashMap<Integer, Integer> payloadRoutingTable = p.payload;
                    for (Integer key : payloadRoutingTable.keySet()) {
                        // check to see if values are the same
                        if (routingTable.get(key) != payloadRoutingTable.get(key)) {
                            // if not, change our value to time to get to neighbor + neighbor values
                            routingTable.put(key, payloadRoutingTable.get(key));
                            changedTable = true;
                            debug.println(2, "(RouterPacket) Updating Router Table for Router: " + nsap);
                        }
                    }
                } else if (toRoute.data instanceof PingPacket) {
                    PingPacket p = (PingPacket) toRoute.data;
                    p.hopCount--;
                    if (p.hopCount == 0) { // Packet has been returned
                        // The ping between two routers
                        int ping = (int) ((System.currentTimeMillis() - p.ping) / 2);
                        if (routingTable.get(p.source) == null || routingTable.get(p.source) > ping) {
                            masterRoutingTable.get(p.source).put(p.dest, ping);
                            changedTable = true;
                        }
                        debug.println(2, "(PingPacket) received ping packet for: " + nsap);
                    } else if (p.hopCount == 1) { // Send back to the original source
                        p.dest = p.source; // Change the destination to the source to be returned
                        routePing(p.source, p);
                    }
                } else {
                    debug.println(0,
                            "Error.  The packet being transmitted is not a recognized Packet.  Not processing");
                }
            }

            if (!process) {
                // Didn't do anything, so sleep a bit
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                }
            }
        }
    }

    // Returns the key to the shortest path
    private int[] calculateFastestNeighbor(int destination) {
        HashMap<Integer, Integer> destinationTable = masterRoutingTable.get(destination);
        int key = 0;// (int) destinationTable.keySet().toArray()[0]; // Pick the first key as the
                    // fastest
        int minSpeed = Integer.MAX_VALUE;
        for (int destinations : masterRoutingTable.keySet()) {
            for (int fastestPath : destinationTable.keySet()) {
                if (destinationTable.get(fastestPath) + masterRoutingTable.get(nsap).get(destinations) < minSpeed) {
                    key = destinationTable.get(fastestPath) + masterRoutingTable.get(nsap).get(destinations);
                    minSpeed = destinationTable.get(key);
                }
            }
        }
        int[] arr = { key, minSpeed };
        // x to y = min {neighbors distance to y + my distance to that neighbor}
        // x to y = min {distance from x to neighbor + neighbor to y}
        return arr;
    }

    // Set distance to self to 0 and all other connections to maxInt
    private void initializeRoutingTable(int nsap, NetworkInterface nic) {
        masterRoutingTable = new HashMap<Integer, HashMap<Integer, Integer>>();
        routingTable = new HashMap<Integer, Integer>();

        routingTable.put(nsap, 0);
        masterRoutingTable.put(nsap, routingTable); // Initialize 2d Table

        ArrayList<Integer> out = nic.getOutgoingLinks();
        for (int i = 0; i < out.size(); i++) {
            debug.println(2, "(initializeRoutingTable) Adding route " + out.get(i) + " to routing Table " + nsap);
            // initializes every connection distance to max_value
            // Initialize table column for neighbors
            if (masterRoutingTable.get(out.get(i)) == null) {
                routingTable.put(out.get(i), Integer.MAX_VALUE);
                masterRoutingTable.put(out.get(i), routingTable);
                // Check speed
                // mrt.get(nsap).put(D, mrt.get(nsap).get(C) + mrt.get(C).get(D))
            }
        }
        changedTable = true;
    }

    // Send an initial ping to each router's neighbors
    // Unsure if necessary
    private void initialPing() {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            PingPacket p = new PingPacket(nsap, i, 2, System.currentTimeMillis());
            routePing(-1, p);
        }
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
        // int min = Integer.MAX_VALUE;
        // int minKey = (int) routingTable.keySet().toArray()[0];
        int[] keySpeed = calculateFastestNeighbor(p.dest);
        int min = keySpeed[1];
        int minKey = keySpeed[0];

        for (int i = 0; i < outLinks.size(); i++) {
            if (outLinks.get(i) == minKey && outLinks.get(i) != linkOriginator) {
                p.previous = minKey;
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
                nic.sendOnLink(p.dest, p);
            } else { // Send back to source
                nic.sendOnLink(linkOriginator, p);
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
