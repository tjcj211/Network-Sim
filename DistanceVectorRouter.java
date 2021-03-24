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
                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }

            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                // There is something to route through - or it might have arrived at destination
                process = true;
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
}

