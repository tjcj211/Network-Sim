/***************
 * LinkStateRouter
 * Author: Christian Duncan
 * Modified by: Charles Rescsanski, 
 * Represents a router that uses a Link State Routing algorithm.
 ***************/
import java.util.ArrayList;
import java.util.HashMap;

public class LinkStateRouter extends Router {
    // A generator for the given LinkStateRouter class
    public static class Generator extends Router.Generator {
        public Router createRouter(int id, NetworkInterface nic) {
            return new LinkStateRouter(id, nic);
        }
    }

    //represents an LSP
    //this is a special packet designed to communicate information about each router's neighborhood in the network
    //it does not contain the desired message or payload
    public static class LSP {
        //packet header info

        int source; //the "owner" of the neighborhood represented
        int dest; //where to send the packet
        int sequence; //the sequence "number"
        int age; //a "fail safe" in case the sequence gets corrupted or the router crashes

        //the first integer (keys) correspond to ID of given neighbor
        //the second integer (values) represent the edge weight
        HashMap<Integer, Integer> neighborEdgeWeights; //the costs associated with each neighbor of the owner

        public LSP(int source, int dest, int sequence, int age, HashMap<Integer, Integer> neighborEdgeWeights)
        {
            this.source = source;
            this.dest = dest; 
            this.sequence = sequence;
            this.age = age;
            this.neighborEdgeWeights = neighborEdgeWeights;
        }

    }

    //a type of packet designed for each router to learn its own neighborhood
    public static class learnReplyPacket
    {
        int sender; //the router that is replying to the neighbor's request for information
        int dest; //the router requesting the information
        int name; //the ID (or nsap) of the neighbor

        public learnReplyPacket(int sender, int dest, int name) 
        {
            this.sender = sender;
            this.dest = dest;
            this.name = name;
        }

    } 

    //a type of packet used to request the names (or unique IDs) of all neighboring routers
    public static class helloPacket{
        int sender; //the router that is requesting information about a neighbor
        int dest; //the router supplying the information

        public helloPacket(int sender, int dest)
        {
            this.sender = sender;
            this.dest = dest;
        }
    }

    //a special type of packet used to determine edge weights (to neighbors)
    public static class echoPacket{
        int sender;
        int dest;

        public echoPacket(int sender, int dest)
        {
            this.sender = sender;
            this.dest = dest;
        }

    }

    //these cannot be sent out until the router has a topology of the entire network
    public static class dijkstraPacket{
        int finalDest; //the intended recipient of message
        int source; //the originator of the message
        int[] shortestPath;  //the "shortest" sequence of nodes from the source to the destination
        int progress; //corresponds to current index position in the calculated path
        Object payload;  //the message to send

        public dijkstraPacket(int finalDest, int source, int[] shortestPath, int progress, Object payload)
        {
            this.finalDest = finalDest;
            this.source = source;
            this.shortestPath = shortestPath;
            this.progress = progress;
            this.payload = payload;
        }
    }

    Debug debug;
    
    public LinkStateRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance();  // For debugging!
    }

    public void run() {
        while (true) {
            // See if there is anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();
            if (toSend != null) {
                // There is something to send out
                process = true;
                debug.println(3, "(LinkStateRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }

            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                // There is something to route through - or it might have arrived at destination
                process = true;
                debug.println(3, "(LinkStateRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }

            if (!process) {
                // Didn't do anything, so sleep a bit
                try { Thread.sleep(1); } catch (InterruptedException e) { }
            }
        }
    }
}
