/***************
 * DistanceVectorRouter
 * Author: Christian Duncan
 * Modified by: Charles Rescanski and Lauren Atkinson
 * Represents a router that uses a Distance Vector Routing algorithm.
 * buildTable function: (screenshot)
 ***************/
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

public class DistanceVectorRouter extends Router {
    // A generator for the given DistanceVectorRouter class
    public static class Generator extends Router.Generator {
        public Router createRouter(int id, NetworkInterface nic) {
            return new DistanceVectorRouter(id, nic);
        }
    }
//duncans
    Debug debug;
    HashMap<Integer,DvPair> routingMap;
    ArrayList<HashMap<Integer,DvPair>>neighborMaps;
    LinkedList<Packet> transmitQueue;

    // create a private class to create a tuple to store hashmap values -- faster than a list/array
    public DistanceVectorRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance();  // For debugging
        routingMap= new HashMap<>();  //
        this.transmitQueue = new LinkedList<Packet>();
        neighborMaps= new ArrayList<>(nic.getOutgoingLinks().size());
        for(int i=0;i<nic.getOutgoingLinks().size();i++){
            neighborMaps.add(null);
        }
    }
//duncans
    public void run() {
    	//add a 0.5sec delay
        int delay=1000; 
        long currentTime= System.currentTimeMillis();
        long timeToRebuild = currentTime + delay;

        //routing map should initially have a distance of 0 to itself and a distance of infinity to all neighbors
        //initially, it will only have entries for its neighbors because it doesn't know about any other routers
        DvPair itself= new DvPair(0,-1);
        routingMap.put(nsap,itself);
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            routingMap.put(outLinks.get(i), new DvPair(Integer.MAX_VALUE, i));
        }
      

        while (true) {
            currentTime=System.currentTimeMillis();
            if(currentTime > timeToRebuild){
                timeToRebuild = currentTime + delay;
                buildTable(nsap);
            }

            //route out old packets that we couldn't transmit before
            int index = 0;
            while (index < this.transmitQueue.size())
            {
                int dest = transmitQueue.get(index).dest;
                if (routingMap.containsKey(dest) && routingMap.get(dest) != null)
                {
                    this.route(transmitQueue.get(index));
                    this.transmitQueue.remove(index);
                }
                else
                {
                    index ++;
                }
            }
            //duncans
            // check for anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();
            if (toSend != null) {
                /*
                if (toSend.data instanceof DistanceVectorRouter.PingPacket) {
                    DistanceVectorRouter.PingPacket p = (DistanceVectorRouter.PingPacket) toSend.data;
                    System.out.println(p.sendBack);
                }
                */
                Packet p = new Packet(nsap, toSend.destination, 10, toSend.data);
                if (this.routingMap.containsKey(toSend.destination) && this.routingMap.get(toSend.destination) != null)
                {
                    route(p);
                }
                else
                {
                    this.transmitQueue.add(p);
                }
                // something should be be sent out 
                process = true;
                debug.println(3, "(DistanceVectorRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
            }
            //duncans
            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                //route or destinatiokn 
                process = true;
                if(toRoute.data instanceof DvPacket){
                    int linkIndex= getLinkIndex(toRoute.originator);
                    debug.println(3, nsap +":just a hashmap to neighbor: " +toRoute.originator +" "+linkIndex);
                    DvPacket packet=(DvPacket) toRoute.data;
                    neighborMaps.set(linkIndex,packet.mapToSend);
                }
                
                else if (toRoute.data instanceof DistanceVectorRouter.PingPacket) {
                    DistanceVectorRouter.PingPacket p = (DistanceVectorRouter.PingPacket) toRoute.data;
                    if (p.sendBack == false) {
                        p.sendBack =true;
                        PingTest(toRoute.originator,p);
                    }
                   // need to figure out how to use linkindex -- maybe as dylan or harrison 
                    else {
                    	//figure out the distance using the table
                        long startTime = p.time;
                        long endTime = System.currentTimeMillis();
                        long dist = (endTime - startTime)/2;
                        int linkInd = nic.getOutgoingLinks().indexOf(toRoute.originator);
                        routingMap.put(toRoute.originator, new DvPair(dist, linkInd));
                        debug.println(3, "LinkInd(Park):"+linkInd);
                        
                    }
                }
                else if (toRoute.data instanceof Packet) {
                    Packet p = (Packet) toRoute.data;
                    if (p.dest == nsap)
                    {
                        // It made it!  Inform the "network" for statistics tracking purposes
                        debug.println(4, "(FloodRouter.run): Packet has arrived!  Reporting to the NIC - for accounting purposes!");
                        debug.println(6, "(FloodRouter.run): Payload: " + p.payload);
                        nic.trackArrivals(p.payload);
                    }
                    else if (p.hopCount > 0)
                    {   
                        p.hopCount --;
                        //we have more routing to do
                        if (this.routingMap.containsKey(p.dest) && routingMap.get(p.dest) != null)
                        {
                            this.route(p);
                        }
                        else
                        {
                            this.transmitQueue.add(p);
                        }
                    }
                    else
                    {
                        debug.println(5, "Packet has too many hops.  Dropping packet from " + p.source + " to " + p.dest + " by router " + nsap);
              
                    }

                }
                else
                {
                    debug.println(4, "Error.  The packet being tranmitted is not a recognized DistanceVector Packet.  Not processing");
                }
            }
           
            //duncans
            if (!process) {
                // mini time out 
                try { Thread.sleep(1); } catch (InterruptedException e) { }
            }
        }
    }
// use a method within a timer to add a delay 
//should use a hashmap or arraylist -- possible both to store the values     
    public void buildTable(int nsap){
    	//create an arraylist of  index and NSAP going out 
        ArrayList<Integer>linkIndex=nic.getOutgoingLinks();
        //use ping to get the distance from  router
        for(int i: nic.getOutgoingLinks()){
            PingTest(i,new PingPacket(System.currentTimeMillis(),false));
        }

        //update our map using the neighbor maps to store the minimum known distance to each destination
        for (int i = 0; i < this.neighborMaps.size(); i++)
        {
            int neighborId = linkIndex.get(i);
            int link = i;
            if (this.neighborMaps.get(link) != null)
            {
                this.neighborMaps.get(link).forEach((destId, valuePair) ->
                {
                    if (!this.routingMap.containsKey(destId) || this.routingMap.get(destId) == null || this.routingMap.get(neighborId).distance + valuePair.distance < this.routingMap.get(destId).distance)
                    {

                        this.routingMap.put(destId, new DvPair(this.routingMap.get(neighborId).distance + valuePair.distance, link));
                    }
                });
            }
            
        }

        if(nsap==14){ 
            debug.println(0, "Start Routing Table for Router " + nsap + "\n");
            this.routingMap.forEach((key, value) ->
            {
                debug.println(0, "Dest: " + key + ", Best Link: " + value.linkIndex + ", Est Distance: " + value.distance);
            });
        }
        
        //hashmap to router
        ////special packet w contains map the needs to be sent to the neighboring router 
        //if
        boolean sendMap = true;
        for(int i=0;i<linkIndex.size();i++){
            if (this.routingMap.get(linkIndex.get(i)).distance == Integer.MAX_VALUE)
            {
                sendMap = false;
            }
        }
        if (sendMap)
        {
            for(int i=0;i<linkIndex.size();i++){
                DvPacket mapPacket = new DvPacket((HashMap<Integer, DvPair>) this.routingMap.clone());
                nic.sendOnLink(i, mapPacket);
            }
        }

    }
    private int getLinkIndex(int nsap){
        ArrayList<Integer> outLinks= nic.getOutgoingLinks();
        return outLinks.indexOf(nsap);
    }
   
    //Custom packet class that holds hashmap that will be sent to a router's direct neighbor(s)
    public static class DvPacket {
        HashMap<Integer,DvPair> mapToSend;
        //the hashmap is the only thing that the neighbors need receive from the router
        public DvPacket(HashMap<Integer,DvPair> mapToSend) {
            this.mapToSend=mapToSend;
        }
    }
    
    //class to make our little tuple for the hashmap
    public class DvPair {
        double distance;
        int linkIndex;
        public DvPair(double distance, int linkIndex) {
                 this.distance = distance;
                 this.linkIndex = linkIndex;
        }
    }

    public static class Packet {
        // This is how we will store our Packet Header information
        int source;
        int dest;
        int hopCount;
        Object payload;  // The payload!

        public Packet(int source, int dest, int hopCount, Object payload) {
            this.source = source;
            this.dest = dest;
            this.hopCount = hopCount;
            this.payload = payload;
        }
    }

    public static class PingPacket {
        long time;  //time
        boolean sendBack;
        
        public PingPacket(long time, boolean sendBack) {
            this.time = time;
            this.sendBack = sendBack;
        }
    }
    
//send to the next link based on the shortest path
    private void route(Packet p) {
        int link = this.routingMap.get(p.dest).linkIndex;
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        nic.sendOnLink(link, p);
    }
//not mine
    public void PingTest(int dest, PingPacket PP) {
         ArrayList<Integer> outGo = nic.getOutgoingLinks();
         int size = outGo.size();
         for (int i = 0; i < size; i++) {
            if (outGo.get(i) == dest) {
                nic.sendOnLink(i, PP);
            }
        }
    }

}
