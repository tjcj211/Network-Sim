/***************
 * LinkStateRouter
 * Author: Christian Duncan
 * Modified by: Charles Rescsanski, 
 * Represents a router that uses a Link State Routing algorithm.
 ***************/
import java.util.*;


public class LinkStateRouter extends Router {
    // A generator for the given LinkStateRouter class
    public static class Generator extends Router.Generator {
        public Router createRouter(int id, NetworkInterface nic) {
            return new LinkStateRouter(id, nic);
        }
    }

    //use to construct an adjacency matrix to represent a graph of the network
    public class NetGraph {
        ArrayList<ArrayList<DijkstraNode>> adjacencyList;
        HashMap<Integer, Integer> indexes; //use to associate an NSAP with a vertex index
        ArrayList<Integer> backToID; //use to link indexes back to ID
        int index = 0;

        public NetGraph()
        {
            this.indexes = new HashMap<Integer, Integer>();
            this.backToID = new ArrayList<Integer>();

            this.adjacencyList = new ArrayList<ArrayList<DijkstraNode>>();

            this.addIndex(nsap); //let's first add ourself

        }

        public int getNumVertices() 
        {
            return this.adjacencyList.size();
        }

        public void addOrReplaceEdge(int source, int destination, int weight)
        {
            if (!this.indexes.containsKey(source))
            {
                this.addIndex(source);
            }
            if (!this.indexes.containsKey(destination))
            {
                this.addIndex(destination);
            }

            //we cannot allow duplicate mappings, so we must first remove any preexisting ones
            this.adjacencyList.get(this.indexes.get(source)).removeIf((edge) -> edge.vertex == this.indexes.get(destination));

            DijkstraNode edge = new DijkstraNode(this.indexes.get(destination), weight);

            this.adjacencyList.get(this.indexes.get(source)).add(edge);
        }

        private void addIndex(int routerID)
        {
            this.indexes.put(routerID, index);
            if (index >= this.backToID.size())
            {
                this.backToID.add(routerID);
            }
            else
            {
                this.backToID.set(index, routerID);
            }
                      
            if (index >= this.adjacencyList.size())
            {
                this.adjacencyList.add(new ArrayList<DijkstraNode>());
            }
            else
            {
                this.adjacencyList.set(index, new ArrayList<DijkstraNode>());
            }
            index++;
        }
    }

    //used to store current distance of each vertex in djkstra algorithm
    public static class DijkstraNode implements Comparator<DijkstraNode>
    {
        int vertex;
        int cost;

        public DijkstraNode(int vertex, int cost)
        {
            this.vertex = vertex;
            this.cost = cost;
        }
        public DijkstraNode() { }

        @Override 
        public int compare(DijkstraNode node1, DijkstraNode node2)
        {
            if (node1.cost < node2.cost)
            {
                return -1;
            }
            else if (node1.cost > node2.cost)
            {
                return 1;
            }
            else
            {
                return 0;
            }   
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
        HashMap<Integer, Long> neighborEdgeWeights; //the costs associated with each neighbor of the owner

        public LSP(int source, int sequence, int age, HashMap<Integer, Long> neighborEdgeWeights)
        {
            this.source = source;
            this.sequence = sequence;
            this.age = age;
            this.neighborEdgeWeights = neighborEdgeWeights;
        }

    }

    //a special type of packet used to determine edge weights (to neighbors)
    public static class EchoPacket{
        int requester; //the router trying to calculate an edge weight
        int neighbor;  //the link to which to calculate the cost

        public EchoPacket(int requester, int neighbor)
        {
            this.requester = requester;
            this.neighbor = neighbor;
        }

    }

    //these cannot be sent out until the router has a topology of the entire network
    public static class DijkstraPacket{
        int finalDest; //the intended recipient of message
        int source; //the originator of the message
        Stack<Integer> shortestPath;  //the "shortest" sequence of nodes from the source to the destination (excluding source)
        Object payload;  //the message to send
        int hopCount; //time to live

        public DijkstraPacket(int finalDest, int source, Stack<Integer> shortestPath, Object payload, int hopCount)
        {
            this.finalDest = finalDest;
            this.source = source;
            this.shortestPath = shortestPath;
            this.payload = payload;
            this.hopCount = hopCount;
        }
    }

    //Represents a transmission to send later (once network graph is determined)
    public static class TransmitRequest{
        Object payload;
        int dest;
        boolean processed;

        public TransmitRequest(Object payload, int dest)
        {
            this.payload = payload;
            this.dest = dest;
            this.processed = false;
        }

    }

    //use to calculate edge weights for each neighbor
    public static class WeightCalc{
        Long sendTime;
        Long receiveTime;
        
        public WeightCalc(Long sendTime) { this.sendTime = sendTime;}

        public void setSendTime(long time) {this.sendTime = time;}
        public void setReceiveTime(long time) {this.receiveTime = time;}

        public Long getWeight()
        {
            if (this.sendTime != null && this.receiveTime != null)
            {
                return (receiveTime - sendTime) / 2;
            }
            return null;  
        }
    }
    LSP myLSP;
    HashMap<Integer, LSP> linkSet; //stores most current LSPs of each router on the network
    int sequence; //32 bit sequence number for distributing LSP; increment for every new LSP sent out
    int lspTimer; //use to dictate how frequently to send out new LSPs
    int pingTimer;  //how often to send out echo packets
    int pathTableTimer;  //dicates how frequently to rebuild network graph and run dijkstra
    int decrementAgeTimer; //dicates how frequently to decrease age value of LSP packets
    Debug debug;
    NetGraph fullNetworkGraph; //represents the full graph of the entire network
    Queue<TransmitRequest> toSendList; //a queue of payloads to send out with a destination 
    HashMap<Integer, TreeSet<Integer>> lspHistory; //list of all LSP sequence numbers from each neighbor
                                                   //should periodically remove the ones that have expired
    HashMap<Integer, WeightCalc> neighborEdgeCalcs; // used to calculate costs using send and receive time
    HashMap<Integer, Long> neighborEdgeWeights; //the costs associated with each neighbor of the owner
    HashMap<Integer, Stack<Integer>> routingTable; //stores the currently calculated shortest paths from source to all other nodes
    

    public LinkStateRouter(int nsap, NetworkInterface nic) {
        super(nsap, nic);
        debug = Debug.getInstance();  // For debugging!
        this.toSendList = new LinkedList<TransmitRequest>();
        this.routingTable = new HashMap<Integer, Stack<Integer>>();
        this.lspHistory = new HashMap<Integer, TreeSet<Integer>>();
        this.neighborEdgeCalcs = new HashMap<Integer, WeightCalc>();
        this.neighborEdgeWeights = new HashMap<Integer, Long>();
        this.linkSet = new HashMap<Integer, LSP>();
        this.fullNetworkGraph = new NetGraph();
        this.sequence = 0; 
        this.lspTimer = 0;
        this.decrementAgeTimer = 10000;
        this.pingTimer = 500000;
        this.pathTableTimer = 1000;
    }

    public void run() {
        //before we enter the loop, let's first send out echo (PING) requests to the neighbors
        this.echoOut();


        while (true) {
            // See if there is anything to process
            boolean process = false;
            NetworkInterface.TransmitPair toSend = nic.getTransmit();

            if (this.decrementAgeTimer <= 0)
            {
                this.decrementLSPAge();
                this.decrementAgeTimer = 10000;
            }
            else
            {
                this.decrementAgeTimer --;
            }

            //let's periodically try to recalculate weights
            if (this.pingTimer <= 0)
            {
                this.echoOut();
                this.pingTimer = 500000;
            }
            else
            {
                this.pingTimer --;
            }

            //First, let's build an LSP (if possible)
            if (this.checkAllWeights_UpdateMaster() && this.lspTimer <= 0)
            {
                //Yes, all neighbor weights have been calculated, so we can build our LSP packet
                this.myLSP = new LSP(nsap, this.sequence, 60, this.neighborEdgeWeights);
                this.sequence++; //so that next LSP sent out can be distinguished
                //LSP will go to everyone except the direct link from which we just received it.
                this.lspRoute(-1, this.myLSP); //we'll use the flooding algorithm to distribute LSP

                this.lspTimer = 500000; //reset the timer

            }
            else if (this.lspTimer > 0)
            {
                this.lspTimer --;
            }
            
            
            if (this.pathTableTimer <= 0 && this.myLSP != null)
            {
                //this.fullNetworkGraph = new NetGraph();

                //first, we'll add our own edges from our LSP
                for(Map.Entry<Integer, Long> myNeigh : this.myLSP.neighborEdgeWeights.entrySet())
                {
                    this.fullNetworkGraph.addOrReplaceEdge(nsap, myNeigh.getKey(), myNeigh.getValue().intValue());
                }
    
                for (Map.Entry<Integer, LSP> owner : this.linkSet.entrySet())
                {
                    for (Map.Entry<Integer, Long> neighbor : owner.getValue().neighborEdgeWeights.entrySet())
                    {
                        this.fullNetworkGraph.addOrReplaceEdge(owner.getKey(), neighbor.getKey(), neighbor.getValue().intValue());
                    }
                }

                //runs dijkstra's algorithm on graph and builds a routing table
                this.routingTable = this.dijkstraAlg();

                //reset timer
                this.pathTableTimer = 1000;
            }
            else if (this.pathTableTimer > 0)
            {
                this.pathTableTimer --;
            }

            
           
            //Next, let's route existing packets in the queue (if possible)
            //those that can be routed are removed from the queue
            
            for (TransmitRequest t : this.toSendList)
            {
                if (this.routingTable.containsKey(t.dest) && this.routingTable.get(t.dest).size() > 0)
                {
                    process = true;
                    Stack<Integer> path = this.routingTable.get(t.dest);
                    int nextDest = path.pop();
                    dijkstraRoute(new DijkstraPacket(t.dest, nsap, this.routingTable.get(t.dest), t.payload, 20), nextDest);
                    t.processed = true;
                }
                
            }
            this.toSendList.removeIf((request) -> request.processed);
                
            
            if (toSend != null) {
                // There is something new to send out
                process = true;
                debug.println(0, "(LinkStateRouter.run): I am being asked to transmit: " + toSend.data + " to the destination: " + toSend.destination);
                if (this.routingTable.containsKey(toSend.destination) && this.routingTable.get(toSend.destination).size() > 0)
                {
                    Stack<Integer> path = this.routingTable.get(toSend.destination);
                    int nextDest = path.pop();
                    dijkstraRoute(new DijkstraPacket(toSend.destination, nsap, this.routingTable.get(toSend.destination), toSend.data, 2000), nextDest);
                }
                else
                {
                    //payloads cannot be sent until the router has built a graph of the entire network
                    //we'll store them temporarily in a queue to transmit later
                    this.toSendList.add(new TransmitRequest(toSend.data, toSend.destination));
                }
            }

            NetworkInterface.ReceivePair toRoute = nic.getReceived();
            if (toRoute != null) {
                // There is something to route through - or it might have arrived at destination
                process = true;
                
                // We need to first determine which type of packet has been received.  
                if (toRoute.data instanceof DijkstraPacket)
                {
                    //in this case, we have an actual packet (not used for learning the network)
                    DijkstraPacket p = (DijkstraPacket) toRoute.data;
                    if (p.finalDest == nsap) {
                        // It made it!  Inform the "network" for statistics tracking purposes
                        debug.println(0, "(LinkStateRouter.run): Packet has arrived!  Reporting to the NIC - for accounting purposes!");
                        debug.println(0, "(LinkStateRouter.run): Payload: " + p.payload);
                        nic.trackArrivals(p.payload);
                    }
                    else  
                    {
                        if (p.hopCount > 0)
                        {
                             //still more routing to do
                            p.hopCount --; //decrement hop count
                            Stack<Integer> curPath = p.shortestPath;
                            try 
                            {
                                int nextDest = curPath.pop();
                                debug.println(0, "Packet originating from " + p.source + "is being sent to " + nextDest);
                                dijkstraRoute(p, nextDest);  
                            }
                            catch (EmptyStackException e)
                            {
                                debug.println(0, "Dropping packet due to invalid path");
                            }    
                        }
                        else 
                        {
                            //packet is too old, we must discard it
                            debug.println(0, "Packet has too many hops; Router " + nsap + " dropping packet from " + p.source + "intended for " + p.finalDest);
                        }
                        
                                        
                    }
                   
                }
                else if (toRoute.data instanceof LSP)
                {
                    LSP p = (LSP) toRoute.data;
                    //This may be an old or duplicate packet, so we shouldn't flood it immediately.
                    //2.  Discard if:
                        //a) it's a duplicate (identical sequence number - originator combination)
                        //b) if its sequence number is lower than the highest one seen from the originating router.
                        //c) If age == 0
                    
                    if(p.age > 0)
                    {
                        if (this.lspHistory.containsKey(p.source))
                        {
                            //adds sequence number; checks if it's a duplicate
                            if (this.lspHistory.get(p.source).add(p.sequence))
                            {
                                //checks whether sequence number is NOT lower than the highest one seen
                                if (!(p.sequence < this.lspHistory.get(p.source).last()))
                                {
                                    p.age --;
                                    debug.println(0, "Router " + nsap + " has received an updated LSP from router " + p.source);
                                    //replace LSP info to our master table
                                    this.linkSet.put(p.source, p);
                                    lspRoute(toRoute.originator, p);
                                }
                                else 
                                {
                                    debug.println(1, "LSP is outdated.  Dropping LSP from " + p.source + " to " + p.dest + " by router " + nsap);
                                }
                            }
                            else 
                            {
                                debug.println(1, "LSP is a duplicate.  Dropping LSP from " + p.source + " to " + p.dest + " by router " + nsap);
                            }
                        }
                        else 
                        {
                            //we've never received an LSP packet of this router's neighborhood
                            TreeSet<Integer> list = new TreeSet<Integer>();
                            list.add(p.sequence);
                            p.age --;
                            this.lspHistory.put(p.source, list);
                            debug.println(0, "Router " + nsap + " has received its first LSP from router " + p.source);
                             //add LSP info to our master table
                             this.linkSet.put(p.source, p);
                            lspRoute(toRoute.originator, p);
                        }
                    }
                    else 
                    {
                        debug.println(1, "LSP is expired.  Dropping LSP from " + p.source + " to " + p.dest + " by router " + nsap);
                    }
                }
                else if (toRoute.data instanceof EchoPacket)
                {
                    EchoPacket p = (EchoPacket) toRoute.data;
                    if (p.requester == nsap)
                    {
                        //the echo request has been returned!!!
                        WeightCalc weCalc = this.neighborEdgeCalcs.get(p.neighbor);
                        weCalc.setReceiveTime(System.currentTimeMillis());

                        debug.println(1, "Edge weight from (" + nsap + " --> " + p.neighbor + "): " + weCalc.getWeight());
                    }
                    else
                    {
                        //no need to check destination for these because they are on sent by immediate neighbors
                        debug.println(1, "Neighbor " + nsap + " RETURNS echo to router " + p.requester);
                        this.returnEcho(p);
                    }
               
                }
                else 
                {
                    debug.println(1, "Error.  The packet being tranmitted is not a recognized LinkStateRouter Packet.  Not processing");
                  
                }
                
            
            }

            
            if (!process) {
                // Didn't do anything, so sleep a bit
                try { Thread.sleep(1); } catch (InterruptedException e) { }
            }
            
        }
    }

        /** Route the given packet out.
        In our case, we go to the node specified in the calculated path
    **/
    private void dijkstraRoute(DijkstraPacket p, int nextDest) {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            if (outLinks.get(i) == nextDest) {
                // Send packet to ONLY the next link in the calculated path
                nic.sendOnLink(i, p);
            }
        }
    }

      /** Route the given packet out.
        LSP packets should be flooded
    **/
    private void lspRoute(int linkOriginator, LSP p) {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            if (outLinks.get(i) != linkOriginator) {
                // Send packet to ONLY the next link in the calculated path
                nic.sendOnLink(i, p);
            }
        }
    }

          /** Route the given packet out.
        Echo packet should be sent immediately back to source
    **/
    private void returnEcho(EchoPacket p) {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
            if (outLinks.get(i) == p.requester) {
                // Send packet to ONLY back to the originator
                nic.sendOnLink(i, p);
            }
        }
    }

    //checks whether the router has a calculated weight value for all neighbors
    //also updates the HashMap to send out on our LSP that only has the weights 
    private Boolean checkAllWeights_UpdateMaster()
    {
        for (Map.Entry<Integer, WeightCalc> pair : this.neighborEdgeCalcs.entrySet())
        {
            if (pair.getValue().getWeight() == null)
            {
                return false;
            }
            else 
            {
                this.neighborEdgeWeights.put(pair.getKey(), pair.getValue().getWeight());
            }
        }
       return true;
    }

    private void echoOut() {
        ArrayList<Integer> outLinks = nic.getOutgoingLinks();
        int size = outLinks.size();
        for (int i = 0; i < size; i++) {
                EchoPacket p = new EchoPacket(this.nsap, outLinks.get(i));
                this.neighborEdgeCalcs.put(outLinks.get(i), new WeightCalc(System.currentTimeMillis()));
                debug.println(1, "Router " + nsap + " SENDS echo to neighbor " + p.neighbor);
                nic.sendOnLink(i, p);
        }
    }

    //computes the shortest path from a single source to all destinations using dijkstra's algorithm
    //requires knowledge of the entire network 
    //returns a routing table containing the shortest path to every other router on the network
    private HashMap<Integer, Stack<Integer>> dijkstraAlg()
    {
        //the source vertex is assumed to be the vertex corresponding to this router
        //we'll use a min heap of size V (or the number of vertices) to store the vertices not yet included in SPT (shortest path tree)
        //each element in the heap contains a vertex and its current distance value

        int vertices = this.fullNetworkGraph.getNumVertices();

        PriorityQueue<DijkstraNode> minHeap = new PriorityQueue<DijkstraNode>(vertices, new DijkstraNode());
        Set<Integer> settled = new HashSet<Integer>();
        int dist[] = new int[vertices];
        //we'll traverse "prev" in reverse to get the shortest path from node u to v
        int prev[] = new int[vertices]; //stores values representing the next location to get to the shortest route to the source

        for (int i = 0; i < vertices; i++)
        {
            dist[i] = Integer.MAX_VALUE;
            prev[i] = -1; //we haven't computed the shortest paths, so these will be undefined "-1" at first

        }

        //distance to source vertex is 0 at start
        dist[this.fullNetworkGraph.indexes.get(nsap)] = 0;

        //set source vertex to have initial cost of 0
        minHeap.add(new DijkstraNode(this.fullNetworkGraph.indexes.get(nsap), 0));
        //System.out.println("Adjacency List: " + vertices + "   Indexes: " + this.fullNetworkGraph.indexes.size() + "    BackToID: " + this.fullNetworkGraph.backToID.size());
        while(settled.size() != vertices)
        {
            //remove minimum distance node from priority queue
            int min;
            try
            {
                min = minHeap.remove().vertex;
            }
            catch (NoSuchElementException e)
            {
                break;
            }

            //add node to the finalized hashset
            settled.add(min);

            int edgeDistance = -1;
            int newDistance = -1;
    
            for (int i = 0; i < this.fullNetworkGraph.adjacencyList.get(min).size(); i++)
            {
                DijkstraNode v = this.fullNetworkGraph.adjacencyList.get(min).get(i);
    
                //if current node hasn't yet been processed
                if (!settled.contains(v.vertex))
                {
                    edgeDistance = v.cost;
                    newDistance = dist[min] + edgeDistance;

                    //if new distance is cheaper in cost
                    if (newDistance < dist[v.vertex])
                    {
                        dist[v.vertex] = newDistance;
                        prev[v.vertex] = min; //min is the next node to take from v to form the shortest route to the source
                    }

                    //Add current node to the priority queue
                    minHeap.add(new DijkstraNode(v.vertex, dist[v.vertex]));
                }
    
    
            }
        }

        HashMap<Integer, Stack<Integer>> routingTable = new HashMap<Integer, Stack<Integer>>();
        //next, let's build the routing table
        for (int i = 0; i < dist.length; i++)
        {
            if (i != this.fullNetworkGraph.indexes.get(nsap))
            {
                if (dist[i] >= 0) //otherwise, the path doesn't exist
                {
                    int z = i;
                    routingTable.put(this.fullNetworkGraph.backToID.get(z), new Stack<Integer>());
                    Stack<Integer> path = routingTable.get(this.fullNetworkGraph.backToID.get(z));
                    while (z >= 0)
                    {
                        path.push(this.fullNetworkGraph.backToID.get(z));
                        z = prev[z];
                    }

                    if (path.peek() == this.nsap)
                    {
                        path.pop(); //we want the first node in the path to contain the first destination
                    }

                    if (path.size() <= 0)
                    {
                        this.routingTable.remove(this.fullNetworkGraph.backToID.get(i));
                    }
                }
            }
        }

        return routingTable;


    }

    public void decrementLSPAge()
    {
        Set<Integer> keys = this.linkSet.keySet();
        Set<Integer> toRemove = new HashSet<Integer>();
        for (int key : keys)
        {
            if (this.linkSet.get(key).age <= 0)
            {
                //this.linkSet.remove(key);
                toRemove.add(key);
            }
            else
            {
                this.linkSet.get(key).age --;
            }
        }

        toRemove.forEach((key) -> this.linkSet.remove(key));

        if (this.myLSP != null)
        {
            if(this.myLSP.age <= 0)
            {
                this.myLSP = null;
                this.sequence ++;
            }
            else 
            {
                this.myLSP.age --;
            }
        }
    }

   
}
