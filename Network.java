/***************
 * Network
 * Author: Christian Duncan
 *
 * Represents a network with routers (nodes) and links (edges)
 ***************/
import java.io.PrintWriter;
import java.io.PrintStream;
import java.io.FileReader;
import java.io.BufferedReader;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.BiConsumer;

public class Network {
    public class Node {
        int nsap;  // The NSAP (ID) of this node
        double probDown;   // Probability that node goes down every second (0 = never, 1 = always)
        double downTime;   // Average time node stays down (in seconds)
        double downDev;    // Standard deviation for downtime
        Router r;  // The router associated with this node
        double remainingDown;  // If >0, node is down... decrements every second until 0 and back up.
        ArrayList<Connection> outgoingLinks;   // An ArrayList of Connections (the outgoing links in the network from this node)
        ArrayList<Connection> incomingLinks;   // An ArrayList of Connections (the incoming links in the network to this node)
        
        public Node(int nsap, double probDown, double downTime, double downDev, Router r) {
            this.nsap = nsap;
            this.probDown = probDown;
            this.downTime = downTime;
            this.downDev = downDev;
            this.r = r;
            this.remainingDown = 0;
            this.outgoingLinks = new ArrayList<Connection>();
            this.incomingLinks = new ArrayList<Connection>();
        }

        public String toString() {
            StringBuilder res = new StringBuilder();
            res.append("Node ");
            res.append(nsap);
            res.append(":");
            res.append("\n   Outgoing ");
            for (Connection c: outgoingLinks) {
                res.append(c.destination.nsap);
                res.append(" ");
            }
            res.append("\n   Incoming ");
            for (Connection c: incomingLinks) {
                res.append(c.source.nsap);
                res.append(" ");
            }
            return res.toString();
        }
    }

    public class Connection {
        Node source;       // The source of this link
        Node destination;  // The destination of this link

        // Used for simulating nosiy/corrupt networks
        double meanSpeed;      // The "average" speed for this link
        double stdSpeed;     // The standard deviation of the speed for this link

        public Connection(Node source, Node destination, double meanSpeed, double stdSpeed) {
            this.source = source;
            this.destination = destination;
            this.meanSpeed = meanSpeed;
            this.stdSpeed = stdSpeed;
        }

        public synchronized boolean sendOnLink(Object packet) {
            // Register the start of transmission on this link - for visualization
            // TBD
            
            // Sleep for the average speed for this link -- simulating a delay
            try {
                long delay = Math.round(rand.nextGaussian()*stdSpeed + meanSpeed);
                if (delay > 0) 
                    Thread.sleep(delay);

                // Inform the receiving router of the new incoming packet - place it on its receiving queue
                debug.println(5, "Transmitting on link from " + source.nsap + " to " + destination.nsap);
                destination.r.nic.receive(source.nsap, packet);
                return true;  // Success
            } catch (InterruptedException e) {
                // We should not be interrupted while trying to transmit.  But if so, it fails to transmit!
                return false;
            }
        }
    }

    /**
     * The total stats for the given network.
     **/
    public class Stat {
        private int packetsReceived;
        private int duplicatePackets;
        private double meanTimeTaken;  // average time taken per packet
        private double ewmaTimeTaken;  // exponentially weighted moving average of time taken
        private double alpha;          // The exponentially weighting decrease
        private ArrayList<PacketStat> packetsSent;  // Used to track the number of packets transmitted
        public Stat() {
            packetsReceived = 0;
            duplicatePackets = 0;
            meanTimeTaken = 0;
            ewmaTimeTaken = 0;
            alpha = 0.1;
            packetsSent = new ArrayList<>();
        }

        /**
         * Clone/copy other stat to this new Stat
         **/
        private Stat(Stat other) {
            this.packetsReceived = other.packetsReceived;
            this.duplicatePackets = other.duplicatePackets;
            this.meanTimeTaken = other.meanTimeTaken;
            this.ewmaTimeTaken = other.ewmaTimeTaken;
            this.alpha = other.alpha;
            this.packetsSent = other.packetsSent;  // WARNING: NOT A DEEP COPY!!!
        }

        @Override
        protected synchronized Object clone() {
            return new Stat(this);
        }

        public synchronized void add(PacketStat p) { packetsSent.add(p); }
        public synchronized int getTotalPacketsSent() { return packetsSent.size(); }
        public synchronized int getPacketsReceived() { return packetsReceived; }
        public synchronized void increaseDuplicatePackets() { duplicatePackets++; }
        public synchronized int getDuplicatePackets() { return duplicatePackets; }
        public void updateTimeTakenForNewArrival(long tt) {
            packetsReceived++;
            meanTimeTaken = meanTimeTaken + (tt - meanTimeTaken)/packetsReceived;  // Update the average: [att*(n-1) + tt]/n
            ewmaTimeTaken = ewmaTimeTaken*(1-alpha) + tt*alpha;
        }
        public double getEWMATimeTaken() { return ewmaTimeTaken; }
        public double getMeanTimeTaken() { return meanTimeTaken; }
    }

    // Statistics to track for a SINGLE packet
    private int packetNumberCount = 1;
    private class PacketStat {
        int source;
        int dest;
        int packetNumber;       // The specific one being created (an ID)
        long startTime;         // Time at which packet was created
        long timeTaken;         // Time taken to arrive, -1 means not yet arrived.
        long arrivals;          // Number of times arrived (to track duplicates)
        public PacketStat(int source, int dest) {
            this.source = source;
            this.dest = dest;
            this.packetNumber = packetNumberCount;
            packetNumberCount++;
            this.startTime = System.currentTimeMillis();
            this.timeTaken = -1;
            this.arrivals = 0;
        }

        public String toString() {
            return "Packet #" + packetNumber + " (" + source + "->" + dest + ")";
        }
    }
    
    private Random rand;  // Random number generator for randomizing behaviour
    private HashMap<Integer, Node> nodes;
    private Debug debug;
    private Stat stats;  // Stats for this network
    
    public Network() {
        nodes = new HashMap<>();
        rand = new Random();
        debug = Debug.getInstance();
        stats = new Stat();
    }

    /**
     * Load a network from the given text file - using the described format
     * (See testGraph.txt)
     * @param fileName The name of the network file to load
     * @returns Network - a new Network containing the nodes/edges described in the file
     *                  - null if there is an error
     **/
    public void loadNetwork(String fileName) throws Exception {
        BufferedReader in = new BufferedReader(new FileReader(fileName));
        String[] line = null;
        // First get the size of the network (nodes and edges)
        line = getNextLine(in);
        int n = Integer.parseInt(line[0]);
        int m = Integer.parseInt(line[1]);
        for (int i = 0; i < n; i++) {
            line = getNextLine(in);
            int nsap = Integer.parseInt(line[0]);
            double prob = Double.parseDouble(line[1]);
            double meanDown = Double.parseDouble(line[2]);
            double stdDown = Double.parseDouble(line[3]);
            Node node = new Node(nsap, prob, meanDown, stdDown, null);
            nodes.put(nsap, node);  // Save the node in the hashmap for quick lookup
        }
        for (int i = 0; i < m; i++) {
            line = getNextLine(in);
            int nsapA = Integer.parseInt(line[0]);
            int nsapB = Integer.parseInt(line[1]);
            double meanSpeed = Double.parseDouble(line[2]);
            double stdSpeed = Double.parseDouble(line[3]);
            Node a = nodes.get(nsapA);
            if (a == null) {
                throw new Exception("Node (" + nsapA + ") not found.");
            }
            Node b = nodes.get(nsapB);
            if (b == null) {
                throw new Exception("Node (" + nsapB + ") not found.");
            }
            Connection c = new Connection(a, b, meanSpeed, stdSpeed);
            a.outgoingLinks.add(c);
            b.incomingLinks.add(c);
        }
    }

    /**
     * Get the next line from the buffered reader that is NOT just a pure comment
     * Returns it as an array of Strings (tokenied by spaces) and with comments removed
     */
    private static String[] getNextLine(BufferedReader in) throws Exception {
        String[] line;
        do {
            line = parseLine(in.readLine());
        } while (line.length == 0 || line[0].length() == 0);
        return line;
    }
    
    private static String[] parseLine(String line) {
        // First remove comments (if any)
        String[] commentLess = line.split("#", 2);
        return commentLess[0].split(" ");
    }

    /**
     * Print the entire network
     **/
    public void printNetwork(PrintWriter out) {
        nodes.forEach((id, n) -> out.println(n));
    }

    public void printNetwork(PrintStream out) {
        nodes.forEach((id, n) -> out.println(n));
    }

    /**
     * Apply a function to each node in the network - mostly hopefully for display purposes only!
     **/
    public void forEachNode(BiConsumer<Integer, Node> action) {
        nodes.forEach(action);
    }
    
    /**
     * Create routers for all the nodes on the network
     **/
    public void createRouters(Router.Generator gen) {
        nodes.forEach((id, n) -> {
                // Create a network interface card for each router
                // Build an Integer only list of links to this NIC.
                ArrayList<Integer> outgoingLinks = new ArrayList<Integer>(n.outgoingLinks.size());
                ArrayList<Integer> incomingLinks = new ArrayList<Integer>(n.incomingLinks.size());
                n.outgoingLinks.forEach(c -> outgoingLinks.add(c.destination.nsap));
                n.incomingLinks.forEach(c -> incomingLinks.add(c.source.nsap));
                NetworkInterface nic = new NetworkInterface(this, n.nsap, outgoingLinks, incomingLinks, 100);
                
                Router r = gen.createRouter(n.nsap, nic);  // Create router using the generator
                n.r = r;                                   // Associate it with the node
                n.r.start();                               // Start it running
            });
    }
    
    private int packetFrequency = 0;
    public void setPacketFrequency(int p) { packetFrequency = p; }
    public int getPacketFrequency() { return packetFrequency; }
    
    /**
     * Simulate the network running for length milliseconds
     * @params out The output stream to use for messages
     * @params length The number of milliseconds to run the simulation
     * @params packetFrequency The number of packets to transmit per second
     * Each second, roughly quantity packets are generated between random nodes
     **/
    public void runNetwork(PrintStream out, long length, int packetFrequency) throws InterruptedException {
        setPacketFrequency(packetFrequency);
        runNetwork(out, length);

        // Finished -- Sleep a few seconds to allow packets to arrive
        Thread.sleep(1000);
        debug.println(1, "Network simulation completed.  Displaying statistics...");
        displayStats();        
    }

    private boolean networkRunning = false;
    public synchronized void setNetworkRunning(boolean flag) { networkRunning = flag; }
    
    /**
     * Simulate the network running for length milliseconds (-1 means "forever")
     * @params out The output stream to use for messages
     * @params length The number of milliseconds to run the simulation
     * Each second roughly packetFrequency packets are generated between random nodes.  Uses instance variable which allows changing value while running.
     **/
    private final int MIN_SLEEP = 10;
    public void runNetwork(PrintStream out, long length) throws InterruptedException {
        List<Integer> nsaps = new ArrayList<Integer>(nodes.keySet());

        long endTime = -1;
        if (length >= 0) {
            // "Infinite" time
            endTime = System.currentTimeMillis() + length;
        }
        setNetworkRunning(true);
        double minRate = 1000.0/MIN_SLEEP;
        while (networkRunning) {            
            // Determine how many packets to generate (and how long to pause for next generation)
            int pf = getPacketFrequency();
            int sleepTime;
            int generate;
            if (pf == 0) {
                // Nothing to generate
                sleepTime = MIN_SLEEP;
                generate = 0;
            } else if (pf > minRate) { 
                // Too many for one at a time... generate multiple per 10ms pattern
                sleepTime = MIN_SLEEP;
                generate = (int) Math.floor(pf/minRate + rand.nextDouble());
            } else {
                // Can generate 1 and then sleep for a sufficient amount of time to keep up the rate
                generate = 1;
                sleepTime = (int) Math.floor(1000.0/pf + rand.nextDouble());
            }
            for (int i = 0; i < generate; i++) {
                // And generate each packet
                int start = rand.nextInt(nsaps.size());
                int end = rand.nextInt(nsaps.size()-1);
                if (end >= start) end++;   // This way we don't have start to start
                Integer source = nsaps.get(start);
                Integer dest = nsaps.get(end);
                PacketStat aPacket = new PacketStat(source, dest);
                transmit(source, dest, aPacket);
            }                
            // Has time run out? (If it was set at all)
            if (endTime >= 0 && System.currentTimeMillis() > endTime) setNetworkRunning(false);
            else {
                Thread.sleep(sleepTime);  // Pause for a few milliseconds and resume
            }
        }
    }

    /**
     * Return a (copy of) the current stats
     **/
    public Stat getStats() {
        return (Stat) stats.clone();
    }
    
    /**
     * Report some statistics on the network performance
     **/
    private void displayStats() {
        // First compute the statistics
        int packetsTransmitted = stats.packetsSent.size();
        int packetsReceived = 0;
        int duplicatePackets = 0;
        double averagePacketTime = 0.0;
        for (PacketStat s: stats.packetsSent) {
            if (s.arrivals > 0) {
                packetsReceived++;
                duplicatePackets += (s.arrivals - 1);  // How many duplicates were sent to final destination
                averagePacketTime = (averagePacketTime*(packetsReceived-1) + s.timeTaken)/packetsReceived;
            } 
        }
        System.out.println("Network Statistics");
        System.out.println("   Packets transmitted:     " + packetsTransmitted);
        System.out.println("   Packets received:        " + packetsReceived);
        System.out.println("   Success percentage:      " + ((double) packetsReceived/packetsTransmitted)*100);
        System.out.println("   Duplicate packets:       " + duplicatePackets);
        System.out.println("   Average time taken (ms): " + averagePacketTime);
    }
    
    /**
     * "Transmit" data from source to destination in the network
     **/
    private void transmit(Integer source, Integer dest, PacketStat data) {
        Node s = nodes.get(source);
        if (s.remainingDown > 0) return;   // Source is still down, can't transmit.
        debug.println(3, "Transmitting from " + source + " to " + dest);
        stats.add(data);  // Record the transmission
        s.r.nic.transmit(dest, data);
    }

    /**
     * "Received" data at destination - for tracking purposes
     **/
    public void receive(Integer dest, Object data) {
        if (data instanceof PacketStat) {
            PacketStat payload = (PacketStat) data;
            synchronized (payload) {
                if (payload.dest != dest) {
                    debug.println(0, "Coding Error: The payload did not arrive at the proper destination.");
                } else if (payload.timeTaken == -1) {
                    // Packet has newly arrived
                    payload.timeTaken = System.currentTimeMillis() - payload.startTime;
                    payload.arrivals++;
                    stats.updateTimeTakenForNewArrival(payload.timeTaken);
                } else {
                    debug.println(5, "Duplicate packet arrived. Packet: " + payload);
                    payload.arrivals++;
                    stats.increaseDuplicatePackets();
                }
            }
        } else {
            debug.println(0, "Error: The payload received was NOT an initially transmitted packet!");
        }
    }
    
    /**
     * "Transmit" a packet on a specific link
     **/
    public boolean sendOnLink(int source, int linkIndex, Object packet) {
        Node sourceNode = nodes.get(source);
        if (linkIndex < 0 || linkIndex >= sourceNode.outgoingLinks.size())
            // No such link exists
            return false;
        Connection c = sourceNode.outgoingLinks.get(linkIndex);
        return c.sendOnLink(packet);
    }
}    
