public class Main {
    public static String NETWORK_FILE = "testGraph.gqu";
    
    public static void main(String[] args) {
        String networkFile = NETWORK_FILE;
        if (args.length > 0) {
            networkFile = args[0];
        }
        Debug.getInstance().setLevel(1);  // Set debug level for more verbose output (higher = more verbose)
        
        Network net = new Network();
        try {
            net.loadNetwork(networkFile);
        } catch (Exception e) {
            System.err.println("Error loading network: " + networkFile);
            System.err.println(e.getMessage());
        }

        net.printNetwork(System.out);
        net.createRouters(new FloodRouter.Generator());
        
        try {
            net.runNetwork(System.out, 10000, 100);
        } catch (Exception e) {
            System.err.println("Error running the network.");
            System.err.println(e.getMessage());
        }
        System.exit(0);  // Finished, kill all the threads!
    }
}
