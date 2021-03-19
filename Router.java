/***************
 * Router
 * Author: Christian Duncan
 *
 * Represents a router on the network
 ***************/

public abstract class Router extends Thread {
    // A generator for the given Router class
    public static abstract class Generator {
        public abstract Router createRouter(int id, NetworkInterface nic);
    }
    
    protected int nsap;   // The NSAP (ID) of this node
    protected NetworkInterface nic;  // The routers "access" to the network

    public Router(int nsap, NetworkInterface nic) {
        this.nsap = nsap;
        this.nic = nic;
    }
}
