/***************
 * Debug
 * Author: Christian Duncan
 *
 * A class use to turn on/off debugging output (with given level)
 ***************/
import java.io.PrintStream;

public class Debug {
    static Debug singleton = null;
    private int level;
    private PrintStream err;
    
    public static Debug getInstance() {
        if (singleton == null)
            singleton = new Debug();
        return singleton;
    }

    private Debug() {
        level = 0;  // 0 means no output
        err = System.err;
    }

    public int getLevel() { return level; }
    public void setLevel(int level) { this.level = level; }
    public void setStream(PrintStream err) { this.err = err; }
    public void println(int level, String message) {
        if (this.level >= level) {
            err.println("DEBUG (" + level + "): " + message);
        }
    }
}
