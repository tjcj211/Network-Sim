/***************
 * App
 * Author: Christian Duncan
 * Spring 21: CSC340
 * 
 * This is the Main GUI interface to the Network Simulator
 * It allows selecting various options as well as
 * visualizing the network and monitoring statistics on the
 * network.
 ***************/
import java.awt.*;        // import statements to make necessary classes available
import java.awt.geom.*;
import javax.swing.*;
import java.awt.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.*;
import java.util.HashMap;
import java.text.NumberFormat;

public class App extends JFrame {
    /**
     * The main entry point that sets up the window and basic functionality
     */
    public static void main(String[] args) {
        App frame = new App();

        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setResizable(true);
        frame.setVisible(true);
    }

    private Network net;
    private VisPanel visPane;
    private Debug debug = Debug.getInstance();
    JComboBox<String> routerBox = null;
    JButton runButton = null;
    JTextField pps = null;
    final private String[] routerChoices = { "Flood", "Distance Vector", "Link State" };
    JDialog debugWindow = null;
    JDialog statsWindow = null;
    
    /* Constructor: Sets up the initial look-and-feel */
    public App() {
        // Network administrative stuff first
        net = null;  // No network loaded/created yet.
        
        JLabel label;  // Temporary variable for a label
        JButton button; // Temporary variable for a button

        // Set up the initial size and layout of the frame
        // For this we will keep it to a simple BoxLayout
        setLocation(100, 100);
        setPreferredSize(new Dimension(800, 800));
        setTitle("CSC340 Basic Network Simulator");
        Container mainPane = getContentPane();
        mainPane.setLayout(new BoxLayout(mainPane, BoxLayout.Y_AXIS));
        mainPane.setPreferredSize(new Dimension(1000, 500));

        // Create the Visualization Panel
        visPane = new VisPanel();
        mainPane.add(visPane);
        
        // Create the buttons at bottom
        JPanel buttonPanel = new JPanel();
        buttonPanel.setLayout(new BoxLayout(buttonPanel, BoxLayout.X_AXIS));

        // Drop down list (Flood, Distance Vector, Link State)
        JLabel routerLabel = new JLabel("Router:");
        routerBox = new JComboBox<String>(routerChoices);
        routerBox.setEnabled(true);
        buttonPanel.add(routerBox);
        
        // Run/Reset network button
        Action runAction = new AbstractAction("Run") {
                public void actionPerformed(ActionEvent e) {
                    if (runButton.getText().equals("Run")) {
                        // Run the network
                        routerBox.setEnabled(false);  // No more changes allowed
                        Thread netRun = new Thread() {
                                public void run() {
                                    setupNetwork();
                                    runNetwork();
                                }
                            };
                        runButton.setText("Pause");
                        netRun.start();
                    } else if (runButton.getText().equals("Pause")) {
                        // Stop the network
                        net.setNetworkRunning(false);
                        runButton.setText("Resume");
                    } else if (runButton.getText().equals("Resume")) {
                        Thread netRun = new Thread() {
                                public void run() {
                                    runNetwork();
                                }
                            };
                        runButton.setText("Pause");
                        netRun.start();
                    }
                }
            };
        runAction.putValue(Action.SHORT_DESCRIPTION, "Start the network running.");
        runButton = new JButton(runAction);
        runButton.setAlignmentX(JButton.CENTER_ALIGNMENT);
        runButton.setEnabled(false);  // Not enabled until a network is loaded!
        buttonPanel.add(runButton);
        
        // JTextField: Packets/Second
        pps = new JTextField("0");
        pps.setEditable(false);  // Not editable until network is loaded
        pps.setHorizontalAlignment(JTextField.RIGHT);
        JLabel ppsLabel = new JLabel("Packets/sec");
        ppsLabel.setAlignmentX(JLabel.LEFT_ALIGNMENT);
        Action ppsAction = new AbstractAction("Packets/Second") {
                public void actionPerformed(ActionEvent e) {
                    if (net == null) {
                        debug.println(0, "ppsAction: Coding error.  Net should be active before this field is editable!");
                        pps.setText("0");
                        return;
                    }
                    try {
                        // Packets per second has changed!
                        int ppsValue = Integer.parseInt(pps.getText());
                        if (ppsValue < 0) {
                            JOptionPane.showMessageDialog(visPane, "The Packets Per Second must be a positive integer.",
                                                          "Error", JOptionPane.ERROR_MESSAGE);
                            pps.setText(net.getPacketFrequency()+"");
                        } else {
                            net.setPacketFrequency(ppsValue);
                        }
                    } catch (NumberFormatException ignored) {
                        JOptionPane.showMessageDialog(visPane, "The Packets Per Second must be a valid positive integer.",
                                                      "Error", JOptionPane.ERROR_MESSAGE);
                        pps.setText(net.getPacketFrequency()+"");
                    }
                }
            };
        pps.addActionListener(ppsAction);
        buttonPanel.add(pps);
        buttonPanel.add(ppsLabel);
        mainPane.add(buttonPanel);

        // Set up the debug window
        setupDebugWindow();

        // Set up the stats window
        setupStatsWindow();
        
        // Setup the menubar
        setupMenuBar();

        // Create animation
        Timer animationTimer;  // A Timer that will emit events to drive the animation.
        animationTimer = new Timer(16, new ActionListener() {
                long lastTimeCheck = System.currentTimeMillis();
                public void actionPerformed(ActionEvent arg0) {
                    long currTimeCheck = System.currentTimeMillis();
                    advance(currTimeCheck - lastTimeCheck);
                    lastTimeCheck = currTimeCheck;
                    visPane.repaint();
                }
            });
        animationTimer.start();
    }

    // Basically a scrollable text area that shows contents of the debug output
    // stream.  Which we will also create here.
    private void setupDebugWindow() {
        debugWindow = new JDialog(this, "Debug Output (Level " + debug.getLevel() + ")");
        JTextArea debugTextArea;
        debugTextArea  = new JTextArea(50,60);
        debugTextArea.setEditable(false);
        JScrollPane scrollPane = new JScrollPane(debugTextArea);
        debugWindow.add(scrollPane);
        debugWindow.setVisible(false);
        debugWindow.pack();
        debugWindow.setResizable(true);
        debug.setStream(new PrintStream(new TextStreamer(debugTextArea)));
    }

    // Basically a window that periodically monitors the network stats and updates
    // the information.
    private JTextArea statsTextArea = null;
    private void setupStatsWindow() {
        statsWindow = new JDialog(this, "Statistics");
        statsTextArea = new JTextArea(10, 40);
        statsTextArea.setEditable(false);
        JScrollPane scrollPane = new JScrollPane(statsTextArea);
        statsWindow.add(scrollPane);
        statsWindow.pack();
        statsWindow.setVisible(false);
        statsWindow.setResizable(true);
    }

    private NumberFormat statsFormat = null;
    private Font statsFont = null;
    private void refreshStatsWindow() {
        if (statsFont == null) {
            // Create the node font
            statsFont = new Font("Monospaced", Font.BOLD, 14);
        }
        statsTextArea.setFont(statsFont);
        
        if (net == null) {
            statsTextArea.setText("No network loaded yet.");
        } else {
            Network.Stat stat = net.getStats();
            if (stat == null) {
                statsTextArea.setText("Network has no statistics yet ready to track.");
            } else {
                if (statsFormat == null) {
                    statsFormat = NumberFormat.getNumberInstance();
                    statsFormat.setMaximumFractionDigits(3);
                    statsFormat.setMinimumFractionDigits(3);
                }
                    
                StringBuilder sb = new StringBuilder();
                sb.append("Packets transmitted: " + stat.getTotalPacketsSent());
                sb.append("\nPackets received:    " + stat.getPacketsReceived());
                sb.append("\nSuccess percentage:  ");
                if (stat.getTotalPacketsSent() > 0)
                    sb.append(statsFormat.format(100.0 * (double) stat.getPacketsReceived()/(double) stat.getTotalPacketsSent()) + "%");
                else
                    sb.append("---");
                sb.append("\nDuplicate packets:   " + stat.getDuplicatePackets());
                sb.append("\nAverage time taken:  " + statsFormat.format(stat.getMeanTimeTaken()) + " ms");
                sb.append("\nEWMA time taken:     " + statsFormat.format(stat.getEWMATimeTaken()) + " ms");
                String sbString = sb.toString();
                statsTextArea.setText(sbString);
            }
        }
        statsTextArea.setCaretPosition(statsTextArea.getText().length());
    }
    
    private void setupMenuBar() {
        JMenuBar mbar = new JMenuBar();
        JMenu menu;
        JMenuItem menuItem;
        Action menuAction;
        menu = new JMenu("File");
        menuAction = new AbstractAction("Load Network") {
                JFileChooser chooser = null;
                public void actionPerformed(ActionEvent event) {
                    if (net != null) {
                        // Network already loaded
                        JOptionPane.showMessageDialog(visPane, "Sorry.  A network has already been loaded.  Currently, we don't support overriding it.",
                                                      "Error", JOptionPane.ERROR_MESSAGE);
                        return;
                    }
                    if (chooser == null) {
                        chooser = new JFileChooser(new File(System.getProperty("user.dir")));
                        FileNameExtensionFilter filter = new FileNameExtensionFilter("Networks (GQU)", "gqu");
                        chooser.setFileFilter(filter);
                    }
                    int returnVal = chooser.showOpenDialog(visPane);
                    String networkFile = null;
                    if(returnVal == JFileChooser.APPROVE_OPTION) {
                        try {
                            net = new Network();
                            networkFile = chooser.getSelectedFile().getName();
                            net.loadNetwork(networkFile);
                            runButton.setEnabled(true);  // Enable the run button so network can be run!
                            pps.setEditable(true);       // Enable editing this value now that network has loaded.
                        } catch (Exception e) {
                            JOptionPane.showMessageDialog(visPane, "Error loading network: " + networkFile + "\n", "Error", JOptionPane.ERROR_MESSAGE);
                            net = null;  // Clear it back out.  So, it can be loaded again.
                        }
                    }
                }
            };
        menuAction.putValue(Action.SHORT_DESCRIPTION, "Load a network to simulate");
        menuItem = new JMenuItem(menuAction);
        menu.add(menuItem);
        mbar.add(menu);
        
        menu = new JMenu("Monitor");
        menuAction = new AbstractAction("Show Stats") {
                public void actionPerformed(ActionEvent event) {
                    statsWindow.setLocationRelativeTo(statsWindow.getParent());
                    statsWindow.setVisible(true);
                }
            };
        menuAction.putValue(Action.SHORT_DESCRIPTION, "Show network statistics");
        menuItem = new JMenuItem(menuAction);
        menu.add(menuItem);
        menuAction = new AbstractAction("Debug Console") {
                public void actionPerformed(ActionEvent event) {
                    debugWindow.setLocationRelativeTo(debugWindow.getParent());
                    debugWindow.setVisible(true);
                }
            };
        menuAction.putValue(Action.SHORT_DESCRIPTION, "Show debug console");
        menuItem = new JMenuItem(menuAction);
        menu.add(menuItem);
        menuAction = new AbstractAction("Debug Level") {
                public void actionPerformed(ActionEvent e) {
                    String debugLevel = JOptionPane.showInputDialog("Please enter an integer to select debug level.");
                    if (debugLevel != null && debugLevel.length() > 0) {
                        try {
                            int dl = Integer.parseInt(debugLevel);
                            debug.setLevel(dl);
                            debugWindow.setTitle("Debug Output (Level " + debug.getLevel() + ")");
                        } catch (NumberFormatException ignore) {
                            JOptionPane.showMessageDialog(null, "The debug level [" + debugLevel + "] must be an integer.", "Number Format Error", JOptionPane.ERROR_MESSAGE);
                        }
                    }
                }
            };
        menuAction.putValue(Action.SHORT_DESCRIPTION, "Show network statistics");
        menuItem = new JMenuItem(menuAction);
        menu.add(menuItem);
        mbar.add(menu);
        setJMenuBar(mbar);
    }

    // Assigns routers and then starts the network running
    public void setupNetwork() {
        try {
            Router.Generator gen = null;
            switch (routerBox.getSelectedIndex()) {
            case 0: gen = new FloodRouter.Generator(); break;
            case 1: gen = new DistanceVectorRouter.Generator(); break;
            case 2: gen = new LinkStateRouter.Generator(); break;
            default: debug.println(0, "Coding error.  Router not recognized.  Using Flood.");
                gen = new FloodRouter.Generator();
            }
            net.createRouters(gen);
        } catch (Exception e) {
            JOptionPane.showMessageDialog(visPane, "Error setting up the network.",
                                          "Error", JOptionPane.ERROR_MESSAGE);
            debug.println(0, "Error setting up the network: " + e.getMessage());
        }
    }

    public void runNetwork() {
        try {
            net.runNetwork(debug.getStream(), -1);
        } catch (Exception e) {
            JOptionPane.showMessageDialog(visPane, "Error running the network.",
                                          "Error", JOptionPane.ERROR_MESSAGE);
            debug.println(0, "Error running network: " + e.getMessage());
        }
    }
    
    /**
     *  Update any features of the images that need to be shown.
     * In particular, update the stats file (or every so often)
     * @params timeGapInMS How much time has elapsed since last call
     **/
    private long statsDelayRemaining = 0;
    private final long STATS_DELAY = 1000;  // How long between stats updates
    private void advance(long timeGapInMS) {
        statsDelayRemaining -= timeGapInMS;
        if (statsDelayRemaining <= 0) {
            // Reset
            statsDelayRemaining = STATS_DELAY;

            // And update the stats panel
            refreshStatsWindow();
        } 
    }
    
    public class VisPanel extends JPanel {
        Graphics2D g2;
        double viewportSize = 100.0;
        
        public VisPanel() {
            setPreferredSize(new Dimension(1000,1000) ); // Set size of drawing area, in pixels.
        }

        /* Used for drawing the network */
        protected void paintComponent(Graphics g) {
            g2 = (Graphics2D) g.create();
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setPaint(new Color(200, 200, 220));
            g2.fillRect(0, 0, getWidth(), getHeight());
            setupViewport(-viewportSize, viewportSize, -viewportSize, viewportSize);

            drawNetwork();
        }

        /**
         * Set up the viewport so dimensions of window are in range (left to right) and (bottom to top)
         * ... roughly, aspect ratio is still preserved.
         * @param left left edge of drawing area
         * @param right right edge of drawing area
         * @param bottom bottom edge of drawing area
         * @param top top edge of drawing area
         */
        private void setupViewport(double left, double right, double bottom, double top) {
            // Get width and height in pixels of panel.
            int width = getWidth();  
            int height = getHeight();

            // Correct viewport dimensions to preserve aspect ratio
            double panelAspect = Math.abs((double)height / width);
            double viewAspect = Math.abs(( bottom-top ) / ( right-left ));
            if (panelAspect > viewAspect) {
                // Expand the viewport vertically.
                double padding = (bottom-top)*(panelAspect/viewAspect - 1)/2;
                bottom += padding;
                top -= padding;
            }
            else { 
                // Expand the viewport horizontally
                double padding = (right-left)*(viewAspect/panelAspect - 1)/2;
                right += padding;
                left -= padding;
            }

            g2.scale(width/(right-left), height/(bottom-top));
            g2.translate(-left, -top);
        }

        private void drawNetwork() {
            if (net == null) return;   // No network to display yet!
            
            if (nodeFont == null) {
                // Create the node font
                nodeFont = new Font("Serif", Font.BOLD, 18);
            }

            if (localNet == null) {
                loadGraph();
            }

            // Draw the links
            localNet.forEach((id, data) -> drawLinks(data));
            
            nodeFontMetrics = g2.getFontMetrics(nodeFont);  // Not sure if it changes as screen size changes for example. So getting it each redisplay
            localNet.forEach((id, data) -> drawNode(data.x, data.y, 5, data.n.nsap+""));
            // drawNode(0, 0, 5, "123");
            // drawNode(0, 30, 5, "456");
            // drawNode(30, 30, 5, "789");
        }

        /**
         * Stores a local copy of the graph into memory for displaying
         **/
        private class NodeData {
            Network.Node n;   // The node itself
            double x; // The x position of the node
            double y; // The y position of the node

            NodeData(Network.Node n) { this.n = n; x = 0; y = 0; }
        }
            
        private HashMap<Integer, NodeData> localNet = null;
        private void loadGraph() {
            // First copy the nodes into the hashmap with space for the extra info needed
            localNet = new HashMap<>();
            net.forEachNode((id, node) -> localNet.put(id, new NodeData(node)));

            // Space the nodes out evenly on a circle
            double angleCount = Math.PI * 2.0/(localNet.size());
            double angle = 0.0;
            for (NodeData d: localNet.values()) {
                d.x = viewportSize * 0.9 * Math.cos(angle);
                d.y = viewportSize * 0.9 * Math.sin(angle);
                angle += angleCount;
            }
        }

        // Could make it configurable but why...
        private Font nodeFont = null;
        private FontMetrics nodeFontMetrics = null;
        private Color nodeColor = new Color(0x87CEEB);
        private Color linkColor = new Color(0x000080);
        
        private void drawNode(double cx, double cy, double radius, String text) {
            AffineTransform cs = g2.getTransform();
            g2.translate(cx, cy);   // Make cx,cy the center... easier to think about.
            
            g2.setPaint(nodeColor);
            g2.setStroke(new BasicStroke(1));
            double diam = radius*2;
            Ellipse2D circ = new Ellipse2D.Double(-radius, -radius, diam, diam);
            g2.fill(circ);

            g2.setColor(Color.BLACK);
            g2.draw(circ);

            if (text != null) {
                // Compute the starting position of text so it is centered at cx,cy
                // Determine the X coordinate for the text
                // double x = cx - metrics.stringWidth(text) / 2.0;
                // double y = cy + (metrics.getHeight()-metrics.getLeading()) / 2.0
                Rectangle2D rec = nodeFontMetrics.getStringBounds(text, g2);
                double x = -rec.getCenterX();
                double y = -rec.getCenterY();  // Adding since y increases downward but still drops upward
                double scaleFactor = radius*1.8/rec.getWidth();
                g2.scale(scaleFactor, -scaleFactor);

                // Set the font and draw String
                g2.setColor(Color.BLACK);
                g2.setFont(nodeFont);
                g2.drawString(text, (float) x, (float) y);
            }

            g2.setTransform(cs);
        }

        /**
         * Draw the link between this node and all of its outgoing neighbors
         **/
        private void drawLinks(NodeData source) {
            g2.setPaint(linkColor);
            g2.setStroke(new BasicStroke(0.2f));
            
            for (Network.Connection c: source.n.outgoingLinks) {
                NodeData destination = localNet.get(c.destination.nsap);
                Path2D arc = new Path2D.Double();
                arc.moveTo(source.x, source.y);
                double dx = destination.x - source.x;
                double dy = destination.y - source.y;
                double norm = viewportSize*0.1/Math.sqrt(dx*dx + dy*dy);
                double midx = (destination.x + source.x)*0.5 - dy*norm;
                double midy = (destination.y + source.y)*0.5 + dx*norm;
                arc.quadTo(midx, midy, destination.x, destination.y);
                g2.draw(arc);
                // g2.draw(new Line2D.Double(source.x, source.y, destination.x, destination.y));
            }
        }
    }

    private class TextStreamer extends OutputStream {
        JTextArea txt;
        StringBuilder buffer;
        static final int MAX_LENGTH = 100000;   // Maximum length of text area (in characters)
        public TextStreamer(JTextArea txt) {
            this.txt = txt;
            this.buffer = new StringBuilder(256);
        }

        @Override
        public void write(int b) throws IOException {
            char c = (char) b;
            this.buffer.append(c);
            if (c == '\n') {
                // Flush the buffer
                flush();
            }
        }

        @Override
        public void flush() throws IOException {
            if (this.buffer.length() > 0) {
                // Transfer buffer to the TextArea, clear buffer, truncate if needed, and move caret
                this.txt.append(this.buffer.toString());
                this.buffer.setLength(0);  // Clear the buffer
                String fullText = this.txt.getText();
                int fullLen = fullText.length();
                if (fullLen > MAX_LENGTH) {
                    // Truncate (trim about half the text)
                    String trunc = "..." + fullText.substring(fullLen - MAX_LENGTH/2);
                    this.txt.setText(trunc);
                    fullLen = trunc.length();
                }
                this.txt.setCaretPosition(fullLen);
            }                
        }
    }
}

