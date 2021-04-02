Router Algorithms Project
------------

This project is a very simple network simulator that is designed mostly to practice
creating router algorithms.

Team Members:
------------
Charles Rescanski, Timothy Carta, Ryan Hayes, Griffin King, and Lauren Atkinson

Instructions:
------------
1. Download the code in this repository.
2. Extract ZIP file in a folder on your computer.
3. Open a command terminal.
4. Navigate to the folder containing the code.
5. Compile all Java files by typing "javac *.java."
6. Start the application by entering "java App.java."
7. Choose the desired routing algorithm for the network (Flood, Distance Vector, or Link State) by using the drop-down in the lower-left hand corner of the window.
8. Click File -> Load Network:
   * Navigate to the location of the network graph file (.gqu).
   * NOTE that the Distance Vector and Link State algorithms REQUIRE a BI-DIRECTIONAL graph to function properly!
     * Routers use PING to determine costs in relation to neighbors, which requires a BI-directional link.
   * The "testGraph2.gqu" may be used as an example.
9. In the text box located at the bottom of the window, enter the desired number of packets/second that you would like to be transmitted.
   * NOTE that you MUST press the ENTER key while the field is in focus (after typing the number) in order for the change to be made.
10. Finally, click the "Run" button to start the network simulator with the chosen configuration.
11. You may monitor the status of the network by clicking Monitor -> Show Stats.  

Contributions:
------------
Charles:
* Setup LinkStateDev branch in GitHub repository for developing the Link State Routing algorithm.
* Designed, Built, and Tested the LinkStateRouter.java file. 
  * Dijkstra's shortest path algorithm
  * Construction of Routing Table
  * LSP packet creation and transmission to all routers on network
  * Recalculation of PING periodically to construct new LSP (before old ones expire)
* Wrote the "Instructions" section in the README

Timothy:

Ryan:  

Griffin:  

Lauren:  

Process for Distributing Tasks:
------------

We did not have a formal methodology for task distribution.  At the start of the project, team members made it clear which routing algorithm they desired to build.  A separate development branch in GitHub was setup for each routing algorithm.  In addition, the team established a group chat early on to share progress, issues, and/or updates among all members.  The intention to perform a specific task was announced and recognized by all other group members by sending a request in the chat.  The development for both routing algorithms was done entirely separately; each algorithm had its own designated branch in the GitHub repository.   
