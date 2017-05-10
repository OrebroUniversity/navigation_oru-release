/**
 * @file
 * @brief This file contains only doxygen definitions and should not be 
 *  included anywhere.
 *
 * @author Alexander Sherikov
 */


#ifndef DOXYGEN_H
#define DOXYGEN_H

/**
 * @mainpage An MPC-based tracking controller.
 *
 * @par Contents
 * - @ref pIntro
 * - @ref pDepRun
 * - @ref pDirs
 * - @ref pSoftware
 *
 * @par Other pages
 * - @ref pIntegration
 * - @ref pInteraction
 * - @ref swAssume
 * - @ref todo
 * \n
 *
 *
 * @section pIntro Introduction
 * This work is a part of two projects:
 *  - <a href="http://www.oru.se/English/Research/Research-environments/Research-environment/Centre-for-Applied-Autonomous-Sensor-Systems-AASS/Research/Research-projects/Research-project/?rdb=693">
 *      SAUNA - SAFE AUTONOMOUS NAVIGATION</a> 
 *  - <a href="http://www.aass.oru.se/Research/Learning/savie/index.html">SAVIE</a> 
 *
 * The purpose of this module is to control a car while tracking a certain trajectory.
 *
 * The module is adapted for SnowWhite car, which has VMC500 onboard controller that is capable
 * of executing given steering angles and velocities of the steering wheel. This controller
 * is produced by Kollmorgen (http://www.ndc8.com/ndc8.html). 
 *
 * Tracking is realized using model predictive control (MPC). The main purpose of this module is 
 * to form and solve respective quadratic programming (QP) problem. All fromulations are 
 * available in LaTeX format in 'formulations' folder.
 *
 * \n
 *
 * @section pDepRun Dependencies and running
 *
 * @verbinclude "README"
 *
 * \n
 *
 * @section pDirs Supplementary files and source code
 * 
 * The purpose of directories in the root of the project:
 *
 *  - 'config': sample configuration files of the module.
 *
 *  - 'dummy_node': dummy control node, which was used for integration tests.
 *
 *  - 'formulations': formulations of the QP problem and its constraints.
 *
 *  - 'func_gen': a set of functions generated in Maple. They form several matrices, which
 *  are necessary for generation of the QP problem. Each function is generated for a particular
 *  length of the preview window of MPC problem.
 *
 *  - 'qpOASES': third party QP solver.
 *
 *  - 'traj_gen': a set of simple Matlab scripts, which are used for generation of test 
 *  trajectories and generation of reference QP solutions for tests. 'spline_demo'
 *  subfolder contains an example of smooth trajectory generation with respect to
 *  time constraints. The test trajectories do not take into account initial and final 
 *  steering angles (phi), because these angles depend on the curvature of a polynomial, 
 *  which is used for interpolation.
 *
 * \n
 *
 * @section pSoftware Software
 * Some useful software, most of this stuff lies in /home/asv/SAUNA/ directory on the lab 
 * server.
 *
 * @subsection pSoftKoll From Kollmorgen
 * - Vehicle Application Designer
 * - OpenPCS
 * - Layout Designer: maps and trajectories
 *
 * @subsubsection pSoftKollFirmware Firmware upgrade
 * Load the .vaf file into the vehicle application designer software. From there you can 
 * select tools->export to vehicle controller. This will compile the PCL software -> check 
 * the output on the bottom window. To transfer the files to snowwhite select (in 
 * application designer): file-> transfer... connect to 192.168.100.100 (yes, make sure 
 * you're IP is set up to the 192.168.100.x network) and update the system -> click OK, 
 * during this clicking you could check that the dates fits with the current date / time 
 * you did the compilation. It will take some time and then the system will reboot. 
 * 
 *
 * @subsection pSoftKvaser From Kvaser
 * - Kvaser CAN King: monitor the CAN bus, send arbitrary messages.
 */


/**
 * @page pIntegration Integration with other modules
 *
 * This page contains links to the methods, which communicate with other modules.
 * Interfaces are described in comments of these methods.
 *
 * @section pModuleOut Output of the control module
 *  - In simulation:
 *      - sending commands:  commandSender#send;
 *      - trajectory evaluation:  trajectoryEvaluator#report.
 *  - On a car:
 *      - trajectory evaluation:  trajectoryEvaluator#report.
 *
 * Messages from trajectory evaluator are send periodically and can be used as a heartbeat
 * signal. The delay between two messages depends on the number of trajectories and time
 * required for solution of a QP. The minimal delay is equal to the control sampling time.
 * 
 * \n
 *
 * @section pModuleIn Input of the control module
 *  - In simulation:
 *      - current state of a car:  ROSSensorSubscriber#readLoop;
 *      - trajectory:  trajectoryReader#processTrajectoryChunk;
 *      - various commands: trajectoryReader#actOnCommand;
 *      - heartbeat signal: trajectoryReader#checkActivity.
 *  - On a car:
 *      - trajectory:  trajectoryReader#processTrajectoryChunk;
 *      - various commands: trajectoryReader#actOnCommand.
 *      - heartbeat signal: trajectoryReader#checkActivity.
 *
 * \n
 *
 * @section pIntegrationLogic Logic of interaction
 *
 *  - Transmission of a trajectory
 *      - The sequence number of the first chunk of a trajectory must be equal to 0.
 *      - All following chunks must be submitted in the same order as they appear in
 *      trajectory with appropriately set sequence numbers.
 *      - The last chunk must be explicitly indicated. No chunks are accepted after 
 *      the last one.
 *      - If the sequence number of a newly arrived chunk has been already received, 
 *      the respective trajectory chunk is replaced by the new one. The new chunk may
 *      contain new constraints and trajectory points.
 *
 *  - Activation of tracking
 *      - The tracking can be activated, when at least one chunk of one trajectory has 
 *      been added.
 *      - It is necessary to activate a particular trajectory with a respective command.
 *      - Then the time, when tracking must start, should be set using a special command.
 *
 *  - Transmission of trajectories after termination
 *      - Whenever tracking is stopped due to an error or successfuly completed all
 *      trajectories are removed.
 *      - If the goal is reached successfully, the controller enters the initial state 
 *      and transmission of new trajectories can be started at once.
 *      - If there was an error, the controller must be switched to a normal state with
 *      a special command explicitly. Otherwise, tracking would not start.
 */


// ===================================================================================
// ===================================================================================
// ===================================================================================


/**
 * @page pInteraction Interaction with the onboard controller
 *
 * There are two methods to communicate with the onboard controller in order
 * to send commands and receive sensor readings. The first one is telnet connection
 * via Ethernet cable to Navserver, which runs on the controller. This connection
 * can be used only to read sensor data. The second one is the CAN bus, which can
 * be used for sending commands as well. However, the CAN bus baudrate is rather
 * low, and there is likely to be high delay in receiving the messages from onboard 
 * controller due to high load on the bus.
 *
 * Refer to one of these pages for more information on CAN:
 * - @ref pCANOverview
 *      - @ref pCANOverviewCAN
 *      - @ref pCANOverviewCANopen
 * - @ref pCANFormat
 *      - @ref pCANSend
 *      - @ref pCANRecv
 *      - @ref pCANStandard
 *
 * \n
 *
 * @section pCANCable CAN connection
 * We use a USB cable to connect to the CAN bus on SnowWhite. The cable is produced by Kvaser.
 *  - http://www.kvaser.com/en/developer/canlib.html
 *  - http://www.kvaser.com/canlib-webhelp/
 *
 * The Linux version of the driver currently supports only version 2.6 of the kernel and
 * may have problems with SMP, 64bit and real-time priorities.
 * 
 * There is one more option to connect to CAN bus (Abdel has used it under Linux) using
 * hardware and drivers from http://www.peak-system.com/fileadmin/media/linux/index.htm
 *
 * \n
 *
 * @section pIntRecv Reading data from sensors
 * The position of the car is updated each 60 ms.
 *
 *
 * @subsection pIntRecvTelnet Connection to Navserver
 * Connection to the Navserver: address = 192.168.100.100, port = 5432. 
 *
 * Navserver outputs data in a simple textual format. We can get the necessary data by 
 * subscribing for the 'env' and 'state' updates, which are sent each 60ms. 'env' and
 * 'state' readings are printed on separate lines. Both lines are sent in one tcp packet.
 * The interval between reception of two packets varies from 50ms to 70ms. 
 *
 * @todo There is a a significant delay in execution of requested steering angle (or in 
 * sensor reports), @see stateContainer. It would be nice to know more about operation of
 * the onboard controller.
 *
 *
 * @subsection pIntRecvCAN CAN bus
 * The Kvaser CAN cable controller has internal clock, which is used to assign 
 * timestamps to messages. The precision of these stamps is 100 microseconds. 
 * The clock starts after activation of the bus. 
 *
 * Sample dump from the CAN bus (old version of the SnowWhite firmware):
 * @verbatim
Channel   ID    Flag   DLC D0  1   2   3   4   5   6   D7      Time     Direction
---------------------------------------------------------------------------------
 0         524         2   0   0                               0.021390 R
 0         779         7   0   0   0   0   0   0   0           0.021900 R
 0         780         7   0   0   0   0   0   0   0           0.022410 R
 0         128         0                                       0.022670 R
 0         515         8   4   0 128   0   0   0   0   0       0.023180 R
 0         652         8   0   0   0   0   0   0   0   0       0.023820 R
 0         387         3  31   0   0                           0.024080 R
 0         395         7   0   1   0   0   0   0   0           0.024780 R
 0         396         7  32  60   0   0   0   0   0           0.025230 R
 0         651         8   0   0   0   0   0   0   0   0       0.025740 R
 0         907         6   0   0   0   0   0   0               0.026120 R
 0        1163         6   0   0   0   0   0   0               0.026570 R
 0        1803  R      1                                       0.041100 R
 0        1804  R      1                                       0.041360 R
 0        1803         1   5                                   0.041800 R
 0        1804         1   5                                   0.042000 R
 0         523         2 128   2                               0.081160 R
 0         524         2   0   0                               0.081420 R
 0         779         7   0   0   0   0   0   0   0           0.081930 R
 0         780         7   0   0   0   0   0   0   0           0.082440 R
 0         128         0                                       0.082700 R
 0         515         8   4   0 128   0   0   0   0   0       0.083280 R
 0         652         8   0   0   0   0   0   0   0   0       0.083790 R
 0         387         3  31   0   0                           0.084110 R
 0         395         7   0   1   0   0   0   0   0           0.084750 R
 0         396         7  32  60   0   0   0   0   0           0.085200 R
 0         651         8   0   0   0   0   0   0   0   0       0.085710 R
 0         907         6   0   0   0   0   0   0               0.086160 R
 0        1163         6   0   0   0   0   0   0               0.086540 R
 0         523         2 128   2                               0.141200 R
 0         524         2   0   0                               0.141450 R
 0         779         7   0   0   0   0   0   0   0           0.141960 R
 0         780         7   0   0   0   0   0   0   0           0.142480 R
 0         128         0                                       0.142730 R
 0         515         8   4   0 128   0   0   0   0   0       0.143310 R
 0         387         3  31   0   0                           0.143760 R
 0         395         7   0   1   0   0   0   0   0           0.144780 R
 0         396         7  32  60   0   0   0   0   0           0.145230 R
 0         651         8   0   0   0   0   0   0   0   0       0.145740 R
 0         652         8   0   0   0   0   0   0   0   0       0.146190 R
 0         907         6   0   0   0   0   0   0               0.146640 R
 0        1163         6   0   0   0   0   0   0               0.147020 R
 0         523         2 128   2                               0.201100 R
 0         524         2   0   0                               0.201420 R
 0         779         7   0   0   0   0   0   0   0           0.201930 R
 0         780         7   0   0   0   0   0   0   0           0.202440 R
 0         128         0                                       0.202700 R
 0         515         8   4   0 128   0   0   0   0   0       0.203210 R
 0         387         3  31   0   0                           0.203790 R
 0         396         7  32  60   0   0   0   0   0           0.204240 R
 0         652         8   0   0   0   0   0   0   0   0       0.204680 R
 0         395         7   0   1   0   0   0   0   0           0.205200 R
 0         651         8   0   0   0   0   0   0   0   0       0.205640 R
 0         907         6   0   0   0   0   0   0               0.206090 R
 0        1163         6   0   0   0   0   0   0               0.206540 R
 0        1803  R      1                                       0.221070 R
 0        1804  R      1                                       0.221320 R
 0        1803         1 133                                   0.221710 R
 0        1804         1 133                                   0.221960 R
   @endverbatim
 *
 * \n
 *
 * @section pIntSend Sending commands
 * The commands can be sent only via CAN bus. Format of the respective CAN 
 * messages is described in @ref pCANFormat.
 * 
 * The sampling time of the internal control loop is 20 ms. The execution of 
 * received commands begins on the next iteration of the internal control 
 * loop (20 ms). The flags in the last CAN message with the commands determine
 * in how many loops the commands are reused.
 *
 * \n
 *
 * @section pIntSync Synchronization
 * There are several synchronization options.
 *
 * In the first case the MPC controller starts when the data is received from
 * connection to navserver (see test_01_telnet.cpp).
 *
 * In the second case MPC controller is synchronized with CAN messages (id = 128)
 * since we cannot synchronize with messages with sensor readings. The sensor 
 * readings are obtained from the CAN messages. This approach is more difficult 
 * to implement.
 *
 * In the third case MPC controller is also synchronized with CAN messages, 
 * however data is obtained from Navserver. This approach is more difficult to 
 * implement than the first one. The delay between the reception of the data and 
 * the start of the controller is bigger than in the first case.
 */


/**
 * @page pCANOverview Controller area network (CAN) and CANopen overview
 *
 * @section pCANOverviewCAN Controller area network (CAN)
 *
 * http://en.wikipedia.org/wiki/Controller_area_network
 * 
 * CAN bus (for controller area network) is a vehicle bus standard designed 
 * to allow microcontrollers and devices to communicate with each other 
 * within a vehicle without a host computer. CAN bus is a message-based 
 * protocol, designed specifically for automotive applications but now also 
 * used in other areas such as industrial automation and medical equipment.
 *
 * CAN is a multi-master broadcast serial bus standard for connecting electronic 
 * control units (ECUs). Each node is able to send and receive messages, but not 
 * simultaneously. A message consists primarily of an ID (identifier), which 
 * represents the priority of the message, and up to eight data bytes.
 *
 * If the bus is free, any node may begin to transmit. If two or more nodes 
 * begin sending messages at the same time, the message with the more dominant 
 * ID (which has more dominant bits, i.e., zeroes) will overwrite other nodes' 
 * less dominant IDs. A CAN message that is transmitted with highest priority 
 * will succeed, and the node transmitting the lower priority message will 
 * sense this and back off and wait.
 *
 * Message IDs must be unique on a single CAN bus, otherwise two nodes would 
 * continue transmission beyond the end of the arbitration field (ID) causing 
 * an error.
 * 
 * \n
 *
 *
 * @section pCANOverviewCANopen CANopen
 *
 * http://en.wikipedia.org/wiki/Canopen
 *
 * Note, that car controller uses CANopen - a communication protocol build on the
 * top of CAN. Some of the features of CANopen, which are used on the car are 
 * desribed here.
 *
 * The Process Data Object (PDO) protocol is used to process real time data among 
 * various nodes. You can transfer up to 8 bytes (64 bits) of data per one PDO 
 * either from or to the device. One PDO can contain multiple object dictionary 
 * entries and the objects within one PDO are configurable using the mapping and 
 * parameter object dictionary entries.
 *
 * The Heartbeat protocol is used to monitor the nodes in the network and verify 
 * that they are alive. A heartbeat producer (usually a slave device) periodically 
 * sends a message with the binary function code of 1110 and its node ID 
 * (COB-ID = 0x700 (1800) + node ID). The data part of the frame contains a byte indicating 
 * the node status. The heartbeat consumer reads these messages. If the messages 
 * fail to arrive within a certain time limit (defined in the object dictionary of 
 * the devices) the consumer can take action to, for example, reset the device or 
 * indicate an error.
 *
 * http://www.can-cia.org/index.php?id=206
 *
 * Synchronization protocol. The Sync Object is implemented with the synchronization 
 * device acting as the producer of that object. In order to guarantee timely access 
 * to the CAN bus the Sync Object is given a very high priority identifier. CANopen 
 * suggests the identifier 128 (0x80) which is a value of the highest priority group 
 * used in the pre-defined connection set. Within the Sync object no application data are 
 * transported. By default, the Sync Object does not carry any data.
 */


/**
 * @page pCANFormat Format of the CAN messages
 *
 * NodeID = 30 (in the old version NodeID = 5)
 *
 * @section pCANSend Messages to the onboard controller
 * @verbatim
PDO No  CobID org       CobID Hex   CobID Dec
1       0x180+NodeID    19E         414

SymbolName   DataType   Offset  Max/Min       Description
SetSpeed     INTEGER16  0                     Speed of the steering wheel (mm/s?)
SetAngle     INTEGER16  16      1.5/-1.5 Rad  Steering angle (centidegree)
Counter      UNSIGNED8  32                    If the counter is not updated, 
                                              the command is not executed
Override     BOOLEAN    40      -             If set, the commands are executed 4x20ms 
                                              and then are set to 0, otherwise they are 
                                              repeated until new commands are received
DriveEnable  BOOLEAN    41      -             Execute velocity command
SteerEnable  BOOLEAN    42      -             Execute steering command
--           BOOLEAN    43                    \
--           BOOLEAN    44                     |
--           BOOLEAN    45                     | Not used
--           BOOLEAN    46                     |
--           BOOLEAN    47                     |
Spare16      INTEGER16  48                    /
   @endverbatim
 *
 * \n
 * 
 * @section pCANRecv Messages from the onboard controller
 * The following messages were added for us:
 * @verbatim
PDO No  CobID org       CodID Hex   CodID Dec
1       0x200+NodeID    21E         542

SymbolName  DataType   Offset  Max/Min  Description
PositionX   INTEGER32  0                Position of the reference point (mm)
PositionY   INTEGER32  32               Position of the reference point (mm)


PDO No  CobID org       CodID Hex   CodID Dec
2       0x300+NodeID    31E         798

SymbolName      DataType   Offset  Max/Min  Description
PositionAngle   UNSIGNED16 0                Orientation of the reference point (centidegree)
NavigationLevel INTEGER8   16               
PositionCOunter INTEGER8   24               
Enc1Angle       INTEGER16  32               Steering angle (centidegree)
Enc2Speed       INTEGER16  48               Speed of the steering wheel (mm/s)
   @endverbatim
 *
 * The following messages are standard:
 * @verbatim
(NodeID = 11)
PDO No  CobID org       CodID Hex   CodID Dec
1       0x480+NodeID    48B         1152/1163

SymbolName  DataType   Offset  Max/Min  Description
Enc2Dist    INTEGER16  0                ?
Enc2Speed   INTEGER16  16               Speed of the steering wheel (mm/s)

PDO No  CobID org      CodID Hex   CodID Dec
2       0x380+NodeID   38B         896/907

SymbolName  DataType   Offset  Max/Min  Description
Enc1Angle   INTEGER16  0                Steering angle (centidegree)
   @endverbatim
 *
 * \n
 * 
 * @section pCANStandard Standard CANopen messages
 *
 * Heartbeat protocol (see @ref pCANOverviewCANopen) -- 0x700 + NodeID.
 *
 * Synchronization protocol (see @ref pCANOverviewCANopen) -- 0x80.
 */


// ===================================================================================
// ===================================================================================
// ===================================================================================



#endif /*DOXYGEN_H*/
