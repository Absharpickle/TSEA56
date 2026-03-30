// ==========================================
// 1. CONFIGURATION & STATE
// ==========================================
DEFINE I2C_ADDR_DRIVE  = 0x10
DEFINE I2C_ADDR_SENSOR = 0x11

GLOBAL SYSTEM_MODE = AUTONOMOUS   // Can be AUTONOMOUS or MANUAL
GLOBAL MISSION_STATE = IDLE       // Can be IDLE, DRIVING_TO_TARGET, RETURNING
GLOBAL CURRENT_ROUTE = []         // List of upcoming commands (e.g., ['f', 'r', 'l', 's'])
GLOBAL ROUTE_INDEX = 0            // Where we are in the route list

// ==========================================
// 2. HARDWARE ABSTRACTION (I2C)
// ==========================================
FUNCTION Send_Drive_Command(Command):
    // As the I2C Master, push a byte to the Slave
    Write_I2C_Byte(I2C_ADDR_DRIVE, Command)

FUNCTION Get_Sensor_Data():
    // As the I2C Master, request data from the Slave
    Data = Read_I2C_Struct(I2C_ADDR_SENSOR)
    RETURN Data (Obstacle_Detected, At_Intersection, Line_Error)

// ==========================================
// 3. PATHFINDING & PLANNING
// ==========================================
FUNCTION Calculate_Mission(Target_Node):
    // Use Breadth-First Search to find shortest path on the map
    Path_Nodes = BFS_Shortest_Path(Current_Node, Target_Node)
    
    // Translate nodes into physical motor commands based on compass direction
    CURRENT_ROUTE = Convert_Nodes_To_Turns(Path_Nodes)
    ROUTE_INDEX = 0
    
    MISSION_STATE = DRIVING_TO_TARGET
    Send_Drive_Command('f') // Tell ATmega to start moving forward

// ==========================================
// 4. AUTONOMOUS BEHAVIOR
// ==========================================
FUNCTION Execute_Autonomous_Step():
    IF MISSION_STATE == IDLE:
        RETURN // Nothing to do
        
    // 1. Listen to the Nervous System (Sensors)
    Sensors = Get_Sensor_Data()

    // 2. Safety First: Obstacle Check
    IF Sensors.Obstacle_Detected == TRUE:
        Send_Drive_Command('s') // Emergency Stop
        RETURN

    // 3. Navigation: Are we at the end of the route?
    IF CURRENT_ROUTE[ROUTE_INDEX] == 'END_OF_ROUTE':
        Send_Drive_Command('s') // Stop exactly at the target
        MISSION_STATE = IDLE
        Trigger_Robotic_Arm_Pickup()
        RETURN

    // 4. Navigation: Handle Intersections
    IF Sensors.At_Intersection == TRUE:
        // Get the next pre-calculated turn ('f', 'r', 'l')
        Next_Turn = CURRENT_ROUTE[ROUTE_INDEX]
        Send_Drive_Command(Next_Turn)
        ROUTE_INDEX = ROUTE_INDEX + 1 // Advance to the next step
    
    // If no intersection and no obstacle, the ATmega just keeps following the line.

// ==========================================
// 5. MANUAL OVERRIDE BEHAVIOR
// ==========================================
FUNCTION Execute_Manual_Step():
    // Camera feed bypasses this loop directly to PC
    
    Input = Read_PC_Network_Socket()
    IF Input is Valid_Joystick_Command:
        Send_Drive_Command(Input.Translated_Command)

// ==========================================
// 6. MAIN SYSTEM LOOP
// ==========================================
FUNCTION Main():
    Initialize_I2C_Bus()
    Initialize_Map_Grid()
    
    // Example: Computer says "Go pick up item at Node 12"
    Calculate_Mission(Target_Node = 12)

    WHILE System_Power_Is_On:
        
        // 1. Check for high-level mode switches from PC
        IF PC_Sends("SWITCH_TO_MANUAL"):
            SYSTEM_MODE = MANUAL
            Send_Drive_Command('s') // Safety halt on switch
            
        // 2. Branch logic based on current mode
        IF SYSTEM_MODE == AUTONOMOUS:
            Execute_Autonomous_Step()
        ELSE IF SYSTEM_MODE == MANUAL:
            Execute_Manual_Step()
            
        // Wait 10ms to prevent the Pi's CPU from running at 100%
        Sleep(10 milliseconds)