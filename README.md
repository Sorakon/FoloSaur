# FoloSaur
A line following bulky robot...


  
________________________________________
1. Introduction
A line-following robot is a type of autonomous robotic vehicle that has the ability to detect and follow a specific path or line that is usually visible on the ground. Most commonly, this path is a black line on a white surface, but it can also be a white line on a black surface depending on the configuration of the sensors and design. The robot follows the line without any manual guidance or external control. It uses infrared (IR) sensors to differentiate between the line and the background surface. These sensors detect the contrast between black and white surfaces by sensing the amount of infrared light reflected back to them.
The concept of line-following robots is one of the most basic and widely implemented projects in robotics, especially in educational and research environments. Despite being relatively simple in terms of hardware and software, the underlying principles introduce students and developers to key areas such as autonomous navigation, real-time decision making, and sensor-based control systems.
The robot works by continuously reading data from multiple IR sensors placed at the front of the chassis. These readings are processed by a microcontroller, such as an Arduino Uno, which decides the direction of movement based on which sensors detect the line. The microcontroller then sends appropriate signals to a motor driver, which in turn controls the direction and speed of the robot’s wheels using DC motors.
The significance of this project extends beyond academic learning. Line-following robots represent a foundational model for more complex systems used in various real-life applications, such as:
•	Industrial automation for material handling and internal logistics,
•	Warehouse robots that follow set paths to move goods,
•	Self-driving vehicle algorithms, where lane detection is a crucial function,
•	Medical transport bots used in hospitals to deliver medicines or lab samples autonomously.
This particular project was developed as part of the engineering curriculum at Modi Institute of Technology, with the goal of combining practical hands-on experience in electronics with theoretical knowledge in programming and circuit design. Through this project, students gain a deep understanding of how electronic sensors, actuators, and embedded systems work together to create a fully functional automated machine.
Moreover, working on such a project helps in building core competencies in problem-solving, debugging, hardware-software integration, and system design—skills that are essential for anyone pursuing a career in robotics, automation, or embedded systems.
In conclusion, the line-following robot project is a stepping stone to advanced robotics. It introduces the core ideas in a manageable scope and provides a strong platform for learning and future innovation.
________________________________________


2. Objectives
The line-following robot project is not only an engaging hands-on task but also a practical way to apply several core concepts of electronics and programming. This section outlines the primary goals and learning outcomes that this project aims to achieve:
•	To design and build an autonomous robot capable of following a black line on a white surface using IR sensors:
The fundamental objective is to construct a functional robotic vehicle that can independently detect and follow a specific path. This involves understanding how IR sensors operate, how they detect contrasts between black and white surfaces, and how this data can be used to guide the robot's movement.
•	To gain hands-on experience with Arduino microcontrollers and basic robotics:
Through this project, students are introduced to the Arduino Uno microcontroller, a powerful yet beginner-friendly platform for embedded system development. Learners get practical exposure to writing Arduino code, uploading programs, testing sensors, and controlling motors, which strengthens their grasp of embedded programming and robotics control logic.
•	To understand how sensor inputs are used for decision-making in embedded systems:
One of the key goals is to help students learn how microcontrollers interpret real-world inputs through sensors. The robot makes decisions based on IR sensor readings—such as whether to turn left, right, or continue forward—which simulates the process of environmental perception and response in autonomous systems.
•	To apply logical thinking and control strategies through programming to manage hardware behaviour effectively:
Students must write control logic in the Arduino IDE that interprets sensor data and determines motor actions. This involves conditional programming, understanding control structures like if statements, and using functions like digitalRead(), digitalWrite(), and analogWrite() to manage outputs. It teaches how software logic is translated into physical actions.
•	To promote system integration skills by combining electronic, mechanical, and software components:
The project helps develop interdisciplinary skills by combining mechanical assembly (chassis and motors), electronics (circuits and sensors), and software (Arduino programming). It encourages a holistic understanding of how complete systems are designed and integrated.
•	To encourage experimentation, debugging, and iterative improvement:
Students are motivated to experiment with different sensor placements, speed settings, and logic flows. Through trial and error, they learn the importance of testing, calibration, and debugging—critical skills in engineering.
•	To build a foundation for advanced robotics and automation concepts:
Finally, this project lays the groundwork for more complex robotics topics such as obstacle avoidance, PID control, path planning, and machine learning-based navigation. It serves as a stepping stone for higher-level projects in industrial automation or AI-powered robotics.
________________________________________

3. Components Used

Component	Quantity	Description
Arduino Uno	1	Main microcontroller board used to control the robot
L298N Motor Driver Module	1	Used to control the direction and speed of DC motors
5-Channel IR Sensor Module	1	Detects black or white surface under each sensor
BO Motors (DC Geared Motor)	2	Drives the wheels
Wheels	2	Attached to the motors
Chassis Frame	1	Holds all components together
Battery Pack (9V or 12V)	1	Provides power to the motors and Arduino
Jumper Wires	As needed	For connections between components
Breadboard (optional)	1	For prototyping and testing connections
________________________________________





4. Circuit Diagram:
 
The robot's control system consists of connections between the Arduino Uno, motor driver, motors, and the 5-channel IR sensor module.
Pin Configuration:
•	IR Sensors (A0 - A4): Connected to analogue pins A0 through A4 on Arduino
•	Motors:
o	Motor 1 (Right): Controlled using pins 4 and 5; speed via pin 9
o	Motor 2 (Left): Controlled using pins 2 and 3; speed via pin 10
•	Power Supply:
o	Arduino is powered through USB or a 9V battery
o	Motor driver powered using a separate battery pack (9V/12V)

Each IR sensor has three connections: VCC (power), GND (ground), and signal (S1–S5). These signal pins go into the Arduino and are used to determine the colour beneath the sensor.
________________________________________



5. Working Principle 
The fundamental working principle of a line-following robot revolves around its ability to detect and react to visual cues on the ground using infrared (IR) sensors. The robot continuously scans the surface beneath it using a set of strategically placed IR sensors, and then makes decisions about its movement based on the pattern of sensor readings.
Infrared Sensor Behaviour
IR sensors operate on the principle of light reflection:
•	White surfaces reflect infrared light strongly. When an IR sensor is placed over a white surface, it receives a significant amount of reflected infrared light, and the sensor output is HIGH (1).
•	Black surfaces absorb most of the infrared light. When a sensor is over a black surface (such as the guiding line), very little light is reflected back, and the sensor outputs a LOW (0).
By placing multiple IR sensors side-by-side (typically 5 sensors), the robot gains a field of vision that can detect where the line is in relation to the robot’s current position.
________________________________________
Sensor Arrangement and Data Interpretation
In this project, a 5-channel IR sensor module is used, with the sensors aligned horizontally in front of the robot and facing downward. These are labelled as:
•	S1: Leftmost sensor
•	S2: Left sensor
•	S3: Centre sensor
•	S4: Right sensor
•	S5: Rightmost sensor
As the robot moves forward, these sensors continuously provide real-time feedback to the Arduino microcontroller. The Arduino reads this data and matches the pattern of black (0) and white (1) readings to predefined rules that determine the direction of movement.
________________________________________
Sensor Output Patterns and Corresponding Actions
Here are some of the common sensor output patterns and the robot’s corresponding behaviour:
Sensor Pattern (S1 S2 S3 S4 S5)	Interpretation	Robot Action
1 1 0 1 1	Robot is centred on the line	Move forward
0 1 1 1 1	Line is on the far left	Sharp right turn
1 1 1 0 0	Line is on the far right	Sharp left turn
1 0 0 1 1	Line slightly left	Slight right adjustment
1 1 0 0 1	Line slightly right	Slight left adjustment
0 0 0 0 0	Robot is fully on the black line	Stop (possible junction or end)
1 1 1 1 1	Robot is off the track	Search/realign
Note: The robot compares the current sensor readings with these patterns inside the main loop of the Arduino code. Based on this, it sends specific commands to the motor driver to either go forward, stop, or turn left/right.
________________________________________
Decision-Making Process
The entire decision-making process is embedded into the Arduino program logic. It follows a loop that:
1.	Continuously reads each sensor’s digital value.
2.	Checks the combination of these readings against known patterns.
3.	Sends appropriate signals to the motors for movement:
o	If the line is centred, both motors move forward at equal speed.
o	If the line is drifting left or right, one motor slows down or stops while the other continues, causing the robot to turn.
o	If no line is detected or all sensors read black, the robot stops to avoid going off-track or misinterpreting a junction.

This reactive mechanism gives the robot the ability to follow the line autonomously in real time, without requiring any preprogrammed path.
________________________________________





6. Software Design

The logic for the robot is implemented using the Arduino IDE, programmed in the C/C++ language. The core functionality revolves around continuously monitoring sensor inputs and making real-time decisions to control the motors, enabling the robot to follow a line smoothly.
The program is organized into four main sections:
A. Initialization
At the start, in the setup() function, the Arduino microcontroller configures the hardware pins. The pins connected to the motors are set as outputs so the program can control their direction and speed. Similarly, the pins connected to the five IR sensors are set as inputs to receive sensor signals. This step is crucial for proper communication between the Arduino and the robot’s hardware components.
B. Sensor Reading
Inside the loop() function, which runs repeatedly, the program continuously reads the states of the five IR sensors using the digitalRead() function. Each sensor detects whether it is over the line or not, typically returning a LOW or HIGH signal depending on the presence of the line (usually a dark line on a lighter surface or vice versa). Reading these values repeatedly allows the robot to keep track of its position relative to the line at all times.
C. Decision Making
Once the sensor values are obtained, the program analyses the combination of these inputs to determine the robot’s next movement. Different sensor patterns correspond to different positions of the robot relative to the line:
•	If the centre sensor detects the line, the robot should move straight forward.
•	If sensors on the left side detect the line, it indicates the robot needs to turn left to realign.
•	Similarly, if sensors on the right side detect the line, the robot should turn right.
•	If no sensors detect the line, the robot might stop or execute a search manoeuvre to relocate the line.
This decision logic is typically implemented using conditional statements (if-else or switch-case) to trigger specific motor commands based on sensor input combinations.
D. Motor Control
After deciding on the action, the robot controls its motors accordingly. Direction control is handled by setting digital pins HIGH or LOW using digitalWrite(). Speed control is achieved with Pulse Width Modulation (PWM) signals sent through the analogWrite() function, allowing smooth adjustments of motor speed. For example, during a turn, one motor may slow down while the other runs at full speed to facilitate a smooth curve. The combination of direction and speed control ensures the robot can accurately follow the line, even when navigating sharp turns or curves.
________________________________________
7. Hardware Implementation
Robot Chassis and Component Assembly
The robot’s chassis serves as a sturdy, flat platform on which all electronic and mechanical components are mounted to form a complete functional system. The layout and arrangement are carefully planned for optimal performance and stability.
________________________________________
Component Placement:
•	Front Side:
The IR sensor array is mounted at the front of the chassis with the sensors facing downward toward the ground. This orientation allows the sensors to effectively detect the line on the surface below, providing real-time feedback about the robot’s position relative to the track.
•	Middle Section:
The Arduino Uno microcontroller board and the L298N motor driver module are securely fixed in the middle area of the chassis. Positioning these components centrally helps in maintaining balance and minimizes the length of wiring required to reach sensors and motors.
•	Rear Side:
The battery pack, which supplies power to the entire system, is attached at the rear of the chassis. Placing the battery here helps balance the overall weight distribution between the front and rear ends.
•	Left and Right Sides:
Two BO (Brushed) motors are mounted on either side of the chassis, each connected to a wheel. These motors are responsible for driving the robot forward, backward, and turning by controlling wheel speeds independently.
________________________________________
Important Assembly Considerations:
•	Balanced Weight Distribution:
Components are positioned strategically to ensure that the robot’s weight is evenly spread across the chassis. This balance prevents tipping and contributes to smooth movement.
•	Secure and Short Wiring:
Wires connecting sensors, motors, and controllers are kept as short as possible and firmly secured. This reduces the risk of loose connections or wire damage during the robot’s motion, which can cause malfunctions.
•	Alignment of Motors and Wheels:
The motors and wheels are carefully aligned to be straight and parallel to each other. Proper alignment ensures the robot travels in a straight path without unwanted drifting, enhancing control and precision.
________________________________________
Final Assembly and Testing:
Once all components are mounted and wired:
1.	Connection Testing:
Each electrical connection is tested individually to verify continuity and correct wiring.
2.	Calibration:
The IR sensors are calibrated for the specific surface and line type to ensure accurate line detection.
3.	Code Upload and Final Testing:
After assembly and calibration, the Arduino code controlling the robot’s logic is uploaded. The robot is then tested on the track to observe its line-following behavior and make any necessary adjustments.
________________________________________
This structured assembly process ensures that the robot’s hardware foundation is reliable and optimized for consistent and precise line-following performance. If you want, I can also help you with tips on calibration or provide troubleshooting advice!
________________________________________
8. Arduino Code Explanation
Complete Code:
#define m1 4  //Right Motor MA1
#define m2 5  //Right Motor MA2
#define m3 2  //Left Motor MB1
#define m4 3  //Left Motor MB2
#define e1 9  //Right Motor Enable Pin EA
#define e2 10 //Left Motor Enable Pin EB

//**********5 Channel IR Sensor Connection**********//
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
//*************************************************//

void setup() {
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
}

void loop() {
  //Reading Sensor Values
  int s1 = digitalRead(ir1);  //Left Most Sensor
  int s2 = digitalRead(ir2);  //Left Sensor
  int s3 = digitalRead(ir3);  //Middle Sensor
  int s4 = digitalRead(ir4);  //Right Sensor
  int s5 = digitalRead(ir5);  //Right Most Sensor

  //if only middle sensor detects black line
  if((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 1) && (s5 == 1))
  {
    //going forward with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, HIGH);
    digitalWrite(m4, LOW);
  }
  
  //if only left sensor detects black line
  if((s1 == 1) && (s2 == 0) && (s3 == 1) && (s4 == 1) && (s5 == 1))
  {
    //going right with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, LOW);
  }
  
  //if only left most sensor detects black line
  if((s1 == 0) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 1))
  {
    //going right with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, HIGH);
  }

  //if only right sensor detects black line
  if((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 0) && (s5 == 1))
  {
    //going left with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, HIGH);
    digitalWrite(m4, LOW);
  }

  //if only right most sensor detects black line
  if((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 0))
  {
    //going left with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, LOW);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, HIGH);
    digitalWrite(m4, LOW);
  }

  //if middle and right sensor detects black line
  if((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 0) && (s5 == 1))
  {
    //going left with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, HIGH);
    digitalWrite(m4, LOW);
  }

  //if middle and left sensor detects black line
  if((s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1))
  {
    //going right with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, LOW);
  }

  //if middle, left and left most sensor detects black line
  if((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1))
  {
    //going right with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, LOW);
  }

  //if middle, right and right most sensor detects black line
  if((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 0) && (s5 == 0))
  {
    //going left with full speed 
    analogWrite(e1, 255); //you can adjust the speed of the motors from 0-255
    analogWrite(e2, 255); //you can adjust the speed of the motors from 0-255
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, HIGH);
    digitalWrite(m4, LOW);
  }

  //if all sensors are on a black line
  if((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0))
  {
    //stop
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, LOW);
  }
}
Below is a simplified explanation of how the Arduino code works:
Pin Definition Section
#define m1 4  // Right Motor MA1
#define m2 5  // Right Motor MA2
#define m3 2  // Left Motor MB1
#define m4 3  // Left Motor MB2
#define e1 9  // Right Motor Enable Pin
#define e2 10 // Left Motor Enable Pin

#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
•	Pins are assigned for motor control and IR sensors.
•	m1-m4 control motor directions.
•	e1 and e2 control motor speed (via PWM).
•	ir1-ir5 read inputs from 5 IR sensors.
Setup Function
void setup() {
  pinMode(m1, OUTPUT); pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT); pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT); pinMode(e2, OUTPUT);
  pinMode(ir1, INPUT); pinMode(ir2, INPUT);
  pinMode(ir3, INPUT); pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
}
•	Sets pin modes: motor pins as OUTPUT, IR sensor pins as INPUT.
Loop Function
The loop() continuously runs and does three things:
1.	Read IR sensor values using digitalRead().
2.	Match sensor pattern to known conditions (e.g., center aligned, veered left/right).
3.	Control motor actions using digitalWrite() and analogWrite().
Example snippet:
if((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 1) && (s5 == 1)) {
  // Move Forward
  analogWrite(e1, 255); analogWrite(e2, 255);
  digitalWrite(m1, HIGH); digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH); digitalWrite(m4, LOW);
}
Each condition block corresponds to a specific pattern of sensor readings and results in a suitable movement (turn or move straight).
________________________________________
9. IR Sensor Functioning
Operation of the 5-Channel IR Sensor Array
The 5-channel infrared (IR) sensor array is a critical component of the robot’s line-following system. Each individual IR sensor functions by emitting infrared light onto the surface beneath the robot and then detecting the amount of reflected light returned. This reflection pattern helps the robot distinguish between the line and the surrounding surface, enabling precise navigation.
________________________________________
How Each IR Sensor Works:
•	Infrared Emission and Reflection:
Every sensor in the array contains an IR LED that continuously emits infrared light downwards towards the ground. Simultaneously, a photodiode or phototransistor in the sensor detects the infrared light that bounces back from the surface.
•	Surface Detection Logic:
o	White Surface Detection:
When the sensor faces a white or light-colored surface, the infrared light is reflected strongly back to the sensor’s photodiode. This results in a HIGH signal (logic 1) being output by the sensor because of the strong reflection.
o	Black Line Detection:
Conversely, when the sensor is positioned over a black or dark-colored line, the surface absorbs most of the IR light, resulting in very little to no reflected light reaching the photodiode. This absence of reflection causes the sensor to output a LOW signal (logic 0).
•	Digital Output:
The sensor outputs a binary digital signal (HIGH or LOW) rather than an analog value, simplifying integration with the Arduino’s digital input pins.
________________________________________
Sensor Placement and Naming Convention:
The IR sensors are arranged in a linear array, mounted at the front of the robot with the following layout and Arduino analog pin connections:
•	S1 – Leftmost Sensor: Connected to analog pin A0
•	S2 – Left Sensor: Connected to analog pin A1
•	S3 – Center Sensor: Connected to analog pin A2
•	S4 – Right Sensor: Connected to analog pin A3
•	S5 – Rightmost Sensor: Connected to analog pin A4
This sequential placement allows the robot to detect the position of the black line relative to its center.
________________________________________
How the Arrangement Helps in Navigation:
•	By continuously reading the five sensors, the robot can determine exactly where the black line lies beneath it. For instance:
o	If only the center sensor (S3) reads LOW, the robot is perfectly aligned with the line and should continue moving straight.
o	If the left sensors (S1 or S2) detect the line (LOW), it indicates the line is drifting towards the left side, prompting the robot to steer left to re-center.
o	Similarly, if the right sensors (S4 or S5) detect the line, the robot knows to steer right.
o	If none of the sensors detect the line (all HIGH), the robot may have lost the track and needs to stop or search.
•	This configuration enables the robot to make nuanced and precise corrections, allowing smooth and accurate line following even on curves or irregular paths.
________________________________________
10. Motor Driver Working
The L298N Motor Driver is a widely used dual H-bridge motor controller module designed to interface easily with microcontrollers like the Arduino. It enables independent control over two DC motors — both in terms of their direction and speed — using just a few input pins. This makes it an ideal component for driving your robot’s left and right motors smoothly and responsively.
________________________________________
How the L298N Controls Motors
1. Direction Control Using Input Pins
•	Each motor is connected to two input pins on the L298N driver (commonly referred to as IN1 and IN2 for motor 1, and IN3 and IN4 for motor 2).
•	By setting these two pins to HIGH or LOW in specific combinations, you control the rotation direction of the motor:
o	Clockwise Rotation:
One input pin is set HIGH and the other LOW (e.g., IN1 = HIGH, IN2 = LOW)
o	Counterclockwise Rotation:
The inputs are reversed (e.g., IN1 = LOW, IN2 = HIGH)
•	If both pins are the same (both HIGH or both LOW), the motor stops (brakes or free runs depending on the wiring).
This binary input pattern controls the H-bridge inside the L298N, which switches the voltage polarity across the motor terminals accordingly.
________________________________________
2. Speed Control Using Enable Pins and PWM
•	Each motor driver channel has an enable pin (EN1 for motor 1, EN2 for motor 2) that controls whether the motor is powered and at what speed.
•	The Arduino uses the analogWrite() function on these enable pins to send Pulse Width Modulation (PWM) signals.
•	PWM rapidly switches the motor’s power ON and OFF at a frequency imperceptible to the motor, effectively controlling the average voltage and thus the motor speed.
•	The PWM value ranges from 0 to 255:
o	255 means 100% duty cycle, i.e., full power → full speed.
o	0 means 0% duty cycle → motor stopped.
By adjusting the PWM duty cycle, the robot can smoothly accelerate, decelerate, or maintain a precise speed.
________________________________________
Practical Benefits for the Robot
Using the L298N motor driver makes it easy for the Arduino to:
•	Drive each motor independently, allowing for differential steering where the left and right wheels can rotate at different speeds or directions.
•	Execute smooth turns by varying motor speeds instead of abrupt stops.
•	Quickly respond to sensor inputs by adjusting motor speed and direction dynamically.
This level of control is essential for a line-following robot to maintain its path accurately, especially when navigating curves or correcting deviations from the line.
________________________________________
15. References
•	Arduino Official Documentation – https://www.arduino.cc
•	IR Sensor Module Datasheet
•	L298N Motor Driver Module Datasheet
•	Tutorials Point and Electronics Hub articles on line-following robots
•	YouTube channels and blogs on Arduino-based robotics
•	Online robotics forums and GitHub repositories for sample codes
________________________________________
16. Acknowledgements
I would like to sincerely thank:
•	My Faculty and Mentors at Modi Institute of Technology for their continuous support, valuable guidance, and encouragement throughout the entire duration of this project. Their expertise and motivation were instrumental in helping me overcome challenges and achieve my goals.
•	My Teammates and Classmates who generously contributed their time and effort during the testing and debugging phases. Their collaboration and constructive feedback greatly improved the quality and success of this project.
•	Online Communities and Forums that offered open-source resources, insightful discussions, and troubleshooting advice. The shared knowledge from these communities played a vital role in solving technical issues and expanding my understanding.
This project would not have been possible without the help, encouragement, and collective effort of all these individuals and groups. I am deeply grateful for their support.
________________________________________
17. Appendices
Appendix A – Arduino Pin Mapping Table
Component	Arduino Pin
Right Motor MA1	4
Right Motor MA2	5
Left Motor MB1	2
Left Motor MB2	3
Right Motor Enable EA	9
Left Motor Enable EB	10
IR Sensor 1 (Leftmost)	A0
IR Sensor 2 (Left)	A1
IR Sensor 3 (Middle)	A2
IR Sensor 4 (Right)	A3
IR Sensor 5 (Rightmost)	A4
________________________________________
Prepared By: [Your Full Name]
Roll Number: [Your Roll No]
Branch: Electronics / Computer Science
College: Modi Institute of Technology
________________________________________
