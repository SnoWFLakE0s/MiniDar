/* RADAR CONTROLLER FIRMWARE v.2022.3.29.b
    This is code written for the Arduino, designed to simultaneously control a servo,
    a laser rangefinder (IR TOF laser distance sensor), and display the info to a IPS
    display (in this case the ST7789). A servo with the LRF module attach to it sweeps
    in an arc, making distance measurements along each point of the sweep. This is put
    onto the display in a PPI (plan-position indicator) grid.

    The range of scanning, field of view, scanning resolution, and grid increments are
    fully adjustable. They have been defined in the earlier section of the code for ease
    of access to any user wishing to change them.
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>
#include <Servo.h>
#include <DFRobot_LIDAR07.h>

#define USE_IIC
#ifdef USE_IIC
DFROBOT_LIDAR07_IIC RDR_LRF;
#endif

//DISPLAY CONSTS
#define TFT_DC    4
#define TFT_RST   2
#define SCR_WD   240
#define SCR_HT   240

//OBJECT DECLARATIONS
Arduino_ST7789 RDR_DISP = Arduino_ST7789(TFT_DC, TFT_RST);
Servo RDR_SERV;

/* VARIABLES
    =========================================================================
*/

//Servo Properties
int SERVOPIN = 3; // servo output PWM pin
int ANGLERNG = 60; // search angle FOV, degrees
float ANGLEMIN = - PI / 6; // search angle lower limit (angle from center, CW)
float ANGLEMAX = PI / 6; // search angle upper limit (angle from center, CW)
const int PPS = 45; // points per scan/sweep
float ANGLEINC = (ANGLERNG * (PI / 180)) / float(PPS); // increment angle, in radians, for servo per scan
int pos_low = 651.31 * ANGLEMIN + 474.188; // lower bound for servo attach
int pos_high = 651.31 * ANGLEMAX + 474.188; // upper bound for servo attach

//LIDAR Properties
float SRCRNG = 5; // search range, meters
int RINGS = 5; // amount of screen rings
int distanceMM; // distance in millimeters
float distanceM; // distance in meters

//Display Properties
int C_y = SCR_HT / 2; // center of the x-axis on display
float dataPoints[PPS]; // data storage array for the points plotted by LRF
float targetPoints[PPS][3]; // data stoarge array for the targets identified
int scanID = 0;

/* FUNCTIONS
    =========================================================================
*/

// Function to set up and initialize comms with all modules
int RadarSetup() {

  // Set up LIDAR module
  uint32_t version;
#ifdef USE_IIC
  RDR_LRF.begin();
  while (!RDR_LRF.begin()) {
    delay(500);
  }
#endif
  Serial.println("LRF INIT COMPLETE");

  // Set up servo motor
  RDR_SERV.attach(SERVOPIN, pos_low, pos_high);
  Serial.println("SERVO INIT COMPLETE");

  // Set up screen display
  RDR_DISP.init(SCR_WD, SCR_HT);
  RDR_DISP.fillScreen(BLACK);
  Serial.println("DISP INIT COMPLETE");
}

// Function to draw screen UI elements
int RadarInterface() {
  RDR_DISP.setRotation(2);
  RDR_DISP.setCursor(0, 213);
  RDR_DISP.setTextColor(GREEN);
  RDR_DISP.print("RANGE ");
  RDR_DISP.setTextColor(BLACK, GREEN);
  RDR_DISP.print(SRCRNG);
  RDR_DISP.setTextColor(GREEN);
  RDR_DISP.println(" M");
  RDR_DISP.print("RINGS ");
  RDR_DISP.setTextColor(BLACK, GREEN);
  RDR_DISP.print(SRCRNG / RINGS);
  RDR_DISP.setTextColor(GREEN);
  RDR_DISP.println(" M");
  RDR_DISP.print("SWEEP ");
  RDR_DISP.setTextColor(BLACK, GREEN);
  RDR_DISP.print(ANGLERNG);
  RDR_DISP.setTextColor(GREEN);
  RDR_DISP.println(" DEG");
}

// Function to draw grid for data returns
int RadarGrid() {
  RDR_DISP.setRotation(1);
  // Arc drawing function
  for (int r = SCR_WD; r > 0; r = r - SCR_WD / RINGS) {
    for (float i = ANGLEMIN; i < ANGLEMAX; i = i + 2 / float(r)) {
      RDR_DISP.drawPixel(r * cos(i), C_y + r * sin(i), GREEN); // Draws an arc with radius "r"
    }
  }
  //Line drawing
  RDR_DISP.drawLine(0, C_y, SCR_WD * cos(ANGLEMIN), C_y + SCR_WD * sin(ANGLEMIN), GREEN); // Line from left center (origin) to right extreme (low)
  RDR_DISP.drawLine(0, C_y, SCR_WD * cos(ANGLEMAX), C_y + SCR_WD * sin(ANGLEMAX), GREEN); // Line from left center (origin) to right extreme (high)
}

// Function to perform LIDAR measurement
int GetRange() {
  while (!RDR_LRF.startMeasure()) {
    distanceMM = 0;
  }
  distanceMM = RDR_LRF.getDistanceMM();
  distanceM = float(distanceMM) / 1000;
}

// Function to control servo from center. Update servo object name as needed
int writeFromCenter(float radians) {
  // Given angle, rotate CW [x] radians from the centerpoint
  int center_offset = 1525; // servo center offset
  int microSeconds = 651.31 * radians;
  int dirConst = 1; // (+-)1 depending on servo face direction
  RDR_SERV.writeMicroseconds(center_offset - dirConst * microSeconds);
  delay(ANGLEINC / 0.0104719755); // Angle increment divided by the servo speed (rad/ms) to allow for maximal scan speed
}

// Function to sweep left (in direction of positive theta)
int RDR_LEFTSCAN() {
  scanID = 0;
  for (float i = ANGLEMIN; i < ANGLEMAX; i += ANGLEINC) {
    float prev_i;

    // Move and get data
    writeFromCenter(i); // turn servo to desired angle
    GetRange(); // get LRF measurement

    // Display sweep line
    RDR_DISP.drawLine(0, C_y, SCR_WD * cos(i), C_y + SCR_WD * sin(i), GREEN); // Draw sweep line from center to terminal point
    if (i > ANGLEMIN) {
      RDR_DISP.drawCircle((SCR_WD * (dataPoints[scanID] / SRCRNG)) * cos(i), C_y + (SCR_WD * (dataPoints[scanID] / SRCRNG)) * sin(i), 2, BLACK); // delete the plotted point previously at this sweep position
      RDR_DISP.drawLine(0, C_y, SCR_WD * cos(prev_i), C_y + SCR_WD * sin(prev_i), BLACK); // erase previous sweep line
    }
    if (i < ANGLEMIN + 2 * ANGLEINC) {
      RDR_DISP.drawLine(0, C_y, SCR_WD * cos(ANGLEMIN), C_y + SCR_WD * sin(ANGLEMIN), GREEN); // manual fix to restore side line
    }

    // Plot points on grid
    RDR_DISP.drawCircle((SCR_WD * (distanceM / SRCRNG)) * cos(i), C_y + (SCR_WD * (distanceM / SRCRNG)) * sin(i), 2, GREEN);
    dataPoints[scanID] = distanceM; // adds current point to the data array
    //RadarFeed();

    prev_i = i;
    scanID++;

  }

  RadarGrid();
}

// Function to sweep right (in direction of negative theta)
int RDR_RIGHTSCAN() {
  scanID = PPS;
  for (float i = ANGLEMAX; i > ANGLEMIN; i -= ANGLEINC) {
    float prev_i;

    // Move and get data
    writeFromCenter(i); // turn servo to desired angle
    GetRange(); // get LRF measurement

    // Display sweep line
    RDR_DISP.drawLine(0, C_y, SCR_WD * cos(i), C_y + SCR_WD * sin(i), GREEN); // Draw sweep line from center to terminal point
    if (i < ANGLEMAX) {
      RDR_DISP.drawCircle((SCR_WD * (dataPoints[scanID] / SRCRNG)) * cos(i), C_y + (SCR_WD * (dataPoints[scanID] / SRCRNG)) * sin(i), 2, BLACK); // delete the plotted point previously at this sweep position
      RDR_DISP.drawLine(0, C_y, SCR_WD * cos(prev_i), C_y + SCR_WD * sin(prev_i), BLACK); // erase previous sweep line
    }
    if (i > ANGLEMAX - 2 * ANGLEINC) {
      RDR_DISP.drawLine(0, C_y, SCR_WD * cos(ANGLEMAX), C_y + SCR_WD * sin(ANGLEMAX), GREEN);
    }

    // Plot points on grid
    RDR_DISP.drawCircle((SCR_WD * (distanceM / SRCRNG)) * cos(i), C_y + (SCR_WD * (distanceM / SRCRNG)) * sin(i), 2, GREEN);
    dataPoints[scanID] = distanceM; // adds current point to the data array
    //RadarFeed();

    prev_i = i;
    scanID--;

  }

  RadarGrid();
}

int IdentifyTargets() {

  float sizeThreshold = 0.05; // how far apart data points can be before it is considered a separate entity, in meters
  
  int counter = 0;
  float prevDistance = 0;
  
  float angleInitial;
  float angleFinal;
  float distanceSum;
  float distanceAvg;

  int targetCount = 0;

  // routine to find targets
  for (int i = 0; i < PPS; i++) {

    /* 
     * Compare previous distance measurement to current. 
     * If the difference remains within the defined target distinguishing threshold, consider it the same entity.
     * Average the distances to get the "average" distance of the object.
    */
    if (abs(dataPoints[i] - prevDistance) < sizeThreshold) {
      counter++; // add to the counter of number of datapoints
      distanceSum += dataPoints[i]; // add distance value to the sum
    } 

    /* 
     * When previous datapoint is significantly different from the previous, consider it the new object.
     * Get the center angle point, the average distance, and draw a red circle at that location.
     * Reset all storage variables after finished.
    */
    else if ((abs(dataPoints[i] - prevDistance) > sizeThreshold) && (counter != 0)) {
      targetCount++;
      
      // calculates the average distance, in meters, of all distance points accrued by object
      distanceAvg = distanceSum / float(counter); 
      
      // calculate the initial and final angle of the object
      angleInitial = ANGLEMIN + ANGLEINC * (i - counter);
      angleFinal = ANGLEMIN + ANGLEINC*i;
      // calculate the angular midpoint of the object
      float angleAvg = (angleInitial + angleFinal)/2;

      // calculate the cartesian coordinates of the object
      float coord_x = (SCR_WD * (distanceAvg / SRCRNG)) * cos(angleAvg);
      float coord_y = C_y + (SCR_WD * (distanceAvg / SRCRNG)) * sin(angleAvg);
      float targetSize = SCR_WD * ((2*PI*distanceAvg*(abs(angleAvg)/(2*PI)))/SRCRNG);
      // add into target storage array
      targetPoints[targetCount][0] = coord_x;
      targetPoints[targetCount][1] = coord_y;
      targetPoints[targetCount][2] = targetSize;
      
      Serial.print("Object detected at ");Serial.print(coord_x);Serial.print(", ");Serial.print(coord_y);Serial.print(", with size of ");Serial.println(targetSize);
      if (distanceAvg < SRCRNG) {
        RDR_DISP.drawCircle(coord_x, coord_y, targetSize, RED);
      }
      // Reset storage variables
      counter = 0;
      distanceSum = 0;
    } 
    
    prevDistance = dataPoints[i];
  }
}

int ClearTargets() {
  for (int i = 0; i < PPS; i++) {
    RDR_DISP.drawCircle(targetPoints[i][0], targetPoints[i][1], targetPoints[i][2], BLACK);
  }
}

/* EXECUTION
    =========================================================================
*/

void setup()
{
  Serial.begin(9600);
  RadarSetup();
  RadarInterface();
  RadarGrid();
}

void loop()
{
  RDR_LEFTSCAN();
  ClearTargets();
  IdentifyTargets();
  RDR_RIGHTSCAN();
  ClearTargets();
  IdentifyTargets();
}