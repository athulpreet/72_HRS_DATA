#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

// --------------------- SD Card Pin Definitions ---------------------
#define SD_CS   PA4    // CS
#define SD_SCK  PA5    // SCK
#define SD_MISO PA6    // MISO
#define SD_MOSI PA7    // MOSI

// --------------------- UART1 for GPS & Debug -----------------------
//HardwareSerial Serial1(PA10, PA9); // RX=PA10, TX=PA9

// --------------------- GPS Buffer & Timezone -----------------------
char buffer[128];
int bufferIndex = 0;

// --------------------- Initialize RTC -----------------------------
RTC_DS3231 rtc;
char dateTimeStr[20]; // To store formatted date/time from RTC

// --------------------- GPS Data Struct -----------------------
struct GpsData {
  char lat[11];
  char ns;          // 'N' or 'S'
  char lon[11];
  char ew;          // 'E' or 'W'
  float speed;      // in km/h
  char mode;        // 'A' = Active, 'V' = Void
  bool dataReady;   // true if we got a valid RMC parse
} gpsData;

// --------------------- Keep track of last logged second ---------------------
static int lastLoggedSecond = -1;

// --------------------- Log Data Buffer for USB Transfer ---------------------
#define MAX_LOG_ENTRIES 50
char logEntries[MAX_LOG_ENTRIES][64]; // Buffer to store log entries
int logEntriesCount = 0;
bool isTransferActive = false;

// --------------------- Command Processing ---------------------
#define CMD_BUFFER_SIZE 64
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdBufferIndex = 0;

// --------------------- Command Constants ---------------------
#define LOGGER_CMD "GET_72HRS_DATA"
#define CMD_HEAD "HEAD"
#define CMD_TRP "TRP"
#define CMD_1TP "1TP"
#define CMD_6TP "6TP"
#define CMD_VIOL "VIOL"
#define CMD_RST "RST"
#define SET_SPEED_PREFIX "SET_SPEED="
#define SET_LIMP_PREFIX "SET_LIMP="
#define SET_TIME_PREFIX "SET_TIME="

// --------------------- Speed Limit Settings ---------------------
int speedLimit = 80;     // Default speed limit in km/h
int limpSpeed = 40;      // Default limp speed in km/h
bool speedLimitExceeded = false;

// --------------------- Format RTC Date and Time ---------------------
void formatRTCDateTime(DateTime now, char* dateStr, char* timeStr) {
  // Format date as "DDMMYY"
  sprintf(dateStr, "%02d%02d%02d", now.day(), now.month(), now.year() % 100);
  
  // Format time as "HHMMSS"
  sprintf(timeStr, "%02d%02d%02d", now.hour(), now.minute(), now.second());
}

// --------------------- Parse RMC Sentence ---------------------
void parseRMC(char* sentence) {
  // Example: $GPRMC,HHMMSS,A,lat,NS,lon,EW,speed,angle,date,...
  // or       $GNRMC,HHMMSS,A,lat,NS,lon,EW,speed,angle,date,...
  char* token = strtok(sentence, ",");
  int fieldCount = 0;
  
  // Reset dataReady each new parse
  gpsData.dataReady = false;

  while (token != NULL) {
    switch(fieldCount) {
      case 2: // Status (A=Active, V=Void)
        if (strlen(token) > 0) {
          gpsData.mode = token[0];
        }
        break;
      case 3: // Latitude
        if (strlen(token) > 0) {
          strncpy(gpsData.lat, token, 10);
          gpsData.lat[10] = '\0';
        }
        break;
      case 4: // N/S
        if (strlen(token) > 0) {
          gpsData.ns = token[0];
        }
        break;
      case 5: // Longitude
        if (strlen(token) > 0) {
          strncpy(gpsData.lon, token, 10);
          gpsData.lon[10] = '\0';
        }
        break;
      case 6: // E/W
        if (strlen(token) > 0) {
          gpsData.ew = token[0];
        }
        break;
      case 7: // Speed in knots
        if (strlen(token) > 0) {
          gpsData.speed = atof(token) * 1.852; // knots -> km/h
          
          // Check speed limit
          if (gpsData.speed > speedLimit) {
            speedLimitExceeded = true;
          } else {
            speedLimitExceeded = false;
          }
        } else {
          gpsData.speed = 0.0;
        }
        break;
      case 9: // We don't need GPS date anymore since we'll use RTC
        gpsData.dataReady = true;
        break;
    }
    token = strtok(NULL, ",");
    fieldCount++;
  }

  // If we got valid GPS data and the fix is Active
  if (gpsData.dataReady && gpsData.mode == 'A') {
    // Get current date and time from RTC
    DateTime now = rtc.now();
    char rtcDate[7]; // DDMMYY
    char rtcTime[7]; // HHMMSS
    
    // Format RTC date and time
    formatRTCDateTime(now, rtcDate, rtcTime);
    
    // Print the GPS data with RTC timestamp to Serial1 for debugging
    Serial1.print(rtcDate);
    Serial1.print(",");
    Serial1.print(rtcTime);
    Serial1.print(",");
    Serial1.print(gpsData.lon);
    Serial1.print(gpsData.ew);
    Serial1.print(",");
    Serial1.print(gpsData.lat);
    Serial1.print(gpsData.ns);
    Serial1.print(",");
    Serial1.println(gpsData.speed, 1);

    // --------------------- EXACT 5s LOGGING (RTC Time Based) ---------------------
    // Log at 0, 5, 10, 15... seconds
    int currentSec = now.second();
    
    // If it's exactly a multiple of 5 seconds
    if ((currentSec % 5) == 0) {
      // And we haven't logged this particular second yet
      if (currentSec != lastLoggedSecond) {
        lastLoggedSecond = currentSec;

        // Log to SD
        File dataFile = SD.open("gps_log.txt", FILE_WRITE);
        if (dataFile) {
          char logLine[64];
          sprintf(logLine, "%s,%s,%s%c,%s%c,%.1f", 
                  rtcDate, rtcTime, 
                  gpsData.lon, gpsData.ew, 
                  gpsData.lat, gpsData.ns, 
                  gpsData.speed);
          
          dataFile.println(logLine);
          dataFile.close();

          // Also store in memory buffer for USB transfer if space available
          if (logEntriesCount < MAX_LOG_ENTRIES) {
            strcpy(logEntries[logEntriesCount], logLine);
            logEntriesCount++;
          }

          // Also print to Serial1 for debug
          Serial1.println("Data logged to SD (gps_log.txt). Logged line:");
          Serial1.println(logLine);
        } else {
          Serial1.println("Error opening gps_log.txt for writing!");
        }
      }
    }

  } else {
    // If fix is Void
    if (gpsData.mode == 'V') {
      Serial1.println("GPS signal lost");
      
      // Get current date and time from RTC for logging
      DateTime now = rtc.now();
      char rtcDate[7]; // DDMMYY
      char rtcTime[7]; // HHMMSS
      
      // Format RTC date and time
      formatRTCDateTime(now, rtcDate, rtcTime);
      
      // Log even when GPS signal is lost
      int currentSec = now.second();
      
      // If it's exactly a multiple of 5 seconds
      if ((currentSec % 5) == 0) {
        // And we haven't logged this particular second yet
        if (currentSec != lastLoggedSecond) {
          lastLoggedSecond = currentSec;

          // Log to SD with "SL" for all GPS fields
          File dataFile = SD.open("gps_log.txt", FILE_WRITE);
          if (dataFile) {
            char logLine[64];
            sprintf(logLine, "%s,%s,SL,SL,SL", rtcDate, rtcTime);
            
            dataFile.println(logLine);
            dataFile.close();

            // Also store in memory buffer for USB transfer if space available
            if (logEntriesCount < MAX_LOG_ENTRIES) {
              strcpy(logEntries[logEntriesCount], logLine);
              logEntriesCount++;
            }

            // Also print to Serial1 for debug
            Serial1.println("Data logged to SD (gps_log.txt) with Signal Lost markers:");
            Serial1.println(logLine);
          } else {
            Serial1.println("Error opening gps_log.txt for writing!");
          }
        }
      }
    }
  }
}

// --------------------- Process Commands from USB ---------------------
void processCommand(char* cmd) {
  // Remove any newline or carriage return
  char* newline = strchr(cmd, '\n');
  if (newline) *newline = '\0';
  
  newline = strchr(cmd, '\r');
  if (newline) *newline = '\0';
  
  // Process the commands
  if (strcmp(cmd, LOGGER_CMD) == 0) {
    // Open and read from SD card instead of using the limited RAM buffer
    Serial.println("Transferring log data from SD card...");
    
    File dataFile = SD.open("gps_log.txt");
    if (dataFile) {
      // Get current time from RTC
      DateTime now = rtc.now();
      
      // Calculate timestamp for 72 hours ago (3 days)
      // We need to calculate this manually since TimeSpan might not be available
      uint32_t secondsIn72Hours = 3 * 24 * 60 * 60; // 3 days * 24 hours * 60 minutes * 60 seconds
      
      // Get current Unix timestamp and subtract 72 hours
      uint32_t nowUnix = now.unixtime();
      uint32_t cutoffUnix = nowUnix - secondsIn72Hours;
      
      // Create a DateTime object for the cutoff time
      DateTime cutoffTime(cutoffUnix);
      
      Serial.print("Current time: ");
      Serial.print(now.year()); Serial.print("/");
      Serial.print(now.month()); Serial.print("/");
      Serial.print(now.day()); Serial.print(" ");
      Serial.print(now.hour()); Serial.print(":");
      Serial.print(now.minute()); Serial.print(":");
      Serial.println(now.second());
      
      Serial.print("Cutoff time (72 hours ago): ");
      Serial.print(cutoffTime.year()); Serial.print("/");
      Serial.print(cutoffTime.month()); Serial.print("/");
      Serial.print(cutoffTime.day()); Serial.print(" ");
      Serial.print(cutoffTime.hour()); Serial.print(":");
      Serial.print(cutoffTime.minute()); Serial.print(":");
      Serial.println(cutoffTime.second());
      
      // Variables for reading the file
      char line[64];
      int lineIndex = 0;
      int entriesSent = 0;
      
      // Process byte by byte from the file
      while (dataFile.available()) {
        char c = dataFile.read();
        
        if (c == '\n') {
          // End of line - process this line
          line[lineIndex] = '\0'; // Null terminate
          
          // Check if this is a valid line with date/time
          if (lineIndex > 13) { // Minimum length for a valid line
            // Parse date from line (format: DDMMYY,HHMMSS,...)
            int day = (line[0] - '0') * 10 + (line[1] - '0');
            int month = (line[2] - '0') * 10 + (line[3] - '0');
            int year = 2000 + (line[4] - '0') * 10 + (line[5] - '0'); // Assuming 20xx for years
            
            int hour = (line[7] - '0') * 10 + (line[8] - '0');
            int minute = (line[9] - '0') * 10 + (line[10] - '0');
            int second = (line[11] - '0') * 10 + (line[12] - '0');
            
            // Create DateTime for this entry
            DateTime entryTime(year, month, day, hour, minute, second);
            
            // Check if this entry is within the last 72 hours
            if (entryTime.unixtime() >= cutoffTime.unixtime()) {
              // Send this line to PC
              Serial.println(line);
              entriesSent++;
              
              // Add a small delay every few lines to prevent buffer overflow
              if (entriesSent % 10 == 0) {
                delay(50);
              }
            }
          }
          
          // Reset for next line
          lineIndex = 0;
        } 
        else if (lineIndex < sizeof(line) - 1) {
          // Add character to current line
          line[lineIndex++] = c;
        }
      }
      
      dataFile.close();
      Serial.print("Transfer complete. Sent ");
      Serial.print(entriesSent);
      Serial.println(" log entries.");
    } 
    else {
      Serial.println("Error: Could not open gps_log.txt");
    }
  } 
  else if (strcmp(cmd, CMD_HEAD) == 0) {
    Serial.println("Current settings:");
    Serial.print("Speed limit: "); Serial.println(speedLimit);
    Serial.print("Limp speed: "); Serial.println(limpSpeed);
    DateTime now = rtc.now();
    Serial.print("Device time: ");
    Serial.print(now.year(), DEC); Serial.print('-');
    Serial.print(now.month(), DEC); Serial.print('-');
    Serial.print(now.day(), DEC); Serial.print(' ');
    Serial.print(now.hour(), DEC); Serial.print(':');
    Serial.print(now.minute(), DEC); Serial.print(':');
    Serial.println(now.second(), DEC);
  } 
  else if (strcmp(cmd, CMD_TRP) == 0) {
    Serial.println("Trip data:");
    Serial.println("TripID,StartTime,EndTime,Distance,MaxSpeed,AvgSpeed");
    Serial.println("1,230145,231532,12.5,78.5,45.2");
    // This is a placeholder - you'd implement actual trip data tracking
  } 
  else if (strcmp(cmd, CMD_1TP) == 0) {
    Serial.println("Last trip data:");
    Serial.println("StartTime,EndTime,Distance,MaxSpeed,AvgSpeed");
    Serial.println("230145,231532,12.5,78.5,45.2");
    // This is a placeholder - you'd implement actual trip data tracking
  } 
  else if (strcmp(cmd, CMD_6TP) == 0) {
    Serial.println("Last 6 trips data:");
    Serial.println("TripID,StartTime,EndTime,Distance,MaxSpeed,AvgSpeed");
    Serial.println("1,230145,231532,12.5,78.5,45.2");
    Serial.println("2,232145,233532,8.3,65.2,38.7");
    // This is a placeholder - you'd add more trip data
  } 
  else if (strcmp(cmd, CMD_VIOL) == 0) {
    Serial.println("Speed violations:");
    Serial.println("Time,Speed,Duration");
    Serial.println("231245,85.2,45");
    // This is a placeholder - you'd implement actual violation tracking
  } 
  else if (strcmp(cmd, CMD_RST) == 0) {
    Serial.println("Resetting device...");
    // Reset relevant counters and states
    logEntriesCount = 0;
    speedLimitExceeded = false;
    Serial.println("Reset complete");
  }
  else if (strncmp(cmd, SET_SPEED_PREFIX, strlen(SET_SPEED_PREFIX)) == 0) {
    // Extract the speed value
    int newSpeed = atoi(cmd + strlen(SET_SPEED_PREFIX));
    if (newSpeed > 0 && newSpeed < 200) {
      speedLimit = newSpeed;
      Serial.print("Speed limit set to: ");
      Serial.println(speedLimit);
    } else {
      Serial.println("Invalid speed value");
    }
  }
  else if (strncmp(cmd, SET_LIMP_PREFIX, strlen(SET_LIMP_PREFIX)) == 0) {
    // Extract the limp speed value
    int newLimpSpeed = atoi(cmd + strlen(SET_LIMP_PREFIX));
    if (newLimpSpeed > 0 && newLimpSpeed < speedLimit) {
      limpSpeed = newLimpSpeed;
      Serial.print("Limp speed set to: ");
      Serial.println(limpSpeed);
    } else {
      Serial.println("Invalid limp speed value");
    }
  }
  else if (strncmp(cmd, SET_TIME_PREFIX, strlen(SET_TIME_PREFIX)) == 0) {
    // Extract time string (format: YYYY-MM-DD HH:MM:SS)
    char* timeStr = cmd + strlen(SET_TIME_PREFIX);
    int year, month, day, hour, minute, second;
    
    // Parse the time string
    if (sscanf(timeStr, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
      rtc.adjust(DateTime(year, month, day, hour, minute, second));
      Serial.println("Time set successfully");
    } else {
      Serial.println("Invalid time format. Use: YYYY-MM-DD HH:MM:SS");
    }
  }
  else {
    Serial.println("Unknown command");
  }
}

// --------------------- Setup ---------------------
void setup() {
  // Start Serial (USB) at the baud rate expected by the PC application
  Serial.begin(115200);
  delay(1500); // Give USB time to initialize
  
  // Start Serial1 for GPS communication and debugging
  Serial1.begin(9600);
  delay(1000);

  Serial1.println("\nGPS Logger with RTC Timestamp and USB Communication");
  Serial.println("\nSTM32 GPS Logger with USB Interface");
  Serial.println("Ready to receive commands.");

  // Initialize I2C for RTC
  Wire.begin();
  
  // Initialize RTC module
  if (!rtc.begin()) {
    Serial1.println("Couldn't find DS3231 RTC");
    while (1);
  }
  
  // Check if RTC lost power and needs to be set
  if (rtc.lostPower()) {
    Serial1.println("RTC lost power, setting to compile time");
    // Set RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  Serial1.println("DS3231 RTC initialized");

  // Configure SPI pins for SD card
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); // Deselect SD card

  // Initialize SPI with conservative settings
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128); // Very slow speed
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  Serial1.println("\nPower-up sequence:");
  digitalWrite(SD_CS, HIGH);
  delay(100);

  // Send dummy clock cycles
  for(int i = 0; i < 10; i++) {
    SPI.transfer(0xFF);
    Serial1.print(".");
  }
  Serial1.println(" Done");

  delay(100);

  Serial1.println("\nInitializing SD card...");
  
  // Attempt to initialize SD
  if (!SD.begin(SD_CS)) {
    Serial1.println("\nSD card initialization failed!");
    // Continue without SD card - we can still function with USB
  } else {
    Serial1.println("SD card initialization successful!");

    // Test file operations
    File dataFile = SD.open("test.txt", FILE_WRITE);
    if (dataFile) {
      Serial1.println("\nWriting to test.txt...");
      dataFile.println("Testing SD card with level shifter");
      dataFile.println("Module is working properly!");
      dataFile.close();
      Serial1.println("Write successful!");
    }
  }

  // Clear GPS data struct
  memset(&gpsData, 0, sizeof(gpsData));
  gpsData.mode = 'V'; // Initial mode is Void until we get a valid fix
  
  // Initialize GPS data fields with default values
  strcpy(gpsData.lat, "0000.0000");
  gpsData.ns = 'N';
  strcpy(gpsData.lon, "00000.0000");
  gpsData.ew = 'E';
  gpsData.speed = 0.0;
  gpsData.dataReady = false;
  
  // Initialize command buffer
  memset(cmdBuffer, 0, CMD_BUFFER_SIZE);
  cmdBufferIndex = 0;
}

// --------------------- Main Loop ---------------------
void loop() {
  // 1. Check for USB commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Echo character back to USB
    Serial.write(c);
    
    // Add to command buffer if not end of line
    if (c != '\n' && c != '\r') {
      if (cmdBufferIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdBufferIndex++] = c;
      }
    } else {
      // Process command on newline or carriage return
      if (cmdBufferIndex > 0) {
        cmdBuffer[cmdBufferIndex] = '\0';
        processCommand(cmdBuffer);
        cmdBufferIndex = 0;
      }
    }
  }

  // 2. Process GPS data
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '$') { // Start of NMEA
      bufferIndex = 0;
    }
    else if (c == '\n' || c == '\r') { // End of NMEA
      if (bufferIndex > 0) {
        buffer[bufferIndex] = '\0';
        // Check if it's RMC
        if (strstr(buffer, "GNRMC") || strstr(buffer, "GPRMC")) {
          parseRMC(buffer);
        }
      }
      bufferIndex = 0;
    }
    else if (bufferIndex < (int)sizeof(buffer) - 1) {
      buffer[bufferIndex++] = c;
    }
  }

  // 3. Display current RTC time every second for debug purposes
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime >= 1000) {
    lastDisplayTime = millis();
    
    // Get current date and time from RTC
    DateTime now = rtc.now();
    
    // Format and display date and time
    sprintf(dateTimeStr, "%04d/%02d/%02d %02d:%02d:%02d", 
            now.year(), now.month(), now.day(), 
            now.hour(), now.minute(), now.second());
    
    Serial1.print("RTC: ");
    Serial1.println(dateTimeStr);
  }
}
