/*
 * SIM800L FULL TEST SUITE for ESP32
 * Uses HardwareSerial2 (pins 16=RX, 17=TX)
 * 
 * Tests:
 * 1. Basic AT communication
 * 2. SIM card presence & readiness
 * 3. Signal quality (CSQ)
 * 4. Network registration (CREG)
 * 5. Operator name (COPS)
 * 6. SMS sending & receiving
 * 7. Voice call (make & hang up)
 * 8. GPRS activation & HTTP GET test
 * 
 * Change the TEST_PHONE and APN settings below!
 */

#include <HardwareSerial.h>

// ========== USER CONFIGURATION ==========
#define SIM800L_SERIAL Serial2
#define RX2_PIN 16      // ESP32 RX2 (connect to SIM800L TX)
#define TX2_PIN 17      // ESP32 TX2 (connect to SIM800L RX)

// Optional: PWR_KEY pin (if your module does not auto-power-on)
// #define PWR_PIN 18

// Test phone number (international format, without '+', e.g., "1234567890")
#define TEST_PHONE "01XXXXXXXXX"    // <-- CHANGE THIS

// GPRS APN settings (ask your mobile operator)
#define APN      "your.apn.here"   // e.g., "internet"
#define APN_USER ""                // leave empty if none
#define APN_PASS ""                // leave empty if none

// ========== END USER CONFIGURATION ==========

String response;   // stores last AT command response
unsigned long timeout = 10000;    // 10 seconds default timeout

// ----------------------------------------------------------------------
// Send AT command and check for expected response (default "OK")
bool sendAT(const String& cmd, const String& expected = "OK", unsigned long wait = timeout) {
  SIM800L_SERIAL.println(cmd);
  response = "";
  unsigned long start = millis();
  while (millis() - start < wait) {
    while (SIM800L_SERIAL.available()) {
      char c = SIM800L_SERIAL.read();
      response += c;
      if (response.indexOf(expected) != -1) {
        return true;
      }
    }
  }
  return false;
}

// ----------------------------------------------------------------------
// Wait for an unsolicited result code (e.g., +CMTI: for SMS)
bool waitForResponse(const String& target, unsigned long wait = 20000) {
  unsigned long start = millis();
  while (millis() - start < wait) {
    if (SIM800L_SERIAL.available()) {
      String line = SIM800L_SERIAL.readStringUntil('\n');
      if (line.indexOf(target) != -1) {
        response = line;
        return true;
      }
    }
  }
  return false;
}

// ----------------------------------------------------------------------
// Read SMS from a given index
void readSMS(int index) {
  sendAT("AT+CMGF=1", "OK");                     // Text mode
  sendAT("AT+CNMI=2,1,0,0,0", "OK");            // New SMS indication
  String cmd = "AT+CMGR=" + String(index);
  if (sendAT(cmd, "+CMGR:")) {
    Serial.println("--- SMS Content ---");
    Serial.println(response);
    // Extract message text (simplified)
    int start = response.indexOf('\n') + 1;
    int end = response.lastIndexOf('\r');
    if (start > 0 && end > start) {
      String msg = response.substring(start, end);
      Serial.println("Message: " + msg);
    }
    Serial.println("-------------------");
  } else {
    Serial.println("Failed to read SMS");
  }
}

// ----------------------------------------------------------------------
// Delete all SMS (optional)
void deleteAllSMS() {
  sendAT("AT+CMGD=1,4", "OK");
  Serial.println("All SMS deleted.");
}

// ----------------------------------------------------------------------
// Test GPRS: activate PDP context, get local IP, HTTP GET example
void testGPRS() {
  Serial.println("\n=== GPRS Test ===");
  
  // Set APN
  String apnCmd = "AT+CSTT=\"" + String(APN) + "\",\"" + String(APN_USER) + "\",\"" + String(APN_PASS) + "\"";
  if (!sendAT(apnCmd, "OK")) {
    Serial.println("ERROR: Failed to set APN");
    return;
  }
  Serial.println("APN set");

  // Bring up wireless connection
  if (!sendAT("AT+CIICR", "OK", 15000)) {
    Serial.println("ERROR: CIICR failed (no carrier?)");
    return;
  }
  Serial.println("Wireless connection established");

  // Get local IP address
  if (sendAT("AT+CIFSR", "OK")) {
    Serial.println("Local IP: " + response);
  } else {
    Serial.println("Failed to get IP");
    return;
  }

  // Start HTTP service
  sendAT("AT+HTTPINIT", "OK");
  
  // Set HTTP parameters (URL)
  if (sendAT("AT+HTTPPARA=\"URL\",\"http://httpbin.org/ip\"", "OK")) {
    // Perform HTTP GET
    if (sendAT("AT+HTTPACTION=0", "200", 15000)) {  // 200 = HTTP OK
      Serial.println("HTTP GET successful");
      // Read response
      sendAT("AT+HTTPREAD", "OK");
      Serial.println("HTTP Response:\n" + response);
    } else {
      Serial.println("HTTP GET failed");
    }
  } else {
    Serial.println("Failed to set URL");
  }

  // Clean up
  sendAT("AT+HTTPTERM", "OK");
  Serial.println("GPRS test completed.");
}

// ----------------------------------------------------------------------
// Setup 
void setup() {
  Serial.begin(115200);
  SIM800L_SERIAL.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
  Serial.println("\n\n==================================");
  Serial.println("SIM800L Full Test Suite for ESP32");
  Serial.println("==================================\n");

  // Optional: Pulse power key if module is off
  #ifdef PWR_PIN
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(1000);
    digitalWrite(PWR_PIN, HIGH);
    delay(2000);
    digitalWrite(PWR_PIN, LOW);
    delay(3000);
  #endif

  // Test 1: Basic AT communication
  Serial.print("1. Testing AT communication... ");
  if (!sendAT("AT", "OK")) {
    Serial.println("FAILED. Check wiring and power.");
    while (1) { delay(1000); }
  }
  Serial.println("OK");
  delay(1000);

  // Test 2: SIM card presence
  Serial.print("2. Checking SIM card... ");
  if (sendAT("AT+CPIN?", "READY")) {
    Serial.println("OK (SIM ready)");
  } else if (response.indexOf("SIM PIN") != -1) {
    Serial.println("SIM locked. Please enter PIN using AT+CPIN=xxxx");
    while (1);
  } else {
    Serial.println("No SIM card or error");
    while (1);
  }

  // Test 3: Signal quality
  Serial.print("3. Signal quality (CSQ): ");
  if (sendAT("AT+CSQ", "+CSQ:")) {
    // Extract RSSI value (first number)
    int idx = response.indexOf("+CSQ:") + 5;
    String rssiStr = response.substring(idx, response.indexOf(',', idx));
    int rssi = rssiStr.toInt();
    if (rssi == 99) Serial.println("No signal");
    else if (rssi >= 10) Serial.println("Good (" + String(rssi) + ")");
    else Serial.println("Weak (" + String(rssi) + ")");
  } else {
    Serial.println("Failed");
  }

  // Test 4: Network registration
  Serial.print("4. Network registration: ");
  if (sendAT("AT+CREG?", "+CREG:")) {
    if (response.indexOf(",1") != -1 || response.indexOf(",5") != -1)
      Serial.println("Registered on home network");
    else if (response.indexOf(",2") != -1)
      Serial.println("Searching...");
    else if (response.indexOf(",3") != -1)
      Serial.println("Registration denied");
    else
      Serial.println("Unknown status");
  } else {
    Serial.println("Failed");
  }

  // Test 5: Operator name
  Serial.print("5. Operator name: ");
  if (sendAT("AT+COPS?", "+COPS:")) {
    int start = response.indexOf('\"');
    int end = response.indexOf('\"', start + 1);
    if (start != -1 && end != -1) {
      String op = response.substring(start + 1, end);
      Serial.println(op);
    } else {
      Serial.println("Unknown");
    }
  } else {
    Serial.println("Failed");
  }

  // Test 6: SMS receive (wait 20 seconds for incoming SMS)
  Serial.println("\n6. Waiting for incoming SMS (20 sec)...");
  if (waitForResponse("+CMTI:", 20000)) {
    // Extract storage index
    int idxStart = response.lastIndexOf(',') + 1;
    int idxEnd = response.length();
    int index = response.substring(idxStart, idxEnd).toInt();
    Serial.println("New SMS received in slot " + String(index));
    readSMS(index);
    // Delete this SMS
    sendAT("AT+CMGD=" + String(index), "OK");
    Serial.println("SMS deleted.");
  } else {
    Serial.println("No SMS received within timeout.");
  }

  // Test 7: Send SMS
  Serial.println("\n7. Sending SMS to " + String(TEST_PHONE) + "...");
  sendAT("AT+CMGF=1", "OK");                  // Text mode
  sendAT("AT+CMGS=\"" + String(TEST_PHONE) + "\"", ">", 5000);
  SIM800L_SERIAL.print("Hello from ESP32 SIM800L test.");
  SIM800L_SERIAL.write(26);                   // Ctrl+Z to send
  if (waitForResponse("+CMGS:", 30000)) {
    Serial.println("SMS sent successfully");
  } else {
    Serial.println("SMS sending failed");
  }

  // Test 8: Voice call (make and auto-hangup after 10 seconds)
  Serial.println("\n8. Making voice call to " + String(TEST_PHONE) + " (will hang up in 10s)...");
  sendAT("AT+CMGF=1", "OK");          // Ensure text mode (some voice commands need it)
  if (sendAT("ATD" + String(TEST_PHONE) + ";", "OK")) {
    Serial.println("Call initiated. Waiting 10 seconds...");
    delay(10000);
    sendAT("ATH", "OK");
    Serial.println("Call ended.");
  } else {
    Serial.println("Call failed.");
  }

  // Test 9: GPRS (Internet)
  testGPRS();

  // Final
  Serial.println("\n==================================");
  Serial.println("All tests completed.");
  Serial.println("==================================");
}

// ----------------------------------------------------------------------
void loop() {
  // Just echo any serial input from the user to the SIM800L (manual command mode)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length()) {
      sendAT(cmd);
      Serial.println("Response: " + response);
    }
  }
  // Forward unsolicited SIM800L messages to Serial
  while (SIM800L_SERIAL.available()) {
    Serial.write(SIM800L_SERIAL.read());
  }
}