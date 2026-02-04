// base_station/firmware/BaseCrowPanelAppRadio/BaseCrowPanelAppRadio.ino
#include <Arduino.h>
#include <Preferences.h>

// ---------------- CONFIG (EDIT THESE) ----------------
// Choose pins for the UART that goes to your APP-DATA radio module.
// These pins depend on your DIS08070H variant & what headers are actually broken out.
// Many ESP32-S3 boards can map Serial1 pins freely.
//
// IMPORTANT: Do NOT use the same UART pins as USB-Serial debug.
static const int RADIO_RX_PIN = 17;  // <-- confirm on your board silkscreen/manual
static const int RADIO_TX_PIN = 18;  // <-- confirm
static const uint32_t RADIO_BAUD = 115200;

// If you want to monitor base GNSS later, add a second UART here.
// static const int GNSS_RX_PIN = ...;
// static const int GNSS_TX_PIN = ...;
// static const uint32_t GNSS_BAUD = 115200;

// -----------------------------------------------------

Preferences prefs;

// Helper: send "TYPE k=v k=v\n"
void sendKV(const String& type, const String& kv) {
  String line = type;
  if (kv.length() > 0) {
    line += " ";
    line += kv;
  }
  line += "\n";
  Serial1.print(line);  // goes out to APP radio
}

// Minimal input helpers over USB Serial Monitor (replace with LVGL + keypad later)
String readLineBlocking() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      if (s.length() > 0) return s;
    }
    delay(10);
  }
}

bool confirmYN(const String& prompt) {
  Serial.println(prompt + " (y/n)");
  String s = readLineBlocking();
  s.toLowerCase();
  return s.startsWith("y");
}

String promptUserId() {
  while (true) {
    Serial.println("Enter 2-digit User ID (00-99) or N for new:");
    String s = readLineBlocking();
    s.toLowerCase();
    if (s == "n") {
      Serial.println("Enter new 2-digit ID to use:");
      s = readLineBlocking();
    }
    if (s.length() == 2 && isDigit(s[0]) && isDigit(s[1])) return s;
    Serial.println("Invalid. Must be two digits like 03.");
  }
}

float promptFloat(const String& prompt, float minv, float maxv) {
  while (true) {
    Serial.println(prompt + " [" + String(minv) + " .. " + String(maxv) + "]");
    String s = readLineBlocking();
    float v = s.toFloat();
    if (v >= minv && v <= maxv) return v;
    Serial.println("Invalid number.");
  }
}

// Storage schema: we store last-known good dimensions per user ID.
// Key names like: "u03_len", "u03_wid"
void saveUserDims(const String& userId, float length_m, float width_m) {
  String kL = "u" + userId + "_len";
  String kW = "u" + userId + "_wid";
  prefs.putFloat(kL.c_str(), length_m);
  prefs.putFloat(kW.c_str(), width_m);
}

bool loadUserDims(const String& userId, float& length_m, float& width_m) {
  String kL = "u" + userId + "_len";
  String kW = "u" + userId + "_wid";
  if (!prefs.isKey(kL.c_str()) || !prefs.isKey(kW.c_str())) return false;
  length_m = prefs.getFloat(kL.c_str(), 0);
  width_m  = prefs.getFloat(kW.c_str(), 0);
  return (length_m > 0 && width_m > 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // App-data radio UART
  Serial1.begin(RADIO_BAUD, SERIAL_8N1, RADIO_RX_PIN, RADIO_TX_PIN);

  prefs.begin("autofield", false);

  Serial.println("\n=== Base CrowPanel (App Radio) ===");
  Serial.println("This demo uses Serial Monitor as UI. Replace with LVGL + keypad later.\n");
}

void loop() {
  // 1) User ID
  String userId = promptUserId();

  // Optional: recall last stored dims
  float storedL = 0, storedW = 0;
  bool hasStored = loadUserDims(userId, storedL, storedW);
  if (hasStored) {
    Serial.println("Found stored dims for user " + userId +
                   ": L=" + String(storedL) + " W=" + String(storedW));
    if (confirmYN("Use stored dimensions?")) {
      // continue with stored
    } else {
      hasStored = false;
    }
  }

  // 2) Length/Width input
  float length_m, width_m;
  if (hasStored) {
    length_m = storedL;
    width_m  = storedW;
  } else {
    length_m = promptFloat("Enter field LENGTH (m)", 0.1, 500.0);
    width_m  = promptFloat("Enter field WIDTH  (m)", 0.1, 500.0);
  }

  // Enforce "width is shorter side"
  float L = max(length_m, width_m);
  float W = min(length_m, width_m);

  Serial.println("Interpreting as length=" + String(L) + " width=" + String(W));

  // 3) Origin placement instruction
  if (!confirmYN("Place rover under base. Ensure both are at origin (0,0) and field is +x,+y. Confirm ready")) {
    Serial.println("Cancelled.\n");
    return;
  }

  // 4) Origin GNSS: for now we ask manually.
  // Later: read from base ZED-F9P and lock this automatically.
  float originLat = promptFloat("Enter ORIGIN LAT (deg) (temporary manual)", -90.0, 90.0);
  float originLon = promptFloat("Enter ORIGIN LON (deg) (temporary manual)", -180.0, 180.0);

  // 5) Sanity check overlay stub
  Serial.println("Overlay sanity check: (stub) OK");

  // 6) Generate waypoints stub (replace with partner mapping code)
  // For now: send a tiny rectangle plan with 4 waypoints.
  float d = 0.00005f;

  // Transmit plan over APP radio:
  sendKV("FIELD", "user=" + userId + " length=" + String(L, 2) + " width=" + String(W, 2));
  sendKV("ORIGIN", "lat=" + String(originLat, 7) + " lon=" + String(originLon, 7));

  sendKV("WP", "i=0 lat=" + String(originLat, 7) + " lon=" + String(originLon, 7));
  sendKV("WP", "i=1 lat=" + String(originLat + d, 7) + " lon=" + String(originLon, 7));
  sendKV("WP", "i=2 lat=" + String(originLat + d, 7) + " lon=" + String(originLon + d, 7));
  sendKV("WP", "i=3 lat=" + String(originLat, 7) + " lon=" + String(originLon + d, 7));
  sendKV("PLAN_END", "n=4");

  Serial.println("Plan transmitted over app radio.");

  // 7) Dry run confirm + start
  if (confirmYN("Ready for dry run START?")) {
    sendKV("START", "dry=1");
    Serial.println("Sent START dry=1");
  } else {
    Serial.println("Not starting.");
  }

  // 8) Store dims after “correct run has gone”
  // You said: save bounds once a correct run has occurred.
  // For now, we let you manually declare success.
  if (confirmYN("Mark this run successful and store dims under user ID?")) {
    saveUserDims(userId, L, W);
    Serial.println("Saved dims for user " + userId);
  }

  Serial.println("\nCycle complete.\n");
  delay(250);
}
