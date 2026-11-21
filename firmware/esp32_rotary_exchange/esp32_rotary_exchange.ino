/*
  ESP32 rotary-phone exchange node
  ---------------------------------
  One sketch per telephone. Each node has a unique 3-digit number, discovers
  the others by UDP broadcast, and places calls when the rotary dial enters
  three digits. A PC running software/softphone/softphone.py is an ordinary
  peer (111, unless you change it).

  Call control:
    * UDP broadcast discovery, up to five remembered peers
    * No hard-coded peer address
    * Dial tone while the handset is off-hook and idle
    * Ringback on an outgoing call, physical ringing on an incoming call
    * Answer and hangup in both directions
    * 8 kHz 16-bit PCM audio over UDP
    * SHK is debounced so rotary pulses are not treated as hook events
    * A dial pulse counts only when its break width and spacing match a
      10 pulse/sec rotary dial
    * The AG1170 DC/DC is off while the phone is idle and on-hook, and is
      woken briefly to sample SHK

  Pins, matching hardware/esp32-phone-exchange:
    GPIO25  DAC1 -> AG1170 VIN, through C6
    GPIO34  <- AG1170 SHK, through the R5/R6 divider
    GPIO18  -> Q3 base; Q3 pulls F/R down (R1 pulls F/R up to 5 V)
    GPIO19  -> Q4 base; Q4 pulls RM down (R4 pulls RM up to 5 V)
    GPIO4   -> Q5 base; Q5 collector is AG1170 PD, with no pull-up

  The NPN inverts F/R and RM:
    GPIO LOW  -> AG1170 input HIGH
    GPIO HIGH -> AG1170 input LOW

  PD is not that circuit. GPIO LOW leaves PD open and the DC/DC free-runs.
  GPIO HIGH pulls PD to ground and powers the converter down. A logic high
  on PD, or a pull-up to +5 V, damages the converter. On the Silvertel
  module, PD is pin 4. Keep that wire short.

  PCM1808 is the I2S slave. The ESP32 drives SCK, BCK, and LRCK.
  MD0, MD1, and FMT are tied to ground on the board:
    MD0 low + MD1 low = slave mode
    FMT low = 24-bit Philips I2S
    GPIO1  = I2S1 MCLK -> PCM1808 SCK
    GPIO26 = I2S1 BCLK -> PCM1808 BCK
    GPIO27 = I2S1 LRCK -> PCM1808 LRC
    GPIO35 = I2S1 DIN  <- PCM1808 DOUT
    AG1170 VOUT -> PCM1808 LIN, through C5

  The original ESP32 can route I2S MCLK only to GPIO0, GPIO1, or GPIO3.
  GPIO0 is not broken out on this DevKit, so SCK is GPIO1. GPIO1 is also
  the serial TX pin. Wi-Fi is brought up first so the address is still
  visible. After the codec starts, the pin is a 2.048 MHz clock and the
  serial monitor stops.

  Requires an original ESP32 (ESP-WROOM-32) and Arduino-ESP32 2.0.x.
  The sketch uses the legacy driver/i2s.h API so call audio and tones can
  stream to the built-in DAC on GPIO25. ESP32-C3, S2, and S3 boards do not
  have that DAC and will not run this sketch.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "soc/io_mux_reg.h"
#include <math.h>

// Wi-Fi credentials and NODE_NUMBER. Edit config.h before uploading.
#include "config.h"

// UDP discovery/signaling/audio port.
constexpr uint16_t UDP_PORT = 5000;

// Broadcast address used for discovery.
// 255.255.255.255 is enough on a simple home LAN. If the router drops it,
// use the subnet broadcast (for example 192.168.1.255) and set the same
// address in DISCOVERY_ADDR in software/softphone/softphone.py.
IPAddress DISCOVERY_IP(255, 255, 255, 255);

// Audio
constexpr uint32_t SAMPLE_RATE = 8000;
constexpr uint16_t FRAME_SAMPLES = 160;  // 20 ms

// AG1170 / ESP32 pins
constexpr int PIN_AUDIO_DAC = 25;  // internal DAC1
constexpr int PIN_SHK       = 34;  // AG1170 SHK
constexpr int PIN_FR        = 18;  // external NPN level shifter -> F/R
constexpr int PIN_RM        = 19;  // external NPN level shifter -> RM
constexpr int PIN_PD        = 4;   // open-collector NPN -> AG1170 PD
// GPIO LOW  = transistor off = PD open = DC/DC running
// GPIO HIGH = transistor on  = PD low  = DC/DC off

// PCM1808 I2S (I2S1 master). MCLK must be GPIO0, GPIO1, or GPIO3.
constexpr int PIN_I2S_MCLK = 1;   // SCK
constexpr int PIN_I2S_BCLK = 26;  // BCK
constexpr int PIN_I2S_WS   = 27;  // LRCK
constexpr int PIN_I2S_DIN  = 35;  // PCM1808 DOUT

// Extra left shift after taking the top 16 bits of the 24-bit sample.
constexpr int PCM1808_GAIN_SHIFT = 0;

// Peer table
constexpr uint8_t MAX_PEERS = 5;
constexpr uint32_t PEER_TIMEOUT_MS = 30000;
constexpr uint32_t HELLO_INTERVAL_MS = 5000;

// Dialing
constexpr uint32_t DIGIT_GAP_MS = 3000;
constexpr uint8_t MAX_DIAL_DIGITS = 3;

// Rotary dial timing for a 10 pulse/sec dial: ~100 ms between breaks and a
// ~60 ms break (SHK low). A trailing chatter edge on an otherwise clean
// train is what turns a dialed 3 into a 4, so a low interval counts only
// when it is long enough to be a real break and far enough from the last one.
constexpr uint32_t ROTARY_BREAK_MIN_US = 25000;
constexpr uint32_t ROTARY_BREAK_MAX_US = 100000;
constexpr uint32_t ROTARY_CHATTER_HIGH_US = 12000;
// High time after a counted pulse before another break may start.
// Real make is ~40 ms; contact chatter is only a few milliseconds.
constexpr uint32_t ROTARY_MIN_MAKE_US = 20000;
constexpr uint32_t ROTARY_MIN_PERIOD_US = 50000;
constexpr uint32_t ROTARY_DIGIT_GAP_US = 300000;
constexpr uint32_t OFFHOOK_CONFIRM_MS = 250;

// AG1170 PD. Outputs, including SHK, are invalid for 50 ms after release.
// SHK also pulses high for about 25 ms during that wake. While idle and
// on-hook the DC/DC stays off and is woken once a second to sample SHK.
constexpr uint32_t SLIC_WAKE_MS = 50;
constexpr uint32_t SLIC_POLL_ON_MS = 80;
constexpr uint32_t SLIC_POLL_PERIOD_MS = 1000;

// ===================== PACKET PROTOCOL =====================

constexpr uint32_t MAGIC = 0x544C5032UL; // "TLP2"
constexpr uint8_t PROTOCOL_VERSION = 2;

enum PacketType : uint8_t {
  PKT_HELLO   = 1,
  PKT_AUDIO   = 2,
  PKT_CONTROL = 3
};

enum ControlCode : uint8_t {
  CTRL_CALL       = 1,
  CTRL_RINGING    = 2,
  CTRL_ANSWER     = 3,
  CTRL_HANGUP     = 4,
  CTRL_BUSY       = 5,
  CTRL_REJECT     = 6,
  CTRL_PING       = 7
};

#pragma pack(push, 1)

struct PacketHeader {
  uint32_t magic;
  uint8_t type;
  uint8_t version;
  uint16_t sequence;
  uint16_t samples;
  uint16_t reserved;
};

// HELLO contains the node's phone number and its LAN IPv4 address.
struct HelloPacket {
  PacketHeader h;
  uint16_t phoneNumber;
  uint8_t ip[4];
  uint8_t reserved2[2];
};

// CALL/ANSWER/etc. identifies both ends of the call.
struct ControlPacket {
  PacketHeader h;
  uint8_t code;
  uint8_t value;
  uint16_t callerNumber;
  uint16_t calledNumber;
  uint16_t reserved2;
};

#pragma pack(pop)

// ===================== STATE =====================

WiFiUDP udp;

struct Peer {
  bool valid;
  uint16_t phoneNumber;
  IPAddress ip;
  uint32_t lastSeenMs;
  bool helloPrinted;
};

Peer peers[MAX_PEERS];

enum CallState : uint8_t {
  CALL_IDLE,
  CALL_DIALING,
  CALL_OUTGOING_RING,
  CALL_INCOMING_RING,
  CALL_CONNECTED
};

volatile CallState callState = CALL_IDLE;

enum ToneType : uint8_t {
  TONE_NONE,
  TONE_DIAL,
  TONE_RINGBACK,
  TONE_BUSY
};

volatile ToneType currentTone = TONE_NONE;

uint16_t remoteNumber = 0;
// Written from the Arduino loop, read from the audio task. A 32-bit store
// is atomic on the ESP32, so the address cannot tear mid-packet.
volatile uint32_t remoteIpRaw = 0;
uint8_t dialCount = 0;
uint32_t lastDialActivityMs = 0;

volatile bool phoneOffHook = false;

// AG1170 DC/DC power. slicReleasedMs is the last time PD was released.
bool slicPowered = false;
uint32_t slicReleasedMs = 0;
bool shkInterruptOn = false;

// Rotary ISR state. SHK high is off-hook; each dial pulse pulls it low.
enum PulseRx : uint8_t {
  PULSE_IDLE,
  PULSE_BREAK,
  PULSE_CHATTER
};

volatile bool rotaryArmed = false;
volatile bool shkIsLow = true;
volatile PulseRx pulseRx = PULSE_IDLE;
volatile uint32_t breakStartUs = 0;
volatile uint32_t chatterHighUs = 0;
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastAcceptedBreakUs = 0;
volatile uint16_t pulseCount = 0;
volatile bool pulseActivity = false;
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

// WiFiUDP is used from loop() and from the capture task.
SemaphoreHandle_t udpMutex = nullptr;

// Ringing
bool ringing = false;
bool ringPhaseOn = false;
uint32_t ringPhaseStartedMs = 0;
uint32_t lastRingToggleMs = 0;
bool frState = false;

// Audio
uint16_t txSequence = 0;
uint32_t toneSampleCounter = 0;

constexpr size_t RX_RING_SAMPLES = 4096;
int16_t rxRing[RX_RING_SAMPLES];
volatile uint32_t rxHead = 0;
volatile uint32_t rxTail = 0;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// ===================== PEER DISCOVERY =====================

bool sameIP(const IPAddress& a, const IPAddress& b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2] && a[3] == b[3];
}

void printIP(const IPAddress& ip) {
  Serial.printf("%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

int findPeer(uint16_t number) {
  for (int i = 0; i < MAX_PEERS; ++i) {
    if (peers[i].valid && peers[i].phoneNumber == number) {
      return i;
    }
  }
  return -1;
}

void storePeer(uint16_t number, const IPAddress& ip) {
  if (number == NODE_NUMBER) return;

  int idx = findPeer(number);

  if (idx < 0) {
    // Find an empty slot.
    for (int i = 0; i < MAX_PEERS; ++i) {
      if (!peers[i].valid) {
        idx = i;
        break;
      }
    }
  }

  if (idx < 0) {
    // Replace the oldest entry.
    uint32_t oldest = UINT32_MAX;
    for (int i = 0; i < MAX_PEERS; ++i) {
      if (peers[i].lastSeenMs < oldest) {
        oldest = peers[i].lastSeenMs;
        idx = i;
      }
    }
  }

  bool wasValid = peers[idx].valid;
  bool changed = !wasValid ||
                 peers[idx].phoneNumber != number ||
                 !sameIP(peers[idx].ip, ip);

  peers[idx].valid = true;
  peers[idx].phoneNumber = number;
  peers[idx].ip = ip;
  peers[idx].lastSeenMs = millis();

  if (!wasValid || changed || !peers[idx].helloPrinted) {
    Serial.printf("PEER DISCOVERED: %03u @ ", number);
    printIP(ip);
    Serial.println();
    peers[idx].helloPrinted = true;
  }
}

void expirePeers() {
  uint32_t now = millis();

  for (int i = 0; i < MAX_PEERS; ++i) {
    if (peers[i].valid && (now - peers[i].lastSeenMs > PEER_TIMEOUT_MS)) {
      Serial.printf("PEER EXPIRED: %03u\n", peers[i].phoneNumber);
      peers[i].valid = false;
      peers[i].helloPrinted = false;
    }
  }
}

void setRemoteIp(const IPAddress& ip) {
  remoteIpRaw = uint32_t(ip);
}

IPAddress getRemoteIp() {
  uint32_t raw = remoteIpRaw;
  return IPAddress(raw);
}

bool lockUdp() {
  if (udpMutex == nullptr) return false;
  return xSemaphoreTake(udpMutex, pdMS_TO_TICKS(30)) == pdTRUE;
}

void unlockUdp() {
  xSemaphoreGive(udpMutex);
}

void sendHello() {
  HelloPacket p{};
  p.h.magic = MAGIC;
  p.h.type = PKT_HELLO;
  p.h.version = PROTOCOL_VERSION;
  p.phoneNumber = NODE_NUMBER;

  IPAddress ip = WiFi.localIP();
  p.ip[0] = ip[0];
  p.ip[1] = ip[1];
  p.ip[2] = ip[2];
  p.ip[3] = ip[3];

  if (!lockUdp()) return;
  udp.beginPacket(DISCOVERY_IP, UDP_PORT);
  udp.write(reinterpret_cast<uint8_t*>(&p), sizeof(p));
  udp.endPacket();
  unlockUdp();
}

int getPeer(uint16_t number, IPAddress& ip) {
  int idx = findPeer(number);
  if (idx < 0) return -1;

  if (millis() - peers[idx].lastSeenMs > PEER_TIMEOUT_MS) {
    peers[idx].valid = false;
    return -1;
  }

  ip = peers[idx].ip;
  return idx;
}

// ===================== CONTROL MESSAGES =====================

void sendControlTo(const IPAddress& ip, uint8_t code,
                   uint16_t callerNumber, uint16_t calledNumber) {
  ControlPacket p{};
  p.h.magic = MAGIC;
  p.h.type = PKT_CONTROL;
  p.h.version = PROTOCOL_VERSION;
  p.h.sequence = 0;
  p.code = code;
  p.callerNumber = callerNumber;
  p.calledNumber = calledNumber;

  if (!lockUdp()) return;
  udp.beginPacket(ip, UDP_PORT);
  udp.write(reinterpret_cast<uint8_t*>(&p), sizeof(p));
  udp.endPacket();
  unlockUdp();
}

void sendCall() {
  sendControlTo(getRemoteIp(), CTRL_CALL, NODE_NUMBER, remoteNumber);
}

void sendAnswer() {
  sendControlTo(getRemoteIp(), CTRL_ANSWER, NODE_NUMBER, remoteNumber);
}

void sendHangup() {
  if (remoteNumber != 0) {
    sendControlTo(getRemoteIp(), CTRL_HANGUP, NODE_NUMBER, remoteNumber);
  }
}

void sendRinging() {
  sendControlTo(getRemoteIp(), CTRL_RINGING, NODE_NUMBER, remoteNumber);
}

void sendBusy() {
  sendControlTo(getRemoteIp(), CTRL_BUSY, NODE_NUMBER, remoteNumber);
}

// ===================== AUDIO RING BUFFER =====================

void clearAudioRing() {
  portENTER_CRITICAL(&rxMux);
  rxHead = rxTail = 0;
  portEXIT_CRITICAL(&rxMux);
}

void rxPush(const int16_t* samples, size_t count) {
  portENTER_CRITICAL(&rxMux);

  for (size_t i = 0; i < count; ++i) {
    uint32_t next = (rxHead + 1) % RX_RING_SAMPLES;

    if (next == rxTail) {
      rxTail = (rxTail + 1) % RX_RING_SAMPLES;
    }

    rxRing[rxHead] = samples[i];
    rxHead = next;
  }

  portEXIT_CRITICAL(&rxMux);
}

size_t rxPop(int16_t* samples, size_t count) {
  size_t n = 0;

  portENTER_CRITICAL(&rxMux);

  while (n < count && rxTail != rxHead) {
    samples[n++] = rxRing[rxTail];
    rxTail = (rxTail + 1) % RX_RING_SAMPLES;
  }

  portEXIT_CRITICAL(&rxMux);
  return n;
}

// ===================== TONE GENERATION =====================

void setTone(ToneType tone) {
  currentTone = tone;
}

int16_t toneSample(float frequency1, float frequency2 = 0.0f,
                   float amplitude = 0.16f) {
  float t = float(toneSampleCounter % SAMPLE_RATE) / float(SAMPLE_RATE);
  float a = sinf(2.0f * PI * frequency1 * t);

  if (frequency2 > 0.0f) {
    a = 0.5f * (a + sinf(2.0f * PI * frequency2 * t));
  }

  ++toneSampleCounter;

  return (int16_t)(a * 32767.0f * amplitude);
}

int16_t generateToneSample() {
  ToneType tone = currentTone;

  switch (tone) {
    case TONE_DIAL:
      // North-American style dial tone: 350 + 440 Hz.
      return toneSample(350.0f, 440.0f, 0.16f);

    case TONE_RINGBACK: {
      // North-American style ringback: 440 + 480 Hz, 2 sec on / 4 sec off.
      uint32_t pos = toneSampleCounter % (SAMPLE_RATE * 6);
      if (pos >= SAMPLE_RATE * 2) {
        ++toneSampleCounter;
        return 0;
      }
      return toneSample(440.0f, 480.0f, 0.14f);
    }

    case TONE_BUSY: {
      // 480 Hz, 0.5 sec on / 0.5 sec off.
      uint32_t phase = (toneSampleCounter / (SAMPLE_RATE / 2)) % 2;
      if (phase == 1) {
        ++toneSampleCounter;
        return 0;
      }
      return toneSample(480.0f, 0.0f, 0.16f);
    }

    default:
      return 0;
  }
}

// ===================== TELEPHONE RINGING =====================

void startPhysicalRinging() {
  // Ring voltage needs the DC/DC. Release PD before RM is asserted.
  slicPowerUp();
  if (ringing) return;

  Serial.println("PHYSICAL RING START");

  ringing = true;
  ringPhaseOn = true;
  ringPhaseStartedMs = millis();
  lastRingToggleMs = millis();
  frState = false;

  // RM active through external NPN level shifter:
  // ESP32 LOW = AG1170 RM HIGH.
  digitalWrite(PIN_RM, LOW);
  digitalWrite(PIN_FR, LOW);
}

void stopPhysicalRinging() {
  if (!ringing) return;

  Serial.println("PHYSICAL RING STOP");

  ringing = false;

  // RM off: ESP32 HIGH -> AG1170 RM low.
  digitalWrite(PIN_RM, HIGH);
  digitalWrite(PIN_FR, LOW);
}

void servicePhysicalRinging() {
  if (!ringing) return;

  uint32_t now = millis();

  // 2 sec on / 4 sec off cadence.
  uint32_t phaseLength = ringPhaseOn ? 2000 : 4000;

  if (now - ringPhaseStartedMs >= phaseLength) {
    ringPhaseOn = !ringPhaseOn;
    ringPhaseStartedMs = now;

    if (ringPhaseOn) {
      digitalWrite(PIN_RM, LOW);
      lastRingToggleMs = now;
    } else {
      digitalWrite(PIN_RM, HIGH);
      digitalWrite(PIN_FR, LOW);
    }
  }

  if (ringPhaseOn && now - lastRingToggleMs >= 25) {
    lastRingToggleMs = now;

    // External NPN inverts F/R.
    frState = !frState;
    digitalWrite(PIN_FR, frState ? LOW : HIGH);
  }
}

// ===================== ROTARY DIAL =====================

void setRotaryArmed(bool armed) {
  portENTER_CRITICAL(&pulseMux);
  rotaryArmed = armed;
  pulseCount = 0;
  pulseActivity = false;
  lastPulseUs = 0;
  lastAcceptedBreakUs = 0;
  pulseRx = PULSE_IDLE;
  portEXIT_CRITICAL(&pulseMux);
}

bool rotaryIsBusy() {
  bool busy;
  portENTER_CRITICAL(&pulseMux);
  busy = pulseActivity || pulseRx != PULSE_IDLE;
  portEXIT_CRITICAL(&pulseMux);
  return busy;
}

void IRAM_ATTR shkISR() {
  uint32_t now = micros();
  bool isLow = digitalRead(PIN_SHK) == LOW;

  portENTER_CRITICAL_ISR(&pulseMux);

  // A second interrupt while the pin has not changed is not a new pulse.
  if (isLow == shkIsLow) {
    portEXIT_CRITICAL_ISR(&pulseMux);
    return;
  }
  shkIsLow = isLow;

  // Armed only after off-hook is confirmed, so pickup itself is not a pulse.
  if (!rotaryArmed) {
    pulseRx = PULSE_IDLE;
    portEXIT_CRITICAL_ISR(&pulseMux);
    return;
  }

  if (isLow) {
    if (pulseRx == PULSE_CHATTER &&
        (uint32_t)(now - chatterHighUs) < ROTARY_CHATTER_HIGH_US) {
      // High twitch inside one break. Keep the original break start.
      pulseRx = PULSE_BREAK;
    } else if (pulseRx != PULSE_BREAK &&
               (lastPulseUs == 0 ||
                (uint32_t)(now - lastPulseUs) >= ROTARY_MIN_MAKE_US) &&
               (lastAcceptedBreakUs == 0 ||
                (uint32_t)(now - lastAcceptedBreakUs) >= ROTARY_MIN_PERIOD_US)) {
      breakStartUs = now;
      pulseRx = PULSE_BREAK;
    }
  } else if (pulseRx == PULSE_BREAK) {
    uint32_t width = (uint32_t)(now - breakStartUs);

    if (width < ROTARY_BREAK_MIN_US) {
      // Too short to commit. It may be bounce in the middle of a real break.
      pulseRx = PULSE_CHATTER;
      chatterHighUs = now;
    } else if (width <= ROTARY_BREAK_MAX_US) {
      if (pulseCount < 15) {
        pulseCount++;
      }
      pulseActivity = true;
      lastPulseUs = now;
      lastAcceptedBreakUs = breakStartUs;
      pulseRx = PULSE_IDLE;
    } else {
      // Longer than a dial break: hook flash or hangup, not a digit.
      pulseRx = PULSE_IDLE;
    }
  }

  portEXIT_CRITICAL_ISR(&pulseMux);
}

// ===================== AG1170 POWER DOWN =====================

void slicSetShkInterrupt(bool enable) {
  if (enable == shkInterruptOn) return;
  if (enable) {
    shkIsLow = (digitalRead(PIN_SHK) == LOW);
    attachInterrupt(digitalPinToInterrupt(PIN_SHK), shkISR, CHANGE);
  } else {
    detachInterrupt(digitalPinToInterrupt(PIN_SHK));
  }
  shkInterruptOn = enable;
}

void slicPowerUp() {
  if (slicPowered) return;
  // SHK is not valid until the converter has been up for SLIC_WAKE_MS.
  slicSetShkInterrupt(false);
  digitalWrite(PIN_PD, LOW);  // transistor off, PD open, DC/DC free-runs
  slicPowered = true;
  slicReleasedMs = millis();
}

void slicPowerDown() {
  if (!slicPowered) return;
  slicSetShkInterrupt(false);
  digitalWrite(PIN_PD, HIGH);  // transistor on, PD grounded, DC/DC off
  slicPowered = false;
}

bool slicOutputsValid() {
  return slicPowered &&
         (uint32_t)(millis() - slicReleasedMs) >= SLIC_WAKE_MS;
}

void serviceSlicPower() {
  // Off-hook, dialing, ringing, and an active call all need loop feed.
  if (phoneOffHook || callState != CALL_IDLE || ringing) {
    slicPowerUp();
    return;
  }

  // wakeStartedMs is the moment the converter was last turned off, so the
  // compare below is the off interval. Silvertel recommends <= 1 second.
  static uint32_t wakeStartedMs = 0;
  uint32_t now = millis();

  if (!slicPowered) {
    if ((uint32_t)(now - wakeStartedMs) >= SLIC_POLL_PERIOD_MS) {
      wakeStartedMs = now;
      slicPowerUp();
    }
    return;
  }

  if (!slicOutputsValid()) return;

  // Past the power-up pulse. SHK still high means the handset is off-hook,
  // so leave the converter running for the normal debounce.
  if (digitalRead(PIN_SHK) == HIGH) return;

  if ((uint32_t)(now - slicReleasedMs) >= SLIC_POLL_ON_MS) {
    wakeStartedMs = now;
    slicPowerDown();
  }
}

void processRotaryDial() {
  if (!phoneOffHook) return;

  uint16_t pulses = 0;
  uint16_t liveCount = 0;

  portENTER_CRITICAL(&pulseMux);
  uint32_t nowUs = micros();

  // A short spike that never went low again must not hold the digit open.
  if (pulseRx == PULSE_CHATTER &&
      (uint32_t)(nowUs - chatterHighUs) >= ROTARY_CHATTER_HIGH_US &&
      digitalRead(PIN_SHK) == HIGH) {
    pulseRx = PULSE_IDLE;
  }

  liveCount = pulseCount;

  if (rotaryArmed &&
      pulseRx == PULSE_IDLE &&
      pulseActivity &&
      pulseCount > 0 &&
      (uint32_t)(nowUs - lastPulseUs) >= ROTARY_DIGIT_GAP_US) {
    pulses = pulseCount;
    pulseCount = 0;
    pulseActivity = false;
    lastPulseUs = 0;
    lastAcceptedBreakUs = 0;
  }
  portEXIT_CRITICAL(&pulseMux);

  static uint16_t shownCount = 0;
  if (liveCount != shownCount) {
    shownCount = liveCount;
    if (liveCount > 0) {
      Serial.printf("pulse %u\n", liveCount);
    }
  }

  if (pulses == 0) return;

  if (pulses > 10) {
    Serial.printf("DIAL IGNORED: %u pulses\n", pulses);
    return;
  }

  CallState state = callState;
  if (state != CALL_IDLE && state != CALL_DIALING) {
    return;
  }

  uint8_t digit = (pulses == 10) ? 0 : (uint8_t)pulses;

  setTone(TONE_NONE);

  if (state == CALL_IDLE) {
    callState = CALL_DIALING;
    dialCount = 0;
    remoteNumber = 0;
  }

  if (dialCount < MAX_DIAL_DIGITS) {
    remoteNumber = remoteNumber * 10 + digit;
    dialCount++;
    lastDialActivityMs = millis();

    Serial.printf("DIAL DIGIT: %u (%u pulses)  number=%03u\n",
                  digit, pulses, remoteNumber);

    if (dialCount == 3) {
      IPAddress target;

      if (getPeer(remoteNumber, target) >= 0) {
        setRemoteIp(target);
        callState = CALL_OUTGOING_RING;
        setRotaryArmed(false);

        Serial.printf("CALLING %03u @ ", remoteNumber);
        printIP(getRemoteIp());
        Serial.println();

        clearAudioRing();
        setTone(TONE_RINGBACK);
        sendCall();
      } else {
        Serial.printf("NO SUCH PEER: %03u\n", remoteNumber);
        setTone(TONE_BUSY);
      }
    }
  }
}

// ===================== PHONE STATE =====================

void beginIdleOffHook() {
  callState = CALL_IDLE;
  remoteNumber = 0;
  dialCount = 0;
  setRemoteIp(IPAddress((uint32_t)0));
  setTone(TONE_DIAL);
  setRotaryArmed(true);
  clearAudioRing();

  Serial.println("OFF HOOK - DIAL TONE");
}

void answerIncomingCall() {
  stopPhysicalRinging();
  callState = CALL_CONNECTED;
  setTone(TONE_NONE);
  setRotaryArmed(false);
  clearAudioRing();

  Serial.printf("ANSWERING CALL FROM %03u\n", remoteNumber);
  sendAnswer();
}

void hangupCall(bool notifyPeer) {
  stopPhysicalRinging();

  // Stop audio before notifying, so the capture task cannot send another
  // frame to a call that is already over.
  callState = CALL_IDLE;

  if (notifyPeer && remoteNumber != 0) {
    sendHangup();
  }

  remoteNumber = 0;
  dialCount = 0;
  setRemoteIp(IPAddress((uint32_t)0));
  setTone(TONE_NONE);
  setRotaryArmed(false);
  clearAudioRing();

  Serial.println("CALL ENDED");
}

void processPhoneState() {
  /*
    IMPORTANT:
    Rotary dialing deliberately opens/closes SHK many times. Therefore we
    must NOT interpret every raw SHK transition as a handset transition.

    A genuine handset pickup/replacement is a stable state change. Rotary
    pulses are short-lived transitions and return to the previous state before
    OFFHOOK_CONFIRM_MS expires.

    We therefore keep:
      rawLevel       = instantaneous SHK input
      candidateLevel = state currently being considered
      phoneOffHook   = confirmed handset state

    SHK is ignored while the DC/DC is off and for SLIC_WAKE_MS after it
    starts. The idle poll in serviceSlicPower() holds the converter on
    once SHK stays high past that wake pulse.
  */
  static bool initialized = false;
  static bool candidateLevel = false;
  static uint32_t candidateSinceMs = 0;

  if (!slicOutputsValid()) return;
  slicSetShkInterrupt(true);

  bool rawLevel = digitalRead(PIN_SHK);
  uint32_t now = millis();

  if (!initialized) {
    initialized = true;
    candidateLevel = rawLevel;
    candidateSinceMs = now;
    // phoneOffHook stays false until SHK has been stable for
    // OFFHOOK_CONFIRM_MS. A handset already off-hook at boot then takes
    // the normal pickup path, including dial tone.
  } else {
    // A raw transition starts/restarts the debounce timer.
    if (rawLevel != candidateLevel) {
      candidateLevel = rawLevel;
      candidateSinceMs = now;
    }

    // Only commit a handset-state change after SHK has stayed there long
    // enough. Rotary dial pulses are much shorter than this.
    if (candidateLevel != phoneOffHook &&
        (now - candidateSinceMs) >= OFFHOOK_CONFIRM_MS) {

      phoneOffHook = candidateLevel;

      if (phoneOffHook) {
        // Genuine ON-HOOK -> OFF-HOOK transition.
        Serial.println("OFF HOOK");
        stopPhysicalRinging();

        if (callState == CALL_INCOMING_RING) {
          answerIncomingCall();
        } else if (callState == CALL_IDLE) {
          beginIdleOffHook();
        } else if (callState == CALL_OUTGOING_RING) {
          // We are still waiting for the remote answer.
          // Keep ringback running.
          Serial.println("LOCAL PHONE OFF HOOK DURING OUTGOING CALL");
        }
      } else {
        // Genuine OFF-HOOK -> ON-HOOK transition.
        Serial.println("ON HOOK");

        CallState state = callState;
        bool notify = (state == CALL_OUTGOING_RING ||
                       state == CALL_INCOMING_RING ||
                       state == CALL_CONNECTED);
        if (state != CALL_IDLE) {
          hangupCall(notify);
        } else {
          setTone(TONE_NONE);
          setRotaryArmed(false);
        }
      }
    }
  }

  // Dialing timeout: fewer than 3 digits -> reset and return dial tone.
  // Pulses still arriving belong to the next digit, so they hold the timer.
  if (phoneOffHook &&
      callState == CALL_DIALING &&
      dialCount > 0 &&
      !rotaryIsBusy() &&
      millis() - lastDialActivityMs > DIGIT_GAP_MS) {

    Serial.println("DIAL TIMEOUT - returning to dial tone");
    remoteNumber = 0;
    dialCount = 0;
    callState = CALL_IDLE;
    setTone(TONE_DIAL);
  }
}

// ===================== NETWORK RECEIVE =====================

void handleHelloPacket(const uint8_t* data, int len,
                       const IPAddress& senderIP) {
  if (len < (int)sizeof(HelloPacket)) return;

  HelloPacket p{};
  memcpy(&p, data, sizeof(p));

  IPAddress advertisedIP(p.ip[0], p.ip[1], p.ip[2], p.ip[3]);

  // Prefer advertised address, but fall back to source address if needed.
  if (advertisedIP == IPAddress(0, 0, 0, 0)) {
    advertisedIP = senderIP;
  }

  storePeer(p.phoneNumber, advertisedIP);
}

void handleControlPacket(const uint8_t* data, int len,
                         const IPAddress& senderIP) {
  if (len < (int)sizeof(ControlPacket)) return;

  ControlPacket p{};
  memcpy(&p, data, sizeof(p));

  switch (p.code) {
    case CTRL_CALL: {
      // A call addressed to this node.
      if (p.calledNumber != NODE_NUMBER) return;

      Serial.printf("INCOMING CALL FROM %03u\n", p.callerNumber);

      // Off-hook with dial tone is still CALL_IDLE. Ringing that handset
      // would put ring voltage on a phone that is already at the ear.
      if (callState != CALL_IDLE || phoneOffHook) {
        Serial.println("BUSY - rejecting incoming call");
        sendControlTo(senderIP, CTRL_BUSY, NODE_NUMBER, p.callerNumber);
        return;
      }

      remoteNumber = p.callerNumber;
      setRemoteIp(senderIP);
      callState = CALL_INCOMING_RING;
      setRotaryArmed(false);
      setTone(TONE_NONE);
      clearAudioRing();

      startPhysicalRinging();
      sendRinging();
      break;
    }

    case CTRL_RINGING:
      if (callState == CALL_OUTGOING_RING) {
        Serial.printf("REMOTE %03u IS RINGING\n", remoteNumber);
        setTone(TONE_RINGBACK);
      }
      break;

    case CTRL_ANSWER:
      if (callState == CALL_OUTGOING_RING) {
        Serial.printf("CALL ANSWERED BY %03u\n", remoteNumber);
        callState = CALL_CONNECTED;
        setTone(TONE_NONE);
        setRotaryArmed(false);
        clearAudioRing();
      }
      break;

    case CTRL_HANGUP:
      if (callState == CALL_IDLE || callState == CALL_DIALING) break;

      Serial.printf("REMOTE %03u HUNG UP\n", remoteNumber);
      stopPhysicalRinging();
      if (phoneOffHook) {
        beginIdleOffHook();
      } else {
        callState = CALL_IDLE;
        remoteNumber = 0;
        dialCount = 0;
        setRemoteIp(IPAddress((uint32_t)0));
        setTone(TONE_NONE);
        setRotaryArmed(false);
        clearAudioRing();
      }
      break;

    case CTRL_BUSY:
      if (callState != CALL_OUTGOING_RING && callState != CALL_CONNECTED) break;
      Serial.printf("REMOTE %03u IS BUSY\n", remoteNumber);
      callState = CALL_IDLE;
      setTone(TONE_BUSY);
      remoteNumber = 0;
      dialCount = 0;
      setRemoteIp(IPAddress((uint32_t)0));
      setRotaryArmed(false);
      break;

    case CTRL_REJECT:
      if (callState != CALL_OUTGOING_RING && callState != CALL_CONNECTED) break;
      Serial.println("CALL REJECTED");
      callState = CALL_IDLE;
      setTone(TONE_BUSY);
      remoteNumber = 0;
      dialCount = 0;
      setRemoteIp(IPAddress((uint32_t)0));
      setRotaryArmed(false);
      break;

    default:
      break;
  }
}

void processUdp() {
  if (!lockUdp()) return;

  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    unlockUdp();
    return;
  }

  uint8_t buffer[1500];

  if (packetSize > (int)sizeof(buffer)) {
    while (udp.available()) udp.read();
    unlockUdp();
    return;
  }

  int len = udp.read(buffer, sizeof(buffer));
  IPAddress senderIP = udp.remoteIP();
  unlockUdp();

  if (len < (int)sizeof(PacketHeader)) return;

  PacketHeader h{};
  memcpy(&h, buffer, sizeof(h));

  if (h.magic != MAGIC || h.version != PROTOCOL_VERSION) return;

  if (h.type == PKT_HELLO) {
    handleHelloPacket(buffer, len, senderIP);
    return;
  }

  if (h.type == PKT_CONTROL) {
    handleControlPacket(buffer, len, senderIP);
    return;
  }

  if (h.type == PKT_AUDIO) {
    size_t payloadBytes = size_t(h.samples) * sizeof(int16_t);

    if (h.samples > FRAME_SAMPLES ||
        payloadBytes > size_t(len - sizeof(PacketHeader))) {
      return;
    }

    if (callState != CALL_CONNECTED) return;

    rxPush(reinterpret_cast<int16_t*>(buffer + sizeof(PacketHeader)),
           h.samples);
  }
}

// ===================== I2S =====================

// Arduino-ESP32 2.0.0 has no mck_io_num. fixed_mclk still generates MCLK
// inside I2S1; these are the CLK_OUT pad selects later cores use for I2S1.
bool routeI2s1Mclk(int pin) {
  if (pin == 0) {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFFF);
  } else if (pin == 1) {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
    WRITE_PERI_REG(PIN_CTRL, 0xF0FF);
  } else if (pin == 3) {
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
    WRITE_PERI_REG(PIN_CTRL, 0xFF0F);
  } else {
    return false;
  }
  return true;
}

void setupI2SAudio() {
  i2s_config_t dac = {};

  dac.mode = (i2s_mode_t)(
      I2S_MODE_MASTER |
      I2S_MODE_TX |
      I2S_MODE_DAC_BUILT_IN
  );

  dac.sample_rate = SAMPLE_RATE;
  dac.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  dac.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
  dac.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  dac.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  dac.dma_buf_count = 8;
  dac.dma_buf_len = FRAME_SAMPLES;
  dac.use_apll = false;
  dac.tx_desc_auto_clear = true;
  dac.fixed_mclk = 0;

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &dac, 0, NULL);

  if (err != ESP_OK) {
    Serial.printf("I2S DAC driver install failed: %d\n", err);
    while (true) delay(1000);
  }

  // Right channel is GPIO25 only. i2s_set_pin(NULL) would also enable
  // the left DAC on GPIO26 and take that pin away from PCM1808 BCK.
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);

  Serial.println("I2S DAC initialized");

  // PCM1808 is an I2S slave. ESP32 is master and supplies MCLK, BCLK, and
  // LRCK. 32-bit stereo Philips slots, MCLK = 256 * fs. APLL is the one
  // audio PLL; the DAC port above does not use it.
  i2s_config_t adc = {};

  adc.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  adc.sample_rate = SAMPLE_RATE;
  adc.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  adc.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  adc.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  adc.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  adc.dma_buf_count = 8;
  adc.dma_buf_len = FRAME_SAMPLES * 2;
  adc.use_apll = true;
  adc.tx_desc_auto_clear = false;
  adc.fixed_mclk = SAMPLE_RATE * 256;

  err = i2s_driver_install(I2S_NUM_1, &adc, 0, NULL);

  if (err != ESP_OK) {
    Serial.printf("I2S PCM1808 driver install failed: %d\n", err);
    while (true) delay(1000);
  }

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_DIN;

  err = i2s_set_pin(I2S_NUM_1, &pins);

  if (err != ESP_OK) {
    Serial.printf("I2S PCM1808 pin config failed: %d\n", err);
    while (true) delay(1000);
  }

  // Print before the clock takes GPIO1, which is also the serial TX pin.
  Serial.println("I2S PCM1808 initialized");

  if (!routeI2s1Mclk(PIN_I2S_MCLK)) {
    Serial.printf("I2S MCLK pin %d is not GPIO0, GPIO1, or GPIO3\n", PIN_I2S_MCLK);
    while (true) delay(1000);
  }
}

void audioCaptureTask(void* parameter) {
  // One task, so these stay off the 4 KB stack. A 20 ms chunk is 160
  // stereo frames, 8 bytes each.
  static int32_t raw[FRAME_SAMPLES * 2];
  static int16_t pcm[FRAME_SAMPLES];

  while (true) {
    size_t bytesRead = 0;

    i2s_read(I2S_NUM_1, raw, sizeof(raw), &bytesRead, portMAX_DELAY);

    size_t samples = bytesRead / (sizeof(int32_t) * 2);
    if (samples > FRAME_SAMPLES) {
      samples = FRAME_SAMPLES;
    }

    CallState state = callState;
    bool offHook = phoneOffHook;

    if (state != CALL_CONNECTED || !offHook || samples == 0) {
      continue;
    }

    for (size_t i = 0; i < samples; ++i) {
      // Left slot is the even sample. The 24-bit value is left-aligned
      // in the 32-bit word, so the top 16 bits are the PCM sample.
      int32_t s = raw[i * 2] >> 16;
      s <<= PCM1808_GAIN_SHIFT;

      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;

      pcm[i] = (int16_t)s;
    }

    IPAddress dest = getRemoteIp();
    if (uint32_t(dest) == 0 || !lockUdp()) {
      continue;
    }

    PacketHeader h{};
    h.magic = MAGIC;
    h.type = PKT_AUDIO;
    h.version = PROTOCOL_VERSION;
    // Advance only after the datagram is queued. A failed send that still
    // bumps the sequence shows up on the receiver as a missing frame.
    h.sequence = txSequence;
    h.samples = samples;

    bool sent = false;
    if (udp.beginPacket(dest, UDP_PORT)) {
      udp.write(reinterpret_cast<uint8_t*>(&h), sizeof(h));
      udp.write(reinterpret_cast<uint8_t*>(pcm),
                samples * sizeof(int16_t));
      sent = udp.endPacket() == 1;
    }
    unlockUdp();

    if (sent) {
      txSequence++;
    }
  }
}

void audioPlaybackTask(void* parameter) {
  int16_t pcm[FRAME_SAMPLES];
  uint16_t dacWords[FRAME_SAMPLES];

  while (true) {
    CallState state = callState;
    bool offHook = phoneOffHook;

    if (state == CALL_CONNECTED) {
      size_t got = rxPop(pcm, FRAME_SAMPLES);

      for (size_t i = 0; i < FRAME_SAMPLES; ++i) {
        int16_t s = (i < got) ? pcm[i] : 0;

        int32_t u8 = (s >> 8) + 128;
        if (u8 < 0) u8 = 0;
        if (u8 > 255) u8 = 255;

        dacWords[i] = uint16_t(u8) << 8;
      }
    } else if (offHook &&
               (state == CALL_IDLE ||
                state == CALL_DIALING ||
                state == CALL_OUTGOING_RING)) {

      // Generate dial tone / ringback / busy tone locally.
      for (size_t i = 0; i < FRAME_SAMPLES; ++i) {
        int16_t s = generateToneSample();
        int32_t u8 = (s >> 8) + 128;

        if (u8 < 0) u8 = 0;
        if (u8 > 255) u8 = 255;

        dacWords[i] = uint16_t(u8) << 8;
      }
    } else {
      // Built-in DAC silence is midscale, not zero.
      for (size_t i = 0; i < FRAME_SAMPLES; ++i) {
        dacWords[i] = 0x8000;
      }
    }

    size_t written = 0;
    i2s_write(I2S_NUM_0, dacWords, sizeof(dacWords),
              &written, portMAX_DELAY);
  }
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32 ROTARY EXCHANGE");
  Serial.printf("Phone number: %03u\n", NODE_NUMBER);
  Serial.println("======================================");

  udpMutex = xSemaphoreCreateMutex();
  if (udpMutex == nullptr) {
    Serial.println("UDP mutex failed");
  }

  pinMode(PIN_SHK, INPUT);

  // External NPN inverts F/R and RM:
  // GPIO HIGH = AG1170 input LOW
  // GPIO LOW  = AG1170 input HIGH
  // PD is the opposite open-collector sense: GPIO HIGH powers the DC/DC down.
  // Latch that level before the pin becomes an output.
  digitalWrite(PIN_PD, HIGH);
  pinMode(PIN_PD, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_RM, OUTPUT);

  digitalWrite(PIN_FR, LOW);  // F/R high
  digitalWrite(PIN_RM, HIGH); // RM low

  slicPowerUp();
  while (!slicOutputsValid()) delay(1);
  shkIsLow = (digitalRead(PIN_SHK) == LOW);
  slicSetShkInterrupt(true);
  Serial.println("AG1170 DC/DC up; idle on-hook powers it down between SHK polls");

  // Join Wi-Fi while GPIO1 is still the serial pin. setupI2SAudio()
  // then routes the PCM1808 master clock onto GPIO1 and the monitor stops.
  WiFi.mode(WIFI_STA);
  // Modem sleep drops UDP while the radio dozes. Audio is a steady
  // 50 packets/s, so the STA stays awake for the life of the link.
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  WiFi.setSleep(false);

  udp.begin(UDP_PORT);
  sendHello();
  Serial.println("Discovery hello sent. Serial stops when the codec clock starts.");

  setupI2SAudio();

  // Core 0 runs the Wi-Fi stack. Keep the 20 ms audio work on core 1
  // so a transmit does not stall the radio.
  xTaskCreatePinnedToCore(
      audioCaptureTask, "AudioCapture",
      4096, NULL, 2, NULL, 1);

  xTaskCreatePinnedToCore(
      audioPlaybackTask, "AudioPlayback",
      4096, NULL, 2, NULL, 1);
}

void serviceWiFi() {
  static uint32_t lastCheckMs = 0;
  static bool wasUp = true;

  if (WiFi.status() == WL_CONNECTED) {
    if (!wasUp) {
      if (!lockUdp()) return;
      udp.begin(UDP_PORT);
      unlockUdp();
      WiFi.setSleep(false);
      wasUp = true;
      Serial.print("Wi-Fi reconnected: ");
      Serial.println(WiFi.localIP());
      sendHello();
    }
    return;
  }

  uint32_t now = millis();
  if ((uint32_t)(now - lastCheckMs) < 5000) return;
  lastCheckMs = now;

  if (wasUp) {
    Serial.println("Wi-Fi lost, reconnecting");
    wasUp = false;
  }
  WiFi.reconnect();
}

void loop() {
  static uint32_t lastHello = 0;
  static uint32_t lastPeerExpiry = 0;

  serviceWiFi();
  processUdp();
  processPhoneState();
  processRotaryDial();
  servicePhysicalRinging();
  serviceSlicPower();

  if (millis() - lastHello >= HELLO_INTERVAL_MS &&
      WiFi.status() == WL_CONNECTED) {
    lastHello = millis();
    sendHello();
  }

  if (millis() - lastPeerExpiry >= 5000) {
    lastPeerExpiry = millis();
    expirePeers();
  }

  delay(1);
}
