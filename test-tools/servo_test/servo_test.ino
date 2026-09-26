// Bench tool: drive one servo from the serial monitor and read the pot wiper.
// Arduino Uno, 115200 baud, newline line ending. Type ? for commands.
//
// Servo signal on D9, wiper (through the divider, see README) on A0.

#include <Servo.h>

const int SERVO_PIN = 9;
const int WIPER_PIN = A0;
// Measured on the first modded MG996R: 1000 us of pulse = 90 deg of real rotation, linear
// from 400 to 2600 us. So 0-180 deg is 500-2500 us.
const int US_MIN = 500;     // pulse width for 0 deg
const int US_MAX = 2500;    // pulse width for 180 deg

Servo servo;
int current_us = 1500;      // target pulse width
float sent_us = 1500;       // pulse width being sent right now (ramps toward the target)
float slew = 500;           // us per second (500 = ~90 deg/s over 1000-2000 us), 0 = jump
unsigned long last_step = 0;
bool attached = false;
float vref_mv = 5000.0;    // measure the Uno's 5V pin and set it with "v"
float divider = 1.0;       // wiper volts = A0 volts * divider (1.0 = wiper straight to A0)

// Wiper calibration of the first modded servo (2026-09-26, same at 5 V and 6 V supply).
const float CAL_MV_AT_1000 = 958.0;
const float CAL_MV_PER_US = 1.364;

char line[48];
byte len = 0;

int deg_to_us(long deg) {
  return map(constrain(deg, 0, 180), 0, 180, US_MIN, US_MAX);
}

// A big jump makes the servo pull its full stall current at once, which can sag the
// supply enough to reset the servo's own controller. Ramping avoids that.
void step_servo() {
  if (!attached) return;
  unsigned long now = millis();
  float dt = (now - last_step) / 1000.0;
  last_step = now;
  if (slew <= 0) {
    sent_us = current_us;
  } else {
    float max_step = slew * dt;
    float err = current_us - sent_us;
    sent_us += constrain(err, -max_step, max_step);
  }
  servo.writeMicroseconds((int)(sent_us + 0.5));
}

float read_wiper_mv(int *raw_out);

// Pulse width that matches where the shaft is now, from the wiper.
int measured_us() {
  float mv = read_wiper_mv(NULL);
  return constrain((int)(1000 + (mv - CAL_MV_AT_1000) / CAL_MV_PER_US), 400, 2600);
}

void set_us(int us) {
  current_us = constrain(us, 400, 2600);
  if (!attached) {
    // Resume from where the servo actually is. If the first pulse is well below the held
    // position, this MG996R ignores all commands until sent past that position.
    sent_us = measured_us();
    servo.writeMicroseconds((int)sent_us);  // set before attach so the first pulse is this
    servo.attach(SERVO_PIN, 400, 2600);
    attached = true;
    Serial.print("resume from ");
    Serial.print((int)sent_us);
    Serial.println(" us (measured)");
  }
  last_step = millis();
  // ramp there before returning
  while ((int)(sent_us + 0.5) != current_us) {
    step_servo();
    delay(5);
  }
  step_servo();
}

// Average of 64 readings, in mV at the wiper.
float read_wiper_mv(int *raw_out) {
  long sum = 0;
  analogRead(WIPER_PIN);  // settle the ADC mux
  for (int i = 0; i < 64; i++) {
    sum += analogRead(WIPER_PIN);
  }
  int raw = sum / 64;
  if (raw_out) *raw_out = raw;
  return raw * vref_mv / 1023.0 * divider;
}

void report() {
  int raw;
  float mv = read_wiper_mv(&raw);
  float deg = (current_us - US_MIN) * 180.0 / (US_MAX - US_MIN);
  Serial.print(attached ? "us " : "us (off) ");
  Serial.print(current_us);
  Serial.print("  deg ");
  Serial.print(deg, 1);
  Serial.print("  adc ");
  Serial.print(raw);
  Serial.print("  wiper ");
  Serial.print(mv, 0);
  Serial.println(" mV");
}

// Min/max/spread of the wiper over 1 s, to see noise while holding or under load.
void noise() {
  int lo = 1023, hi = 0;
  long sum = 0;
  unsigned long end = millis() + 1000;
  int n = 0;
  while (millis() < end) {
    int r = analogRead(WIPER_PIN);
    lo = min(lo, r);
    hi = max(hi, r);
    sum += r;
    n++;
  }
  float k = vref_mv / 1023.0 * divider;
  Serial.print("1 s, ");
  Serial.print(n);
  Serial.print(" samples: min ");
  Serial.print(lo * k, 0);
  Serial.print("  avg ");
  Serial.print(sum / (float)n * k, 0);
  Serial.print("  max ");
  Serial.print(hi * k, 0);
  Serial.print(" mV  (spread ");
  Serial.print((hi - lo) * k, 0);
  Serial.println(" mV)");
}

// Step through angles and print a CSV table: deg,us,adc,wiper_mv
void sweep(long from, long to, long step, long wait_ms) {
  if (step <= 0) step = 10;
  if (wait_ms < 100) wait_ms = 500;
  Serial.println("deg,us,adc,wiper_mv");
  long dir = from <= to ? 1 : -1;
  for (long d = from; dir > 0 ? d <= to : d >= to; d += dir * step) {
    set_us(deg_to_us(d));
    delay(wait_ms);
    int raw;
    float mv = read_wiper_mv(&raw);
    Serial.print(d);
    Serial.print(',');
    Serial.print(current_us);
    Serial.print(',');
    Serial.print(raw);
    Serial.print(',');
    Serial.println(mv, 0);
  }
}

// Time our own output on D9 (reading an output pin returns its real level), to check the
// pulse width and frame period the servo is actually being sent.
void pulse_check() {
  if (!attached) {
    Serial.println("servo off, no pulses");
    return;
  }
  unsigned long high = pulseIn(SERVO_PIN, HIGH, 50000UL);
  unsigned long low = pulseIn(SERVO_PIN, LOW, 50000UL);
  Serial.print("D9 high ");
  Serial.print(high);
  Serial.print(" us, low ");
  Serial.print(low);
  Serial.print(" us, period ");
  Serial.print(high + low);
  Serial.print(" us (");
  Serial.print(high + low ? 1000000.0 / (high + low) : 0, 1);
  Serial.println(" Hz)");
}

// Is A0 really on the wiper? With the internal ~35k pull-up on, a floating pin reads ~5 V,
// while a pin tied to the pot (a few k) barely moves.
void float_check() {
  int raw_off, raw_on;
  read_wiper_mv(&raw_off);
  pinMode(WIPER_PIN, INPUT_PULLUP);
  delay(5);
  read_wiper_mv(&raw_on);
  pinMode(WIPER_PIN, INPUT);
  Serial.print("A0 raw: pull-up off ");
  Serial.print(raw_off);
  Serial.print(", on ");
  Serial.print(raw_on);
  Serial.println(raw_on - raw_off > 300 ? "  -> A0 looks FLOATING (not connected to the wiper)"
                                        : "  -> A0 is connected to something low-impedance");
}

void help() {
  Serial.println(F("a <deg>        go to angle 0-180"));
  Serial.println(F("u <us>         go to pulse width 400-2600 us"));
  Serial.println(F("s <from> <to> <step> <ms>   sweep, prints CSV (default step 10, 500 ms)"));
  Serial.println(F("r              read wiper once"));
  Serial.println(F("n              wiper noise over 1 s"));
  Serial.println(F("p              measure the pulses on D9 (width, period)"));
  Serial.println(F("f              check whether A0 is actually connected (pull-up test)"));
  Serial.println(F("o              servo off (no pulses, goes limp)"));
  Serial.println(F("w <us/s>       move speed, default 500 (~90 deg/s); 0 = jump straight there"));
  Serial.println(F("v <mV>         set measured 5V pin voltage (default 5000)"));
  Serial.println(F("k <x100>       set divider ratio x100 (200 = 100k/100k, 100 = no divider)"));
  Serial.print(F("pulse range for 0-180 deg: "));
  Serial.print(US_MIN);
  Serial.print(F("-"));
  Serial.print(US_MAX);
  Serial.println(F(" us"));
}

void handle(char *cmd) {
  char c = cmd[0];
  char *p = cmd + 1;
  long a = strtol(p, &p, 10);
  long b = strtol(p, &p, 10);
  long s = strtol(p, &p, 10);
  long w = strtol(p, &p, 10);

  switch (c) {
    case 'a': set_us(deg_to_us(a)); delay(300); report(); break;
    case 'u': set_us(a); delay(300); report(); break;
    case 's': sweep(a, b, s, w); break;
    case 'r': report(); break;
    case 'n': noise(); break;
    case 'p': pulse_check(); break;
    case 'f': float_check(); break;
    case 'o': servo.detach(); attached = false; Serial.println("servo off"); break;
    case 'w': slew = max(0L, a); Serial.print("speed "); Serial.print(slew, 0); Serial.println(" us/s"); break;
    case 'v': if (a > 3000) vref_mv = a; Serial.print("vref "); Serial.println(vref_mv, 0); break;
    case 'k': if (a >= 100) divider = a / 100.0; Serial.print("divider "); Serial.println(divider, 2); break;
    case '?': case 'h': help(); break;
    default: if (c) Serial.println("? for help");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("servo_test ready, servo off. ? for help"));
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[len] = 0;
      if (len) handle(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}
