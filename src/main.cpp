#include <Arduino.h>
#include <pins_arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define PIN_A1 13
#define PIN_A2 15
#define PIN_A3 14
#define PIN_A4 18
#define PIN_C1 19
#define PIN_MSYN 21
#define PIN_SSYN 22
#define PIN_D00 23
#define PIN_D01 25
#define PIN_D02 26
#define PIN_D03 27
#define PIN_D04 32
#define PIN_D05 33
#define PIN_D06 34
#define PIN_D07 35
#define PIN_D08 4
#define PIN_D09 5

#define LOGIC_HIGH 0
#define LOGIC_LOW 1
#define LOGIC(x) (!(x))

#define STATE_IDLE 0
#define STATE_DEBONK 1
#define STATE_S8 2
#define STATE_S7 3
#define STATE_S6 4
#define STATE_S5 5
#define STATE_S4 6
#define STATE_S3 7
#define STATE_S2 8
#define STATE_S1 9
#define STATE_S0 10
#define STATE_R8 11
#define STATE_R7 12
#define STATE_R6 13
#define STATE_R5 14
#define STATE_R4 15
#define STATE_R3 16
#define STATE_R2 17
#define STATE_R1 18
#define STATE_R0 19
#define STATE_TRANSMIT 20
#define STATE_RELEASE 21

#define MS_TICKS(x) ((TickType_t)(x) / portTICK_PERIOD_MS)

int INOUT_DATA_PINS[] = {PIN_D00, PIN_D01, PIN_D02, PIN_D03, PIN_D04, PIN_D05};

char ssid[] = "ssid";
char password[] = "password";
int bit = 0;

int state = STATE_IDLE;
bool is_express = false;

// these must be zero
uint8_t distpts_cache[4] = {0, 0, 0, 0};

struct card_t
{
  uint16_t relays[9][16];
};

card_t read_card, transmit_card;

char request_body[1024];

/* Distribute points
"S4",     "S3",     "S2",     "S1",     "S0",     "RSV0.5",
"RSV1.0", "S8",     "S7",     "S6",     "S5",     "TRC",
"MB",     "CMJ",    "CMN",    "ARLK",   "STRA",   "STR",
"RSV3.0", "RSV3.1", "RSV3.2", "RSV3.3", "RSV3.4", "RSV3.5"
*/

#define DIST_S0 0
#define DIST_S1 1
#define DIST_S2 2
#define DIST_S3 3
#define DIST_S4 4
#define DIST_S5 5
#define DIST_S6 6
#define DIST_S7 7
#define DIST_S8 8
#define DIST_TRC 9
#define DIST_MB 10
int distpt_rows[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2};
int distpt_cols[] = {4, 3, 2, 1, 0, 4, 3, 2, 1, 5, 0};

/* Scan points
7,         6,         5,          4,        3,   2,   1,   0,   "STRA1",   "STR",
15,        14,        13,         12,       11,  10,  9,   8,   "TRC",     "SPL",
23,        22,        21,         20,       19,  18,  17,  16,  "ROS",     "MB",
"BWX1",    "BWX0",    29,         28,       27,  26,  25,  24,  "RSV3.8",  "RSV3.9",
37,        36,        35,         34,       33,  32,  31,  30,  "RSV4.8",  "RSV4.9",
45,        44,        43,         42,       41,  40,  39,  38,  "RSV5.8",  "RSV5.9",
53,        52,        51,         50,       49,  48,  47,  46,  "RSV6.8",  "RSV6.9",
61,        60,        59,         58,       57,  56,  55,  54,  "RSV7.8",  "RSV7.9",
69,        68,        67,         66,       65,  64,  63,  62,  "RSV8.8",  "RSV8.9",
77,        76,        75,         74,       73,  72,  71,  70,  "RSV9.8",  "RSV9.9",
85,        84,        83,         82,       81,  80,  79,  78,  "RSV10.8", "RSV10.9",
91,        90,        "BWX3",     "BWX2",   89,  88,  87,  86,  "RSV11.8", "RSV11.9",
99,        98,        97,         96,       95,  94,  93,  92,  "RSV12.8", "RSV12.9",
107,       106,       105,        104,      103, 102, 101, 100, "RSV13.8", "RSV13.9",
115,       114,       113,        112,      111, 110, 109, 108, "RSV14.8", "RSV14.9",
"RSV15.0", "RSV15.1", "RSV15.2", "RSV15.3", 119, 118, 117, 116, "RSV15.8", "RSV15.9"
*/

#define SCAN_STRA1 0
#define SCAN_STR 1
#define SCAN_MB 2
int scanpt_rows[] = {0, 0, 2};
int scanpt_cols[] = {8, 9, 9};

QueueHandle_t queue;

uint16_t read_row(int row)
{
  // for (int pin : INOUT_DATA_PINS)
  // {
  //   pinMode(pin, INPUT);
  // }

  digitalWrite(PIN_C1, LOGIC_LOW);

  digitalWrite(PIN_A1, LOGIC(row & 1));
  digitalWrite(PIN_A2, LOGIC((row >> 1) & 1));
  digitalWrite(PIN_A3, LOGIC((row >> 2) & 1));
  digitalWrite(PIN_A4, LOGIC((row >> 3) & 1));

  digitalWrite(PIN_MSYN, LOGIC_HIGH);

  int j = 0;
  int64_t start = esp_timer_get_time();
  while (digitalRead(PIN_SSYN) == LOGIC_LOW)
  {
    vTaskDelay(MS_TICKS(1));
    j += 1;
    if (j >= 32)
    {
      int64_t end = esp_timer_get_time();
      Serial.print("read timed out after ");
      Serial.print(end - start);
      Serial.println(" us");
      return 0xffff;
    }
  }

  int d0 = LOGIC(digitalRead(PIN_D00));
  int d1 = LOGIC(digitalRead(PIN_D01));
  int d2 = LOGIC(digitalRead(PIN_D02));
  int d3 = LOGIC(digitalRead(PIN_D03));
  int d4 = LOGIC(digitalRead(PIN_D04));
  int d5 = LOGIC(digitalRead(PIN_D05));
  int d6 = LOGIC(digitalRead(PIN_D06));
  int d7 = LOGIC(digitalRead(PIN_D07));
  int d8 = LOGIC(digitalRead(PIN_D08));
  int d9 = LOGIC(digitalRead(PIN_D09));

  digitalWrite(PIN_MSYN, LOGIC_LOW);

  digitalWrite(PIN_A1, LOGIC_LOW);
  digitalWrite(PIN_A2, LOGIC_LOW);
  digitalWrite(PIN_A3, LOGIC_LOW);
  digitalWrite(PIN_A4, LOGIC_LOW);

  return d0 | (d1 << 1) | (d2 << 2) | (d3 << 3) | (d4 << 4) |
         (d5 << 5) | (d6 << 6) | (d7 << 7) | (d8 << 8) | (d9 << 9);
}

void write_row(int rownum)
{
  // for (int pin : INOUT_DATA_PINS)
  // {
  //   pinMode(pin, OUTPUT);
  // }

  digitalWrite(PIN_A1, LOGIC(rownum & 1));
  digitalWrite(PIN_A2, LOGIC((rownum >> 1) & 1));

  int data = distpts_cache[rownum];
  digitalWrite(PIN_D00, LOGIC(data & 1));
  digitalWrite(PIN_D01, LOGIC((data >> 1) & 1));
  digitalWrite(PIN_D02, LOGIC((data >> 2) & 1));
  digitalWrite(PIN_D03, LOGIC((data >> 3) & 1));
  digitalWrite(PIN_D04, LOGIC((data >> 4) & 1));
  digitalWrite(PIN_D05, LOGIC((data >> 5) & 1));

  digitalWrite(PIN_C1, LOGIC_HIGH);

  digitalWrite(PIN_MSYN, LOGIC_HIGH);

  int j = 0;
  int64_t start = esp_timer_get_time();
  while (digitalRead(PIN_SSYN) == LOGIC_LOW)
  {
    vTaskDelay(MS_TICKS(1));
    j += 1;
    if (j >= 32)
    {
      int64_t end = esp_timer_get_time();
      Serial.print("write timed out after ");
      Serial.print(end - start);
      Serial.println(" us");
      return;
    }
  }
  // delay slightly to allow write?
  // vTaskDelay(MS_TICKS(1));

  int pins_to_reset[] = {PIN_MSYN, PIN_C1, PIN_A1, PIN_A2, PIN_A3, PIN_A4,
                         PIN_D00, PIN_D01, PIN_D02, PIN_D03, PIN_D04, PIN_D05};
  for (int pin : pins_to_reset)
  {
    digitalWrite(pin, LOGIC_LOW);
  }
}

void write_distpt(int distpt, bool val)
{
  int row = distpt_rows[distpt];
  int col = distpt_cols[distpt];
  distpts_cache[row] &= ~(1 << col);
  distpts_cache[row] |= val << col;

  write_row(row);
}

void read_relay_row(int group)
{
  for (int row = 0; row < 16; row++)
  {
    uint16_t val = read_row(row);
    read_card.relays[group][row] = val;
  }

  write_distpt(group, 0);
}

bool read_scanpt(int scanpt)
{
  int row = scanpt_rows[scanpt];
  int col = scanpt_cols[scanpt];

  uint16_t val = read_row(row);
  return (val >> col) & 1;
}

inline TickType_t state_idle()
{
  uint16_t val = read_row(scanpt_rows[SCAN_STR]);

  if ((val >> scanpt_cols[SCAN_STRA1]) & 1)
  {
    state = STATE_DEBONK;
    Serial.println("state: moving to debonk");
    return MS_TICKS(64);
  }
  else if ((val >> scanpt_cols[SCAN_STR]) & 1)
  {
    state = STATE_DEBONK;
    Serial.println("state: moving to debonk");
    return MS_TICKS(64);
  }
  return MS_TICKS(100);
}

inline TickType_t state_debonk()
{
  if (read_scanpt(SCAN_STR))
  {
    is_express = false;
    state = STATE_S8;
    return MS_TICKS(100);
  }
  else if (read_scanpt(SCAN_STRA1))
  {
    is_express = true;
    state = STATE_S8;
    return MS_TICKS(100);
  }
  else
  {
    Serial.printf("state: debonk failed, returning to idle\n");
    state = STATE_IDLE;
    return MS_TICKS(100);
  }
}

void state_machine(void *_params)
{
  TickType_t delay = 0;
  while (1)
  {
    delay = MS_TICKS(32);
    switch (state)
    {
    case STATE_IDLE:
      delay = state_idle();
      break;
    case STATE_DEBONK:
      delay = state_debonk();
      break;
    case STATE_S8:
      state = STATE_R8;
      write_distpt(DIST_S8, 1);
      break;
    case STATE_S7:
      state = STATE_R7;
      write_distpt(DIST_S7, 1);
      break;
    case STATE_S6:
      state = STATE_R6;
      write_distpt(DIST_S6, 1);
      break;
    case STATE_S5:
      state = STATE_R5;
      write_distpt(DIST_S5, 1);
      break;
    case STATE_S4:
      state = STATE_R4;
      write_distpt(DIST_S4, 1);
      break;
    case STATE_S3:
      state = STATE_R3;
      write_distpt(DIST_S3, 1);
      break;
    case STATE_S2:
      state = STATE_R2;
      write_distpt(DIST_S2, 1);
      break;
    case STATE_S1:
      state = STATE_R1;
      write_distpt(DIST_S1, 1);
      break;
    case STATE_S0:
      state = STATE_R0;
      write_distpt(DIST_S0, 1);
      break;
    case STATE_R8:
      state = STATE_S7;
      read_relay_row(8);
      break;
    case STATE_R7:
      state = STATE_S6;
      read_relay_row(7);
      break;
    case STATE_R6:
      state = STATE_S5;
      read_relay_row(6);
      break;
    case STATE_R5:
      state = STATE_S4;
      read_relay_row(5);
      break;
    case STATE_R4:
      state = STATE_S3;
      read_relay_row(4);
      break;
    case STATE_R3:
      state = STATE_S2;
      read_relay_row(3);
      break;
    case STATE_R2:
      state = STATE_S1;
      read_relay_row(2);
      break;
    case STATE_R1:
      state = STATE_S0;
      read_relay_row(1);
      break;
    case STATE_R0:
      state = STATE_TRANSMIT;
      read_relay_row(0);
      break;
    case STATE_TRANSMIT:
      state = STATE_RELEASE;
      write_distpt(DIST_TRC, 1);
      if (xQueueSendToBack(queue, &read_card, 0) != pdTRUE) {
        Serial.println("state: failed to enqueue");
      }
      break;
    case STATE_RELEASE:
      state = STATE_IDLE;
      write_distpt(DIST_TRC, 0);
      Serial.println("state: released");
      break;
    }
    vTaskDelay(delay);
  }
}

void transmit(void *_params)
{

  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.print("connected to wifi as ");
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  write_distpt(DIST_MB, 1);
  vTaskDelay(MS_TICKS(500));
  write_distpt(DIST_MB, 0);

  HTTPClient client;
  while (1)
  {
    while (!WiFi.isConnected()) {
      vTaskDelay(1000);
      Serial.println("transmit: reconnecting...");
      write_distpt(DIST_MB, 1);
      vTaskDelay(MS_TICKS(500));
      write_distpt(DIST_MB, 0);
    }
    BaseType_t res = xQueueReceive(queue, &transmit_card, portMAX_DELAY);
    if (res == pdFALSE)
    {
      Serial.println("transmit: continuing loop (this is exceptionally rare :3)");
      continue;
    }
    client.begin("http://0.0.0.0:5000/card");
    for (int i = 0; i < 143; i++)
    {
      uint16_t val = transmit_card.relays[i / 16][i % 16];
      sprintf(request_body + i * 5, "%04x,", val);
    }
    uint16_t val = transmit_card.relays[8][15];
    sprintf(request_body + 143 * 5, "%04x", val);

    Serial.println("transmit: sending request");
    Serial.println(request_body);

    int rc = client.POST(request_body);

    Serial.print("transmit: got response: ");
    Serial.println(rc);
    Serial.println("transmit: done");
    client.end();

    memset(request_body, 0, 1024);
  }
}

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(PIN_A1, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_A2, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_A3, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_A4, OUTPUT_OPEN_DRAIN);

  pinMode(PIN_C1, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_MSYN, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_SSYN, INPUT);

  pinMode(PIN_D00, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D01, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D02, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D03, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D04, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D05, INPUT | OUTPUT_OPEN_DRAIN);
  pinMode(PIN_D06, INPUT);
  pinMode(PIN_D07, INPUT);
  pinMode(PIN_D08, INPUT);
  pinMode(PIN_D09, INPUT);

  queue = xQueueCreate(8, sizeof(card_t));

  // Audibly ready
  write_distpt(DIST_MB, 1);
  vTaskDelay(MS_TICKS(100));
  write_distpt(DIST_MB, 0);
  vTaskDelay(MS_TICKS(100));
  write_distpt(DIST_MB, 1);
  vTaskDelay(MS_TICKS(100));
  write_distpt(DIST_MB, 0);
  vTaskDelay(MS_TICKS(100));
  write_distpt(DIST_MB, 1);
  vTaskDelay(MS_TICKS(100));
  write_distpt(DIST_MB, 0);

  Serial.println("creating tasks");
  xTaskCreate(state_machine, "sm", 2048, 0, 10, 0);
  xTaskCreate(transmit, "transmit", 32000, 0, 5, 0);
  Serial.println("finished creating tasks");
}

void loop(){}
