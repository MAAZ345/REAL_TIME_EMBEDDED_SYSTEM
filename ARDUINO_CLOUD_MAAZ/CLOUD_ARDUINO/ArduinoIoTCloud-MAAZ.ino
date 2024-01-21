/*
  CloudHeartRate bPM;
  CloudLight lED;
  int steps;
  bool _switch_;
  CloudTime rTC;
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include "time.h"
#include "sntp.h"

#include "thingProperties.h"




#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define BPM_PIN 32
#define Button_PIN 34
const char* ssid       = "StoodLemming7";
const char* password   = "Faiez123";
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 5 * 3600;
const int   daylightOffset_sec = 3600;


Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


QueueHandle_t steps_Queue = NULL, BPM_Queue = NULL, time_Queue = NULL, screen_Queue = NULL;
QueueSetHandle_t xQueueSet = NULL;
TaskHandle_t HandleDisplay = NULL;
SemaphoreHandle_t I2C;
TimerHandle_t xOneShotTimer;
BaseType_t xTimerStarted;

void TaskBlink( void *pvParameters );
void Display( void *pvParameters );
void BPM_task(void *pvParameters) ;
void step_func(void *pvParameters);
void button_press();
void printLocalTime(void *pvParameters);
void initMPU();


typedef struct timeData
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
} timeData;

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");

  
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);

  Serial.println("Starting0");
  ArduinoCloud.printDebugInfo();

  pinMode(Button_PIN, INPUT_PULLUP);
  pinMode(BPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Button_PIN), button_press, FALLING);

  initMPU();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  I2C = xSemaphoreCreateMutex();
  time_Queue = xQueueCreate( 1, sizeof( timeData * ) );
  BPM_Queue = xQueueCreate( 1, sizeof( int * ) );
  steps_Queue = xQueueCreate( 1, sizeof( int * ) );
  screen_Queue = xQueueCreate( 1, sizeof( bool * ) );
  xQueueSet = xQueueCreateSet( 1 * 4 );


  xQueueAddToSet( time_Queue, xQueueSet );
  xQueueAddToSet( BPM_Queue, xQueueSet );
  xQueueAddToSet( steps_Queue, xQueueSet );
  xQueueAddToSet( screen_Queue, xQueueSet );

  xOneShotTimer = xTimerCreate("OneShot", pdTICKS_TO_MS(30000), pdFALSE, 0,
                               prvOneShotTimerCallback );

  Serial.println("Starting");
  delay(1000);
  //core 0
  uint32_t blink_delay = 700;
  xTaskCreatePinnedToCore(TaskBlink,  "Task Blink",  2048,  (void*) &blink_delay,  2,  NULL, 0);
  xTaskCreatePinnedToCore(BPM_task,  "BPM",  2048,  NULL ,  2,  NULL, 0);
  xTaskCreatePinnedToCore(printLocalTime,  "ntp",  2048,  NULL ,  2,  NULL, 0);

  //core 1
  xTaskCreatePinnedToCore(Display, "Display", 16000, NULL, 1, &HandleDisplay, 1);
  xTaskCreatePinnedToCore(step_func,  "step",  2048,  NULL ,  2,  NULL, 1);
}

void loop() {
}

void TaskBlink(void *pvParameters) { // This is a task.
  uint32_t blink_delay = *((uint32_t*)pvParameters);
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(blink_delay));
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(pdMS_TO_TICKS(blink_delay));
  }
}


void onSwitchChange()  {
  static bool screen_state = true;
  if (xTimerStarted != pdPASS)
    xTimerStarted = xTimerStart( xOneShotTimer, 0 );
  else if (xTimerStarted == pdPASS)
    xTimerReset( xOneShotTimer, 0 );
  xQueueSend( screen_Queue, &screen_state, 0 );
}
void button_press() {
  static bool screen_state = true;
  if (xTimerStarted != pdPASS)
    xTimerStarted = xTimerStart( xOneShotTimer, 0 );
  else if (xTimerStarted == pdPASS)
    xTimerReset( xOneShotTimer, 0 );
  xQueueSend( screen_Queue, &screen_state, 0);

}
static void prvOneShotTimerCallback( TimerHandle_t xTimer ) {
  bool screen_state = false;
  Serial.println("oneshot timer");
  xQueueSend( screen_Queue, &screen_state, 0 );
}
void Display(void *pvParameters) {
  QueueSetMemberHandle_t xHandle;
  static timeData Display_time;
  static int Display_BPM = 0;
  static int Display_steps = 0;
  static bool screen_state = false;
  while (1) {
    xHandle = xQueueSelectFromSet( xQueueSet, 0 );
    if ( xHandle == NULL ) {
      ;
    }
    else if ( xHandle == ( QueueSetMemberHandle_t ) time_Queue ) {
      xQueueReceive( time_Queue, &Display_time, 0 );
    }
    else if ( xHandle == ( QueueSetMemberHandle_t ) BPM_Queue ) {
      xQueueReceive( BPM_Queue, &Display_BPM, 0 );
    }
    else if ( xHandle == ( QueueSetMemberHandle_t ) steps_Queue ) {
      xQueueReceive( steps_Queue, &Display_steps, 0 );
    }
    else if ( xHandle == ( QueueSetMemberHandle_t ) screen_Queue ) {
      xQueueReceive( screen_Queue, &screen_state, 0 );
      Serial.print("state received:");
      Serial.println(screen_state);
    }
    if (xSemaphoreTake(I2C, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (screen_state) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(1, 1);
        display.print("BPM: ");
        display.print(Display_BPM, DEC);
        display.setCursor(60, 1);
        display.print("STEPS:");
        display.print(Display_steps, DEC);
        display.setCursor(30, 15);
        display.print(Display_time.hours, DEC);
        display.print(':');
        display.print(Display_time.minutes, DEC);
        display.print(':');
        display.print(Display_time.seconds, DEC);
        display.display();
        xSemaphoreGive(I2C);
      }
      else {
        display.clearDisplay();
        display.display();
        xSemaphoreGive(I2C);
      }
    }
    lED = (bool)digitalRead(LED_BUILTIN);
    steps = Display_steps;
    bPM = Display_BPM;
    rTC = Display_time.hours * 3600 + Display_time.minutes * 60 + Display_time.seconds;
    ArduinoCloud.update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


void BPM_task(void *pvParameters) {
  static uint8_t BPM_pin_curr = 1;
  static uint8_t BPM_pin_prev = 1;
  static uint32_t prev_millis = 0;
  static int BPM_counter = 0;
  int BPM = 0;
  while (1) {
    BPM_pin_curr = digitalRead(BPM_PIN);
    if (BPM_pin_curr + BPM_pin_prev == 1)
      BPM_counter++;
    if (millis() - prev_millis > 15000) {
      prev_millis = millis();
      BPM = BPM_counter * 4;
      BPM_counter = 0;
      xQueueSend( BPM_Queue, &BPM, pdMS_TO_TICKS(10) );
    }
    BPM_pin_prev = BPM_pin_curr;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void step_func(void *pvParameters) {
  static int steps_local = 0;
  float acc_x, acc_y, acc_z;
  float acc = 0;
  sensors_event_t a, g, temp;
  while (1) {
    if (xSemaphoreTake(I2C, pdMS_TO_TICKS(50)) == pdTRUE) {
      mpu.getEvent(&a, &g, &temp);
      acc_x = a.acceleration.x;
      acc_y = a.acceleration.y;
      acc_z = a.acceleration.z;
      acc = (acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z);
      if (acc > (144)) {
        steps_local++;
        xQueueSend( steps_Queue, &steps_local, pdMS_TO_TICKS(10) );
      }
      xSemaphoreGive(I2C);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void initMPU() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}
void printLocalTime(void *pvParameters)
{
  struct tm timeinfo;
  timeData current_time;
  while (1) {
    if (!getLocalTime(&timeinfo)) {
      Serial.println("No time available (yet)");
      return;
    }
    current_time.seconds = timeinfo.tm_sec;
    current_time.minutes = timeinfo.tm_min;
    current_time.hours = timeinfo.tm_hour;
    xQueueSend( time_Queue, &current_time, pdMS_TO_TICKS(10) );
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
