#include <AnalogIO.h>
#include <PulseSensorPlayground.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_FreeRTOS.h>
#include "task.h"
#include "semphr.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3D
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

TaskHandle_t xHandleD = NULL;
TaskHandle_t xHandleT = NULL;
SemaphoreHandle_t sem;

int timeout = 0;
unsigned long seconds;
unsigned int minutes;
unsigned int hours;
unsigned int BPM ;
unsigned int steps = 0;
uint8_t disp_clear = 0;

unsigned long previousMillis = 0;
const long interval = 1000;
const int LEDPIN = 13;
const int inputpin = 2;
unsigned int PULSE_SENSOR_PIN = A0;
unsigned int STEP_SENSOR_PIN = A8;
int STEP_THRESHOLD = 0;

PulseSensorPlayground pulseSensor;

void vibrationMonitor(void *pvParameters)
{
  Serial.println("Starting");

  while (1)
  {
    // Assuming STEP_SENSOR_PIN is an analog pin, read its value
    int vibrationValue = analogRead(STEP_SENSOR_PIN);
    

    if (vibrationValue > STEP_THRESHOLD)
    {
      if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS)
      {
        steps++;
        xSemaphoreGive(sem);
      }
    }
    

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


  void vApplicationIdleHook()
  {
    if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS)
    {
      Serial.println("update test 1");

       BPM = pulseSensor.getBeatsPerMinute();

      Serial.println(BPM);

      xSemaphoreGive(sem);
    }
  }


void setup()
{
  Serial.begin(9600);
  int Buff [7];

  sem = xSemaphoreCreateBinary();
  if (sem != NULL)
  {
    xSemaphoreGive(sem);
  }

  pinMode(LEDPIN, OUTPUT);
  pinMode(inputpin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputpin), button_press, FALLING);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
  {
    Serial.println("SSD1306 allocation failed");
    while (1)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  Serial.println("1");
  xTaskCreate(Display, "Display", 4096, NULL, 1, &xHandleD);
  xTaskCreate(Time, "Time", 128, NULL, 2, &xHandleT);
  xTaskCreate(vibrationMonitor, "VibrationMonitor", 1000, NULL, 3, NULL); // Create the vibration monitoring task
  vTaskStartScheduler();
}

void loop() {}

void button_press()
{
  timeout = 0;
}

void Display(void *pvParameters)
{
  if (xSemaphoreTake(sem, portMAX_DELAY) == pdPASS)
  {
    while (1)
    {
      if (disp_clear == 0)
      {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(1, 1);
        display.print("BPM: ");
        display.print(BPM);
        display.setCursor(90, 1);
        display.print("S:");
        display.print(steps);
        display.setCursor(30, 35);
        display.print(hours % 24);
        display.print(':');
        display.print(minutes % 60);
        display.print(':');
        display.print(seconds % 60);
        display.display();
      }
      else
      {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(1, 1);
        display.display();
      }
    }
  }
  xSemaphoreGive(sem);
  //vTaskDelay(pdMS_TO_TICKS(100));
}

void Time(void *pvParameters)
{
  while (1)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      seconds = currentMillis / 1000;
      minutes = seconds / 60;
      hours = minutes / 60;
      timeout++;
      if (timeout >= 3)
        disp_clear = 1;
      else
        disp_clear = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
