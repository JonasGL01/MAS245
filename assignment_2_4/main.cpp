// Written by K. M. Knausgård 2023-10-21

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

#include "mas245_logo_bitmap.h"

namespace carrier
{
  namespace pin
  {
    constexpr uint8_t joyLeft{18};
    constexpr uint8_t joyRight{17};
    constexpr uint8_t joyClick{19};
    constexpr uint8_t joyUp{22};
    constexpr uint8_t joyDown{23};

    constexpr uint8_t oledDcPower{6};
    constexpr uint8_t oledCs{10};
    constexpr uint8_t oledReset{5};
    constexpr uint8_t joystickPin{A0};  // Joystick connected to A0
  }

  namespace oled
  {
    constexpr uint8_t screenWidth{128};  // OLED display width in pixels
    constexpr uint8_t screenHeight{64};  // OLED display height in pixels
  }
}

namespace {
  CAN_message_t msg;

  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can0;
  Adafruit_SSD1306 display( carrier::oled::screenWidth,
                            carrier::oled::screenHeight,
                            &SPI,
                            carrier::pin::oledDcPower,
                            carrier::pin::oledReset,
                            carrier::pin::oledCs);

  // Ball properties
  int ballX = carrier::oled::screenWidth / 2;
  int ballY = carrier::oled::screenHeight / 2;
  int ballSize = 3;
  int ballVelocityX = 1;  // Horizontal velocity
  int ballVelocityY = 1;  // Vertical velocity

  // Paddle properties
  int paddleHeight = 20;
  int paddleWidth = 5;
  int paddleX = 0;  // Left side of the screen
  int paddleY = carrier::oled::screenHeight / 2 - paddleHeight / 2;

  // Right wall (opponent) properties
  int rightWallX = carrier::oled::screenWidth - 5; // Position at the far right
  int rightWallHeight = carrier::oled::screenHeight;
  int rightWallWidth = 5;
}

struct Message {
    uint8_t sequenceNumber;
    float temperature;
};

void drawSplash();
void sendCan();
void sendCan(const Message& message);
void updateBall();
void updatePaddle();

void setup() {
  Serial.begin(9600);
  can0.begin();
  can0.setBaudRate(250000);

  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("ERROR: display.begin(SSD1306_SWITCHCAPVCC) failed."));
    for (;;);
  }

  display.clearDisplay();
  display.display();
  delay(2000);

  // Display logo and apply invert effects
  drawSplash();
  delay(2000);
  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
  delay(1000);
  display.invertDisplay(true);
  delay(100);
  display.invertDisplay(false);
  delay(1000);

  // Clear the screen after the logo and effects
  display.clearDisplay();
  display.display();
  delay(1000);
}

void loop() {
  updatePaddle();  // Update joystick-controlled paddle position
  updateBall();    // Update ball position and handle bouncing

  // Draw everything on the OLED display
  display.clearDisplay();

  // Draw left paddle
  display.fillRect(paddleX, paddleY, paddleWidth, paddleHeight, SSD1306_WHITE);

  // Draw right wall
  display.fillRect(rightWallX, 0, rightWallWidth, rightWallHeight, SSD1306_WHITE);

  // Draw the ball
  display.fillRect(ballX, ballY, ballSize, ballSize, SSD1306_WHITE);

  display.display();
  delay(10);  // Small delay to control the speed of the game loop
}

void updatePaddle() {
  // Read joystick value from A0
  int joystickValue = analogRead(carrier::pin::joystickPin);

  // Map joystick value to paddle Y position, confined within screen boundaries
  paddleY = map(joystickValue, 250, 1023, carrier::oled::screenHeight - paddleHeight, 0);

  // Constrain paddleY to ensure it stays within screen boundaries
  paddleY = constrain(paddleY, 0, carrier::oled::screenHeight - paddleHeight);
}

void updateBall() {
  // Update ball position
  ballX += ballVelocityX;
  ballY += ballVelocityY;

  // Ball bouncing off top and bottom edges
  if (ballY <= 0 || ballY >= carrier::oled::screenHeight - ballSize) {
    ballVelocityY = -ballVelocityY;  // Reverse vertical direction
  }

  // Ball bouncing off left paddle
  if (ballX <= paddleX + paddleWidth && ballY + ballSize >= paddleY && ballY <= paddleY + paddleHeight) {
    ballVelocityX = -ballVelocityX;  // Reverse horizontal direction
    ballX = paddleX + paddleWidth;   // Adjust ball position to avoid sticking
  }

  // Ball bouncing off right wall
  if (ballX + ballSize >= rightWallX) {
    ballVelocityX = -ballVelocityX;  // Reverse horizontal direction
    ballX = rightWallX - ballSize;   // Adjust ball position to avoid sticking
  }
}

void sendCan() {
  msg.len = 3;
  msg.id = 0x007;
  msg.buf[0] = 0x26;
  msg.buf[1] = 0x42;
  msg.buf[2] = 0x00;
  can0.write(msg);
}

void sendCan(const Message& message) {
    CAN_message_t msg;
    msg.id = 0x245;
    msg.len = 1 + sizeof(float);
    msg.buf[0] = message.sequenceNumber;
    memcpy(&msg.buf[1], &message.temperature, sizeof(float));

    if (can0.write(msg) < 0) {
      Serial.println("CAN send failed.");
    }
}

void drawSplash(void) {
  namespace splash = images::mas245splash;
  display.clearDisplay();
  display.drawBitmap(0, 0, splash::bitmap, splash::width, splash::height, 1);
  display.display();
}
