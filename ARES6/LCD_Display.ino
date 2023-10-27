#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
 
#define TFT_RST 26
#define TFT_DC 27
#define TFT_CS 28

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(9600);
  tft.initR(INITR_144GREENTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(45);
  tft.setTextColor(ST7735_RED);
  tft.setCursor(1,1);
  tft.print("ARES Project");
  tft.setTextColor(ST7735_MAGENTA);
  tft.setCursor(1,20);
  tft.print("ARES5.5");
  tft.setRotation(45);
  tft.setTextColor(ST7735_BLUE);
  tft.setCursor(1,50);
  tft.print("===TESTING NOW===");  
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(1,70);
  tft.println("2023 10/27");
  tft.println("in Tottori ");
  tft.println("Lunar Terrace ");

  
}

void loop() {



}