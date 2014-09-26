/** 
 * Original: @author : Alex Baizeau 
 * 
 * Changed: Serdyukov Anton, devdotnet.org
 * for device Arduino IIC / I2C Serial 3.2" LCD 2004
 * Module Display GY-IICLCD GY-LCD-V1 PCF8574 PCF8574T
 * from http://dx.com/p/arduino-iic-i2c-serial-3-2-lcd-2004-module-display-138611
 */ 

#include <Wire.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

#define LCD_I2C_ADDR 0x20 // Define I2C Address where the PCF8574T is
#define BACKLIGHT 7
#define LCD_EN 4
#define LCD_RW 5
#define LCD_RS 6
#define LCD_D4 0
#define LCD_D5 1
#define LCD_D6 2
#define LCD_D7 3

LiquidCrystal_I2C lcd(LCD_I2C_ADDR,LCD_EN,LCD_RW,LCD_RS,LCD_D4,LCD_D5,LCD_D6,LCD_D7);

void setup()
{
	lcd.begin (20,4);
	lcd.setBacklightPin(BACKLIGHT,NEGATIVE); // init the backlight
	lcd.setBacklight(HIGH); // Backlight on
	lcd.clear(); // Clear the lcd
}

void loop()
{
	lcd.home (); // go home
	lcd.print("111 Hello World");
	lcd.setCursor ( 0, 1 ); // go to the next line
	lcd.print("222 olo Olo");
	lcd.setCursor ( 0, 2 ); // go to the next line
	lcd.print("333 olo Olo");
	lcd.setCursor ( 0, 3 ); // go to the next line
	lcd.print("444 olo Olo");
	lcd.setCursor ( 0, 4 ); // go to the next line
}
