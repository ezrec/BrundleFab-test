/* Read Quadrature Encoder
 * Connect Encoder to Pins encoder0PinA, encoder0PinB, and +5V.
 *
 * Sketch by max wolf / www.meso.net
 * v. 0.1 - very basic functions - mw 20061220
 *
 */  


int val; 
int encoder0PinA = 15;
int encoder0PinB = 19;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int encoder0PinBLast = LOW;
int a = LOW;
int b = LOW;

void setup() { 
  pinMode (encoder0PinA,INPUT_PULLUP);
  pinMode (encoder0PinB,INPUT_PULLUP);
  Serial.begin (9600);
} 

void loop() { 
  int changed = 0;
  a = digitalRead(encoder0PinA);
  b = digitalRead(encoder0PinB);
  if ((encoder0PinALast == LOW) && (a == HIGH)) {
    if (b == LOW) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
    changed = 1;
  }
  if ((encoder0PinBLast == LOW) && (b == HIGH)) {
    if (a == LOW) {
      encoder0Pos++;
    } else {
      encoder0Pos--;
    }
    changed = 1;
  }
  if (changed) {
    Serial.print (encoder0Pos);
    Serial.print ("/");
    Serial.print (a);
    Serial.print (",");
    Serial.print (b);
    Serial.print ("\r\n");
  } 
  encoder0PinALast = a;
  encoder0PinBLast = b;
} 
