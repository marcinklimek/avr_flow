
/*        1          2          3          4         A
 *       ---        ---        ---        ---       ---
 *      |   |      |   |      |   |      |   |    F| G |B
 *       ---        ---        ---        ---       ---
 *      |   |      |   |      |   |      |   |    E| D |C
 *       --- .      --- .      --- .      --- .     --- . DP
 *
 *  Pierwszy wysyłany bajt w formacie MSB steruje włączeniem wyświetlacza, odpowiednio: 1->4, 2->3, 4->3, 8->1, wyświetlacz ze wspólną anodą - zapalamy jedynkę
 *  drugi bajt w MSB cyfry wg tablicy dec_digits, aby zaświecić segment ustawiamy zero
 *
 *      drugi bajt       pierwszy bajt
 *  |A|B|C|D|E|F|G|DP| |4|3|2|1|X|X|X|X|  (MSB)
 *
 *  W danym momencie możę wyświetlać się tylko jedna cyfra więc trzeba multiplexpować i wiświetlać w "tle"

    http://www.mike-gualtieri.com/posts/arduino_word_clock
    
 */
int dataPin = 11;
int latchPin = 8;
int clockPin = 12;

int ByteReceived;

byte dec_digits[] = { 0b11000000, 0b11111001, 0b10100100, 0b10110000, 0b10011001, 0b10010010, 0b10000011, 0b11111000, 0b10000000, 0b10011000, 0b10111111 };
byte segments[] =   { 0b00001000, 0b00000100, 0b00000010, 0b00000001 };

void setup() {
  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  Serial.begin(9600);
}

void display1 () {
  byte digit = 0;
  for (digit = 0; digit < 10; digit++) {
    Serial.print("Write: ");
    Serial.println(digit);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[digit]);
    shiftOut(dataPin, clockPin, MSBFIRST, 255);
    digitalWrite(latchPin, HIGH);
    delay(1000);
  }
}

void display2 () {
  byte digit = 0;
  for (digit = 0; digit < 8; digit++) {
    Serial.print("Write: ");
    Serial.println(1 << digit);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8]);
    shiftOut(dataPin, clockPin, MSBFIRST, 1 << digit);
    digitalWrite(latchPin, HIGH);
    delay(1000);
  }
}

void display3 () {
  byte digit = 0;
  for (digit = 0; digit < 8; digit++) {
    Serial.print("Write: ");
    Serial.println(1 << digit);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, 1 << digit);
    shiftOut(dataPin, clockPin, MSBFIRST, 255);
    digitalWrite(latchPin, HIGH);
    delay(1000);
  }
}

void loop() {

  if (Serial.available() > 0)
  {
    ByteReceived = Serial.read();

    if (ByteReceived == '0')
    {
      display1();
    }
    if (ByteReceived == '1')
    {
      display2();
    }
    if (ByteReceived == '2')
    {
      display3();
    }
    if (ByteReceived == '4')
    {
      Serial.println("8 on segment 1");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8]);
      shiftOut(dataPin, clockPin, MSBFIRST, segments[0]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("8 on segment 2");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8]);
      shiftOut(dataPin, clockPin, MSBFIRST, segments[1]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("8 on segment 3");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8]);
      shiftOut(dataPin, clockPin, MSBFIRST, segments[2]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("8 on segment 4");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8]);
      shiftOut(dataPin, clockPin, MSBFIRST, segments[3]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("- on segment 1");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[10]);
      shiftOut(dataPin, clockPin, MSBFIRST, segments[0]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("8 on segment 2 with dot");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, dec_digits[8] & 0b01111111 );
      shiftOut(dataPin, clockPin, MSBFIRST, segments[1]);
      digitalWrite(latchPin, HIGH);
      delay(1000);

      Serial.println("only dot segment 3");
      digitalWrite(latchPin, LOW);
      shiftOut(dataPin, clockPin, MSBFIRST, 0b01111111 );
      shiftOut(dataPin, clockPin, MSBFIRST, segments[2]);
      digitalWrite(latchPin, HIGH);
      delay(1000);
    }
  }
}

