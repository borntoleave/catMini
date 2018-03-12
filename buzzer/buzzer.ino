// Customized music player
// Rongzhong Li
// August 2017

#define BUZZER 5
void beep(int note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }

  int freq = 220 * pow(1.059463, note);
  float period = 1000000.0 / freq / 2.0;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period * 2) {
      analogWrite(BUZZER, 150);      // Almost any value can be used except 0 and 255
      // experiment to get the best tone
      delayMicroseconds(period);          // wait for a delayms ms
      analogWrite(BUZZER, 0);       // 0 turns it off
      delayMicroseconds(period);          // wait for a delayms ms
    }
    delay(pause);
  }
}
void playMelody(byte m[], int len) {
  for (int i = 0; i < len; i++)
    beep(m[i], 1000 / m[len + i], 100);
}
void setup()
//opening music
{
  Serial.begin(57600);
  pinMode(BUZZER, OUTPUT);
}
int a=0;
void loop() {
  // tone: pause,1,  2,  3,  4,  5,  6,  7,  1,  2
  // code: 0,    1,  3,  5,  6,  8,  10, 12, 13, 15
  /*byte melody[] = {8, 13, 10, 13, 8, 0, 5, 8, 3, 5, 8,
                   8, 8, 32, 32, 8, 32, 32, 32, 32, 32, 8
                   //8,8,16,16,8,16,16,16,16,8
                  };
  playMelody(melody, sizeof(melody) / 2);*/
  Serial.println(a++);
}
