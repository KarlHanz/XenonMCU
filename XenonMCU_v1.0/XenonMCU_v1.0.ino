/*
  https://alexgyver.ru/lessons/serial/
  https://neoworlds.net/arduino/arduino-v-primerakh/delaem-voltmetr-na-baze-arduino
  http://arduino.ru/forum/programmirovanie/pochistil-sketch-primera-raboty-s-ds18b20-iz-bibly-oneware

*/

#include <GyverWDT.h> // Сторжевой таймер библиотека
#include <OneWire.h>
OneWire  ds(2);

float Vcc;  // напряжение питания ардуино
float v_main; // напряжение с аккума
float v_in_main; // напряжение с делителя R1-R2
float v_out; // напряжение после реле на блоки розжига
float v_in_out; // напряжение с делителя R3-R4
float v_currentSens1;  // напряжение с датчика тока 1
float v_currentSens2;  // напряжение с датчика тока 2
float i_right; // ток правой фары
float i_left; // ток правой фары
float temp;
volatile int raw; // температура "сырая"
float R1 = 20010.0; // сопротивление R1 (20K)
float R2 = 4655.0; // сопротивление R2 (4.7K)
float R3 = 19970.0; // сопротивление R3 (20K)
float R4 = 4683.0; // сопротивление R4 (4.7K)


const float typVbg = 1.18; // 1.0 -- 1.2   // внутреннее опорное напряжение для отладочной платы!

float kVolt; // коэфициент чувствительности датчика тока

#define NUM_READS 10 // кол-во считывания данных АЦП для усреднения показаний
#define voltagPin1 A6  // порт ардуины напряжение с аккума
#define voltagPin2 A0  // порт ардуины напряжение после реле на блоки розжига
#define currentPin1 A2 // порт ардуины напряжение с датчика тока 1
#define currentPin2 A4 // порт ардуины напряжение с датчика тока 2




void setup() {
  Serial.begin(9600); //hardware serial
Watchdog.enable(INTERRUPT_RESET_MODE, WDT_PRESCALER_128);   // Комбинированный режим , таймаут ~1c
//  WDTCSR = (1 << WDCE) | (1 << WDE); //установить биты WDCE WDE (что б разрешить запись в другие биты
//  WDTCSR = (1 << WDIE) | (1 << WDP3); // разрешение прерывания + выдержка 4 секунды
  pinMode(7, OUTPUT); // управление модулем RS-485
  digitalWrite(7, LOW); // управление модулем RS-485   // включаем прием, МЫ SLAVE
}





//----------Функция измерения напряжения на пине-------
float readAnalog(int pin) {
  // read multiple values and sort them to take the mode
  int sortedValues[NUM_READS];
  for (int i = 0; i < NUM_READS; i++) {
    delay(5);
    int value = analogRead(pin);
    //----------фильтр данных (для уменьшения шумов и разброса данных)-------
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (int k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  return returnval / 10;
}



//----------Функция точного определения напряжения питания-------
float readVcc() {
  // read multiple values and sort them to take the mode
  float sortedValues[NUM_READS];
  for (int i = 0; i < NUM_READS; i++) {
    float tmp = 0.0;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRA |= _BV(ADSC); // Start conversion
    delay(5);
    while (bit_is_set(ADCSRA, ADSC)); // measuring
    uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both
    tmp = (high << 8) | low;
    float value = (typVbg * 1023.0) / tmp;

    //----------фильтр данных (для уменьшения шумов и разброса данных)-------
    int j;
    if (value < sortedValues[0] || i == 0) {
      j = 0; //insert at first position
    }
    else {
      for (j = 1; j < i; j++) {
        if (sortedValues[j - 1] <= value && sortedValues[j] >= value) {
          // j is insert position
          break;
        }
      }
    }
    for (int k = i; k > j; k--) {
      // move all values higher than current reading up one position
      sortedValues[k] = sortedValues[k - 1];
    }
    sortedValues[j] = value; //insert current reading
  }
  //return scaled mode of 10 values
  float returnval = 0;
  for (int i = NUM_READS / 2 - 5; i < (NUM_READS / 2 + 5); i++) {
    returnval += sortedValues[i];
  }
  return returnval / 10;
}


//----------формирование пакета данных-----------
void sendData() {
  digitalWrite(7, HIGH); // управление модулем RS-485  // включаю ПЕРЕДАЧУ

  Serial.print("L"); // индефикатор
  Serial.print(v_main);
  Serial.print(";");
  Serial.print(v_out);
  Serial.print(";");
  Serial.print(Vcc);
  Serial.print(";");
  Serial.print(temp);
  Serial.print(";");
  Serial.print(i_right);
  Serial.print(";");
  Serial.print(i_left);
  Serial.print("$"); //конец передачи
  Serial.println(); //для отладки!
  Serial.flush(); //ждём окончания передачи всех данных
  delay(5); //задержка для модуля
  digitalWrite(7, LOW); // управление модулем RS-485   // включаем прием, МЫ SLAVE

}

//----------------------ОБРАБОТЧИКИ ПРЕРЫВАНИЙ--------------------------

ISR(WATCHDOG) { //обработка прерывания WDT-таймера
  byte data[2];

  static boolean n = 0; // флаг работы: запрос температуры или её чтение
  n = !n;
  if (n) {
    ds.reset();  // сброс шины
    ds.write(0xCC); // команда пропустить поиск по адресу. В нашем случае только один
    ds.write(0x44); // начать преобразование (без паразитного питания)
  }
  else   {
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);
    data[0] = ds.read();
    data[1] = ds.read();
    // Формируем значение

    //   temp = (data[1] << 8) + data[0];
    //   temp = temp >> 4; // деление на 16.

    raw = (data[1] << 8) + data[0];
    //   raw = raw >> 4; // деление на 16.
      }
  Watchdog.enable(INTERRUPT_RESET_MODE, WDT_PRESCALER_512); // Если перенастроить watchdog здесь - сброса не будет
}


void loop() {

//----------отправка пакета данных-----------
//  if (Serial.available() > 0) {
//    char incomingByte = Serial.read();        // обязательно ЧИТАЕМ входящий символ
//    if (incomingByte == 'L') {   // если это наш ID
      sendData();                          // отправляю пакет данных в ответ
//    }
//  }

//----------считывание аналоговых значений-----------
  Vcc = readVcc(); // считывание напряжения питания ардуины
  v_in_main = (readAnalog(voltagPin1) * Vcc) / 1023.0; //считывание напряжения после делителя R1-R2 с пина
  v_in_out = (readAnalog(voltagPin2) * Vcc) / 1023.0; //считывание напряжения после делителя R3-R4 с пина
  
  v_currentSens1 = (readAnalog(currentPin1) * Vcc) / 1023.0; //считывание напряжения датчика тока
  v_currentSens2 = (readAnalog(currentPin2) * Vcc) / 1023.0; //считывание напряжения датчика тока
  
  v_main = v_in_main / (R2 / (R1 + R2)); // расчёт напряжения аккума
  if (v_main <= 0.09) v_main = 0.00;         // обнуляем нежелательное значение

  v_out = v_in_out / (R4 / (R3 + R4)); // расчёт напряжения выхода реле
  if (v_out <= 0.09) v_out = 0.00;         // обнуляем нежелательное значение

  kVolt = (Vcc / 2 - Vcc * 0.1) / 20; // коэф. датчика тока
  i_right = ((v_currentSens1 - Vcc / 2) / kVolt); // расчёт тока датчика тока 1
  if (i_right <= 0.00) i_right = 0.00;   // обнуляем нежелательное значение
  
  i_left = ((v_currentSens2 - Vcc / 2) / kVolt); // расчёт тока датчика тока 2 
  if (i_left <= 0.00) i_left = 0.00;   // обнуляем нежелательное значение

   temp = (float)raw / 16.0; // перевод "сырой" температуры в формат хх.хх

  
}
