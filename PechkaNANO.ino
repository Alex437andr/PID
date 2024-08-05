
/* Занятые пины:
    PORTD - почти весь на 7SEG (0,1,3-7), кроме PD2 (2) - прерывание; третий сегмент подключен к 13 пину
    PORTB - PB0 (8), PB1(9) - Выбор сегмента
    термопара -  А0 (SO), А1(CS), А2 (SCLK)
    Потенциометр - А7
    Диммер: 2 (PD2), А3.
    Кнопки для настройки PID: 10, 11, 12
    LCD для вывода P, I, D в режиме настроек: A4, A5
    Свободные A6
  Задейстованы таймеры 1 и 2. Hа таймер 0 держутся millis(), delay()
*/
/*
  //************************************************************
  // Секция для блока настроек коэффициентов PID
  // Раскоментировать блок в самом конце loop
  #include "GyverButton.h" // подключение библиотеки кнопок
  GButton bUP (10);        // кнопка для индремента
  GButton bDOWN (11);      // кнопка для декремента
                         // причём +/- для коэф. P инк и дек = 1; для I и D инк и дек = 0.1
  GButton bMODE (12);      // переключение от P до D коэффициентов
  //для работы LCD. подключать к А4 (SDA) А5(SCL)
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x3F, 16, 2); // Включение LCD в работу; адреса 0x27 или 0x3F
  //************************************************************

*/

//#define plot
//#define proverka


int Dimm; // переменная хранения значения времени для "среза" полуволны на диммер

int Temp;                        // переменная хранения температуры от термопары
int RES;                         // хранит текущую настройку температуры
int RESold;                      // хранит прошлую настройку температуры
uint32_t showTemp = 0;           // переменная времени для эффекта переключения температура<->установка температуры
uint32_t incov = 0;              // количество срабатываний таймера 2 за ~10мс. Обнуляется после 75 срабатываний
unsigned long long delayISP = 0; // количество срабатываний таймера 2 за ~10мс. Обнуляется после 20 срабатываний
uint32_t delayPlot, pidincov, decrease, increase, ResScan;
int RESprev;
#include <EEPROM.h>
int addres = 0;
#include "GyverTimer2.h" // библиотека для работы с Timer2. На нём работают переменные времени showTemp и delayISP
//#include <GyverPID.h> // библиотрка для работы с PID регулятором

int16_t dt = 1000 ;
boolean _mode = 0, _direction = 0, _prev_direction = 0;
float _minOut = 0, _maxOut = 120;
float dt_s = 1000;
float pid_prevInput = 0.0;
float integral = 0.0;
uint32_t pidTimer = 0;
float pid_setpoint;    // заданная величина, которую должен поддерживать регулятор
float pid_input;    // сигнал с датчика (например температура, которую мы регулируем)
float pid_output;   // выход с регулятора на управляющее устройство (например величина ШИМ или угол поворота серво)

//PID = 100 - надо проверить T = 100
//PID = 90  - надо проверить T = 95
//PID = 82  удерживается при T = 91 // 90-91-92
//PID = 81  удерживается при T = 90 // 90-91
//PID = 80  удерживается при T = 89 // в среднем, т.е. показывает 88-90, стабильней 89 выдаёт, но может выход на режим у неё большой, ибо 90 выдавало под самый конец; перепрошито на PId = 82
//PID = 74  - надо проверить T = 85
//PID = 67  удерживается при T = 80 // немного качает с 79-80, потом остывает до 78, но вполне неплохо держит 79, мб 67, но было 65
//PID = 63  удерживается при T = 75 // стабильно
//PID = 60  удерживается при T = 72 // но качает с 72 до 68 ,т.е. мб 57
//PID = 57  - надо проверить T = 70
//PID = 51  удерживается при T = 66 // стабильно
//PID = 50  - надо проверить T = 65
//PID = 45  удерживается при T = 60-61-62 - стабильней 61
//PID = 44  удерживается при T = 59-60 - надо проверить
//PID = 40  удерживается при T = 55 // стабильно
//PID = 35  удерживается при T = 50 // стабильно
//PID = 32  удерживается при T = 50 // стабильно на протяжении 8 минут
//PID = 30  удерживается при T = 45 // стабильно
//PID = 25  удерживается при T = 42 // стабильно
//PID = 22  удерживается при T = 40 // стабильно -8выход на режим 25-30 минут
//PID = 20  - надо проверить T = 35
//PID = 19  - надо проверить T = 30

int polynomial [] = {22, 24, 25, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 42, 43, 44, 45, 46, 48, 49, 50, 51, 53, 54, 55, 57, 58, 59, 61, 62, 63, 64, 65, 66, 67, 67, 68, 70, 71, 72, 74, 75, 77, 79, 80, 81, 82};

float pid_res;

//при 8, 0.0065 и RES >=10 систему начинает раскачивать при RES = 70
float kp = 24; // 20, 0, 0 - период 1662 // (при условии out < 120 в интегралке) при 8 и 0.013 недоходит! до RES = 70 detT = 2 и начинает остывать
float ki = 0.005; //0.045; //0.025; //0.1 //
float kd = 0;

float error;
float delta_input;
char trigger_zero = 2, trigger_120 = 2;
float pid_getResult() {

  if (trigger_zero == 2 && trigger_120 == 2)
  {
    #ifdef proverka
    Serial.print("| RESscan"); Serial.print("\t"); Serial.print(ResScan); Serial.print("\t");
    Serial.print("| dec"); Serial.print("\t"); Serial.print(decrease); Serial.print("\t");
    Serial.print("| inc"); Serial.print("\t"); Serial.print(increase); Serial.print("\t");
    #endif
    error = pid_setpoint - pid_input;        // ошибка регулирования
    pid_output = 0;
    int pass = 12;
    if (RES >= 40 && RES <= 43) pass = 13;
    else if (RES <= 55) pass = 13;
    else if (RES <= 80) pass = 13;
    if ((RES - Temp) >= pass)
    {
      pid_output += (float)error * kp;   // пропорционально изменению входного сигнала
    }
    if ((RES - Temp) < pass)
    {
      if ((RES >= 40) && (RES <= 91))
      {
        pid_res = polynomial [RES - 40];
        //pid_res = ceil(-60731.3550042525 + 8655.94718337448*pow(RES,1) - 538.161215853307*pow(RES,2) + 19.11186845652*pow(RES,3) - 0.42605894958254*pow(RES,4) + 0.0061631766329511*pow(RES,5) - 0.0000576055104105417*pow(RES,6) + 3.3341904022365*pow(10,-7)*pow(RES,7) - 1.07386045838*pow(10,-9)*pow(RES,8) + 1.44116053*pow(10,-12)*pow(RES,9));
      }
      if (RES >= 30 && RES < 40)
      {
        pid_res = 0.02 * RES * RES - 1.1 * RES + 34;
      }
      if (RES >= 92 && RES <= 100)
      {
        pid_res = 2 * RES - 100;
      }
      pid_output = pid_res;

      if ((RES - Temp) <= 5 && (RES - Temp != 1))
      {
        integral += (float)error * ki;
       
      }
      pid_output += integral;
      if (pid_output  <= pid_res - 6) pid_output  = pid_res - 6;
      if (pid_output  >= pid_res + 6) pid_output  = pid_res + 6;
    }

    // if (integral >=
#ifdef proverka
    Serial.print("| output="); Serial.print("\t"); Serial.print(pid_output); Serial.print("\t");
#endif

    pid_output = constrain(pid_output, 0, 120);    // ограничиваем

#ifdef proverka
    Serial.print("| output"); Serial.print("\t"); Serial.print(pid_output); Serial.print("\t");
    Serial.print("| Temp"); Serial.print("\t"); Serial.print( Temp); Serial.print("\t");
    Serial.print("| RES"); Serial.print("\t"); Serial.println( RES);

    //Serial.print("| Dim"); Serial.print("\t"); Serial.println(Dimm);
#endif

    return pid_output;
  }
  else if (trigger_zero == 1)
  {
    #ifdef proverka
    Serial.print(" HERE2 "); Serial.print("\t");
    Serial.print("| dec"); Serial.print("\t"); Serial.println(decrease); //Serial.print("\t");
    #endif
    integral = 0;
    pid_output = 0;
    return pid_output;
  }
  else if (trigger_120 == 1)
  {
    #ifdef proverka
    Serial.print(" HERE3 "); Serial.print("\t");
    Serial.print("| inc"); Serial.print("\t"); Serial.println(increase);
    #endif
    pid_output = 120;
    return pid_output;
  }
}

float pid_getResultTimer() {
  if ((long)millis() - pidTimer >= dt) {
    pidTimer = millis();
    return pid_getResult();
  } else {
    return pid_output;
  }
}
void pid_setDirection(uint8_t pid_direction) {
  _direction = pid_direction;
  if (kp > 0 && _direction) {       // смотрим по первому коэффициенту, если > 0 и надо менять на <0, меняем
    kp = 0 - kp;
    ki = 0 - ki;
    kd = 0 - kd;
  } else if (kp < 0 && !_direction) {   // если < 0 и надо менять на >0, меняем
    kp = 0 - kp;
    ki = 0 - ki;
    kd = 0 - kd;
  }
}

void pid_setLimits(float min_output, float max_output) {
  _minOut = min_output;
  _maxOut = max_output;
}


/*
  float P = EEPROM.read(0);
  float I = EEPROM.read(1);
  float D = EEPROM.read(2);
*/
/*
  float P = 10.0;
  float I = 0.8;
  float D = 0;
*/
double PIDres; // переменная хранения результата вычисления PID

//GyverPID PID (P, I, D, 500); //коэффициент П, коэффициент И, коэффициент Д, период дискретизации (мс)


#include <CyberLib.h>// шустрая библиотека для таймера. Нужна для диммера

#include <SPI.h>     // Библиотека для работы с SPI интерфейсом для MAX6675<->термопара
const int SO = A0;   // MISO
const int CS = A1;   // CS
const int SCLK = A2; // SCLK

#define POT A7       // Потенциометр
#define Max 1020
#define Min 10

#define dimPin A3    // Пин для подключения аля ШИМ на диммер // переназначить на А3
#define zeroPin 2    // ПРЕРЫВАНИЕ на втором пине ардуино (PD2 INT0) 

int8_t indiDigits[2]; // массив хранения цифр для вывода на SEG

volatile int tic;
//unsigned long long int timeNOW = 0;



int number [] = // массив вывода на порт D цифры в зависимости от её номера в массиве
{
  0b11000000, //0
  0b11111001, //1
  0b10100000, //2
  0b10110000, //3
  0b10011001, //4
  0b10010010, //5
  0b10000010, //6
  0b11111000, //7
  0b10000000, //8
  0b10010000, //9
};

int number_RES [] = // тоже самое, что и number[], только с выводом "единицы" - отображения пользовательских настроек
{
  0b01000000, //0
  0b01111001, //1
  0b00100000, //2
  0b00110000, //3
  0b00011001, //4
  0b00010010, //5
  0b00000010, //6
  0b01111000, //7
  0b00000000, //8
  0b00010000, //9
};



void outSEG (int F, bool seg) // функция вывода числа на сегмент, seg - нужна ли "единица" на сегменте
{
  indiDigits [0] = F % 10;
  indiDigits [1] = F / 10;
  //if (indiCounter >= 5){

  if (seg) // если надо вывести "единицу"
    for (int count = 0; count <= 1; ++count)
    {
      PORTB &= ~(0x3);
      PORTB = (1 << count);
      if (indiDigits [count] != 2) {
        PORTD = number_RES [indiDigits [count]];
        digitalWrite(13, LOW);
        _delay_ms(1);
      }
      else {
        PORTD = number_RES [indiDigits [count]];
        digitalWrite(13, HIGH);
        _delay_ms(1);
      }

    }
  else // если не требуется
    for (int count = 0; count <= 1; ++count)
    {
      PORTB &= ~(0x3);
      PORTB = (1 << count);
      if (indiDigits [count] != 2) {
        PORTD = number [indiDigits [count]];
        digitalWrite(13, LOW);
        _delay_us(900);
      }
      else
      {
        digitalWrite(13, HIGH);
        PORTD = number [indiDigits [count]];
        _delay_us(950); // 900 мкс поставлено для функции вывода температуры
      }
    }
}



// Т.к. функция ниже читает с MAX6625 байты в цикле, а в цикле стоит задержка 1 мс, то
// в сумме для получения температуры в градусах С тратится ~20 мс. Этого хватает, чтобы сегмент единиц стал моргать.
//Решение: вместо задержки выводим температуру на сегменты (коряво, но работает)

double readThermocouple(int ChipSelect) // функция чтения температуры с термопары
{
  uint16_t v;

  digitalWrite(ChipSelect, LOW);
  //_delay_ms(1);
  outSEG (Temp, false);              // вместо задержки на 1 мс выводит текущую температуру
  v = SPIRead();
  v <<= 8;
  v |= SPIRead();
  digitalWrite(ChipSelect, HIGH);


  if (v & 0x4) {
    return NAN;
  }
  v >>= 3;
  return v * 0.25;
}

byte SPIRead(void) // чтение байта данных с MAX6625
{
  int i;
  byte d = 0;


  for (i = 7; i >= 0; i--)
  {
    digitalWrite(SCLK, LOW);
    //_delay_ms(1);            // вместо задержки на 1 мс выводит текущую температуру
    outSEG (Temp, false);
    if (digitalRead(SO)) {
      d |= (1 << i);
    }
    digitalWrite(SCLK, HIGH);
    //_delay_ms(1);
    outSEG (Temp, false);
  }
  return d;
}



void timer_interrupt() {       // прерывания таймера срабатывают каждые 40 мкс

  tic++;                       // счетчик
  if (tic > Dimm)              // если настало время включать ток
    digitalWrite(dimPin, 1);   // врубить ток
}

void  detect_up() {    // обработка внешнего прерывания на пересекание нуля снизу
  tic = 0;                                  // обнулить счетчик
  ResumeTimer1();                           // перезапустить таймер
  attachInterrupt(0, detect_down, RISING);  // перенастроить прерывание
}

void  detect_down() {  // обработка внешнего прерывания на пересекание нуля сверху
  tic = 0;                                  // обнулить счетчик
  StopTimer1();                             // остановить таймер
  digitalWrite(dimPin, 0);                  // вырубить ток
  attachInterrupt(0, detect_up, FALLING);   // перенастроить прерывание
}


uint32_t stay_off;
void inc (void) // прерывание таймера каждые 10 мс
{
  incov++;
  delayISP++;
  //delayPlot++;
  pidincov++;
  decrease++;
  increase++;
  ResScan++;
  stay_off++;
  if (incov > 75)
  {
    showTemp++;
    incov = 0;
  }
  if (pidincov >= 200)
  {
    pid_setpoint = RES;               // записываем текущую "планку" для PID
    pid_input = Temp;                 // температура, что есть на текущий момент для пересчёта PID
    /*if (PIDreset >= 300 && abs(RES - Temp) <= 10)
      {
      PIDreset = 0;
      integral = 0;
      }
      PIDreset++;
    */
    PIDres = pid_getResultTimer();    // высчитывает, насколько надо прибавить для уменьшения разницы
    //PIDres =  computePID (pid_input, pid_setpoint, 8, 0.025, 0, 2, 0, 120);
    Dimm = 220 - PIDres;              // "обратный" ход PID.
    pidincov = 0;
  }
  if (ResScan >= 1000)
  {
    if ((RESprev - RES) > 10 && (RESprev - RES) > 0)
    {
      trigger_zero = 1;
      decrease = 0;
    }
    else
    if ((RES - RESprev) >= 5 && (RESprev - RES) <= 10)
    {
      trigger_120 = 1;
      increase = 0;
    }
    RESprev = RES;
    ResScan = 0;
  }

  if (decrease >= 120000 && trigger_zero == 1)
  {
    trigger_zero = 2;
    trigger_120 = 1;
    increase = 0;
    if (RESprev != RES)
    {
      trigger_zero = 2;
      trigger_120 = 2;
    }
  }

  if (trigger_120 == 1)
  {
    if (increase >= 9000)
    {
      trigger_120 = 2;
      increase = 0;
      decrease = 0;
      if (RESprev != RES)
      {
        trigger_zero = 2;
        trigger_120 = 2;
      }
    }
  }
  if (stay_off <= 6000)
  {
    trigger_zero = 2;
    trigger_120 = 2;
  }
  if (stay_off > 6200) stay_off = 6200;
  if (increase >= 20000) increase = 0;
  if (decrease >= 100000) decrease = 0;
}



void buttSet(int showme) // функция обработки потенциометра. Сохраняет текущее время "показа" настроек/температуры
{

  RES = constrain (map(analogRead(POT), Min, Max, 30, 99), 30, 99);
  /*Serial.print(analogRead(POT));
    Serial.print("\t");
    Serial.println(constrain (map(analogRead(POT), 10, 850, 30, 99), 30, 99));
  */
  if (RESold != RES)
  {
    showTemp = 0; // т.к.
    RESold = RES;
    while (showTemp <= 4) // ждём 4 секунды
    {
      RES = constrain (map(analogRead(POT), Min, Max, 30, 99), 30, 99);
      /* Serial.print (RES);
        Serial.print ("\t");
        Serial.println (analogRead(POT));
      */
      if (RES != RESold)
      {
        RESold = RES;
        showTemp = 0;
      }
      outSEG(RES, true);
    }
  }
  showTemp = showme; // возврат времени
}
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void setup() {

  dt_s = (float)dt / 1000;
  pidTimer = (long)millis() + dt;
  pinMode(POT, INPUT);
  RES = constrain (map(analogRead(POT), Min, Max, 30, 99), 30, 99);// default для настройки температуры
  RESold = RES;          // default для настройки температуры
  RESprev = RES;
  pinMode(CS, OUTPUT);   // настройка MAX6625
  pinMode(SCLK, OUTPUT);
  pinMode(SO, INPUT);

  digitalWrite(CS, HIGH);

  //DDRD = 0xFF; // обращение к портам напрямую "мешает" работе SPI (не знаю почему)
  //DDRB = 0x03; // Решение: настроить порты средстави Arduino
  for (int i = 0; i < 10; i++) // настройка порта D и B
  {
    pinMode(i, OUTPUT);
  }
  pinMode(13, OUTPUT); // замена пина PD2 для вывода третьего сегмента

  pid_setDirection(0); // направление регулирования PID
  pid_setLimits(0, 120);    // пределы

  pinMode(dimPin, OUTPUT); // пин для диммера
  digitalWrite(dimPin, 0);
  pinMode(zeroPin, INPUT); // настраиваем порт на вход для отслеживания прохождения сигнала через ноль
  attachInterrupt(0, detect_up, FALLING); // настроить срабатывание прерывания interrupt0 на pin 2 на низкий уровень
  StartTimer1(timer_interrupt, 40); // запуск перывания таймера 1 на прерывание каждые 40 мкс
  // StopTimer1(); // остановка таймера

  timer2_setPeriod(10000); // прерывания каждые ~10мс
  timer2_ISR(inc);         // вызов функции при прерывании
  timer2_start();          // включение таймера2

  /*  lcd.init();
    lcd.backlight();
    //pinMode(A6,INPUT);
  */
  
#ifdef proverka
  Serial.begin(9600);
  
#endif
#ifdef plot
 Serial.begin(9600);
Serial.println("TEMP,RES,PID");
#endif

}

//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
//-----------------------------------------------------
bool trig = false;
int showme;

void loop() {
  if (showTemp >= 15) showTemp = 0; // обнуление счётчика через каждые ~30 сек
  //Serial.println(Dimm);
  // для настроек коэффициентов PID в реальном времени (недоделано)

  buttSet(0); // опрос потенциометра

  if ((abs(Temp - RES) <= 10) && (abs(Temp - RES) >= 5))
    // если температура приблизилась к установленной от 10-5 градусов,
    // то показывает настроенную температуру каждые ~5 сек
  {
    showTemp = 0;
    while (showTemp <= 3)
    {
      buttSet(showTemp); // опрашиваем кнопки, иначе изменить настройки больше не получится
      if (trig)
      {
        outSEG(RES, true);
      }
      else {
        outSEG (Temp, false);
      }
    }
    trig = !trig;
    showTemp = 0;
  }
  else if (abs(Temp - RES) < 5)
    // если температура приблизилась менее чем на 5 градусов,
    // то выводим настройки каждые 2 сек
  {
    showTemp = 0;
    while (showTemp <= 1)
    {
      buttSet(showTemp); // опрашиваем кнопки, иначе изменить настройки больше не получится
      if (trig) // что выводим? Температуру - false, или Настройки - true?
      {
        outSEG(RES, true);
      }
      else
      {
        outSEG(Temp, false);
      }
    }
    showTemp = 0;
    trig = !trig; // меняем темпераутур на настройки или обратно
  }

  outSEG (Temp, false); // выводим темературу
  if (delayISP >= 20 )  // считываем температуру через каждые ~200 мс
  {
    Temp = readThermocouple(CS);
    delayISP = 0;
  }


#ifdef plot
  if (delayPlot >= 100)
  {
    Serial.print(Temp);
    Serial.print(',');
    Serial.print(RES);
    Serial.print(',');
    Serial.print(PIDres);
    Serial.println(',');
    delayPlot = 0;
  }
  if (Serial.available () > 1)
  {
    char key = Serial.read();
    int val = Serial.parseInt();
    switch (key)
    {
      case 'p': kp = val; break;
      case 'i': ki = val / 1000; break;
      case 'd': kd = val / 1000; break;
    }
  }

#endif
  /*
    #ifndef plot
      Serial.print("Temp "); Serial.print( Temp);
      Serial.print("| RES "); Serial.print( RES);

      Serial.print("| P= "); Serial.print( P);
      Serial.print( " | I= "); Serial.print( I);
      Serial.print( " | D= "); Serial.print(D);
      Serial.print( " | PID= "); Serial.print(PIDres);
      Serial.print( " | Dim= "); Serial.println(Dimm);
    #endif
  */
}
