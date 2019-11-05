#include <EEPROM.h>
#include<Vector.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "Adafruit_MAX31855.h"


// ***** I2C дисплей *****
#include <LiquidCrystalRus_I2C.h>
#include <LiquidCrystal_I2C.h> // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#define cols 20
#define rows 4
LiquidCrystalRus_I2C lcd(0x27, cols, rows);


char *Blank;
int Phase;

//-------------Tune/PID/aTune--------------------------------------

byte ATuneModeRemember = 2;
double Input = 38, Output = 60, Setpoint = 43;
//Define Variables we'll be connecting to
double kp = 2.67, ki = 0.03, kd = 0;
//----просто чтобы при включении, но не задании PID уже былои каке-то ззаачения

//----------
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);// Данная часть инициализирует PID
PID_ATune aTune(&Input, &Output);// Данная часть инициализирует autoTune - который правда не работает и нужно переписывать библиотеку
//-----------

int WindowSize = 1024;        //Использоуется в PID как окно между открытиями реле
unsigned long windowStartTime;

#define RELAY_PIN 2                         // выход на реле
int PIN_BUTTON = 2;                         // выход на реле(по углпости в коде использовал и тот и другой вариант/от одного можно избавиться)

double aTuneStep = 30, aTuneNoise = 3, aTuneStartValue = 125;//Это для autoTune(Не работает пока-что)
unsigned int aTuneLookBack = 30;    //Это для autoTune(Не работает пока-что)

boolean tuning = false;             //Это для autoTune(Не работает пока-что)
unsigned long  modelTime, serialTime;//Это для autoTune(Не работает пока-что)

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.Для прослушки температуры
#define MAXDO   23
#define MAXCS   24
#define MAXCLK  3

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);//для получения температуры

//set to false to connect to the real world
boolean useSimulation = false;         //Это для autoTune(Не работает пока-что)
//-------------end Tune/PID/aTune--------------------------------------
// ********** Параметры меню **********
#define ShowScrollBar 1     // Показывать индикаторы прокрутки (0/1)
#define ScrollLongCaptions 1// Прокручивать длинные названия (0/1)
#define ScrollDelay 800     // Задержка при прокрутке текста
#define BacklightDelay 200000// Длительность подсветки
#define ReturnFromMenu 1    // Выходить из меню после выбора элемента(0/1)

//-------------То, что ниже - это отрисовка символа "градус"
uint8_t temp_cel[8] =
{
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000
}; //закодировано в двоичной системе значек градуса
//--------------

//-----------------------
// переменная для подсчёта времени
unsigned long prevMillis = 0;
//-----------------------

// ********** Переменные для энкодера ***************
#define pinT 48 // Энкодер пин A
#define pinR  50 // Энкодер пин B
#define pinKey 52 // Кнопка


unsigned long CurrentTime, PrevEncoderTime;
enum eEncoderState {eNone, eLeft, eRight, eButton};
eEncoderState EncoderState;
int EncoderA, EncoderB, EncoderAPrev, counter;
bool ButtonPrev;
eEncoderState GetEncoderState();

// ********** Прототипы функции, которые используется в меню, сначала новую функцию вносить сюда ***************
// А потом докидывать в "sMenuItem Menu[]". И все, меню автоматически работает
void LCDBacklight(byte v = 2); // Это для включение/выключения подсветки

enum eMenuKey {mkNull, mkBack, mkRoot, Heating, StepHeating, getPID, LastProgram, ProgramsList, NewProgram, Settings, Information, TimeAndDate,
               Hysteresis, HeatingTest
              };
eMenuKey DrawMenu(eMenuKey Key);

// ******************** Меню ********************


byte ScrollUp[8]  = {0x4, 0xa, 0x11, 0x1f};
byte ScrollDown[8]  = {0x0, 0x0, 0x0, 0x0, 0x1f, 0x11, 0xa, 0x4};

byte ItemsOnPage = rows;    // Максимальное количество элементов для отображения на экране
unsigned long BacklightOffTime = 0;
unsigned long ScrollTime = 0;
byte ScrollPos;
byte CaptionMaxLength;
struct sMenuItem {
  eMenuKey  Parent;       // Ключ родителя
  eMenuKey  Key;          // Ключ
  char      *Caption;     // Название пункта меню
  void      (*Handler)(); // Обработчик
};

sMenuItem Menu[] = {
  {mkNull, mkRoot, "Menu", NULL},
  {mkRoot, Heating,                 "Heating",     NULL},
  {mkRoot, StepHeating,             "Step heating", NULL},
  {StepHeating, mkBack,             "Back",        NULL},
  {StepHeating, LastProgram,        "Last program", NULL},
  {StepHeating, ProgramsList,       "Programs list", NULL},
  {StepHeating, NewProgram,         "New program", NULL},
  {mkRoot, Settings,                "Settings",    NULL},
  {Settings, mkBack,      "Back"         , NULL},
  {Settings, TimeAndDate, "Time and Date", NULL},
  {Settings, Hysteresis,  "Hysteresis " , NULL},
  {Settings, HeatingTest, "Input PID" , NULL},
  {Settings, getPID,      "See PID", NULL},
  {mkRoot, Information,   "Information", NULL},
  {mkRoot, mkBack, "Back", NULL}
};

// Создаем пользовательскую структуру с участком, температурой и временем для сохранения данных PID
struct KpKiKdStruct {
  double kp;
  double ki;
  double kd;
};

const int MenuLength = sizeof(Menu) / sizeof(Menu[0]);
void LCDBacklight(byte v) { // Управление подсветкой
  if (v == 0) { // Выключить подсветку
    BacklightOffTime = millis();
    lcd.noBacklight();
  }
  else if (v == 1) { //Включить подсветку
    BacklightOffTime = millis() + BacklightDelay;
    lcd.backlight();
  }
  else { // Выключить если время вышло
    if (BacklightOffTime < millis())
      lcd.noBacklight();
    else
      lcd.backlight();
  }
}


eMenuKey DrawMenu(eMenuKey Key) { // Отрисовка указанного уровня меню и навигация по нему
  eMenuKey Result;
  int k, l, Offset, CursorPos, y;
  sMenuItem **SubMenu = NULL;
  bool NeedRepaint;
  String S;
  l = 0;
  LCDBacklight(1);
  // Запишем в SubMenu элементы подменю
  for (byte i = 0; i < MenuLength; i++) {
    if (Menu[i].Key == Key) {
      k = i;
    }
    else if (Menu[i].Parent == Key) {
      l++;
      SubMenu = (sMenuItem**) realloc (SubMenu, l * sizeof(void*));
      SubMenu[l - 1] = &Menu[i];
    }
  }

  if (l == 0) { // l==0 - подменю нет
    if ((ReturnFromMenu == 0) and (Menu[k].Handler != NULL)) (*Menu[k].Handler)(); // Вызываем обработчик если он есть
    LCDBacklight(1);
    return Key; // и возвращаем индекс данного пункта меню
  }

  // Иначе рисуем подменю
  CursorPos = 0;
  Offset = 0;
  ScrollPos = 0;
  NeedRepaint = 1;
  do {
    if (NeedRepaint) {
      NeedRepaint = 0;
      lcd.clear();
      y = 0;
      for (int i = Offset; i < min(l, Offset + ItemsOnPage); i++) {
        lcd.setCursor(1, y++);
        lcd.print(String(SubMenu[i]->Caption).substring(0, CaptionMaxLength));
      }
      lcd.setCursor(0, CursorPos);
      lcd.print(">");
      if (ShowScrollBar) {
        if (Offset > 0) {
          lcd.setCursor(cols - 1, 0);
          lcd.write(0);
        }
        if (Offset + ItemsOnPage < l) {
          lcd.setCursor(cols - 1, ItemsOnPage - 1);
          //lcd.write(1);
        }
      }
    }
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eLeft: {
          // Прокрутка меню вверх
          LCDBacklight(1);
          ScrollTime = millis() + ScrollDelay * 5;
          if (CursorPos > 0) {  // Если есть возможность, поднимаем курсор
            if ((ScrollLongCaptions) and (ScrollPos)) {
              // Если предыдущий пункт меню прокручивался, то выводим его заново
              lcd.setCursor(1, CursorPos);
              lcd.print(Blank);
              lcd.setCursor(1, CursorPos);
              lcd.print(String(SubMenu[Offset + CursorPos]->Caption).substring(0, CaptionMaxLength));
              ScrollPos = 0;
            }
            // Стираем курсор на старом месте, рисуем в новом
            lcd.setCursor(0, CursorPos--);
            lcd.print(" ");
            lcd.setCursor(0, CursorPos);
            lcd.print(">");
          }
          else if (Offset > 0) {
            //Курсор уже в крайнем положении. Если есть пункты выше, то перерисовываем меню
            Offset--;
            NeedRepaint = 1;
          }
          break;
        }
      case eRight: {
          // Прокрутка меню вниз
          LCDBacklight(1);
          ScrollTime = millis() + ScrollDelay * 5;
          if (CursorPos < min(l, ItemsOnPage) - 1) {// Если есть возможность, то опускаем курсор
            if ((ScrollLongCaptions) and (ScrollPos)) {
              // Если предыдущий пункт меню прокручивался, то выводим его заново
              lcd.setCursor(1, CursorPos);
              lcd.print(Blank);
              lcd.setCursor(1, CursorPos);
              lcd.print(String(SubMenu[Offset + CursorPos]->Caption).substring(0, CaptionMaxLength));
              ScrollPos = 0;
            }
            // Стираем курсор на старом месте, рисуем в новом
            lcd.setCursor(0, CursorPos++);
            lcd.print(" ");
            lcd.setCursor(0, CursorPos);
            lcd.print(">");
          }
          else {
            // Курсор уже в крайнем положении. Если есть пункты ниже, то перерисовываем меню
            if (Offset + CursorPos + 1 < l) {
              Offset++;
              NeedRepaint = 1;
            }
          }
          break;
        }
      case eButton: {
          // Выбран элемент меню. Нажатие кнопки Назад обрабатываем отдельно
          LCDBacklight(1);
          ScrollTime = millis() + ScrollDelay * 5;
          if (SubMenu[CursorPos + Offset]->Key == mkBack) {
            free(SubMenu);
            return mkBack;
          }
          Result = DrawMenu(SubMenu[CursorPos + Offset]->Key);
          if ((Result != mkBack) and (ReturnFromMenu)) {
            free(SubMenu);
            return Result;
          }
          NeedRepaint = 1;
          break;
        }
      case eNone: {
          if (ScrollLongCaptions) {
            // При бездействии прокручиваем длинные названия
            S = SubMenu[CursorPos + Offset]->Caption;
            if (S.length() > CaptionMaxLength)
            {
              if (ScrollTime < millis())
              {
                ScrollPos++;
                if (ScrollPos == S.length() - CaptionMaxLength)
                  ScrollTime = millis() + ScrollDelay * 2; // Небольшая задержка когда вывели все название
                else if (ScrollPos > S.length() - CaptionMaxLength)
                {
                  ScrollPos = 0;
                  ScrollTime = millis() + ScrollDelay * 5; // Задержка перед началом прокрутки
                }
                else
                  ScrollTime = millis() + ScrollDelay;
                lcd.setCursor(1, CursorPos);
                lcd.print(Blank);
                lcd.setCursor(1, CursorPos);
                lcd.print(S.substring(ScrollPos, ScrollPos + CaptionMaxLength));
              }
            }
          }
          LCDBacklight();
        }
    }
  } while (1);
}

///-------------------------
int eeAddress = 0;// Данная переменная используется при записи PID в еепром, указывает запись начинается(или продложается)

int heatTemerature;//записывается температура из Heating
int heatSpeed;  //скорость
unsigned long heatTimeOnSecond;//часы+минуты в секундах, т.е. общее.


//-----------------Параметры для списков------------------------
int nameSp = 0;
//int nameSpMassive[];
// Создаем пользовательскую структуру температурой и временем для списка внутри "программы"
struct TempAndTime {
  int Temp;
  unsigned long Time;
};
// Создаем пользовательскую структуру с участком, температурой и временем для хранения списка "программ"
struct NameAndVectorTempAndTime {
  int Name;
  Vector<TempAndTime> vi;
};
const int ELEMENT_COUNT_MAX = 10;// Это число дает возможность создания такого количества элементов списка.
//т.е. в листе програм не может быть более 10.

TempAndTime storage_array22[ELEMENT_COUNT_MAX];
Vector<TempAndTime> vi(storage_array22);                  //Список листов в программе
NameAndVectorTempAndTime storage_array223[ELEMENT_COUNT_MAX];
Vector<NameAndVectorTempAndTime> nameVi(storage_array223);//список программ
//--------------------------------------------------------------
KpKiKdStruct PidStruct;
//----------------------------
//****************************************
void setup() {
  Serial.begin(9600);

//---------------- Задаем параметры для корректного отображения данных о действиях Энкодера----
  pinMode(pinT, INPUT);
  pinMode(pinR,  INPUT);
  pinMode(pinKey, INPUT_PULLUP);
  //  SPI.begin();
//----------------------------------------------------------------------------------------------

  lcd.init();     //----Инициализируем экран
  lcd.backlight();//----Подстветку
  
  //----------ИНициализация меню, вмешиваться или исправлять - не нужно--------
  CaptionMaxLength = cols - 1;
  Blank = (char*) malloc(cols * sizeof(char));
  for (byte i = 0; i < CaptionMaxLength; i++)
    Blank[i] = ' ';
  if (ShowScrollBar) {
    CaptionMaxLength--;
    lcd.createChar(0, ScrollUp);
    lcd.createChar(1, ScrollDown);
  }
  Blank[CaptionMaxLength] = 0;
  LCDBacklight(1); // Включаем подвсветку

//---------------------------------------------------------------------------

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  if (tuning)
  {
    tuning = false;
    changeAutoTune();
    tuning = true;
  }
//-------------↑Чтобы внести определенные измнения в поведения autotune------------------

  serialTime = 0;

//----------PIN_BUTTON==2 - для инициализации Реле и закрытия его
  pinMode(PIN_BUTTON, OUTPUT);
  digitalWrite(PIN_BUTTON, LOW);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long F;

  // В цикле опрашиваем энкодер и отлавиваем какие пункты меню были нажаты
  switch (GetEncoderState()) {
    case eNone: { // При бездействии отключаем подсветку по таймауту
        LCDBacklight(1); // Включаем подсветку
        switch (DrawMenu(mkRoot)) { // Показываем меню и анализируем выбранный пункт
          case Heating: {
              InputHeating();
              break;
            } 

          case StepHeating: {
              lcd.clear();
              lcd.print("Step Heating");
              lcd.setCursor(0, 1);
              lcd.print("dots20101@gmail.com");
              while (GetEncoderState() == eNone) {
                LCDBacklight();//данная конструкция необходима, чтобы держалось окно с "принтом" 
              }                 //неопределенное время, до нажатия
              break;
            }

          case NewProgram: {
              SetDisplayNewProgram();//происходит установка визуацльной части
              InputNewProgram();    //здесь просходит настройка/переходы между пунктами интерфейса, ввод и все остальное
              break;
            }
          case getPID: {//достаем знаения PID которые мы задавали ранее
              getPIDFromEEPROM();  //ДОстаем из памяти записанные значения            
              dispPIDnow();        //выводим их
              delay(3000);
              break;
            }


          case ProgramsList: {
              setListProgram();   //Фуункция, включающая в себя возможность просматривать программы/редактировать некоторые подпрограммы
              break;
            }

          case LastProgram: {
              int n = nameVi.size();
              if (n > 0) {
                getProgram(n - 1);//массив из 10 символов заканчивается 9 элементом (0-9), поэтому n-1, что соответствует "последней программе"
              }
              break;
            }
          case Hysteresis: {
              lcd.clear();
              
              lcd.setCursor(1, 1); 
              lcd.print("Input Temp:0000");
              
              int temperature = inputTemperature(1, 12, 13, 14, 15);// 1 цифра - номер строки, остальные 4 - номер столбца на этой строке, где будут введены значения(цифры)
              heatTimeOnSecond=0;heatSpeed=0;
              DisplayHeatingStart(heatTimeOnSecond, heatSpeed, temperature);

              bool state = false;
              while (1) {
                double c = thermocouple.readCelsius() - 4;//Оринетируемся на возможную погрешность при измерении температуры
                dispTempNow(c);//меняет постоянно значение температуры на нынешнее(выводит)
                lcd.noBlink();
                if (c > temperature-0.25) {
                  digitalWrite (RELAY_PIN, LOW);
                }
                else {
                  digitalWrite (RELAY_PIN, HIGH);//подает напряжение, однако если сюда поставить заранее высчитаный промежуток, то получается отличный результат
                }
                EncoderState = GetEncoderState();//получаем состояние энкодера(повернулся/нажали)
                switch (EncoderState) {
                  case eButton: {
                      if (state) {
                        LCDBacklight(1);
                        Serial.print("Нажатие");
                        SetDisplayHeating();
                        digitalWrite (RELAY_PIN, LOW);
                        return;
                      }
                      break;
                    }
                  case eLeft: {
                      if (!state) {
                        lcd.setCursor(1, 3);  lcd.print("Quit");
                        state = true;
                      } else {
                        lcd.setCursor(1, 3);  lcd.print("Work");
                        state = false;
                      }
                    }
                  case eRight: {
                      if (!state) {
                        lcd.setCursor(1, 3);  lcd.print("Quit");
                        state = true;
                      } else {
                        lcd.setCursor(1, 3);  lcd.print("Work");
                        state = false;
                      }
                    }
                }             
              }
            }

          case Settings: {
              lcd.clear();
              lcd.print("Setting");
              lcd.setCursor(0, 1);
              lcd.print("dots20101@gmail.com");
              while (GetEncoderState() == eNone) {
                LCDBacklight();
              }
              break;
            } 
            
          case HeatingTest: {//сохранение PID //input PID
              lcd.clear();
              lcd.setCursor(1, 1); 
              lcd.print("Kp:");
              lcd.setCursor(1, 2);
              lcd.print("Ki:"); 
              lcd.setCursor(1, 3);
              lcd.print("Kd:"); 
              
              setKpKiKd(1,2,3, 4, 4, 4);              
              EEPROM.put(eeAddress, PidStruct);//PidStruct заполняется в функции setKpKiKd              
              break;
          }
          case Information: {
              lcd.clear();
              lcd.print("Furnace");
              lcd.setCursor(0, 1);
              lcd.print("dots20101@gmail.com");
              while (GetEncoderState() == eNone) {
                LCDBacklight();
              }
              LCDBacklight(1);
              break;
            } // Изменить форму сигнала на синусоидальную
        }
        // После выхода из меню перерисовываем главный экран
        //LCDRepaint();
        return;
      }

    case eButton: { // При нажатии на кнопку показываем меню 
      }
  }
}
// ******************** Энкодер с кнопкой ******************** 
//Просто функция, которая реализует получение состояния энкодера
eEncoderState GetEncoderState() {
  eEncoderState Result = eNone;
  CurrentTime = millis();
  if (CurrentTime >= (PrevEncoderTime + 20)) {
    PrevEncoderTime = CurrentTime;
    if (digitalRead(pinKey)) {           //  Если вывод pinK установлен в 1, значит произошло нажатие на вал энкодера
      if (ButtonPrev) {
        Result = eButton; // Нажата кнопка
        ButtonPrev = 0;
      }
    } else {
      ButtonPrev = 1;
    }
    if (digitalRead(pinT)) {
      ButtonPrev = 1;
      if (digitalRead(pinR)) {
        Result = eLeft;
      }
      else                 {
        Result = eRight;
      }
      delay(2);
      pinMode(pinT, OUTPUT);
      digitalWrite(pinT, INPUT);
      pinMode(pinT, INPUT);
    }
  }
  return Result;
}


//---------------------------------------------------**********************
// ******************** Ввод Времени ******************** .
unsigned long InputHourOrMinuteT(int HourOrMinute) {
  //TempOrSpeed - тут либо 2 либо 3 или 4 или 5; 2/3 - часы, a 4/5 - минуты
  int w = 0;
  int positionCursor;
  if (HourOrMinute == 2) {
    positionCursor = 7;
  } else if (HourOrMinute == 3) {
    positionCursor = 8;
  } else if (HourOrMinute == 4) {
    positionCursor = 10;
  } else {
    positionCursor = 11;
  }

  while (1) {
    lcd.setCursor(positionCursor, 2);
    lcd.blink();
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eButton: {
          LCDBacklight(1);
          return w;
        }
      case eLeft: {
          LCDBacklight(1);
          if (w > 0) w--;
          break;
        }
      case eRight: {
          LCDBacklight(1);
          if (HourOrMinute == 2) {
            if (w < 2) {
              w++; //Ограничение по вводимой величине
            }
          } else if (HourOrMinute == 3) {
            if (w < 4) {
              w++; //Ограничение по вводимой величине
            }
          } else if (HourOrMinute == 4) {
            if (w < 5) {
              w++; //Ограничение по вводимой величине
            }
          } else {
            if (w < 9) {
              w++; //Ограничение по вводимой величине
            }
          }
          break;
        }
    }
    lcd.setCursor(positionCursor, 2);
    lcd.print(w);
  }
}
//---------------------------------------------------**********************
unsigned long Freq;


//-----------------Просто вывод на экран
void SetDisplayHeating() {
  lcd.clear();

  lcd.setCursor(0, 0);  lcd.print('>');

  lcd.setCursor(1, 0);
  lcd.print("Temp: 0     C");
  lcd.setCursor(1, 1);
  lcd.print("Speed:0     C/min");
  lcd.setCursor(1, 2);
  lcd.print("Hold: 00:00");

  lcd.setCursor(1, 3);
  lcd.print("Start");
  lcd.setCursor(8, 3);
  lcd.print("Reset");
  lcd.setCursor(15, 3);
  lcd.print("Quit");
}
//--------------------------------------------
unsigned long InputHeating() {
  unsigned long F;
  int Time;
  int Speed;
  int Positions[] = {6, 7, 8};
  int Digits[3];
  int p = 0;
  //----------Настриваем вид экрана-------------------
  SetDisplayHeating();
  lcd.setCursor(0, 0);  lcd.print('>');
  //-------------------------------------------------
  //Основной цикл - выбор разряда для изменения либо Start/Reset
  while (1)
  {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eLeft: { // Двигаем курсор влево
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {
            lcd.setCursor(0, 0);  lcd.print('>');
            continue;
          }  // Левее перемещаться некуда
          if (p == 5) { 
            lcd.setCursor(14, 3); lcd.print(' ');
            lcd.setCursor(7, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 4) { // перемещаемся к другому элементу
            lcd.setCursor(7, 3); lcd.print(' ');
            lcd.setCursor(0, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 3) { // перемещаемся к другому элементу
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(0, 2);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 2) { // перемещаемся к другому элементу
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 1);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 1) { // перемещаемся к другому элементу
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 0);  lcd.print('>');
            p--;
            lcd.cursor();
            continue;
          }
          // Выбрана частота, перемещаемся к старшему разряду
          p--;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eRight: { // Двигаем курсор вправо
          LCDBacklight(1); // Включаем подсветку
          if (p == 5)continue;
          if (p == 4) {
            lcd.setCursor(7, 3); lcd.print(' ');
            lcd.setCursor(14, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 3) { 
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(7, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 2) {
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 1) {
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 2); lcd.print('>');
            p++;
            continue;
          }
          if (p == 0) {
            lcd.setCursor(0, 0); lcd.print(' ');
            lcd.setCursor(0, 1); lcd.print('>');
            p++;
            continue;
          }
          // Выбрана частота, перемещаемся к младшему разряду
          p++;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eButton: { //Нажата кнопка//в зависисоти от значения p при нажатии кнопки
        //мы производим те или иные действия. р - указывает на 
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {
            lcd.noCursor();
            F = InputTempOrSpeedT(0);//0 - значит вводим температуру
            heatTemerature = F;
          }
          if (p == 1) {
            lcd.noCursor();
            F = InputTempOrSpeedT(1);//1 - значит вводим скорость - описано в самой функции
            heatSpeed = F;
          }
          if (p == 2) {
            //lcd.noCursor();
            lcd.setCursor(1, 2);
            lcd.print("Hold: 00:00");
            F = InputHourOrMinuteT(2);//(2/3/4/5 - это просто поля на экране(от 0 до 20, 
            //но так как мы установили чуть выше курсор на второй символ, то там и вводим )

            heatTimeOnSecond = F * 60 * 60 * 10; //(часы в десятках)10/20 часов
            Serial.println(heatTimeOnSecond);
            F = InputHourOrMinuteT(3);    //Часы в долях 2/3 часа
            Serial.println(F);
            heatTimeOnSecond = F * 60 * 60 + heatTimeOnSecond;
            Serial.println(heatTimeOnSecond);
            F = InputHourOrMinuteT(4);    //Время в минутах(десятки)
            Serial.println(F);
            heatTimeOnSecond = F * 60 * 10 + heatTimeOnSecond;
            Serial.println(heatTimeOnSecond);
            F = InputHourOrMinuteT(5);    //Время в минутах(доли)
            Serial.println(F);
            heatTimeOnSecond = F * 60 + heatTimeOnSecond; //В секундах общее время
            Serial.println(heatTimeOnSecond);
          }
          if (p == 3) {
            bool timeStart = false;//данные boolевы значения необходимы для своевренного выхода из цикла по достижению необходимого условия
            int c;
            Setpoint = heatTemerature; //Изначально сверху мы инициализировали setPoint  с определенной температурой , а затем уже тут вводили
            //нужную температуру, так что теперь приравниваем к ней
            DisplayHeatingStart(heatTimeOnSecond, heatSpeed, heatTemerature);            
            myPID.SetTunings(kp+heatSpeed, ki, kd);//меняем значение PID если установили скорость(добавляем к Кр коэффициенту)
            
            p = 0;
            bool state = false;
            while (1) {

              c = thermocouple.readCelsius() - 4;
              pppeeechkaWork();
              dispTempNow(c);

              //описанное ниже говорит о том, что при достижение температуры поставленной нами
              //активируется выдержка по времени(hold заявленная нами)
              if (c >= heatTemerature) {
                timeStart = true;
              }
              if (timeStart && (heatTimeOnSecond > 0) && ((millis() - prevMillis) > 1000)) {
                DispTime(--heatTimeOnSecond);
                prevMillis = millis();
                if (heatTimeOnSecond == 0) {
                  lcd.clear();
                  lcd.setCursor(6, 1); lcd.print("FINISH");
                  delay(2000);
                  state = true;
                  return;
                }}
                EncoderState = GetEncoderState();
                switch (EncoderState) {
                  case eButton: {
                      if (state) {
                        LCDBacklight(1);
                        Serial.print("Нажатие");
                        SetDisplayHeating();
                        return;
                      }
                      break;
                    }
                  case eLeft: {
                      if (!state) {
                        lcd.setCursor(1, 3);  lcd.print("Quit");
                        state = true;
                      } else {
                        lcd.setCursor(1, 3);  lcd.print("Work");
                        state = false;
                      }
                      break;
                    }
                  case eRight: {
                      if (!state) {
                        lcd.setCursor(1, 3);  lcd.print("Quit");
                        state = true;
                      } else {
                        lcd.setCursor(1, 3);  lcd.print("Work");
                        state = false;
                      }
                      break;
                    }
              }
            }
            continue;
          }
          if (p == 4) {
            SetDisplayHeating();
            heatTemerature = 0;
            heatSpeed = 0;
            heatTimeOnSecond = 0;
            p = 0;
          }
          if (p == 5) {
            lcd.noCursor();
            return Freq; // Cancel.
          }
          LCDBacklight(1);
          lcd.noBlink();
          continue;
        }
    }
  }
}

// ******************** Ввод нового значения количестве участков ********************
unsigned long InputFreq() {
  unsigned long F = Freq;
  int Positions[] = {6, 7, 8};
  int Digits[3];
  int p = 0;

  //----------Настриваем вид экрана-------------------
  SetDisplayHeating();
  //-------------------------------------------------

  // Разбиваем частоту на разряды и выводим на дисплей
  for (int i = 2; i >= 0; i--) {
    Digits[i] = F % 10;
    lcd.setCursor(Positions[i], 1);
    lcd.print(Digits[i]);
    F = F / 10;
  }
  lcd.setCursor(Positions[0], 1);
  lcd.cursor();

  //Основной цикл - выбор разряда для изменения либо Start/Reset
  while (1)
  {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight();
          continue;
        }
      case eLeft: { // Двигаем курсор влево
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) continue; // Левее перемещаться некуда
          if (p == 4) { // Выбран Cancel, перемещаемся к OK
            lcd.setCursor(9, 3); lcd.print(' ');
            lcd.setCursor(0, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 3) { // Выбран OK, перемещаемся к частоте
            lcd.setCursor(0, 3); lcd.print(' ');
            p--;
            lcd.setCursor(Positions[p], 1);
            lcd.cursor();
            continue;
          }
          // Выбрана частота, перемещаемся к старшему разряду
          p--;
          lcd.setCursor(Positions[p], 1);
          continue;
        }
      case eRight: { // Двигаем курсор вправо
          LCDBacklight(1); // Включаем подсветку
          if (p == 4) continue; // Правее перемещаться некуда
          if (p == 3) { // Выбран Ok, перемещаемся к Cancel
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(9, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 2) { // Выбран младший разряд частоты, перемещаемся к OK
            lcd.noCursor();
            lcd.setCursor(0, 3); lcd.print('>');
            p++;
            continue;
          }
          // Выбрана частота, перемещаемся к младшему разряду
          p++;
          lcd.setCursor(Positions[p], 1);
          continue;
        }
      case eButton: { //Нажата кнопка
          LCDBacklight(1); // Включаем подсветку
          if (p == 4) {
            lcd.noCursor();
            return Freq; // Cancel.
          }
          if (p == 3) { // OK. Собираем и возвращаем новое значение частоты
            lcd.noCursor();
            F = 0;
            for (int i = 0; i < 3; i++)
              F = F * 10 + Digits[i];
            return F;
          }
          // Редактирование выбранного разряда частоты
          EncoderState = eNone;
          lcd.setCursor(Positions[p], 1);
          lcd.blink();
          while (EncoderState != eButton)
          {
            EncoderState = GetEncoderState();
            switch (EncoderState) {
              case eNone: {
                  LCDBacklight();
                  continue;
                }
              case eLeft: {
                  LCDBacklight(1); // Включаем подсветку
                  if (Digits[p] == 0) continue;
                  lcd.setCursor(Positions[p], 1);
                  lcd.print(--Digits[p]);
                  lcd.setCursor(Positions[p], 1);
                  continue;
                }
              case eRight: {
                  LCDBacklight(1); // Включаем подсветку
                  if (Digits[p] == 9) continue;
                  lcd.setCursor(Positions[p], 1);
                  lcd.print(++Digits[p]);
                  lcd.setCursor(Positions[p], 1);
                  continue;
                }
            }
          }
          LCDBacklight(1);
          lcd.noBlink();
          continue;
        }
    }
  }
}

//--------Просто вывод на экран Значений уже запущенного действия------------Heating/start
void DisplayHeatingStart(unsigned long heatTimeOnSecond, int heatSpeed, int heatTemerature) {
  lcd.clear();
  lcd.createChar(1, temp_cel);

  lcd.setCursor(1, 0);
  lcd.print(heatTemerature);
  lcd.print("\1");
  lcd.print("C");
  lcd.setCursor(1, 1);
  //lcd.print(); 
  lcd.print("\1C");
  if(heatTimeOnSecond != 0){   
    unsigned long timeH = heatTimeOnSecond / 3600;
    unsigned long timeM = (heatTimeOnSecond - timeH * 3600) / 60;
    unsigned long timeS = (heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10;
   
    lcd.setCursor(1, 2);
    lcd.print(heatTimeOnSecond / 60 / 60 / 10); //часы в десятках
    lcd.print(heatTimeOnSecond / 60 / 60 % 10); //часы в долях
    lcd.print(":");
    lcd.print((heatTimeOnSecond - timeH * 3600) / 60 / 10); //минуты
    lcd.print((heatTimeOnSecond - timeH * 3600) / 60 % 10); //минуты
    lcd.print(":");
    lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10); //секунды в десятках
    lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) % 10); //секунды в долях
  }
  lcd.setCursor(1, 3);  lcd.print("Work");
}
void DispTime(unsigned long heatTimeOnSecond) {
  unsigned long timeH = heatTimeOnSecond / 3600;
  unsigned long timeM = (heatTimeOnSecond - timeH * 3600) / 60;
  unsigned long timeS = (heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10;
  lcd.setCursor(1, 2);
  lcd.print(heatTimeOnSecond / 60 / 60 / 10); //часы в десятках
  lcd.print(heatTimeOnSecond / 60 / 60 % 10); //часы в долях
  lcd.print(":");
  lcd.print((heatTimeOnSecond - timeH * 3600) / 60 / 10); //минуты в десятках
  lcd.print((heatTimeOnSecond - timeH * 3600) / 60 % 10); //минуты в долях
  lcd.print(":");
  lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10); //секунды в десятках
  lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) % 10); //секунды в долях
}

void dispTempNow(int c) {
  lcd.setCursor(1, 1);
  lcd.print(c);
  lcd.print("\1C");
}

//--------Просто вывод на экран Значений уже запущенного действия------------Heating/start
void DisplayLastProgram() {
  unsigned long timeH = heatTimeOnSecond / 3600;
  unsigned long timeM = (heatTimeOnSecond - timeH * 3600) / 60;
  unsigned long timeS = (heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10;

  lcd.clear();
  lcd.createChar(1, temp_cel);

  lcd.setCursor(1, 0);
  lcd.print(heatTemerature);
  lcd.print("\1");
  lcd.print("C");
  lcd.setCursor(1, 1);
  lcd.print(heatSpeed);
  lcd.print("\1C/min");
  lcd.setCursor(1, 2);
  lcd.print(heatTimeOnSecond / 60 / 60 / 10); //часы в десятках
  lcd.print(heatTimeOnSecond / 60 / 60 % 10); //часы в долях
  lcd.print(":");
  lcd.print((heatTimeOnSecond - timeH * 3600) / 60 / 10); //минуты
  lcd.print((heatTimeOnSecond - timeH * 3600) / 60 % 10); //минуты
  lcd.print(":");
  lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) / 10); //минуты
  lcd.print((heatTimeOnSecond - timeH * 3600 - timeM * 60) % 10); //минуты
  lcd.setCursor(1, 3);  lcd.print("Work");
}



// ******************** Ввод Температуры ******************** Для примера.
int InputTempOrSpeedT(int TempOrSpeed) {
  //TempOrSpeed - тут либо 0 либо 1; 0 - Temperature, a 1 - speed
  int w = 0;
  int positionCursor = 7;
  lcd.setCursor(positionCursor, TempOrSpeed);
  if (TempOrSpeed == 0) {
    lcd.print("0     C");
  } else {
    lcd.print("0");
  }
  while (1) {
    lcd.setCursor(positionCursor, TempOrSpeed);
    lcd.blink();
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eButton: {
          LCDBacklight(1);
          return w;
        }
      case eLeft: {
          LCDBacklight(1);
          if (w > 0) w--;
          break;
        }
      case eRight: {
          LCDBacklight(1);
          if (TempOrSpeed == 0) {
            if (w < 1500) w++;//Ограничение по вводимой величине
            break;
          } else {
            if (w < 15) w++;//Ограничение по вводимой величине
            break;
          }
          
        }
    }
    lcd.setCursor(positionCursor, TempOrSpeed);
    lcd.print(w);
  }
}

//--------------------------------------------
int inputTemperature(int posLine, int posStroke1, int posStroke2, int posStroke3, int posStroke4) {
  int F ;
  int sum = 0;
  int numberGran = 9;//органичение по величене вводимого числа, т.е. не более 0123456789, не более 9.
  lcd.blink();
  F = InputNumber(posStroke1, posLine, 1);
  sum = F * 1000;
  F = InputNumber(posStroke2, posLine, 5);
  sum = F * 100 + sum;
  F = InputNumber(posStroke3, posLine, numberGran);
  sum = F * 10 + sum;
  F = InputNumber(posStroke4, posLine, numberGran);
  sum = F + sum;
  return sum;
}
//--------------------------------------------
unsigned long inputTime(int posLine, int posStroke1, int posStroke2, int posStroke3, int posStroke4) {
  unsigned long F ;
  unsigned long sum = 0;

  lcd.blink();
  F = InputNumber(posStroke1, posLine, 9);
  sum = F * 60 * 60 * 10 + sum;
  F = InputNumber(posStroke2, posLine, 9);
  sum = F * 60 * 60 + sum;
  F = InputNumber(posStroke3, posLine, 5);
  sum = F * 60 * 10 + sum;
  F = InputNumber(posStroke4, posLine, 9);
  sum = F * 60 + sum;

  return sum;
}
//-----------------Просто вывод на экран
void SetDisplayNewTempAndTimeForList() {
  lcd.clear();

  lcd.setCursor(0, 1);
  lcd.print("Input Temp: 0000");

  lcd.setCursor(0, 2);
  lcd.print("Input Time: 00:00");
  lcd.setCursor(14, 2);
}
// ******************** Ввод числа (целого числа)******************** Для примера.
int InputNumber(int posStroke, int posLine, int lastN) {
  int w = 0;
  lcd.setCursor(posStroke, posLine);
  while (1) {
    lcd.setCursor(posStroke, posLine);
    lcd.blink();
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eButton: {
          LCDBacklight(1);
          return w;
        }
      case eLeft: {
          LCDBacklight(1);
          if (w > 0) w--;
          break;
        }
      case eRight: {
          LCDBacklight(1);
          if (w < lastN) w++;//Ограничение по вводимой величине
          break;
        }
    }
    lcd.setCursor(posStroke, posLine);
    lcd.print(w);
  }
}

// ******************** Ввод числа (дробь)******************** Для примера.
double InputNumberDouble(int posStroke, int posLine, int lastN, double Nn) {
  //Nn - показывает сколько прибаляться или отниматься будет при прокрутке энкодера
  double w = 0;
  lcd.setCursor(posStroke, posLine);
  while (1) {
    lcd.setCursor(posStroke, posLine);
    lcd.blink();
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eButton: {
          LCDBacklight(1);
          return w;
        }
      case eLeft: {
          LCDBacklight(1);
          if (w > 0) w=w-Nn;
          break;
        }
      case eRight: {
          LCDBacklight(1);
          if (w < lastN) w=w+Nn;//Ограничение по вводимой величине
          break;
        }
    }
    lcd.setCursor(posStroke, posLine);
    lcd.print(w);
  }
}
//---------------------------------------------------**********************
//-----------------Просто вывод на экран
void SetDisplayNewProgram() {
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(8, 0);
  lcd.print("Program 1");
  lcd.setCursor(1, 1);
  lcd.print("List (press)");

  lcd.setCursor(1, 2);
  lcd.print("Delete previousStep");
  lcd.setCursor(1, 3);
  lcd.print("Add");
  lcd.setCursor(13, 3);
  lcd.print("Ok");
}
//--------------------------------------------
unsigned long InputNewProgram() {
  unsigned long F;
  int num = 1;
  int p = 0;
  int w = 0;

  TempAndTime storage_array22[ELEMENT_COUNT_MAX];//Это массив динамический с именем, температурой и временем. Ограничен 1000 элементов
  Vector<TempAndTime> vi(storage_array22);
  //----------Настриваем вид экрана-------------------
  SetDisplayNewProgram();
  lcd.setCursor(0, 0);  lcd.print('>');
  //-------------------------------------------------
  //Основной цикл - выбор разряда для изменения либо Start/Reset
  bool running2 = true;
  while (running2)
  {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eLeft: { // Двигаем курсор влево
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {
            lcd.setCursor(0, 0);  lcd.print('>');
            continue;
          }  // Левее перемещаться некуда
          if (p == 5) { // Выбран Quit, перемещаемся к Cancel
            lcd.setCursor(12, 3); lcd.print(' ');
            lcd.setCursor(0, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 4) { // Выбран Cancel, перемещаемся к OK
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(0, 2);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 3) { // Выбран OK, перемещаемся к частоте
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 1);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 2) { // Выбран Cancel, перемещаемся к delete
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(7, 0);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 1) { // Выбран proargam Name, перемещаемся к add
            lcd.setCursor(7, 0); lcd.print(' ');
            lcd.setCursor(0, 0);  lcd.print('>');
            p--;
            lcd.cursor();
            continue;
          }
          // Выбрана частота, перемещаемся к старшему разряду
          p--;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eRight: { // Двигаем курсор вправо
          LCDBacklight(1); // Включаем подсветку
          if (p == 5)continue;
          if (p == 4) {
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(12, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 3) {
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 2) {
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 2); lcd.print('>');
            p++;
            continue;
          }
          if (p == 1) {
            lcd.setCursor(7, 0); lcd.print(' ');
            lcd.setCursor(0, 1); lcd.print('>');
            p++;
            continue;
          }
          if (p == 0) {
            lcd.setCursor(0, 0); lcd.print(' ');
            lcd.setCursor(7, 0); lcd.print('>');
            p++;
            continue;
          }          
          p++;// перемещаемся к слующему пункту
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eButton: { //Нажата кнопка
          LCDBacklight(1); // Включаем подсветку
          if (p == 1) { //Back
            lcd.noCursor();
            num = InputNumber(16, 0, 999);//номер столбца,строки, ограничение по вводимому числу(воротом энкодера)
            lcd.noBlink();
            continue;
          }
          if (p == 0) {
            vi.clear();//сбрасываем список
            p = 0;
            //lcd.noCursor();
            return;
          }
          if (p == 2) { //list
            lcd.clear();
            displayS(w, vi); //выводит курсор в зависимости от числа w -показатель напротив какого элемента списка находится "курсор" и списка
            bool running = true; // bool определен
            while (running) {
              EncoderState = GetEncoderState();
              switch (EncoderState) {
                case eNone: {

                    lcd.setCursor(0, 0);
                    lcd.print("Temperature and time");
                    continue;
                  }
                case eButton: {
                    running = false;

                    //---------------Возвращаем экран на место

                    SetDisplayNewProgram();
                    lcd.setCursor(16, 0); lcd.print(num);
                    lcd.setCursor(0, 1);  lcd.print('>');
                    p = 2;
                    //-------------------------------------------------

                    break;
                  }
                case eLeft: {
                    if (w > 0) w--;
                    displayS(w, vi);
                    Serial.println(w);
                    continue;
                  }
                case eRight: {
                    //if (w < vi.size()-1) w++;//Ограничение по вводимой величине
                    if (w < vi.size() - 1) {
                      w++;
                      displayS(w, vi);
                    }//else{displaySBack(w,vi);}
                    Serial.println(w);
                    continue;
                  }
              }
            }
            continue;
          }
          if (p == 3) { //remove last
            vi.pop_back();//удаление последнего элемента из списка 
            continue;
          }
          if (p == 4) { //add
            //-----------------------------------------------
            int temperature;
            unsigned long timeOnSecond;

            //---------вывод дисплея-------
            SetDisplayNewTempAndTimeForList();

            //------ввод данных-------------
            temperature = inputTemperature(1, 12, 13, 14, 15);
            timeOnSecond = inputTime(2, 12, 13, 15, 16);
            TempAndTime inputTimeAndTemp;

            //--------приравниваем данные к реализованной и инициализрованной структуре выше, чтобы сохранить данные
            inputTimeAndTemp.Temp = temperature;
            inputTimeAndTemp.Time = timeOnSecond;
            vi.push_back(inputTimeAndTemp);//--добавляем к списку функцией push_back()
            //            inputListProrgam();//производится пока, ввод температуры и времени и сохранение
            SetDisplayNewProgram();
            lcd.setCursor(16, 0); lcd.print(num);
            lcd.setCursor(0, 3); lcd.print('>');
            lcd.noBlink();
            //Serial.println(vi.size());
            continue;
          }
          if (p == 5) { //Ok
            NameAndVectorTempAndTime listVi;//Это не список, инициализируем структура, чтобы потом добавит в список структур(nameVi)
            listVi.Name = num;
            listVi.vi = vi;
            nameVi.push_back(listVi);
            
            for (int i = 0; i < nameVi.size(); i++) {
              Serial.print("Name: ");
              Serial.println(nameVi[i].Name);
              for (int p = 0; p < nameVi[i].vi.size(); p++) {
                Serial.print("Temp: ");
                Serial.print(nameVi[i].vi[p].Temp);
                Serial.print(" ;Time: ");
                Serial.print(nameVi[i].vi[p].Time);
              }
            }
            lcd.noCursor();
            running2 = false;
            break;
          }
          LCDBacklight(1);
          lcd.noBlink();
          continue;
        }
    }
  }
}

//функция ниже позволяет вывести на что указывает курсор ">"
//А также листает список (если список заканчивается то последние элементы не показываются)
//т.е. в списке 10 элементов, мы на 1-ом, то 2/3/4 строка заполняется 1/2/3 названием программы
//если также 10 элементов, но мы на 9-ом, то показывается только 9/10 и заполняется 2/3 строка(4 пустая)
void displayS(int i, Vector<struct TempAndTime> vi) {
  if (vi.size() > 0) {
    lcd.setCursor(0, 1);
    lcd.print('>');

    lcd.setCursor(1, 1);
    lcd.print(vi[i].Temp);
    lcd.print("*C  ");
    lcd.print(vi[i].Time);
    lcd.print("sec");
    Serial.println(vi[i].Temp);
    Serial.println(vi[i].Time);
    if (i + 1 < vi.size()) {
      lcd.setCursor(1, 2);
      lcd.print(vi[i + 1].Temp);
      lcd.print("*C  ");
      lcd.print(vi[i + 1].Time);
      lcd.print("sec");
      Serial.println(vi[i + 1].Temp);
      Serial.println(vi[i + 1].Time);
    } else {
      lcd.setCursor(1, 2); lcd.print("                ");
    }
    if (i + 2 < vi.size()) {
      lcd.setCursor(1, 3);
      lcd.print(vi[i + 2].Temp);
      lcd.print("*C  ");
      lcd.print(vi[i + 2].Time);
      lcd.print("sec");
      Serial.println(vi[i + 2].Temp);
      Serial.println(vi[i + 2].Time);
    } else {
      lcd.setCursor(1, 3); lcd.print("                ");
    }

    lcd.setCursor(19, 1);
    lcd.print('<');
  }
}
//Модификация описанного выше решения, но для писка программ, а не температур и времени
void displaySProgramList(int i, Vector<struct NameAndVectorTempAndTime> nVi) {
  if (nVi.size() > 0) {
    lcd.setCursor(0, 1);
    lcd.print('>');

    lcd.setCursor(1, 1);
    lcd.print("Program ");
    lcd.print(nVi[i].Name);
    lcd.print("  ");
    if (i + 1 < nVi.size()) {
      lcd.setCursor(1, 2);
      lcd.print("Program ");
      lcd.print(nVi[i + 1].Name);
      lcd.print("  ");
    } else {
      lcd.setCursor(1, 2); lcd.print("                ");
    }
    if (i + 2 < nVi.size()) {
      lcd.setCursor(1, 3);
      lcd.print("Program ");
      lcd.print(nVi[i + 2].Name);
      lcd.print("  ");
    } else {
      lcd.setCursor(1, 3); lcd.print("                ");
    }

    lcd.setCursor(19, 1);
    lcd.print('<');
  }
}

//в чем смысл - следует из названия. Суть -арзвертывает список программ.
int setListProgram() {
  int w = 0;
  lcd.clear();
  displaySProgramList(w, nameVi);
  bool running = true; // bool определен
  while (running) {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          if (w == 0) {
            lcd.setCursor(0, 0);
            lcd.print(">Back       ");
            lcd.setCursor(19, 0);
            lcd.print("<");

            lcd.setCursor(0, 1);
            lcd.print(" ");
            lcd.setCursor(19, 1);
            lcd.print(" ");
          } else {
            lcd.setCursor(0, 0);
            lcd.print(" Back       ");
            lcd.setCursor(19, 0);
            lcd.print(" ");
          }
          continue;
        }
      case eButton: {
          if (w == 0) {
            running = false; break;
          }
          if (w > 0) {
            if (nameVi.size() > 0) {
              SetDisplayProgramFromPrgList();
              getProgram(w - 1);
              //-------------------
              lcd.clear();
              displaySProgramList(0, nameVi);
              //----------------------
            }
          }
          break;
        }
      case eLeft: {
          if (w > 0) {
            w--;
          }
          if (w >= 1) {
            displaySProgramList(w - 1, nameVi);
            Serial.println(w - 1);
          }
          continue;
        }
      case eRight: {
          if (w <= nameVi.size() - 1) {
            w++;
            displaySProgramList(w - 1, nameVi);
          }
          Serial.println(nameVi.size());
          Serial.println(w);
          continue;
        }
    }
  }
}


//---------------------------------------------------**********************
//-----------------Просто вывод на экран------------------------------
void SetDisplayProgramFromPrgList() {
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("Back");
  lcd.setCursor(1, 1);
  lcd.print("Program 1");
  lcd.setCursor(1, 2);
  lcd.print("List (press button)");

  lcd.setCursor(1, 3);
  lcd.print("Start");
  lcd.setCursor(7, 3);
  lcd.print("Edit");
  lcd.setCursor(13, 3);
  lcd.print("Delete");
}
//--------------------------------------finish---------------------------
//*********************************************************************

//--------------------------------------------
//открывает программу из списка программ и производит примерно то, что и в inputProgram, с небольшой разницей в зполнении 
//элементов списка температур/названии и прочего.
//Попытка дополнить функции друг другом привели к конфвликтам, поэтому несмотря на повторяющийся в некторых местах код
//Лучшим решением оказалось разделить ана разные фнукции - создание нового(inputProgram) и редактирование/откртие старого(это)
unsigned long getProgram(int nowVector) {
  unsigned long F;
  int num = nameVi[nowVector].Name;
  int p = 0;
  int w = 0;

  //TempAndTime storage_array22[ELEMENT_COUNT_MAX];//Это массив динамический с именем, температурой и временем. Ограничен 100 элементов
  Vector<TempAndTime> vi = nameVi[nowVector].vi;
  //----------Настриваем вид экрана-------------------
  SetDisplayProgramFromPrgList();
  lcd.setCursor(0, 0);  lcd.print('>');
  lcd.setCursor(9, 1);  lcd.print(num);
  //-------------------------------------------------
  //Основной цикл - выбор разряда для изменения либо Start/Reset
  bool running2 = true;
  while (running2)
  {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eLeft: { // Двигаем курсор влево
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {
            lcd.setCursor(0, 0);  lcd.print('>');
            continue;
          }  // Левее перемещаться некуда
          if (p == 5) {
            lcd.setCursor(12, 3); lcd.print(' ');
            lcd.setCursor(6, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 4) {
            lcd.setCursor(6, 3); lcd.print(' ');
            lcd.setCursor(0, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 3) {
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(0, 2);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 2) {
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 1);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 1) {
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 0);  lcd.print('>');
            p--;
            lcd.cursor();
            continue;
          }
          // Выбрана частота, перемещаемся к старшему разряду
          p--;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eRight: { // Двигаем курсор вправо
          LCDBacklight(1); // Включаем подсветку
          if (p == 5)continue;
          if (p == 4) {
            lcd.setCursor(6, 3); lcd.print(' ');
            lcd.setCursor(12, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 3) { // Выбран Ok, перемещаемся к Cancel
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(6, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 2) {
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 1) {
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 2); lcd.print('>');
            p++;
            continue;
          }
          if (p == 0) {
            lcd.setCursor(0, 0); lcd.print(' ');
            lcd.setCursor(0, 1); lcd.print('>');
            p++;
            continue;
          }
          // Выбрана частота, перемещаемся к младшему разряду
          p++;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eButton: { //Нажата кнопка
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {  //Back
            vi.clear();
            running2 = false;
            break;
          }
          if (p == 1) {
            continue;
          }
          if (p == 2) { //list
            lcd.clear();
            displayS(w, vi);
            bool running = true; // bool определен
            while (running) {
              EncoderState = GetEncoderState();
              switch (EncoderState) {
                case eNone: {
                    lcd.setCursor(0, 0);
                    lcd.print("Temperature and time");
                    continue;
                  }
                case eButton: {
                    running = false;
                    //---------------Возвращаем экран на место--------
                    SetDisplayProgramFromPrgList();
                    lcd.setCursor(0, 2);  lcd.print('>');
                    lcd.setCursor(9, 1);  lcd.print(num);
                    p = 2;
                    //-------------------------------------------------
                    break;
                  }
                case eLeft: {
                    if (w > 0) w--;
                    displayS(w, vi);
                    Serial.println(w);
                    continue;
                  }
                case eRight: {
                    if (w < vi.size() - 1) {
                      w++;
                      displayS(w, vi);
                    }
                    Serial.println(w);
                    continue;
                  }
              }
            }
            continue;
          }
          if (p == 3) { //start
            continue;
          }
          if (p == 4) { //edit
            EditProgram(nowVector);
            running2 = false;
            break;
          }
          if (p == 5) { //
            int k = YesOrNot(nowVector); //смотрим на функцию ниже - рисует экран и удаляет его;
            running2 = false;
            break;
          }
          LCDBacklight(1);
          lcd.noBlink();
          continue;
        }
    }
  }
}
//--------------------------------------finish---------------------------
//*********************************************************************

int YesOrNot(int i) {
  lcd.clear();

  lcd.setCursor(8, 1);
  lcd.print("Yes");
  lcd.setCursor(12, 1);
  lcd.print("No");
  int w = 0;
  lcd.setCursor(7, 1);  lcd.print('>');
  lcd.setCursor(11, 1);  lcd.print(' ');

  bool running = true; // bool определен
  while (running) {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          lcd.setCursor(0, 0);
          lcd.print("Temperature and time");
          continue;
        }
      case eButton: {
          //---------------Смотрим выбрали да или нет(Yes or Not)-------
          if (w == 0) { //значит YES
            nameVi.remove(i);//Удаляем
          }
          running = false; break;
          //-------------------------------------------------

        }
      case eLeft: {
          if (w > 0) w--;
          if (w == 0) {
            lcd.setCursor(7, 1);  lcd.print('>');
            lcd.setCursor(11, 1);  lcd.print(' ');
          } else {
            lcd.setCursor(11, 1);  lcd.print('>');
            lcd.setCursor(7, 1);  lcd.print(' ');
          }
          Serial.println(w);
          continue;
        }
      case eRight: {
          if (w < 1) {
            w++;
          }
          if (w == 0) {
            lcd.setCursor(7, 1);  lcd.print('>');
            lcd.setCursor(11, 1);  lcd.print(' ');
          } else {
            lcd.setCursor(11, 1);  lcd.print('>');
            lcd.setCursor(7, 1);  lcd.print(' ');
          }
          Serial.println(w);
          continue;
        }
    }
  }
}

//-------------Ниже редактирование программы------------------------------
//************************************************************************

//--------------------------------------------
unsigned long EditProgram(int nowVector) {

  unsigned long F;
  int num;

  Vector<TempAndTime> vi = nameVi[nowVector].vi;
  num = nameVi[nowVector].Name;

  int p = 0;
  int w = 0;
  //----------Настриваем вид экрана-------------------
  SetDisplayNewProgram();
  lcd.setCursor(0, 0);  lcd.print('>');
  lcd.setCursor(16, 0);  lcd.print(num);
  //-------------------------------------------------
  //Основной цикл - выбор разряда для изменения либо Start/Reset
  bool running2 = true;
  while (running2)
  {
    EncoderState = GetEncoderState();
    switch (EncoderState) {
      case eNone: {
          LCDBacklight(1);
          continue;
        }
      case eLeft: { // Двигаем курсор влево
          LCDBacklight(1); // Включаем подсветку
          if (p == 0) {
            lcd.setCursor(0, 0);  lcd.print('>');
            continue;
          }  // Левее перемещаться некуда
          if (p == 5) { // Выбран Quit, перемещаемся к Cancel
            lcd.setCursor(12, 3); lcd.print(' ');
            lcd.setCursor(0, 3);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 4) { // Выбран Cancel, перемещаемся к OK
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(0, 2);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 3) { // Выбран OK, перемещаемся к частоте
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 1);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 2) { // Выбран Cancel, перемещаемся к delete
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(7, 0);  lcd.print('>');
            p--;
            continue;
          }
          if (p == 1) { // Выбран proargam Name, перемещаемся к add
            lcd.setCursor(7, 0); lcd.print(' ');
            lcd.setCursor(0, 0);  lcd.print('>');
            p--;
            lcd.cursor();
            continue;
          }
          p--;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eRight: { // Двигаем курсор вправо
          LCDBacklight(1); // Включаем подсветку
          if (p == 5)continue;
          if (p == 4) {
            lcd.setCursor(0, 3); lcd.print(' ');
            lcd.setCursor(12, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 3) { // Выбран Ok, перемещаемся к Cancel
            lcd.setCursor(0, 2); lcd.print(' ');
            lcd.setCursor(0, 3); lcd.print('>');
            p++;
            continue;
          }
          if (p == 2) {
            lcd.setCursor(0, 1); lcd.print(' ');
            lcd.setCursor(0, 2); lcd.print('>');
            p++;
            continue;
          }
          if (p == 1) {
            lcd.setCursor(7, 0); lcd.print(' ');
            lcd.setCursor(0, 1); lcd.print('>');
            p++;
            continue;
          }
          if (p == 0) {
            lcd.setCursor(0, 0); lcd.print(' ');
            lcd.setCursor(7, 0); lcd.print('>');
            p++;
            continue;
          }
          p++;
          lcd.setCursor(0, 0);  lcd.print('>');
          continue;
        }
      case eButton: { //Нажата кнопка
          LCDBacklight(1); // Включаем подсветку
          if (p == 1) { //Back
            lcd.noCursor();
            num = InputNumber(16, 0, 999);//х1 - номер столбца//строка//ограничение по величине вводимого числа
            lcd.noBlink();
            continue;
          }
          if (p == 0) {
            vi.clear();
            p = 0;
            return;
          }
          if (p == 2) { //list
            lcd.clear();
            displayS(w, vi);
            bool running = true; // bool определен(для выхода из цикла)
            while (running) {
              EncoderState = GetEncoderState();
              switch (EncoderState) {
                case eNone: {
                    lcd.setCursor(0, 0);
                    lcd.print("Temperature and time");
                    continue;
                  }
                case eButton: {
                    running = false;
                    //---------------Возвращаем экран на место
                    SetDisplayNewProgram();
                    lcd.setCursor(16, 0);  lcd.print(num);
                    lcd.setCursor(0, 1);  lcd.print('>');
                    p = 2;
                    //-------------------------------------------------
                    break;
                  }
                case eLeft: {
                    if (w > 0) w--;
                    displayS(w, vi);
                    Serial.println(w);
                    continue;
                  }
                case eRight: {
                    if (w < vi.size() - 1) {
                      w++;
                      displayS(w, vi);
                    }
                    Serial.println(w);
                    continue;
                  }
              }
            }
            continue;
          }
          if (p == 3) { //remove lastr
            vi.pop_back();
            continue;
          }
          if (p == 4) { //add
            //-----------------------------------------------
            int temperature;
            unsigned long timeOnSecond;
            SetDisplayNewTempAndTimeForList();
            
            temperature = inputTemperature(1, 12, 13, 14, 15);
            timeOnSecond = inputTime(2, 12, 13, 15, 16);
            TempAndTime inputTimeAndTemp;
            
            inputTimeAndTemp.Temp = temperature;
            inputTimeAndTemp.Time = timeOnSecond;
            vi.push_back(inputTimeAndTemp);
            //-----------------------------------------------
            SetDisplayNewProgram();
            lcd.setCursor(16, 0);  lcd.print(num);
            lcd.setCursor(0, 3); lcd.print('>');
            lcd.noBlink();
            continue;
          }
          if (p == 5) { //Ok
            NameAndVectorTempAndTime listVi = {num, vi};
            Serial.println("num:");
            Serial.print(num);
            nameVi[nowVector] = {num, vi};
            Serial.println(nowVector);
            Serial.println(nameVi[nowVector].Name);
          }
          //----------------------------------------
          lcd.noCursor();
          running2 = false;
          break;
        }
        LCDBacklight(1);
        lcd.noBlink();
        continue;
    }
  }
}


void getDataFromEEPROM() {
  //int eeAddress = sizeof(float); // Переместить адрес к байту, следующему после float 'f'.

  //NameAndVectorTempAndTimeM my_point234;
  NameAndVectorTempAndTime my_point234;
  nameVi.clear();
  EEPROM.get(eeAddress, nameVi);

  Serial.print("Read custom object from EEPROM: Name: ");
  // Serial.print(my_point234.Name);
  for (int i; i < nameVi.size(); i++) {
    Serial.print("Name = ");
    Serial.println(nameVi[i].Name);
    for (int j = 0; j < nameVi[i].vi.size(); j++) {
      Serial.print("TempAndTime № ");
      Serial.print(j);
      Serial.print(" Temp= ");
      int sa = nameVi[i].vi[j].Temp;
      Serial.print(sa);
      Serial.print(" Time= ");
      sa = nameVi[i].vi[j].Time;
      Serial.println(sa);
    }
  }
}

void getPIDFromEEPROM() {
  EEPROM.get(eeAddress, PidStruct);
  kp = PidStruct.kp;ki = PidStruct.ki;kd = PidStruct.kd;
  Serial.print("kp: "); Serial.print(kp);
  Serial.print("ki: "); Serial.print(ki);
  Serial.print("kd: "); Serial.print(kd);
}

//--------------------------НИже про нагрев/-----------------------------------
void SerialSend()
{
  Serial.print("Setpoint: "); Serial.print(Setpoint); Serial.print(" ");
  Serial.print("Input: "); Serial.print(Input); Serial.print(" ");
  Serial.print("Output: "); Serial.print(Output); Serial.print(" ");
  if (tuning) {
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: "); Serial.print(myPID.GetKp()); Serial.print(" ");
    Serial.print("ki: "); Serial.print(myPID.GetKi()); Serial.print(" ");
    Serial.print("kd: "); Serial.print(myPID.GetKd()); Serial.println();
    myPID.SetTunings(kp, ki, kd);
  }
}

void pppeeechkaWork() {

  unsigned long now = millis();

  Input = thermocouple.readCelsius() - 4;

  //в данном случае tuning = false, но если поставить true, то реализуется autoTune (пусть и не до конца рабочий)
  //КОгда false - реализуется просто PID регулятор
  if (tuning)
  {
    byte val = (aTune.Runtime());
    if (val != 0)
    {
      tuning = false;
    }
    if (!tuning)
    { //we're done, set the tuning parameters
      SerialSend();
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();//происходят расчеты с помощью которых PID и вычисляет. Резльутат появлется в переменной Output.

  if (useSimulation) {}//не используется(существует возможность моделирования, но по факту она бесполезна)
  else
  {
    //---данная реализация ниже позволяет открывать
    if (millis() - windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (Output < millis() - windowStartTime) digitalWrite(RELAY_PIN, LOW);
    else digitalWrite(RELAY_PIN, HIGH);
    //---------------------------------------
  }

  //send-receive with processing if it's time
  if (millis() > serialTime)
  {
    SerialSend();
    serialTime += 500;
  }
}

void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

void changeAutoTune()
{
  if (!tuning)
  {
    //Set the Output to the desired starting frequency.
    Output = aTuneStartValue;
    aTune.SetControlType(0);
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}
int setKpKiKd(int posLine1,int posLine2,int posLine3, int posStroke1, int posStroke2, int posStroke3){
  int numberGran = 20;//органичение по величене вводимого числа, т.е. не более 20.
  lcd.blink();
  kp = InputNumberDouble(posStroke1, posLine1, numberGran, 0.1);//0.1/0.01 - насколько изменяется число при повороте Энкодедера в ту или иную сторону
  ki = InputNumberDouble(posStroke2, posLine2, numberGran, 0.01);
  kd = InputNumberDouble(posStroke3, posLine3, numberGran, 0.01);
  PidStruct.kp = kp;PidStruct.ki = ki;PidStruct.kd = kd;//Заполняем структуру,которая будут записана в EEPROM памяти
  lcd.noBlink();
  return;
}

//--Просто вывод на дисплей данных о PID
void dispPIDnow(){  
  lcd.clear();
  lcd.setCursor(1,1); lcd.print("kp: "); lcd.print(kp);
  lcd.setCursor(1,2); lcd.print("ki: "); lcd.print(ki);
  lcd.setCursor(1,3); lcd.print("kd: "); lcd.print(kd);
}
