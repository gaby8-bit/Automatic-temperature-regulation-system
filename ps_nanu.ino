#include <LiquidCrystal.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <EEPROM.h>

//    ------------------------------------------ Constante ------------------------------------------
#define BUTTON_OK 6
#define BUTTON_CANCEL 7
#define BUTTON_PREV 8
#define BUTTON_NEXT 9
//#define TP36_SENSOR_CHANNEL 0  // Dacă senzorul de temperatură este conectat la pinul A0
#define ADC_REF_VOLTAGE 5.0    // Tensiune de referință 5V

//    ------------------------------------------ Enum ------------------------------------------
enum Buttons {
  EV_OK,
  EV_CANCEL,
  EV_NEXT,
  EV_PREV,
  EV_NONE,
  EV_MAX_NUM
};

enum Menus {
  // meniuri principale
  MENU_MAIN = 0,
  MENU_PID,
  MENU_TEMP,
  MENU_START,
  // meniuri modificare PID
  SHOW_KP,
  SHOW_KI,
  SHOW_KD,
  MODIFY_KP,
  MODIFY_KI,
  MODIFY_KD,
  // meniuri modificare Temp, timp
  SHOW_TEMP,
  SHOW_Tinc,
  SHOW_Tmen,
  SHOW_Trac,
  MODIFY_TEMP,
  MODIFY_Tinc,
  MODIFY_Tmen,
  MODIFY_Trac,
  //
  MENU_MAX_NUM
};

enum Stari {
  STOP = 0,
  INCALZIRE,
  MENTINERE,
  RACIRE
};

//    ------------------------------------------ Variabile globale si prototipuri ------------------------------------------
// ecran lcd
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// variabile pentru PID
double kp = 5, ki = 1, kd = 0.5; // pot fi modificate in aplicatie
double kpt = kp, kit=ki, kdt=kd;   // variabile temporare
double eroare= 0;
double suma_erori= 0;
double eroare_anterioara = 0;
double derivativa = 0;
double dt; // timp esantionare
double output;
double setpoint = 30;
int transistorPin = 10;   // pinul la care e legat tranzistorul

// variabile pentru timpi si temperatura
double tInc = 10, tMen = 10, tRac = 10;  // pot fi modificate in aplicatie
double tInct = tInc, tMent = tMen, tRact = tRac;  // variabile temporare
double temp = 30;   // poate fi setata in aplicatie
double tempt = temp;    // variabila temporara
double temp_citita = 0;
String mod = "";    // afiseaza starea actuala (Inc, Men, Rac - Incalzire, Mentinere, Racire)
double mod_timp = 0;  // ia una din valorile de timp ale starilor (tInc, tMen, tRac)

// initializari
Menus menu = MENU_MAIN; // meniul e setat pe meniul principal
Stari stare = (Stari)0; // starea e setata pe oprita
bool start = false;   // proces pornit/oprit
bool newState = true;
double timpRamas;
double temp_initiala;
double temp_intermediara;
double pas;
double startTime;

// timer
unsigned long previousMillis = 0;
unsigned long interval = 1000;  // 1 secundă
void displayTime(); // protoripul functiei de afisare pe ecran
int ore=8;
int minute=0;
int secunde=0;

// adresele pentru salvarea in memorie
const int address_kp   = 0;              // Adresa pentru kp
const int address_ki   = address_kp + sizeof(double);  // Adresa pentru ki
const int address_kd   = address_ki + sizeof(double);  // Adresa pentru kd
const int address_temp = address_kd + sizeof(double);  // Adresa pentru temp
const int address_tInc = address_temp + sizeof(double); // Adresa pentru tInc
const int address_tMen = address_tInc + sizeof(double); // Adresa pentru tMen
const int address_tRac = address_tMen + sizeof(double); // Adresa pentru tRac
void salvare_memorie();
bool incarcat = false;

//    ------------------------------------------ Citire temperatura ------------------------------------------
// ----- Incercare de rezolvare cu registri (fail) -----
/*
void init_adc() 
{
      // 1. Configurarea ADC-ului
    R_ADC0->ADCSR_b.ADST = 0;        // Asigură-te că ADC-ul este oprit înainte de configurare
    R_ADC0->SCKDIVCR = 0x01;          // Selectează canalul 0 (A0) pentru conversie
    R_ADC0->ADCER = 0x00;            // Configurare de bază pentru registre
    R_ADC0->ADSTRGR = 0x0000;        // Setează declanșarea software pentru conversie
    R_ADC0->ADCSR_b.ADCS = 0;        // Modul de scanare unică (single scan)
}
R_ADC0->ADCSR_b.ADST = 1;        // Pornește conversia ADC

// 2. Așteaptă finalizarea conversiei
while (R_ADC0->ADCSR_b.ADST == 1);

// 3. Citește valoarea ADC (rezultatul conversiei)
uint16_t adc_value = R_ADC0->ADDRn[0];  // Citește valoarea din registrul canalului 0

// 4. Transformă valoarea ADC în tensiune
float voltage = (float)adc_value * VREF / ADC_RESOLUTION;

// 5. Calculează temperatura în grade Celsius
double temperature = (voltage - 0.5) * 100.0;

return temperature;
*/

double read_temperature() {
  //Citește valoarea analogică de la senzor
  float sensorValue = analogRead(A0);
  // Calculează tensiunea pe baza valorii citite
  float voltage = sensorValue * (ADC_REF_VOLTAGE / 16384.0);
  //transformare din tensiune in grade celsius
  double temperature = voltage * 1000.0;

  // Aproximare temperatura afisata din 0.5 in 0.5 grade
  if ((temperature - (int)temperature) > 0.7)
    return (int)temperature + 1;
  else if ((temperature - (int)temperature) > 0.3)
    return (int)temperature + 0.5;
  else return (int)temperature;
}

//    --------------------- Meniu ---------------------
void state_machine(enum Menus menu, enum Buttons button);
Buttons GetButtons(void);
void print_menu(enum Menus menu);

typedef void (state_machine_handler_t)(void);

void print_menu(enum Menus menu)
{
  lcd.clear();
  switch(menu)
  {
    // 		meniuri principale
    case MENU_PID:
    	lcd.print("KP=" + String(kp) + " KI=" + String(ki));
    	lcd.setCursor(0,1);
    	lcd.print("KD=" + String(kd));
    	break;
    case MENU_TEMP:
    	lcd.print("Ts=");
        lcd.print(temp); 
    	lcd.print(" Tc=");
    	lcd.print(temp_citita);
		lcd.setCursor(0, 1);
		lcd.print("t" + mod + " = " + String(mod_timp) + "s");
    	break;
    // 	START
    case MENU_START:
      if(!start)
    	  lcd.print("Incepeti?");
      else lcd.print("Opriti procesul?");
    	break;
    // 		meniuri secundare
    // pid
    case SHOW_KP:
    	lcd.print("KP = ");
    	lcd.print(kp);
    	break;
    case MODIFY_KP:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("KP = ");
    	lcd.print(kpt);
    	break;
    case SHOW_KI:
    	lcd.print("KI = ");
    	lcd.print(ki);
    	break;
    case MODIFY_KI:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("KI = ");
    	lcd.print(kit);
    	break;
    case SHOW_KD:
    	lcd.print("KD = ");
    	lcd.print(kd);
    	break;
    case MODIFY_KD:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("KD = ");
    	lcd.print(kdt);
    	break;
    // temp
    case SHOW_TEMP:
    	lcd.print("TEMP SET = ");
    	lcd.print(temp);
    	break;
    case MODIFY_TEMP:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("TEMP SET = ");
    	lcd.print(tempt);
    	break;
    case SHOW_Tinc:
    	lcd.print("Timp inc = ");
    	lcd.print(tInc);
    	break;
    case MODIFY_Tinc:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("Timp inc = ");
    	lcd.print(tInct);
    	break;
    case SHOW_Tmen:
    	lcd.print("Timp men = ");
    	lcd.print(tMen);
    	break;
    case MODIFY_Tmen:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("Timp men = ");
    	lcd.print(tMent);
    	break;
    case SHOW_Trac:
    	lcd.print("Timp rac = ");
    	lcd.print(tRac);
    	break;
    case MODIFY_Trac:
    	lcd.print("mofifica");
    	lcd.setCursor(0, 1);
    	lcd.print("Timp rac = ");
    	lcd.print(tRact);
    	break;
    // 			default
    case MENU_MAIN:
    default:
    	lcd.print("PS-NANU 2024-25");
    	displayTime();
   		break;
  }
}

// functii pentru butoane
// comenzi principale
void go_home(void)
{
  menu = MENU_MAIN;
}
void go_pid(void)
{
  menu = MENU_PID;
}
void go_temp(void)
{
  menu = MENU_TEMP;
}
void go_next(void)
{
  menu = (Menus) ((int)menu + 1);
}
void go_prev(void)
{
  menu = (Menus) ((int)menu - 1);
}
void go_start(void)
{
  menu = MENU_START;
}
void start_sra(void)
{
  lcd.clear();
  if(!start) lcd.print("Proces inceput");
  else lcd.print("Proces oprit");
  delay(2000);
  start = !start;
  menu = MENU_MAIN;
  if (temp_citita < temp)
    stare = (Stari)1;
  else if (temp_citita >= temp + 3)
    stare = (Stari)3;
  else 
    stare = (Stari)2;
}

// comenzi de salvare
// pid
void save_kp(void)
{
  kp = kpt;
  menu = SHOW_KP;
  salvare_memorie();
}
void save_ki(void)
{
  ki = kit;
  menu = SHOW_KI;
  salvare_memorie();
}
void save_kd(void)
{
  kd = kdt;
  menu = SHOW_KD;
  salvare_memorie();
}
//temp
void save_temp(void)
{
  temp = tempt;
  menu = SHOW_TEMP;
  salvare_memorie();
}
void save_tinc(void)
{
  tInc = tInct;
  menu = SHOW_Tinc;
  salvare_memorie();
}
void save_tmen(void)
{
  tMen = tMent;
  menu = SHOW_Tmen;
  salvare_memorie();
}
void save_trac(void)
{
  tRac = tRact;
  menu = SHOW_Trac;
  salvare_memorie();
}

// comenzi de incrementare si decrementare
// pid
void inc_kp(void)
{
  kpt += 0.5;
}
void dec_kp(void)
{
  kpt -= 0.5;
}
void inc_ki(void)
{
  kit += 0.1;
}
void dec_ki(void)
{
  kit -= 0.1;
}
void inc_kd(void)
{
  kdt += 0.1;
}
void dec_kd(void)
{
  kdt -= 0.1;
}
// temp
void inc_temp(void)
{
  tempt++;
}
void dec_temp(void)
{
  tempt--;
}
void inc_tinc(void)
{
  tInct++;
}
void dec_tinc(void)
{
  tInct--;
}
void inc_tmen(void)
{
  tMent++;
}
void dec_tmen(void)
{
  tMent--;
}
void inc_trac(void)
{
  tRact++;
}
void dec_trac(void)
{
  tRact--;
}

// meniuri secundare
// pid
void acc_pid(void)
{
  menu = SHOW_KP;
}
// temp
void acc_temp(void)
{
  menu = SHOW_TEMP;
}

// meniuri secundare secundare
// pid
void mod_pid(void)
{
  menu = (Menus) ((int)menu + 3);
}
// temp
void mod_temp(void)
{
  menu = (Menus) ((int)menu + 4);
}

// comenzi anulare
void cancel(void) 
{
  switch (menu)
  {
    case 6:
    	menu = SHOW_KP;
    	break;
    case 7:
    	menu = SHOW_KI;
    	break;
    case 8:
    	menu = SHOW_KD;
      	break;
    case 13:
    	menu = SHOW_TEMP;
    	break;
    case 14:
    	menu = SHOW_Tinc;
    	break;
    case 15:
    	menu = SHOW_Trac;
    	break;
    case 16:
    	menu = SHOW_Tmen;
    	break;
  }
}

// navigare meniuri
state_machine_handler_t* sm[MENU_MAX_NUM][EV_MAX_NUM] = 
{ //events: OK , CANCEL , NEXT, PREV
  {go_home, go_home, go_next, go_start}, 	 // MENU_MAIN
  {acc_pid, go_home, go_next, go_prev},      // MENU_PID
  {acc_temp, go_home, go_next, go_prev},	 // MENU_TEMP
  {start_sra, go_home, go_home, go_prev},	 // MENU_START
    //
    //
  {mod_pid, go_pid, go_next, acc_pid},   // SHOW_KP
  {mod_pid, go_pid, go_next, go_prev},      // SHOW_KI
  {mod_pid, go_pid, acc_pid, go_prev},	// SHOW_KD
    //
  {save_kp, cancel, inc_kp, dec_kp},   // MODIFY_KP
  {save_ki, cancel, inc_ki, dec_ki},   // MODIFY_KI
  {save_kd, cancel, inc_kd, dec_kd},   // MODIFY_KD
    //
    //
  {mod_temp, go_temp, go_next, acc_temp},   	// SHOW_TEMP
  {mod_temp, go_temp, go_next, go_prev},   	// SHOW_Tinc
  {mod_temp, go_temp, go_next, go_prev},   	// SHOW_Tmen
  {mod_temp, go_temp, acc_temp, go_prev},   	// SHOW_Trac
    //
  {save_temp, cancel, inc_temp, dec_temp},	// MODIFY_TEMP
  {save_tinc, cancel, inc_tinc, dec_tinc},	// MODIFY_Tinc
  {save_tmen, cancel, inc_tmen, dec_tmen},	// MODIFY_Tmen
  {save_trac, cancel, inc_trac, dec_trac}	// MODIFY_Trac
};

void state_machine(enum Menus menu, enum Buttons button)
{
  sm[menu][button]();
}

//    ------------------------------------------ Butoane ------------------------------------------
Buttons GetButtons(void)
{
  enum Buttons ret_val = EV_NONE;
  if (digitalRead(BUTTON_OK))
  {
    ret_val = EV_OK;
  }
  else if (digitalRead(BUTTON_CANCEL))
  {
    ret_val = EV_CANCEL;
  }
  else if (digitalRead(BUTTON_NEXT))
  {
    ret_val = EV_NEXT;
  }
  else if (digitalRead(BUTTON_PREV))
  {
    ret_val = EV_PREV;
  }
  Serial.print(ret_val);
  return ret_val;
}

//    ------------------------------------------ Timer ------------------------------------------
// -------- Incercare de rezolvare cu registri pentru R4 (fail) --------
// void setupTimer() {
//   // 1. Activează ceasul pentru timerul GPT0
//   R_MSTP->MSTPCRB &= ~(1 << 0);  // Dezactivează modul de oprire pentru GPT0
  
//   // 2. Resetează timerul GPT0
//   R_GPT0->GTCR = 0x01;          // Resetează configurația timerului
  
//   // 3. Configurează modul timer
//   R_GPT0->GTCR = 0x00;          // Selectează modul normal de funcționare
//   R_GPT0->GTCNT = 0x00;         // Resetează contorul
//   R_GPT0->GTPR = 31250 - 1;     // Setează perioada (16 MHz / 512 prescaler => 1 ms)

//   // 4. Activează întreruperile (opțional)
//   R_GPT0->GTINTAD = 0x01;       // Activează întreruperile la overflow

//   NVIC_EnableIRQ(GPT0_IRQn);    // Activează întreruperea în NVIC

//   // 5. Pornește timerul
//   R_GPT0->GTCR |= 0x01;         // Pornește timerul
// }

// Funcție pentru întreruperea timerului GPT0
// void GPT0_Handler() {
//   if (R_GPT0->GTSR & 0x01) {    // Verifică flag-ul de overflow
//     R_GPT0->GTSR &= ~(0x01);    // Resetează flag-ul
//     seconds++;                  // Incrementare contor de secunde
//   }

// -------- Incercare de rezolvare cu registri pentru R3 (functioneaza in Tinkercad) --------
/*
void setupTimer1()
{
 TCCR1A = 0;  // Nu folosim PWM
    TCCR1B = 0;
    TCNT1 = 0;  // Inițializare contor
    OCR1A = 15624;  // Valoare de comparare pentru frecvență de 1Hz (16 MHz cu prescaler de 1024)
    TCCR1B |= (1 << WGM12);  // Mod CTC
    TCCR1B |= (1 << CS12) | (1 << CS10);  // Prescaler 1024
    TIMSK1 |= (1 << OCIE1A);  // Activare întrerupere CTC
    sei();  // Activare întreruperi globale
}

ISR(TIMER1_COMPA_vect) {
  // Actualizare timp la fiecare întrerupere (o secundă)
  secunde++;
}
*/

void displayTime() {
	// Afișăm ora pe al doilea rând
    lcd.setCursor(0, 1);
    lcd.print("Ora: ");
    if (ore < 10) lcd.print("0");
    lcd.print(ore);
    lcd.print(":");
    if (minute < 10) lcd.print("0");
    lcd.print(minute);
    lcd.print(":");
    if (secunde < 10) lcd.print("0");
    lcd.print(secunde);
    if(secunde>=59)
    { 
      minute++;
      secunde = 0;
    }
     if(minute>=59)
    { 
      ore++;
      minute = 0;
    }
}

//    ------------------------------------------ Calcul PWM ------------------------------------------
void calcul_PWM() 
{
  // Calcul PID
  eroare = temp - temp_initiala;
  suma_erori = suma_erori + eroare * dt;
  derivativa = (eroare - eroare_anterioara) / dt;
  output = (kp * eroare) + (ki * suma_erori) + (kd * derivativa);
  eroare_anterioara = eroare;

  // Reglare output sa fie intre 0 si 255
  if(output>255)
    output=255;
  if(output<0)
    output=0;
  //delay(mod_timp*1000);
}

//    ------------------------------------------ Salvarea si citirea din memorie ------------------------------------------
void salvare_memorie()
{
    // Salvarea fiecărui parametru în EEPROM
  EEPROM.put(address_kp, kp);
  EEPROM.put(address_ki, ki);
  EEPROM.put(address_kd, kd);
  EEPROM.put(address_temp, temp);
  EEPROM.put(address_tInc, tInc);
  EEPROM.put(address_tMen, tMen);
  EEPROM.put(address_tRac, tRac);
}
  
  // Citire parametri din EEPROM
void citire_memorie()
{
  EEPROM.get(address_kp, kp);
  EEPROM.get(address_ki, ki);
  EEPROM.get(address_kd, kd);
  EEPROM.get(address_temp, temp);
  EEPROM.get(address_tInc, tInc);
  EEPROM.get(address_tMen, tMen);
  EEPROM.get(address_tRac, tRac);
}

  // Actualizare variabile temporare
void actualizare_vart()
{
    kpt = kp;
    kit = ki;
    kdt = kd;
    tempt = temp;
    tInct = tInc;
    tMent = tMen;
    tRact = tRac;
}


//    ------------------------------------------ Setup + loop ------------------------------------------
void setup()
{
  Serial.begin(9600);

  // configurare butoane
  lcd.begin(16,2);
  pinMode(6, INPUT);
  digitalWrite(6, LOW); // pull-down
  pinMode(7, INPUT);
  digitalWrite(7, LOW); // pull-down
  pinMode(8, INPUT);
  digitalWrite(8, LOW); // pull-down
  pinMode(9, INPUT);
  digitalWrite(9, LOW); // pull-down

  // Initializari functii
  displayTime();  // afisare timp
  pinMode(transistorPin, OUTPUT); // porneste pwm

  startTime = millis();

  // Salvare in memorie 
  // salvare_memorie();   // decomentat doar la prima incarcare a codului, apoi comentat si incarcat din nou codul 

  // Citire parametri din memorie
  citire_memorie();
  // Actualizare variabile temporare
  actualizare_vart();
}

void loop()
{
  // Timer
   unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { // Verifică dacă a trecut o secundă
    previousMillis = currentMillis;
    secunde++;  // Incrementare contor de secunde
  }

  // Citire butoane
  volatile Buttons event = GetButtons();
  if (event != EV_NONE)
  {
    state_machine(menu, event);
  }
  print_menu(menu); // afisare meniu
 
  if (start) // daca procesul este pornit
  {
    if (stare == (Stari)1) {    // incalzire
      startTime = secunde;
      mod_timp = tInc;
      mod = "Inc";
      if(newState) {
      	timpRamas = mod_timp;
        temp_citita = read_temperature();
        temp_initiala = temp_citita;
        temp_intermediara = temp_initiala;
        newState = false;
        pas = (temp - temp_initiala)/timpRamas;
        calcul_PWM();
	      analogWrite(transistorPin,output);
      } else if (timpRamas == 0) {
        newState = true;
        stare = (Stari)2;
      }
      else {
       	temp_intermediara += pas; 
        temp_citita = temp_intermediara;
        timpRamas--;
        mod_timp = timpRamas;
        // Las temperatura sa creasca putin peste cea setata, ca sa poata sa scada apoi in stadiul de mentinere
        if (temp_intermediara >= temp) {
          temp_intermediara += 2*pas;
        }
      }
      
    } else if (stare == (Stari)3) {   // racire
      mod_timp = tRac;
      mod = "Rac";
      if(newState) {
      	timpRamas = mod_timp;
        temp_citita = read_temperature();
        temp_initiala = temp_citita;
        temp_intermediara = temp_initiala;
        newState = false;
        pas = (temp - temp_initiala)/timpRamas;
        analogWrite(transistorPin, 0);
      } else if (timpRamas == 0) {
        newState = true;
        stare = (Stari)2;
      }
      else {
       	temp_intermediara += pas; 
        temp_citita = temp_intermediara;
        timpRamas--;
        mod_timp = timpRamas;
      }
    }
    else if(stare == (Stari)2) {    // mentinere
      mod_timp = tMen;
      mod = "Men";
      temp_citita = temp_intermediara;
      if(newState) {
      	timpRamas = mod_timp;
        temp_initiala = temp_citita;
        newState = false;
        pas = (temp - temp_initiala)/timpRamas;
        calcul_PWM();
	      analogWrite(transistorPin,output);
      } else if (timpRamas == 0) {
        newState = true;
        stare = (Stari)1;
      }
      else {
       	temp_intermediara += pas; 
        temp_citita = temp_intermediara;
        timpRamas--;
        mod_timp = timpRamas;
      }
    }
  }
  else // daca procesul este oprit
    {
      stare = (Stari)0;
      mod_timp = 0;
      mod = "";
      temp_citita = read_temperature();
      analogWrite(transistorPin, 0);
    }

  delay(1000);
}
