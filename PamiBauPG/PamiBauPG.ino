
#include "arduino.h"
#include "LibPami.h"
// #include "MemoryFree.h"

#define DEBUG_PAMI

#ifndef DEBUG_PAMI
//============= REGLAGE PAMI CONCOURS =========================
#define DUREE_WAIT_TO_RUN_PAMI1  70000 // Durée d'attente avant de partir pour le PAMI 1 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI2  70000 // Durée d'attente avant de partir pour le PAMI 2 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI3  70000 // Durée d'attente avant de partir pour le PAMI 3 (en ms)

#else
//============= REGLAGE PAMI DEBUG =========================
#define DUREE_WAIT_TO_RUN_PAMI1  15000 // Durée d'attente avant de partir pour le PAMI 1 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI2  15000 // Durée d'attente avant de partir pour le PAMI 2 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI3  15000 // Durée d'attente avant de partir pour le PAMI 3 (en ms)

#endif

#define DUREE_RUN_PAMI1  4000   // Durée de mouvement pour le PAMI 1 (en ms)
#define DUREE_RUN_PAMI2  3500   // Durée de mouvement pour le PAMI 2 (en ms)
#define DUREE_RUN_PAMI3  2500   // Durée de mouvement pour le PAMI 3 (en ms)


LibPami pami;

// Gestion de la team
unsigned char team;
unsigned char pamiNb;

// Gestion des bras
#define GPIO_BRAS_GAUCHE    PAMI_GPIO_3 
#define GPIO_BRAS_DROIT     PAMI_GPIO_4
#define DEFAULT_BRAS_ANGLE  90    // L'angle au repos en degré des bras
#define MAX_BRAS_ANGLE      30    // Angle maximum de déplacement d'un bras
#define PERIOD_GESTION_BRAS 300   // Période de battement des bras en ms

// Gestion des Leds et Boule à facettes
#define GPIO_LED_GAUCHE       PAMI_GPIO_1
#define GPIO_LED_DROITE       PAMI_GPIO_2
#define GPIO_BOULE_FACETTE    PAMI_GPIO_3
#define DEFAULT_FACETTE_ANGLE 90    // L'angle au repos en degré de la boule à facette
#define PERIOD_GESTION_BOULE  500   // Période d'oscillation de la boule en ms

bool flagActivity = false; // Indique si l'activité Bras ou Boule doit être activée

// Gestion des moteurs
#define VITESSE_MOTEUR 60
bool runPami = false;

// Gestion de la direction du PAMI
#define GPIO_DIRECTION_TEAM_A    PAMI_GPIO_4
#define GPIO_DIRECTION_TEAM_B    PAMI_GPIO_2
#define DEFAULT_DIRECTION_ANGLE  90   // L'angle au repos en degré
#define PERIOD_GESTION_DIRECTION 100   // Période d'évaluation de la direction en ms
#define TIME_BEFORE_RECALIBRAGE  2000 // Temps d'attente avant recalibrage en ms
unsigned char gpioDirection;

// Gestion des états du PAMI
// Chaque état correspond à une étape du PAMI
typedef enum {
  PAMI_WAIT_RECALIBRAGE = 0, // Le PAMI attente quelques secondes pour se recalibrer
  PAMI_SAY_READY_TO_RUN = 1, // Le PAMI dit qu'il est pret après recalibrage
  PAMI_WAIT_TO_RUN      = 2, // Le PAMI attends pour partir
  PAMI_ON_ROAD          = 3, // Le PAMI est en route
  PAMI_ARRIVED          = 4  // Le PAMI est arrivé => Il applaudit
} t_statusPami;
t_statusPami statusPami = PAMI_WAIT_RECALIBRAGE;

#define DUREE_SAY_READY_TO_RUN  1000 // Durée de battement des bras pour acquitter son initialisation (en ms)

unsigned long dureeWaitToRun; // Temps d'attente avant de démarrer le parcours
unsigned long dureeRunPami;   // Temps de durée du parcours
unsigned long endStatusTime;  // Fin d'un état en cours avant passage vers le suivant


void setup(void){
  Serial.begin(9600);

  pami.afficheur.begin();
  pami.afficheur.displayString("init");

  Serial.println("PAMI controlé par BAU");

  // pami.gpio.setDebug(true);
  // pami.radio.setDebug(true);

  // On initialise la team et le Pami
  team   = pami.jumper.getTeam();
  pamiNb = pami.jumper.getPami();
  Serial.print(" Team="); Serial.print(team);
  Serial.print(" Pami="); Serial.println(pamiNb);

  // En fonction du PAMI, les durées d'attentes et de run sont différentes
  switch(pamiNb) {
    case PAMI_NB_1 : dureeWaitToRun = millis()+DUREE_WAIT_TO_RUN_PAMI1; dureeRunPami = DUREE_RUN_PAMI1; break;
    case PAMI_NB_2 : dureeWaitToRun = millis()+DUREE_WAIT_TO_RUN_PAMI2; dureeRunPami = DUREE_RUN_PAMI2; break;
    case PAMI_NB_3 : dureeWaitToRun = millis()+DUREE_WAIT_TO_RUN_PAMI3; dureeRunPami = DUREE_RUN_PAMI3; break;
  }

  // On configure le moteur
  pami.moteur.moteurDroit(0);

  // On configure les GPIO
  if( team == PAMI_TEAM_A ) {
    pami.gpio.configure(GPIO_LED_GAUCHE, PAMI_GPIO_OUTPUT, 1);
    pami.gpio.configure(GPIO_LED_DROITE, PAMI_GPIO_OUTPUT, 1);
    pami.gpio.configure(GPIO_BOULE_FACETTE, PAMI_GPIO_PWM, DEFAULT_FACETTE_ANGLE);
    pami.gpio.configure(GPIO_DIRECTION_TEAM_A, PAMI_GPIO_PWM, DEFAULT_DIRECTION_ANGLE);
    gpioDirection = GPIO_DIRECTION_TEAM_A;
  } else {
    pami.gpio.configure(GPIO_BRAS_GAUCHE,      PAMI_GPIO_PWM, DEFAULT_BRAS_ANGLE);
    pami.gpio.configure(GPIO_BRAS_DROIT,       PAMI_GPIO_PWM, DEFAULT_BRAS_ANGLE);
    pami.gpio.configure(GPIO_DIRECTION_TEAM_B, PAMI_GPIO_PWM, DEFAULT_DIRECTION_ANGLE);
    gpioDirection = GPIO_DIRECTION_TEAM_B;
  }

  // On initialise le gyroscope
  pami.gyro.begin();
  pami.gyro.selectAxis(GYROSCOPE_AXIS_X);
  pami.gyro.display(true);

  // On lance le PAMI vers sa première étape
  switchToWaitRecalibrage();
}


void loop(void){
   displayStatus(); // On affiche le Status sur la console pour le debug

  // En fonction du status, on appelle la fonction correspondant à l'étape
  switch( statusPami ) {
    case PAMI_WAIT_RECALIBRAGE : pamiWaitRecalibrage(); break;
    case PAMI_SAY_READY_TO_RUN : pamiSayReadyToRun();   break; 
    case PAMI_WAIT_TO_RUN      : pamiWaitToRun();       break; 
    case PAMI_ON_ROAD          : pamiOnRoad();          break; 
    case PAMI_ARRIVED          : pamiArrived();         break; 
  }

  if( team == PAMI_TEAM_A ) {
    gestionBoule(); // Gestion des claps
  } else {
    gestionClap(); // Gestion de la boule à facette
  }
  gestionRun();  // Gestion du run du PAMI
  pami.gestion(); 
}


// Les routines suivantes sont appelées pour passer à une autre étape
void switchToWaitRecalibrage() { statusPami = PAMI_WAIT_RECALIBRAGE; endStatusTime = millis()+TIME_BEFORE_RECALIBRAGE; }
void switchToSayReadyToRun()   { statusPami = PAMI_SAY_READY_TO_RUN; endStatusTime = millis()+DUREE_SAY_READY_TO_RUN;  }
void switchToWaitToRun()       { statusPami = PAMI_WAIT_TO_RUN;      endStatusTime = millis()+dureeWaitToRun;  }
void switchToOnRoad()          { statusPami = PAMI_ON_ROAD;          endStatusTime = millis()+dureeRunPami;    }
void switchToArrived()         { statusPami = PAMI_ARRIVED;                                                    }


// Dans cette étape, le PAMI attends quelques secondes avant de faire un recalibrage
void pamiWaitRecalibrage(void) {
  if( endStatusTime < millis()) {
    pami.gyro.calibrate();
    flagActivity = true;
    switchToSayReadyToRun();
  }
}


// Dans cette étape, le PAMI indique que le recalibrage est terminé
void pamiSayReadyToRun(void) {
  if( endStatusTime < millis()) {
    flagActivity = false;
    switchToWaitToRun();
  }
}


// Dans cette étape, le PAMI attends la fin de la manche pour partir
void pamiWaitToRun() {
  if( dureeWaitToRun < millis()) {
    pami.moteur.moteurDroit(VITESSE_MOTEUR);
    runPami = true;
    switchToOnRoad();
  }
}


// Dans cette étape, le PAMI est en route vers sa destination
void pamiOnRoad() {
  if( endStatusTime < millis()) {
    pami.moteur.moteurDroit(0);
    runPami = false;
    switchToArrived();
  }
}


// Dans cette étape, le PAMI est arrivé et applaudit
void pamiArrived() {
  flagActivity = true;
}


// Cette routine permet de gérer les claps
void gestionClap( void ) {
  static short lastValue = 0;
  static unsigned long nextTime = millis()+PERIOD_GESTION_BRAS;
  static bool resetBrasDone = false;

  // On ne gére les claps que toutes les PERIOD_GESTION_BRAS ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_BRAS;

  // S'il n'y a pas d'activité en cours, on remet les bras au repos
  if( !flagActivity ) {
    if(resetBrasDone == true) return;
    pami.gpio.set(GPIO_BRAS_GAUCHE, DEFAULT_BRAS_ANGLE);
    pami.gpio.set(GPIO_BRAS_DROIT,  DEFAULT_BRAS_ANGLE);
    lastValue = DEFAULT_BRAS_ANGLE;
    resetBrasDone = true;
    return;
  }

  lastValue = ( lastValue == 0 ? MAX_BRAS_ANGLE : 0);
  pami.gpio.set(GPIO_BRAS_GAUCHE, DEFAULT_BRAS_ANGLE - lastValue);
  pami.gpio.set(GPIO_BRAS_DROIT,  DEFAULT_BRAS_ANGLE + lastValue);

  resetBrasDone = false;
}

// Cette routine permet de gérer les claps
void gestionBoule( void ) {
  static short lastValue = 0;
  static unsigned long nextTime = millis()+PERIOD_GESTION_BOULE;
  static bool resetLedsDone = false;

  // On ne gére la boule que toutes les PERIOD_GESTION_OULE ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_BOULE;

  // S'il n'y a pas d'activité en cours, on remet éteint les leds
  if( !flagActivity ) {
    if(resetLedsDone == true) return;
    pami.gpio.set(GPIO_LED_GAUCHE, 0);
    pami.gpio.set(GPIO_LED_DROITE, 0);
    resetLedsDone = true;
    return;
  }

  if( lastValue == 0 ) {
    pami.gpio.set(GPIO_LED_GAUCHE, 0);
    pami.gpio.set(GPIO_LED_DROITE, 1);
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE+45);
    lastValue = 1;
  } else {
    pami.gpio.set(GPIO_LED_GAUCHE, 1);
    pami.gpio.set(GPIO_LED_DROITE, 0);
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE-45);
    lastValue = 0;
  }
}


// Cette routine permet de gérer le run du PAMI
void gestionRun() {
  static unsigned long  nextTime = millis()+PERIOD_GESTION_DIRECTION;
  int anglePami;
  unsigned short anglePwm;

  // On ne gére les claps que toutes les PERIOD_GESTION_DIRECTION ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_DIRECTION;

  if( !runPami ) return; 
  anglePami = pami.gyro.getAngle();
  // anglePwm = anglePami*2;
  anglePwm = anglePami;

  pami.gpio.set(gpioDirection, DEFAULT_DIRECTION_ANGLE+anglePwm);
}


// Cette routine permet d'afficher sur la console et pour le debug, l'étape en cours
void displayStatus( void ) {
  static t_statusPami lastDisplayStatus = 100;

  if( statusPami == lastDisplayStatus ) return;

  switch( statusPami ) {
    case PAMI_WAIT_RECALIBRAGE : Serial.println(F("PAMI_WAIT_RECALIBRAGE")); break;
    case PAMI_SAY_READY_TO_RUN : Serial.println(F("PAMI_SAY_READY_TO_RUN")); break; 
    case PAMI_WAIT_TO_RUN      : Serial.println(F("PAMI_WAIT_TO_RUN"));      break; 
    case PAMI_ON_ROAD          : Serial.println(F("PAMI_ON_ROAD"));          break; 
    case PAMI_ARRIVED          : Serial.println(F("PAMI_ARRIVED"));          break; 
  }
  lastDisplayStatus = statusPami;
}  

