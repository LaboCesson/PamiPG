
#include "arduino.h"
#include "LibPami.h"
// #include "MemoryFree.h"

// #define DEBUG_PAMI

#ifndef DEBUG_PAMI
//============= REGLAGE PAMI CONCOURS =========================
#define DUREE_WAIT_TO_RUN_PAMI1  85000 // Durée d'attente avant de partir pour le PAMI 1 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI2  86000 // Durée d'attente avant de partir pour le PAMI 2 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI3  87000 // Durée d'attente avant de partir pour le PAMI 3 (en ms)

#else
//============= REGLAGE PAMI DEBUG =========================
#define DUREE_WAIT_TO_RUN_PAMI1  10000 // Durée d'attente avant de partir pour le PAMI 1 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI2  11000 // Durée d'attente avant de partir pour le PAMI 2 (en ms)
#define DUREE_WAIT_TO_RUN_PAMI3  12000 // Durée d'attente avant de partir pour le PAMI 3 (en ms)

#endif

// Durée de run des Pamis de la team A => Boule
#define DUREE_RUN_PAMI1_TEAM_A  4200   // Durée de mouvement pour le PAMI 1 (en ms)
#define DUREE_RUN_PAMI2_TEAM_A  3400   // Durée de mouvement pour le PAMI 2 (en ms)
#define DUREE_RUN_PAMI3_TEAM_A  3000   // Durée de mouvement pour le PAMI 3 (en ms)

// Durée de run des Pamis de la team B => Bras
#define DUREE_RUN_PAMI1_TEAM_B  4400   // Durée de mouvement pour le PAMI 1 (en ms)
#define DUREE_RUN_PAMI2_TEAM_B  3600   // Durée de mouvement pour le PAMI 2 (en ms)
#define DUREE_RUN_PAMI3_TEAM_B  3000   // Durée de mouvement pour le PAMI 3 (en ms)


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
#define MAX_BRAS_BOULE        45    // Angle maximum de rotation de la boule
#define PERIOD_GESTION_BOULE  500   // Période d'oscillation de la boule en ms

bool flagActivity = false; // Indique si l'activité Bras ou Boule doit être activée

// Gestion de la radio
typedef enum {
  RADIO_NO_ORDER = 0, ///< Aucun message de reçu
  RADIO_START    = 1, ///< Message de départ
  RADIO_STOP     = 2, ///< Message d'arrêt
} t_radioOrder;

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
bool recalibrationDone = false;       // Permet de recalibrer le gyroscope pendant la période d'attente

// Gestion des états du PAMI
// Chaque état correspond à une étape du PAMI
typedef enum {
  PAMI_SAY_READY    = 0, // Le PAMI dit qu'il est en attente d'un ordre => 1 seconde d'activité
  PAMI_WAIT_START   = 1, // Le PAMI est en attente d'un ordre de départ 
  PAMI_SAY_OK_START = 2, // Le PAMI dit qu'il a recu l'odre de départ   =>  3 secondes d'activité
  PAMI_WAIT_TO_RUN  = 3, // Le PAMI attends pour partir
  PAMI_ON_ROAD      = 4, // Le PAMI est en route
  PAMI_ARRIVED      = 5  // Le PAMI est arrivé => Il applaudit ou fait tourner la boule
} t_statusPami;
t_statusPami statusPami = PAMI_SAY_READY;

#define DUREE_SAY_READY    1000 // Durée de battement des bras pour acquitter son initialisation (en ms)
//#define DUREE_SAY_OK_START 3000 // Durée de battement des bras pour acquitter acquitter la réception radio (en ms)
#define DUREE_SAY_OK_START 1000 // Durée de battement des bras pour acquitter acquitter la réception radio (en ms)

unsigned long dureeWaitToRun; // Temps d'attente avant de démarrer le parcours
unsigned long dureeRunPami;   // Temps de durée du parcours
unsigned long endStatusTime;  // Fin d'un état en cours avant passage vers le suivant


void setup(void){
  Serial.begin(9600);

  pami.afficheur.begin();
  pami.afficheur.displayString("init");

  Serial.print("PAMI controlé par Radio");

  // pami.gpio.setDebug(true);
  // pami.radio.setDebug(true);

  // On initialise la team et le Pami
  team   = pami.jumper.getTeam();
  pamiNb = pami.jumper.getPami();
  Serial.print(" Team="); Serial.print(team);
  Serial.print(" Pami="); Serial.println(pamiNb);

  // En fonction du PAMI, les durées d'attentes et de run sont différentes
  if( team == PAMI_TEAM_A ) {
    switch(pamiNb) {
      case PAMI_NB_1 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI1; dureeRunPami = DUREE_RUN_PAMI1_TEAM_A; break;
      case PAMI_NB_2 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI2; dureeRunPami = DUREE_RUN_PAMI2_TEAM_A; break;
      case PAMI_NB_3 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI3; dureeRunPami = DUREE_RUN_PAMI3_TEAM_A; break;
    }
  }
  else {
    switch(pamiNb) {
      case PAMI_NB_1 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI1; dureeRunPami = DUREE_RUN_PAMI1_TEAM_B; break;
      case PAMI_NB_2 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI2; dureeRunPami = DUREE_RUN_PAMI2_TEAM_B; break;
      case PAMI_NB_3 : dureeWaitToRun = DUREE_WAIT_TO_RUN_PAMI3; dureeRunPami = DUREE_RUN_PAMI3_TEAM_B; break;
    }
  }

  // On configure le moteur
  pami.moteur.moteurDroit(0);

  // On configure la radio
  pami.radio.begin(16);

  // On configure les GPIO
  if( team == PAMI_TEAM_A ) {
    pami.gpio.configure(GPIO_LED_GAUCHE, PAMI_GPIO_OUTPUT, 1);
    pami.gpio.configure(GPIO_LED_DROITE, PAMI_GPIO_OUTPUT, 1);
    pami.gpio.configure(GPIO_BOULE_FACETTE, PAMI_GPIO_PWM, DEFAULT_FACETTE_ANGLE);
    pami.gpio.configure(GPIO_DIRECTION_TEAM_A, PAMI_GPIO_PWM, DEFAULT_DIRECTION_ANGLE);
    gpioDirection = GPIO_DIRECTION_TEAM_A;

    // on signale le démarrage
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE+MAX_BRAS_BOULE);
    delay(500);
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE);
  } else {
    pami.gpio.configure(GPIO_BRAS_GAUCHE,      PAMI_GPIO_PWM, DEFAULT_BRAS_ANGLE);
    pami.gpio.configure(GPIO_BRAS_DROIT,       PAMI_GPIO_PWM, DEFAULT_BRAS_ANGLE);
    pami.gpio.configure(GPIO_DIRECTION_TEAM_B, PAMI_GPIO_PWM, DEFAULT_DIRECTION_ANGLE);
    gpioDirection = GPIO_DIRECTION_TEAM_B;

    // on signale le démarrage
    pami.gpio.set(GPIO_BRAS_GAUCHE, DEFAULT_BRAS_ANGLE-MAX_BRAS_ANGLE);
    delay(500);
    pami.gpio.set(GPIO_BRAS_GAUCHE, DEFAULT_BRAS_ANGLE);
  }

  // On initialise le gyroscope
  pami.gyro.begin();
  pami.gyro.selectAxis(GYROSCOPE_AXIS_X);
  pami.gyro.display(true);

  // On lance le PAMI vers sa première étape
  flagActivity = true;
  recalibrationDone = false;
  switchToSayReady();
}


void loop(void){

// Serial.println("Trace");
// pami.gpio.set(gpioDirection, DEFAULT_DIRECTION_ANGLE+10);
// delay(500);
// pami.gpio.set(gpioDirection, DEFAULT_DIRECTION_ANGLE-10);
// delay(500);
// return;

   displayStatus(); // On affiche le Status sur la console pour le debug

  // En fonction du status, on appelle la fonction correspondant à l'étape
  switch( statusPami ) {
    case PAMI_SAY_READY    : pamiSayReady();   break;
    case PAMI_WAIT_START   : pamiWaitStart();  break; 
    case PAMI_SAY_OK_START : pamiSayOkStart(); break; 
    case PAMI_WAIT_TO_RUN  : pamiWaitToRun();  break; 
    case PAMI_ON_ROAD      : pamiOnRoad();     break; 
    case PAMI_ARRIVED      : pamiArrived();    break; 
  }

  if( team == PAMI_TEAM_A ) {
    gestionBoule(); // Gestion des claps
  } else {
    gestionClap(); // Gestion de la boule à facette
  }
  gestionRun();   // Gestion du run du PAMI
  gestionRadio(); // Gestion des réceptions radio
  pami.gestion(); // Gestion des organes du PAMI
}


// Les routines suivantes sont appelées pour passer à une autre étape
void switchToSayReady()   { statusPami = PAMI_SAY_READY;    endStatusTime = millis()+DUREE_SAY_READY;    }
void switchToWaitStart()  { statusPami = PAMI_WAIT_START;                                                }
void switchToSayOkStart() { statusPami = PAMI_SAY_OK_START; endStatusTime = millis()+DUREE_SAY_OK_START; }
void switchToWaitToRun()  { statusPami = PAMI_WAIT_TO_RUN;  endStatusTime = millis()+dureeWaitToRun;     }
void switchToOnRoad()     { statusPami = PAMI_ON_ROAD;      endStatusTime = millis()+dureeRunPami;       }
void switchToArrived()    { statusPami = PAMI_ARRIVED;                                                   }


// Dans cette étape, le PAMI indique qu'il est prêt à recevoir un ordre de départ
void pamiSayReady(void) {
  if( endStatusTime > millis()) return; 
  flagActivity = false;
  switchToWaitStart();
}

// Dans cette étape, le PAMI attend un ordre radio de départ
void pamiWaitStart(void) {
  if( gestionRadio() != RADIO_START ) return;
  flagActivity = true;
  switchToSayOkStart();
}


// Dans cette étape, le PAMI indique qu'il a bien reçu l'ordre de départ
void pamiSayOkStart(void) {
  if( endStatusTime > millis()) return;
  flagActivity = false;
  switchToWaitToRun();
}


// Dans cette étape, le PAMI attends la fin de la manche pour partir
void pamiWaitToRun() {
  if( recalibrationDone == false ) {
    pami.gyro.calibrate();
    recalibrationDone = true; 
  }

  // // Possiblité d'interrompre le PAMI et d'attendre de nouveau un ordre de départ
  // if( gestionRadio() == RADIO_STOP ) {
  //   switchToWaitStart();
  // }

  if( endStatusTime > millis()) return;
  pami.moteur.moteurDroit(VITESSE_MOTEUR);
  runPami = true;
  switchToOnRoad();
}


// Dans cette étape, le PAMI est en route vers sa destination
void pamiOnRoad() {
  if( endStatusTime > millis()) return;
  pami.moteur.moteurDroit(0);
  runPami = false;
  flagActivity = true;
  switchToArrived();
}


// Dans cette étape, le PAMI est arrivé et applaudit
void pamiArrived() {
  // // Possiblité d'interrompre le clap
  // // UNIQUEMENT POUR LE DEBUG
  // if( gestionRadio() == RADIO_STOP ) {
  //   switchToWaitStart();
  // }
}


// Cette routine permet de tester si un ordre radio est arrivé
t_radioOrder gestionRadio( void ) {
  #define RADIO_MSG_SIZE 40
  char msg[RADIO_MSG_SIZE];

  unsigned char len = pami.radio.getMessage(msg, RADIO_MSG_SIZE);

  if( len == 0 )  return RADIO_NO_ORDER;
  if( msg[0] != 'T' ) return RADIO_NO_ORDER;
  if( msg[1] != ( team == PAMI_TEAM_A ? 'A' : 'B')) return RADIO_NO_ORDER;
  if( msg[2] == 'G' ) return RADIO_START;
  if( msg[2] == 'S' ) return RADIO_STOP;
  return RADIO_NO_ORDER;
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
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE+MAX_BRAS_BOULE);
    lastValue = 1;
  } else {
    pami.gpio.set(GPIO_LED_GAUCHE, 1);
    pami.gpio.set(GPIO_LED_DROITE, 0);
    pami.gpio.set(GPIO_BOULE_FACETTE, DEFAULT_FACETTE_ANGLE-MAX_BRAS_BOULE);
    lastValue = 0;
  }
}


// Cette routine permet de gérer le run du PAMI
void gestionRun() {
  static unsigned long  nextTime = millis()+PERIOD_GESTION_DIRECTION;
  int anglePami;
  unsigned short anglePwm;

  // On ne gére la direction que toutes les PERIOD_GESTION_DIRECTION ms
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
    case PAMI_SAY_READY    : Serial.println("PAMI_SAY_READY");    break;
    case PAMI_WAIT_START   : Serial.println("PAMI_WAIT_START");   break; 
    case PAMI_SAY_OK_START : Serial.println("PAMI_SAY_OK_START"); break; 
    case PAMI_WAIT_TO_RUN  : Serial.println("PAMI_WAIT_TO_RUN");  break; 
    case PAMI_ON_ROAD      : Serial.println("PAMI_ON_ROAD");      break; 
    case PAMI_ARRIVED      : Serial.println("PAMI_ARRIVED");      break; 
  }
  lastDisplayStatus = statusPami;
}  

