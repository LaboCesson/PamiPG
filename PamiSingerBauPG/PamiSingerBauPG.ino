
#include "arduino.h"
#include "LibPami.h"
// #include "MemoryFree.h"

#define DEBUG_PAMI

#ifndef DEBUG_PAMI
//============= REGLAGE PAMI CONCOURS =========================
#define DUREE_WAIT_TO_RUN_PAMI  70000 // Durée d'attente avant de partir pour le PAMI Singer (en ms)

#else
//============= REGLAGE PAMI DEBUG =========================
#define DUREE_WAIT_TO_RUN_PAMI  15000 // Durée d'attente avant de partir pour le PAMI Singer (en ms)

#endif

LibPami pami;

// Gestion de la team et de la direction du virage;
unsigned char team;
unsigned char directionVirage;

// Gestion des bras du tambour
#define GPIO_TAMBOUR_GAUCHE    PAMI_GPIO_4 
#define GPIO_TAMBOUR_DROIT     PAMI_GPIO_3
#define DEFAULT_TAMBOUR_ANGLE  90    // L'angle au repos en degré des bras
#define MAX_TAMBOUR_ANGLE      30    // Angle maximum de déplacement d'un bras
#define PERIOD_GESTION_TAMBOUR 200   // Période de battement sur le tambour en ms

bool flagActivity = false; // Indique si l'activité Tambour ou Display doit être activée

// Gestion des moteurs
#define VITESSE_MOTEUR        80 // Vitesse par défaut des moteurs
#define PAMI_TOURNE_A_GAUCHE  1  // Sur la scene le PAMI tourne à gauche
#define PAMI_TOURNE_A_DROITE  0  // Sur la scene le PAMI tourne à droite

bool runPami = false;
int  angleToRun = 0;                       // Angle que doit gérer l'asservissement du run
char vitesseMoteurDefaut = VITESSE_MOTEUR;

// Gestion de la direction du PAMI pour le virage sur la scene
#define GPIO_DIRECTION           PAMI_GPIO_1 // Indique si le PAMI doit tourner à droite ou à gauche

//#define PERIOD_GESTION_DIRECTION 100  // Période d'évaluation de la direction en ms
#define PERIOD_GESTION_DIRECTION 50   // Période d'évaluation de la direction en ms
#define PERIOD_GESTION_MONTEE    100  // Période d'évaluation de l'axe Y pour la montée  
#define TIME_BEFORE_RECALIBRAGE  2000 // Temps d'attente avant recalibrage en ms
#define PERIOD_GESTION_TURN      20   // Période d'évaluation du virage en ms

// Gestion de l'ultrason
#define ULTRASON_MAX_DISTANCE    50   // Distance maximum à détecter
#define PERIOD_GESTION_ULTRASON  20   // Période d'évaluation de l'ultrason  
#define ULTRASON_SEUIL_SCENE     60   // Seuil de détection du bord de scene

// Gestion du display
#define PERIOD_GESTION_DISPLAY  500   // Période de modification du display en ms

// Gestion des états du PAMI
// Chaque état correspond à une étape du PAMI
typedef enum {
  PAMI_WAIT_RECALIBRAGE = 0, // Le PAMI attente quelques secondes pour se recalibrer
  PAMI_SAY_READY_TO_RUN = 1, // Le PAMI dit qu'il est pret après recalibrage
  PAMI_WAIT_TO_RUN      = 2, // Le PAMI attends pour partir
  PAMI_ON_ROAD          = 3, // Le PAMI est en route
  PAMI_GO_UP            = 4, // Le PAMI monte la pente
  PAMI_TURN             = 5, // Le PAMI tourne
  PAMI_WAIT_EDGE        = 6, // Le PAMI attends d'être au bord de la scene
  PAMI_ARRIVED          = 7  // Le PAMI est arrivé => Il tape sur des tambours
} t_statusPami;
t_statusPami statusPami = PAMI_WAIT_RECALIBRAGE;

#define DUREE_SAY_READY_TO_RUN  1000 // Durée de battement des bras pour acquitter son initialisation (en ms)

unsigned long dureeWaitToRun; // Temps d'attente avant de démarrer le parcours
unsigned long endStatusTime;  // Fin d'un état en cours avant passage vers le suivant


void setup(void){
  Serial.begin(9600);

  pami.afficheur.begin();
  pami.afficheur.displayString("init");

  Serial.println("PAMI Singer controlé par BAU");

  // pami.gpio.setDebug(true);
  // pami.radio.setDebug(true);

  // On initialise la team
  team = pami.jumper.getTeam();
  Serial.print("Team="); Serial.println(team);

  // On initialise les durée d'attente
  dureeWaitToRun = millis()+DUREE_WAIT_TO_RUN_PAMI;

  // On configure les moteurs
  stopRun();
  pami.moteur.setPwmMode(false); 

  // On configure les GPIO
  if( team == PAMI_TEAM_A ) {
    pami.gpio.configure(GPIO_TAMBOUR_GAUCHE, PAMI_GPIO_PWM, DEFAULT_TAMBOUR_ANGLE);
    pami.gpio.configure(GPIO_TAMBOUR_DROIT,  PAMI_GPIO_PWM, DEFAULT_TAMBOUR_ANGLE);
    pami.gpio.configure(GPIO_DIRECTION, PAMI_GPIO_INPUT, 0);
  } else {
    // TODO
  }
  directionVirage = pami.gpio.get(GPIO_DIRECTION);

  // On initialise le gyroscope
  pami.gyro.begin();
  pami.gyro.selectAxis(GYROSCOPE_AXIS_X);
  // pami.gyro.display(true);

  // On initialise l'ultrason
  pami.ultrason.begin();
  pami.ultrason.setMaxDistance(ULTRASON_MAX_DISTANCE);

  // Fin de la phase d'initialisation
  pami.afficheur.displayString("Pret");

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
    case PAMI_GO_UP            : pamiGoUp();            break; 
    case PAMI_TURN             : pamiTurn();            break; 
    case PAMI_WAIT_EDGE        : pamiWaitEdge();        break; 
    case PAMI_ARRIVED          : pamiArrived();         break; 
  }

  gestionTambours();
  gestionDisplay();
  gestionRun();
  pami.gestion(); 
}


// Les routines suivantes sont appelées pour passer à une autre étape
void switchToWaitRecalibrage() { statusPami = PAMI_WAIT_RECALIBRAGE; endStatusTime = millis()+TIME_BEFORE_RECALIBRAGE; }
void switchToSayReadyToRun()   { statusPami = PAMI_SAY_READY_TO_RUN; endStatusTime = millis()+DUREE_SAY_READY_TO_RUN;  }
void switchToWaitToRun()       { statusPami = PAMI_WAIT_TO_RUN;      endStatusTime = millis()+dureeWaitToRun;  }
void switchToOnRoad()          { statusPami = PAMI_ON_ROAD;   }
void switchToGoUp()            { statusPami = PAMI_GO_UP;     }
void switchToTurn()            { statusPami = PAMI_TURN;      }
void switchToTWaitEdge()       { statusPami = PAMI_WAIT_EDGE; }
void switchToArrived()         { statusPami = PAMI_ARRIVED;   }


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
    runPami = true;
    pami.afficheur.displayString(" Go ");
    switchToOnRoad();
  }
}


// Dans cette étape, le PAMI commence son périple et attends d'atteindre la pente
void pamiOnRoad() {

// static unsigned long nextT = millis()+4000;
// if( millis() < nextT) return;
// pami.moteur.moteurDroit(0);
// pami.moteur.moteurGauche(0);
// return;

  static unsigned long nextTime = millis()+PERIOD_GESTION_MONTEE;

  // On ne gére l'axe Z que toutes les PERIOD_GESTION_MONTEE ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_MONTEE;

  if( pami.gyro.getAngle(GYROSCOPE_AXIS_Y) < -5 ) {
    pami.afficheur.displayString(" Up ");
    switchToGoUp();
  }
}


// Dans cette étape, le PAMI monte la pente et attends d'être sur le podium
void pamiGoUp() {
  static unsigned long nextTime = millis()+PERIOD_GESTION_MONTEE;

  // On ne gére l'axe Z que toutes les PERIOD_GESTION_MONTEE ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_MONTEE;

  if( pami.gyro.getAngle(GYROSCOPE_AXIS_Y) > -2  ) {
    pami.afficheur.displayString("Turn");
    switchToTurn();
  }
}


// Dans cette étape, le PAMI se tourne vers les spectateurs
void pamiTurn() {

  runPami = false; // Dans cette étape, on suspend la régulation sur angle

  if( directionVirage == PAMI_TOURNE_A_GAUCHE ) {
    setMoteurDroit(40);
    setMoteurGauche(20);
    angleToRun = -90;
  } else {
    setMoteurDroit(20);
    setMoteurGauche(40);
    angleToRun = 90;
  }

  if( directionVirage == PAMI_TOURNE_A_GAUCHE ) {
    while( pami.gyro.getAngle(GYROSCOPE_AXIS_X) > (angleToRun+15) ) pami.gestion();
  } else {
    while( pami.gyro.getAngle(GYROSCOPE_AXIS_X) < (angleToRun-15) ) pami.gestion();
  }

//  vitesseMoteurDefaut = 20; // On avance tres doucement sur le bord
  switchToTWaitEdge();
}


// Dans cette étape, le PAMI attends d'être au bord
void pamiWaitEdge() {
  unsigned short distance = 400; // 400 pour forcer a entrer dans la boucle

  // runPami = true;  // On redémarre la régulation
  setMoteurDroit(20);
  setMoteurGauche(20);

  while( (distance >= 400) || (distance < ULTRASON_SEUIL_SCENE ) ) {
    distance = pami.ultrason.getDistance();
    // if( (distance < 400) && (distance > ULTRASON_SEUIL_SCENE ) ) break;
    // gestionRun();
    pami.gestion();
  }

  stopRun();
  runPami = false;
  switchToArrived();
}


// Dans cette étape, le PAMI est arrivé et tape du tambour
void pamiArrived() {
  flagActivity = true;
}


// Cette routine permet de gérer les tambours
void gestionTambours( void ) {
  static short lastValue = 0;
  static unsigned long nextTime = millis()+PERIOD_GESTION_TAMBOUR;
  static bool resetTamboursDone = false;

  // On ne gére les tambours que toutes les PERIOD_GESTION_TAMBOUR ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_TAMBOUR;

  // S'il n'y a pas d'activité en cours, on remet les bras au repos
  if( !flagActivity ) {
    if(resetTamboursDone == true) return;
    pami.gpio.set(GPIO_TAMBOUR_GAUCHE, DEFAULT_TAMBOUR_ANGLE);
    pami.gpio.set(GPIO_TAMBOUR_DROIT,  DEFAULT_TAMBOUR_ANGLE);
    lastValue = DEFAULT_TAMBOUR_ANGLE;
    resetTamboursDone = true;
    return;
  }

  if( lastValue == 0) {
    pami.gpio.set(GPIO_TAMBOUR_GAUCHE,DEFAULT_TAMBOUR_ANGLE);
    pami.gpio.set(GPIO_TAMBOUR_DROIT,DEFAULT_TAMBOUR_ANGLE-20);
    lastValue = 1;
  } else {
    pami.gpio.set(GPIO_TAMBOUR_GAUCHE,DEFAULT_TAMBOUR_ANGLE+20);
    pami.gpio.set(GPIO_TAMBOUR_DROIT,DEFAULT_TAMBOUR_ANGLE);
    lastValue = 0;
  }
  resetTamboursDone = false;
}


// Cette routine permet d'animer le display
void gestionDisplay( void ) {
  static unsigned long nextTime = millis()+PERIOD_GESTION_DISPLAY;
  static short index = 0;
  static bool resetDisplayDone = false;

  // On ne gére le display que toutes les PERIOD_GESTION_DISPLAY ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_DISPLAY;

  if( !flagActivity ) {
    if(resetDisplayDone == true) return;
    pami.afficheur.displayString("O  O");
    resetDisplayDone = true;
    return;
  }

  switch(index) {
    case 0: pami.afficheur.displayString("O  O"); index += 1; break;
    case 1: pami.afficheur.displayString("o  O"); index += 1; break;
    case 2: pami.afficheur.displayString("O  O"); index += 1; break;
    case 3: pami.afficheur.displayString("-  O"); index  = 0; break;
  }
  resetDisplayDone = false;
}


// Cette routine permet de gérer le run du PAMI
void gestionRun() {
  static unsigned long  nextTime = millis()+PERIOD_GESTION_DIRECTION;
  int  anglePami;
  char vitesseDroite = vitesseMoteurDefaut;
  char vitesseGauche = vitesseMoteurDefaut;

  // On ne gére les moteurs que toutes les PERIOD_GESTION_DIRECTION ms
  if( millis() < nextTime) return;
  while( nextTime <= millis() ) nextTime += PERIOD_GESTION_DIRECTION;

  if( !runPami ) {
    // setMoteurDroit(0);
    // setMoteurGauche(0);
    return;
  }

  anglePami = pami.gyro.getAngle(GYROSCOPE_AXIS_X);

  if( anglePami > 0 ) {
    vitesseDroite += 20;
  }
  if( anglePami < 0 ) {
    vitesseDroite -= 20;
  }

  setMoteurDroit(vitesseDroite);
  setMoteurGauche(vitesseGauche);
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
    case PAMI_GO_UP            : Serial.println(F("PAMI_GO_UP"));            break; 
    case PAMI_TURN             : Serial.println(F("PAMI_TURN"));             break; 
    case PAMI_WAIT_EDGE        : Serial.println(F("PAMI_WAIT_EDGE"));        break; 
    case PAMI_ARRIVED          : Serial.println(F("PAMI_ARRIVED"));          break; 
  }
  lastDisplayStatus = statusPami;
}  


// Ces fonctions permettent d'éviter de recabler le moteur
void setMoteurDroit (char vitesse) { pami.moteur.moteurDroit(-vitesse); }
void setMoteurGauche(char vitesse) { pami.moteur.moteurGauche(-vitesse); }
void stopRun(void) { pami.moteur.moteurs(0); }
