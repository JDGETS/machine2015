#include "Motors.h"

typedef unsigned char uint8_t;

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- INB4 CLASS -------------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

typedef struct LimitSwitch LimitSwitch;
typedef struct ReedSwitch ReedSwtich;
typedef struct XBeeComm XBeeComm;
typedef struct OpticalSensor OpticalSensor;

namespace States {

    typedef struct ChargementBatterie ChargementBatterie;
    typedef struct NouvellePocheDansMagasin NouvellePocheDansMagasin;

    typedef struct ParcoursAvecLanceur ParcoursAvecLanceur;
    typedef struct MonteeAvecLanceur MonteeAvecLanceur;
    typedef struct DecenteAvecLanceur DecenteAvecLanceur;

    typedef struct VirageEntreeZoneLancement VirageEntreeZoneLancement;
    typedef struct AlignmentReedSwitchZoneLancement AlignmentReedSwitchZoneLancement;
    typedef struct DeploiementCanon DeploiementCanon;

    typedef struct DeploiementLimitSwitchs DeploiementLimitSwitchs;
    typedef struct VerificationDeclenchementLimitSwitchs VerificationDeclenchementLimitSwitchs;

    typedef struct SequenceCalculRotationCible SequenceCalculRotationCible;
    typedef struct SequenceLancementPoches SequenceLancementPoches;

    typedef struct SeparationLanceur SeparationLanceur;
    typedef struct VirageSortieZoneLancement VirageSortieZoneLancement;

    typedef struct ParcoursSansLanceur ParcoursSansLanceur;
    typedef struct MonteeSansLanceur MonteeSansLanceur;
    typedef struct DecenteSansLanceur DecenteSansLanceur;

    const unsigned short CHARGEMENT_BATTERIE_STATE_ID = 1;
    const unsigned short NOUVELLE_POCHE_DANS_MAGASIN_STATE_ID = 2;

    const unsigned short PARCOURS_AVEC_LANCEUR_STATE_ID = 3;
    const unsigned short MONTEE_AVEC_LANCEUR_STATE_ID = 4;
    const unsigned short DECENTE_AVEC_LANCEUR_STATE_ID = 5;

    const unsigned short VIRAGE_ENTREE_ZONE_LANCEMENT_STATE_ID = 6;
    const unsigned short ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT_STATE_ID = 7;
    const unsigned short DEPLOIEMENT_CANON_STATE_ID = 8;

    const unsigned short DEPLOIEMENT_LIMITSWITCHS_STATE_ID = 9;
    const unsigned short VERIFICATION_DECLENCHEMENT_LIMITSWITCHS_STATE_ID = 10;

    const unsigned short SEQUENCE_CALCUL_ROTATION_CIBLE_STATE_ID = 11;
    const unsigned short SEQUENCE_LANCEMENT_POCHES_STATE_ID = 12;

    const unsigned short SEPARATION_LANCEUR_STATE_ID = 13;
    const unsigned short VIRAGE_SORTIE_ZONE_LANCEMENT_STATE_ID = 14;

    const unsigned short PARCOURS_SANS_LANCEUR_STATE_ID = 15;
    const unsigned short MONTEE_SANS_LANCEUR_STATE_ID = 16;
    const unsigned short DECENTE_SANS_LANCEUR_STATE_ID = 17;
}

class INB4 {
public:
    static void setup();
    static void loop();
    static void showXBeeCommRead(const int &read);
};

extern XBeeComm XBEECOMM;
extern LimitSwitch LIMIT_SWITCH;

extern float OPTICAL_SENSOR_MIN_ACTIVE_VALUE;
extern OpticalSensor OPTICAL_SENSOR;

extern unsigned short CURRENT_STATES_ID;

namespace States {

    extern ChargementBatterie CHARGEMENT_BATTERIE;
    extern NouvellePocheDansMagasin NOUVELLE_POCHE_DANS_MAGASIN;

    extern ParcoursAvecLanceur PARCOURS_AVEC_LANCEUR;
    extern MonteeAvecLanceur MONTEE_AVEC_LANCEUR;
    extern DecenteAvecLanceur DECENTE_AVEC_LANCEUR;

    extern VirageEntreeZoneLancement VIRAGE_ENTREE_ZONE_LANCEMENT;
    extern AlignmentReedSwitchZoneLancement ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT;
    extern DeploiementCanon DEPLOIEMENT_CANON;

    extern DeploiementLimitSwitchs DEPLOIEMENT_LIMITSWITCHS;
    extern VerificationDeclenchementLimitSwitchs VERIFICATION_DECLENCHEMENT_LIMITSWITCHS;

    extern SequenceCalculRotationCible SEQUENCE_CALCUL_ROTATION_CIBLE;
    extern SequenceLancementPoches SEQUENCE_LANCEMENT_POCHES;

    extern SeparationLanceur SEPARATION_LANCEUR;
    extern VirageSortieZoneLancement VIRAGE_SORTIE_ZONE_LANCEMENT;

    extern ParcoursSansLanceur PARCOURS_SANS_LANCEUR;
    extern MonteeSansLanceur MONTEE_SANS_LANCEUR;
    extern DecenteSansLanceur DECENTE_SANS_LANCEUR;
}

///////////////////////////////////////////////////////////////////////////////
///                                                                         ///
/// --- MACHINE STATES ---------------------------------------------------- ///
///                                                                         ///
///////////////////////////////////////////////////////////////////////////////

namespace States {

    struct ChargementBatterie {

        ChargementBatterie() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct NouvellePocheDansMagasin {

        NouvellePocheDansMagasin() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct ParcoursAvecLanceur {

        ParcoursAvecLanceur() : active(false), firstpass(true) {};

        void Update() {

            if (firstpass)
            {
                time = millis();
                firstpass = false;
            }

            if (OPTICAL_SENSOR.LongReadInput(false))
            {
                CURRENT_STATES_ID = MONTEE_AVEC_LANCEUR_STATE_ID;
                active = false;
                delay(20);
            }
            else
            {
                active = true;
            }
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.35);
            MOTORS.Speed(MOTOR_RIGHT, 0.35);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;
        bool firstpass;
        unsigned long time;
    };

    struct MonteeAvecLanceur {

        MonteeAvecLanceur() : active(false), sommetDosDane(false), _FirstExecuteTime(0) {};

        void Update() {

            /*
                    27 DEC 00:35 -> le char rentre dans la pente et change d'etat (le vehicule stop) presque immidiatement apr;es
             */


            if (!OPTICAL_SENSOR.LongReadInput(true))
            {
                sommetDosDane = true;
            }
            else if (OPTICAL_SENSOR.LongReadInput(false) && sommetDosDane)
            {
                CURRENT_STATES_ID = DECENTE_AVEC_LANCEUR_STATE_ID;
                active = false;
                delay(20);
            }
            else
            {
                active = true;
            }
        };

        void Execute() {
            MOTORS.Speed(MOTOR_LEFT, 0.35);
            MOTORS.Speed(MOTOR_RIGHT, 0.35);
        };

        const bool & IsActive() { return active; };

    private:
        unsigned long _FirstExecuteTime;
        bool active;
        bool sommetDosDane;
    };

    struct DecenteAvecLanceur {

        DecenteAvecLanceur() : active(false), _FirstExecuteTime(0) {};

        void Update()
        {
            if (!OPTICAL_SENSOR.LongReadInput(true))
            {
                CURRENT_STATES_ID = VIRAGE_ENTREE_ZONE_LANCEMENT_STATE_ID;
                active = false;
                delay(20);
            }
            else
            {
                active = true;
            }
        };

        void Execute()
        {
            MOTORS.Speed(MOTOR_LEFT, -0.34);
            MOTORS.Speed(MOTOR_RIGHT, -0.34);
        };

        const bool & IsActive() { return active; };

    private:
        unsigned long _FirstExecuteTime;
        bool active;
    };
}



namespace States {

    struct DeploiementLimitSwitchs {

        DeploiementLimitSwitchs() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct VerificationDeclenchementLimitSwitchs {

        VerificationDeclenchementLimitSwitchs() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct SequenceCalculRotationCible {

        SequenceCalculRotationCible() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct SequenceLancementPoches {

        SequenceLancementPoches() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct SeparationLanceur {

        SeparationLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct VirageSortieZoneLancement {

        VirageSortieZoneLancement() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}

namespace States {

    struct ParcoursSansLanceur {

        ParcoursSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct MonteeSansLanceur {

        MonteeSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };

    struct DecenteSansLanceur {

        DecenteSansLanceur() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}


namespace States {

    struct VirageEntreeZoneLancement {

        VirageEntreeZoneLancement() : active(false), firstPass(true),
            switchNextState(false) {};

        void Update()
        {
            if (switchNextState)
            {
                CURRENT_STATES_ID = ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT_STATE_ID;
                active = false;
            }
            else
            {
                active = true;
            }
        };

        void Execute()
        {
            static bool firstExecute = true;
            static bool turnTriggered = false;
            static unsigned long turnTime = 0;

            /*
                    26 DEC - 23:57 -> la roue de droite a pogner le rebors alors certain event non pas peut faire se qu'il devait faire a cause de ca ex: reculer avec la roue de droit
                    27 DEC - 00:04 -> la dernier step de RunSpeedScript n'a pas etait pris en compte et l'avant dernier step est rester indefiniment et les dernier step a etait ignorer
                    27 DEC - 00:09 -> la roue de gauche a pogner le rebors alors certain event non pas pu faire se qu'il devait faire
             */


            static const unsigned long startTime        = millis();
            static const unsigned int nbStep            = 7;
            static const float leftSpeeds[nbStep]       = {
                    0.0, 0.0,
                    -0.13, 0.0,
                    0.0, 0.0,
                    0.0
            };

            static const float rightSpeeds[nbStep]      = {
                    0.0, 0.0,
                    -0.13, 0.0,
                    0.35, 0.0,
                    0.0
            };

            static const unsigned long timing[nbStep]   = {
                    500, 1500,
                    2500, 3500,
                    5500, 6500,
                    6501
            };
        /*
            static const unsigned long startTime        = millis();
            static const unsigned int nbStep            = 19;
            static const float leftSpeeds[nbStep]       = {
                    0.0,    0.0,
                    -0.14,  0.0,
                    0.22,   0.0,
                    -0.45,  0.0,
                    -0.80,  0.0,
                    0.0,    0.0,
                    -0.60,  0.0,
                    0.0,    0.0,
                    0.35,   0.0,
                    0.0
             };                  };
         
            static const float rightSpeeds[nbStep]      = {
                    0.0,    0.0,
                    -0.14,  0.0,
                    0.22,   0.0,
                    0.55,   0.0,
                    0.20,   0.0,
                    0.70,   0.0,
                    0.0,    0.0,
                    0.50,   0.0,
                    0.35,   0.0,
                    0.0
            };

            static const unsigned long timing[nbStep]   = {
                    500,    1500,
                    2500,   3500,
                    3650,   4650,
                    4900,   5900,
                    6000,   7000,
                    7100,   8100,
                    8300,   9300,
                    9450,   10450,
                    10950,  11950,
                    11952
            };
        */

            MOTORS.RunSpeedScript(nbStep, leftSpeeds, rightSpeeds, timing, startTime);

        };

        const bool & IsActive() { return active; };

    private:
        bool switchNextState;
        bool active;
        bool firstPass;
        unsigned long time;

    };

    struct AlignmentReedSwitchZoneLancement {

        AlignmentReedSwitchZoneLancement() {};

        void Update() {

            active = true;
        };

        void Execute() {

            MOTORS.Speed(MOTOR_LEFT, 0.00);
            MOTORS.Speed(MOTOR_RIGHT, 0.00);
        };

        const bool & IsActive() { return active; };

    private:
        bool active;

    };

    struct DeploiementCanon {

        DeploiementCanon() {};

        void Update() {

        };

        void Execute() {

        };

        const bool & IsActive() { return active; };

    private:
        bool active;
    };
}



unsigned short CURRENT_STATES_ID = States::PARCOURS_AVEC_LANCEUR_STATE_ID;

namespace States {

    ChargementBatterie CHARGEMENT_BATTERIE = ChargementBatterie();
    NouvellePocheDansMagasin NOUVELLE_POCHE_DANS_MAGASIN = NouvellePocheDansMagasin();

    ParcoursAvecLanceur PARCOURS_AVEC_LANCEUR = ParcoursAvecLanceur();
    MonteeAvecLanceur MONTEE_AVEC_LANCER = MonteeAvecLanceur();
    DecenteAvecLanceur DECENTE_AVEC_LANCER = DecenteAvecLanceur();

    VirageEntreeZoneLancement VIRAGE_ENTREE_ZONE_LANCEMENT = VirageEntreeZoneLancement();
    AlignmentReedSwitchZoneLancement ALIGNEMENT_REEDSWITCH_ZONE_LANCEMENT = AlignmentReedSwitchZoneLancement();
    DeploiementCanon DEPLOIEMENT_CANON = DeploiementCanon();

    DeploiementLimitSwitchs DEPLOIEMENT_LIMITSWITCHS = DeploiementLimitSwitchs();
    VerificationDeclenchementLimitSwitchs VERIFICATION_DECLENCHEMENT_LIMITSWITCHS = VerificationDeclenchementLimitSwitchs();

    SequenceCalculRotationCible SEQUENCE_CALCUL_ROTATION_CIBLE = SequenceCalculRotationCible();
    SequenceLancementPoches SEQUENCE_LANCEMENT_POCHES = SequenceLancementPoches();

    SeparationLanceur SEPARATION_LANCEUR = SeparationLanceur();
    VirageSortieZoneLancement VIRAGE_SORTIE_ZONE_LANCEMENT = VirageSortieZoneLancement();

    ParcoursSansLanceur PARCOURS_SANS_LANCEUR = ParcoursSansLanceur();
    MonteeSansLanceur MONTEE_SANS_LANCEUR = MonteeSansLanceur();
    DecenteSansLanceur DECENTE_SANS_LANCEUR = DecenteSansLanceur();
};

