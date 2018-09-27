#ifndef DEFINELOCALISATIONCONSTS_H
#define DEFINELOCALISATIONCONSTS_H

const static double POSTE_UNCERTAINTY = 0.15; // uncertainty radius for poste position estimates
const static double PDIST_UNCERTAINTY = 0.01; //
const static double A_VALUE_ACCURACY = 1;    //number of a values needed for computing the real value
const static double PUCK_UNCERTAINTY  = 0.25; // uncertainty radius for _GLOBAL_ puck position estimates
const static bool DEBUG = true;
const static double PI = 3.14159265359;
const static double FINAL_CORRECTION = 0.25;    //robot would place the robot slightly off set from the final goal; this is a correction factor
const static double FIELD_TOLERANCE = 0.4;

/* DEFINE THE RELATIONS FOR INTER-POLE DISTANCE PROPORTIONS */
const static double D2_D1 = 3.0;
const static double D3_D2 = 2.0/3.0;
const static double D4_D3 = 1.0;
const static double D5_D4 = 3.0/2.0;
const static double D6_D5 = 1.0/3.0;

const static double a_true = 0.0;           // fill in on demonstation day
const static double b_true = 0.0;           // fill in on demonstation day



#endif // DEFINELOCALISATIONCONSTS_H
