#include "CarFuzzy.h"

namespace fuzzy
{
    namespace parallel
    {
       const uint8_t x_MFNum = 3;
       TrapezoidMF x_S(0, 0, 0.2347, 0.5);
       TriangleMF x_M(0.293, 0.554, 0.9882);
       TrapezoidMF x_L(0.5518, 0.978, 1.2, 1.2);
       MembershipFuction* x_MFs[x_MFNum] = { &x_S,&x_M,&x_L };
       float x_range[2] = { 0,1.2 };
       FuzzyVar x(x_MFs, x_MFNum, x_range);

       const uint8_t y_MFNum = 4;
       TrapezoidMF y_Z(0, 0, 0.3321, 0.3861);
       TriangleMF y_S(0.347, 0.39, 0.5319);
       TriangleMF y_M(0.397, 0.5116, 0.684);
       TrapezoidMF y_L(0.4677, 0.737, 0.8, 0.8);
       MembershipFuction* y_MFs[y_MFNum] = { &y_Z,&y_S,&y_M,&y_L };
       float y_range[2] = { 0,0.8 };
       FuzzyVar y(y_MFs, y_MFNum, y_range);

       const uint8_t theta_MFNum = 4;
       TrapezoidMF theta_N(-120, -120, -20, -10);
       TriangleMF theta_Z(-20, 0, 39.58);
       TriangleMF theta_PS(0, 31.97, 66.4);
       TrapezoidMF theta_PL(43.64, 70, 120, 120);
       MembershipFuction* theta_MFs[theta_MFNum] = { &theta_N,&theta_Z,&theta_PS,&theta_PL };
       float theta_range[2] = { -120,120 };
       FuzzyVar theta(theta_MFs, theta_MFNum, theta_range);

       const uint8_t phi_MFNum = 5;
       TriangleMF phi_NL(-25, -25, -19.98);
       TriangleMF phi_NS(-25, -19.3, -13.42);
       TriangleMF phi_Z(-16.67, 0, 16.67);
       TriangleMF phi_PS(11.11, 18.33, 23.89);
       TriangleMF phi_PL(20, 25, 25);
       MembershipFuction* phi_MFs[phi_MFNum] = { &phi_NL,&phi_NS,&phi_Z,&phi_PS,&phi_PL };
       float phi_range[2] = { -25,25 };
       FuzzyVar phi(phi_MFs, phi_MFNum, phi_range);

       FuzzyVar* inputVars[3] = { &x,&y,&theta };

       uint8_t rules[x_MFNum][y_MFNum][theta_MFNum] =
       {
           {{0,4,4,4},{0,0,0,4},{0,0,0,4},{0,0,0,4}},
           {{0,4,4,4},{0,0,0,3},{0,0,0,2},{0,0,0,2}},
           {{2,4,4,4},{0,2,3,4},{0,1,2,3},{0,0,1,2}}
       };

       Fuzzy3InputsController CarFuzzy(inputVars, phi, &rules[0][0][0]);
   }

   namespace perpendicular
   {
       const uint8_t x_MFNum = 3;
       TrapezoidMF x_S(0, 0, 0.03171, 0.318);
       TriangleMF x_M(0.06469, 0.422, 0.9);
       TrapezoidMF x_L(0.6127, 0.989, 1.2, 1.2);
       MembershipFuction* x_MFs[x_MFNum] = { &x_S,&x_M,&x_L };
       float x_range[2] = { 0,1.2 };
       FuzzyVar x(x_MFs, x_MFNum, x_range);

       const uint8_t y_MFNum = 2;
       TrapezoidMF y_S(0, 0, 0.284, 0.5402);
       TrapezoidMF y_L(0.3259, 0.729, 1, 1);
       MembershipFuction* y_MFs[y_MFNum] = { &y_S,&y_L };
       float y_range[2] = { 0, 1 };
       FuzzyVar y(y_MFs, y_MFNum, y_range);

       const uint8_t theta_MFNum = 5;
       TrapezoidMF theta_N(-120, -120, -20, 0);
       TriangleMF theta_Z(-20, 0, 48.71);
       TriangleMF theta_PS(23.89, 61.9, 90);
       TriangleMF theta_PM(75, 83.21, 91);
       TriangleMF theta_PL(90, 120, 120);
       MembershipFuction* theta_MFs[theta_MFNum] = { &theta_N,&theta_Z,&theta_PS,&theta_PM,&theta_PL };
       float theta_range[2] = { -120,120 };
       FuzzyVar theta(theta_MFs, theta_MFNum, theta_range);

       const uint8_t phi_MFNum = 4;
       TriangleMF phi_NL(-25, -25, -20);
       TriangleMF phi_NS(-25, -15.9, -6.453);
       TriangleMF phi_Z(-16.67, 0, 16.76);
       TriangleMF phi_P(20, 25, 25);
       MembershipFuction* phi_MFs[phi_MFNum] = { &phi_NL,&phi_NS,&phi_Z,&phi_P };
       float phi_range[2] = { -25,25 };
       FuzzyVar phi(phi_MFs, phi_MFNum, phi_range);

       FuzzyVar* inputVars[3] = { &x,&y,&theta };

       uint8_t rules[x_MFNum][y_MFNum][theta_MFNum] =
       {
           {{0,0,0,1,3},{0,0,0,1,3}},
           {{3,3,3,3,3},{0,0,0,3,3}},
           {{3,3,3,3,3},{1,2,3,3,3}}
       };

       Fuzzy3InputsController CarFuzzy(inputVars, phi, &rules[0][0][0]);
   }
}
