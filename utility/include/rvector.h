#ifndef RVECTOR_H
#define RVECTOR_H

#include<math.h>
#include<iomanip>
#include "rmatrix.h"

class RVector
{
public:
    RVector();
    RVector(size_t m);
    RVector(double m[], size_t n);

    ~RVector(void);

    friend RVector operator-(const RVector &a, const RVector &b);
    friend RVector operator-(const RVector &a);
    friend RVector operator+(const RVector &a, const RVector &b);
    friend RVector operator*(const RVector &a, const double b);
    friend RVector operator*(double b, const RVector &a);
    friend RVector operator/(const RVector &a, double b);
    friend RVector operator/(double b, const RVector &a);
    //
    static double norm(RVector d);
    static RVector vex3(RMatrix A);
    static RMatrix Skew(RVector S);
    static RVector cross3(RVector a, RVector b);

    static RVector catRVector(RVector &a, RVector b);
    static void processAngle(RVector &a);
    static RVector diagRVector(RMatrix &A);
    static double dotRVector(RVector &a, RVector &b);
    static RVector TR2RPY(RMatrix RotN);
    static bool Ori2Quaternion(RMatrix rot, RVector &c);

    static RVector sqrtRVector(RVector a);
    //
//    std::vector<double> value;
    size_t size;
    double value[7];
//    string type;
    bool IsAmpty;


};

#endif // RVECTOR_H
