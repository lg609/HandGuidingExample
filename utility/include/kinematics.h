#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>


#include "../include/rmatrix.h"
#include "../include/rvector.h"

#define i5_PARAMS
#define MIN -175    //随机数产生的范围
#define MAX 175
#define MAXIUM_ITERATOR 10

typedef struct AUBO_ROBOT_DH_Parameters {
    double d1;
    double d2;
    double d5;
    double d6;
    double a2;
    double a3;
}ARDP;


const double ZERO_THRESH = 1e-5;
const double PI = M_PI;


//int SIGN(double x)
//{
//    return (x > 0) - (x < 0);
//}


class Kinematics
{
public:
    Kinematics(int type);
    ~Kinematics();
    void homogeneousTransformation(RMatrix& T, double alpha, double a, double d, double theta);
    void getJacobian(RMatrix& J, RMatrix& T6, RMatrix& JJ2, RVector q, int index);
    //void getJacobianofTool(RMatrix& Jtool,RMatrix& A, RVector q, int index);
//    void obtainConstraintForce(RVector q0, RVector& F_constraint, RMatrix& Jt_inv_current);
//    double getPerformanceIndex(RVector q, RMatrix& Jt_inv, RMatrix& Jt_);

//    bool getAngleVelocity(double* q, double *dq);
//    bool getAngleVelocity(const double* ds, double* q, double *dq);

    RMatrix obtainRightInverse(RMatrix J);
   private:
   ARDP ardp_;
};

#endif // KINEMATICS_H
