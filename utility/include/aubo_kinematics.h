#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "aubo_definition.h"

namespace AUBO {

class Kinematics
{
public:
    Kinematics(int type);
    ~Kinematics();
      void getJacobian(RMatrix& J, RMatrix& T6, RMatrix& JJ2, RVector q, int index);
      void setToolProperty();
    //void getJacobianofTool(RMatrix& Jtool,RMatrix& A, RVector q, int index);
//    void obtainConstraintForce(RVector q0, RVector& F_constraint, RMatrix& Jt_inv_current);
//    double getPerformanceIndex(RVector q, RMatrix& Jt_inv, RMatrix& Jt_);

//    bool getAngleVelocity(double* q, double *dq);
//    bool getAngleVelocity(const double* ds, double* q, double *dq);

    RMatrix obtainRightInverse(RMatrix J);
   private:
   ARDP ardp_;
};

}

#endif // KINEMATICS_H
