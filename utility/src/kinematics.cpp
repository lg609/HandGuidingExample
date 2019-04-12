#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>


#include "../include/rmatrix.h"
#include "../include/rvector.h"
#include "../include/robotcontrol.h"


#define i5_PARAMS
#define MIN -175    //随机数产生的范围
#define MAX 175
#define MAXIUM_ITERATOR 10


namespace
{
const double ZERO_THRESH = 1e-5;
const double PI = M_PI;
#ifdef i5_PARAMS
const double d1 =  0.122;
const double d2 =  0.1215;
const double d5 =  0.1025;
const double d6 =  0.094;
const double a2 =  0.408;
const double a3 =  0.376;
#endif

int SIGN(double x)
{
    return (x > 0) - (x < 0);
}
}

// input radian

// output radian//MatrixXd q_sols(6,8);

void transfer(RMatrix& T, double alpha, double a, double d, double theta)
{
    //modified DH
    T.value[0][0] = cos(theta);
    T.value[0][1] = -sin(theta);
    T.value[0][2] = 0;
    T.value[0][3] = a;

    T.value[1][0] = sin(theta)*cos(alpha);
    T.value[1][1] = cos(theta)*cos(alpha);
    T.value[1][2] = -sin(alpha);
    T.value[1][3] = -sin(alpha)*d;

    T.value[2][0] = sin(theta)*sin(alpha);
    T.value[2][1] = cos(theta)*sin(alpha);
    T.value[2][2] = cos(alpha);
    T.value[2][3] = cos(alpha)*d;

    T.value[3][0] = 0;
    T.value[3][1] = 0;
    T.value[3][2] = 0;
    T.value[3][3] = 1;
    //            T <<    cos(theta),            -sin(theta),           0,           a,
    //            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
    //            sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d,
    //            0,                     0,                     0,           1;
}


void GetJacobian(RMatrix& J, RMatrix& A, RVector q)
{
    RMatrix JJ2(3,6);
    RVector z1(3),z2(3),z3(3),z4(3),z5(3),z6(3),z7(3),o1(3),o2(3),o3(3),o4(3),o5(3),o6(3),o7(3),J1(3),J2(3),J3(3),J4(3),J5(3),J6(3),J7(3);
    RMatrix T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4), T67(4,4);
    RMatrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),T6(4,4),T7(4,4);


    transfer(T01, 0, 0, d1, q.value[0] + M_PI);
    transfer(T12, -M_PI / 2, 0, d2, q.value[1] - M_PI / 2);
    transfer(T23, M_PI, a2, 0, q.value[2]);
    transfer(T34, M_PI, a3, 0, q.value[3] - M_PI / 2);
    transfer(T45, -M_PI / 2, 0, d5, q.value[4]);
    transfer(T56, M_PI / 2, 0, d6, q.value[5]);


    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            T67.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
        }
        T67.value[i][3] = RobotControl::m_toolPosition[i];
    }
    T67.value[3][3] = 1;

    //    R = RMatrix.RotZ(M_PI/4);
    //    T67 << 0, -1, 0, 0,
    //           1, 0, 0, 0,
    //           0, 0, 1, 0.07,
    //           0, 0, 0, 1;

    T1 = T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;
    T5 = T4 * T45;
    T6 = T5 * T56;
    T7 = T6 * T67;

    z1 = RMatrix::subRVector(T1,0,2,2,"Column");
    z2 = RMatrix::subRVector(T2,0,2,2,"Column");
    z3 = RMatrix::subRVector(T3,0,2,2,"Column");
    z4 = RMatrix::subRVector(T4,0,2,2,"Column");
    z5 = RMatrix::subRVector(T5,0,2,2,"Column");
    z6 = RMatrix::subRVector(T6,0,2,2,"Column");

    o1 = RMatrix::subRVector(T1,0,2,3,"Column");
    o2 = RMatrix::subRVector(T2,0,2,3,"Column");
    o3 = RMatrix::subRVector(T3,0,2,3,"Column");
    o4 = RMatrix::subRVector(T4,0,2,3,"Column");
    o5 = RMatrix::subRVector(T5,0,2,3,"Column");
    o6 = RMatrix::subRVector(T6,0,2,3,"Column");
    o7 = RMatrix::subRVector(T6,0,2,3,"Column");



    J1 = RVector::cross3(z1, o7 - o1);
    J2 = RVector::cross3(z2, o7 - o2);
    J3 = RVector::cross3(z3, o7 - o3);
    J4 = RVector::cross3(z4, o7 - o4);
    J5 = RVector::cross3(z5, o7 - o5);
    J6 = RVector::cross3(z6, o7 - o6);
    //    J1 = z1.cross(o7 - o1);
    //    J2 = z2.cross(o7 - o2);
    //    J3 = z3.cross(o7 - o3);
    //    J4 = z4.cross(o7 - o4);
    //    J5 = z5.cross(o7 - o5);
    //    J6 = z6.cross(o7 - o6);

    //MatrixXd C(A.rows(), A.cols()+B.cols());
    //C << A, B;
    RMatrix::catRMatrix(J,0,2,0,J1);
    RMatrix::catRMatrix(J,0,2,1,J2);
    RMatrix::catRMatrix(J,0,2,2,J3);
    RMatrix::catRMatrix(J,0,2,3,J4);
    RMatrix::catRMatrix(J,0,2,4,J5);
    RMatrix::catRMatrix(J,0,2,5,J6);

    RMatrix::catRMatrix(JJ2,0,2,0,z1);
    RMatrix::catRMatrix(JJ2,0,2,1,z2);
    RMatrix::catRMatrix(JJ2,0,2,2,z3);
    RMatrix::catRMatrix(JJ2,0,2,3,z4);
    RMatrix::catRMatrix(JJ2,0,2,4,z5);
    RMatrix::catRMatrix(JJ2,0,2,5,z6);

    RMatrix::catRMatrix(J,3,5,0,5,JJ2);

    RMatrix RT(3,3), R(3,3),S(3,3), B(3,3);
    R = RMatrix::subRMatrix(T7,0,2,0,2);
    RT = RMatrix::RTranspose(R);
    S =  RVector::Skew(o7);

    RMatrix C = B-RT*S;
    RMatrix::catRMatrix(A,0,2,0,2,RT);
    RMatrix::catRMatrix(A,0,2,3,5,C);
    RMatrix::catRMatrix(A,3,5,0,2,B);
    RMatrix::catRMatrix(A,3,5,3,5,RT);


    //    for(int i = 0; i < 6; i++)
    //        for(int j = 0; j < 6; j++)
    //        {
    //            JJ.value[i][j] = J(i,j);
    //            AA.value[i][j] = A(i,j);
    //        }


    //    std::cout << J0 << std::endl<< J1 << std::endl;
}


