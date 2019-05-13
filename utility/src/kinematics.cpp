#include "../include/kinematics.h"


Kinematics::Kinematics(int type)
{
    if(type == 5)
    {
        ardp_.d1 =  0.122;
        ardp_.d2 =  0.1215;
        ardp_.d5 =  0.1025;
        ardp_.d6 =  0.094;
        ardp_.a2 =  0.408;
        ardp_.a3 =  0.376;
    }
}

void Kinematics::homogeneousTransformation(RMatrix& T, double alpha, double a, double d, double theta)
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

//index ==0; output J0; index = 1,output Jn
//matrix A transfer ds in flange to ds in base;
//void Kinematics::getJacobian(RMatrix& J, RMatrix& A, RVector q)
//{
//    RMatrix JJ2(3,6);
//    RVector z1(3),z2(3),z3(3),z4(3),z5(3),z6(3),z7(3),o1(3),o2(3),o3(3),o4(3),o5(3),o6(3),on(3),o7(3),J1(3),J2(3),J3(3),J4(3),J5(3),J6(3),J7(3);
//    RMatrix T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4), T67(4,4);
//    RMatrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),T6(4,4),T7(4,4),Tn(4,4);

//    homogeneousTransformation(T01, 0, 0, ardp_.d1, q.value[0] + M_PI);
//    homogeneousTransformation(T12, -M_PI / 2, 0, ardp_.d2, q.value[1] - M_PI / 2);
//    homogeneousTransformation(T23, M_PI, ardp_.a2, 0, q.value[2]);
//    homogeneousTransformation(T34, M_PI, ardp_.a3, 0, q.value[3] - M_PI / 2);
//    homogeneousTransformation(T45, -M_PI / 2, 0, ardp_.d5, q.value[4]);
//    homogeneousTransformation(T56, M_PI / 2, 0, ardp_.d6, q.value[5]);


//    //    for(int i = 0; i < 3; i++)
//    //    {
//    //        for(int j = 0; j < 3; j++)
//    //        {
//    //            T67.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
//    //        }
//    //        T67.value[i][3] = RobotControl::m_toolPosition[i];
//    //    }
//    //    T67.value[3][3] = 1;

//    //    T67.value[0][1] = 1;
//    //    T67.value[1][0] = -1;
//    //    T67.value[2][2] = 1;
//    //    T67.value[3][3] = 1;


//    //    R = RMatrix.RotZ(M_PI/4);
//    //    T67 << 0, -1, 0, 0,
//    //           1, 0, 0, 0,
//    //           0, 0, 1, 0.07,
//    //           0, 0, 0, 1;

//    T1 = T01;
//    T2 = T1 * T12;
//    T3 = T2 * T23;
//    T4 = T3 * T34;
//    T5 = T4 * T45;
//    T6 = T5 * T56;
//    //    T7 = T6 * T67;

//    z1 = RMatrix::subRVector(T1,0,2,2,"Column");
//    z2 = RMatrix::subRVector(T2,0,2,2,"Column");
//    z3 = RMatrix::subRVector(T3,0,2,2,"Column");
//    z4 = RMatrix::subRVector(T4,0,2,2,"Column");
//    z5 = RMatrix::subRVector(T5,0,2,2,"Column");
//    z6 = RMatrix::subRVector(T6,0,2,2,"Column");

//    o1 = RMatrix::subRVector(T1,0,2,3,"Column");
//    o2 = RMatrix::subRVector(T2,0,2,3,"Column");
//    o3 = RMatrix::subRVector(T3,0,2,3,"Column");
//    o4 = RMatrix::subRVector(T4,0,2,3,"Column");
//    o5 = RMatrix::subRVector(T5,0,2,3,"Column");
//    o6 = RMatrix::subRVector(T6,0,2,3,"Column");
//    //    o7 = RMatrix::subRVector(T7,0,2,3,"Column");
//    on = o6;
//    Tn = T6;


//    J1 = RVector::cross3(z1, on - o1);
//    J2 = RVector::cross3(z2, on - o2);
//    J3 = RVector::cross3(z3, on - o3);
//    J4 = RVector::cross3(z4, on - o4);
//    J5 = RVector::cross3(z5, on - o5);
//    J6 = RVector::cross3(z6, on - o6);
//    //    J1 = z1.cross(o7 - o1);
//    //    J2 = z2.cross(o7 - o2);
//    //    J3 = z3.cross(o7 - o3);
//    //    J4 = z4.cross(o7 - o4);
//    //    J5 = z5.cross(o7 - o5);
//    //    J6 = z6.cross(o7 - o6);

//    //MatrixXd C(A.rows(), A.cols()+B.cols());
//    //C << A, B;
//    RMatrix::catRMatrix(J,0,2,0,J1);
//    RMatrix::catRMatrix(J,0,2,1,J2);
//    RMatrix::catRMatrix(J,0,2,2,J3);
//    RMatrix::catRMatrix(J,0,2,3,J4);
//    RMatrix::catRMatrix(J,0,2,4,J5);
//    RMatrix::catRMatrix(J,0,2,5,J6);

//    RMatrix::catRMatrix(JJ2,0,2,0,z1);
//    RMatrix::catRMatrix(JJ2,0,2,1,z2);
//    RMatrix::catRMatrix(JJ2,0,2,2,z3);
//    RMatrix::catRMatrix(JJ2,0,2,3,z4);
//    RMatrix::catRMatrix(JJ2,0,2,4,z5);
//    RMatrix::catRMatrix(JJ2,0,2,5,z6);

//    RMatrix::catRMatrix(J,3,5,0,5,JJ2);

//    RMatrix Rn(3,3),S(3,3), B(3,3);
//    Rn = RMatrix::subRMatrix(Tn,0,2,0,2);
//    S =  RVector::Skew(on);

//    RMatrix C = S*Rn;
//    RMatrix::catRMatrix(A,0,2,0,2,Rn);
//    RMatrix::catRMatrix(A,0,2,3,5,C);
//    RMatrix::catRMatrix(A,3,5,0,2,B);
//    RMatrix::catRMatrix(A,3,5,3,5,Rn);


//    //    for(int i = 0; i < 6; i++)
//    //        for(int j = 0; j < 6; j++)
//    //        {
//    //            JJ.value[i][j] = J(i,j);
//    //            AA.value[i][j] = A(i,j);
//    //        }


//    //    std::cout << J0 << std::endl<< J1 << std::endl;
//    if(index == 1)
//    {
//        RMatrix Rn0(3,3), Trans(6,6), Jn(6,6);

//        Rn0 = RMatrix::RTranspose(Rn);
//        RMatrix::catRMatrix(Trans,0,2,0,2,Rn0);
//        RMatrix::catRMatrix(Trans,0,2,3,5,B);
//        RMatrix::catRMatrix(Trans,3,5,0,2,B);
//        RMatrix::catRMatrix(Trans,3,5,3,5,Rn0);
//        Jn = Trans*J;
//        J = Jn;
//    }
//}
//obtain the jacobian of flange;
//index ==0; output J0; index = 1,output Jn
void Kinematics::getJacobian(RMatrix& J, RMatrix& T6, RMatrix& JJ2, RVector q, int index)
{
    RVector z1(3),z2(3),z3(3),z4(3),z5(3),z6(3),o1(3),o2(3),o3(3),o4(3),o5(3),o6(3),on(3),J1(3),J2(3),J3(3),J4(3),J5(3),J6(3);
    RMatrix T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4);
    RMatrix T1(4,4),T2(4,4),T3(4,4),T4(4,4),T5(4,4),Tn(4,4), zeros_m(3,3);

    homogeneousTransformation(T01, 0, 0, ardp_.d1, q.value[0] + M_PI);
    homogeneousTransformation(T12, -M_PI / 2, 0, ardp_.d2, q.value[1] - M_PI / 2);
    homogeneousTransformation(T23, M_PI, ardp_.a2, 0, q.value[2]);
    homogeneousTransformation(T34, M_PI, ardp_.a3, 0, q.value[3] - M_PI / 2);
    homogeneousTransformation(T45, -M_PI / 2, 0, ardp_.d5, q.value[4]);
    homogeneousTransformation(T56, M_PI / 2, 0, ardp_.d6, q.value[5]);

    T1 = T01;
    T2 = T1 * T12;
    T3 = T2 * T23;
    T4 = T3 * T34;
    T5 = T4 * T45;
    T6 = T5 * T56;

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

    on = o6;
    Tn = T6;


    J1 = RVector::cross3(z1, on - o1);
    J2 = RVector::cross3(z2, on - o2);
    J3 = RVector::cross3(z3, on - o3);
    J4 = RVector::cross3(z4, on - o4);
    J5 = RVector::cross3(z5, on - o5);
    J6 = RVector::cross3(z6, on - o6);

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

    if(index == 1)
    {
        RMatrix Rn0(3,3), Trans(6,6), Jn(6,6), Rn(3,3);
        Rn = RMatrix::subRMatrix(T6,0,2,0,2);
        Rn0 = RMatrix::RTranspose(Rn);
        RMatrix::catRMatrix(Trans,0,2,0,2,Rn0);
        RMatrix::catRMatrix(Trans,0,2,3,5,zeros_m);
        RMatrix::catRMatrix(Trans,3,5,0,2,zeros_m);
        RMatrix::catRMatrix(Trans,3,5,3,5,Rn0);
        Jn = Trans*J;
        J = Jn;
    }
}
// getJacobian of Tool,index ==0; output J0 of tool; index = 1,output Jn of tool
//void Kinematics::getJacobianofTool(RMatrix& Jtool,RMatrix& A, RVector q, int index)
//{
//    RMatrix J6(6,6), Jadd(6,6), T6(4,4), JJ2(3,6);//tool in flange T67
//    RVector tool_position(3),zi(3), Jaddi(3), v_zero(3);
//    RMatrix R0_6(3,3);

//    getJacobian(J6, T6, JJ2, q, 0);
//    R0_6 = RMatrix::subRMatrix(T6,0,2,0,2);

//    for(int i = 0; i < 3; i++)
//    {
//        tool_position.value[i] = m_toolPosition[i];
//    }
//    for(int i = 0; i < 5; i++)
//    {
//        zi = RMatrix::subRVector(JJ2,0,2,i,"Column");
//        Jaddi = Skew(zi) * R0_6 * tool_position;
//        RMatrix::catRMatrix(Jadd,0,2,i,Jaddi);
//        RMatrix::catRMatrix(Jadd,3,5,i,v_zero);
//    }

//    Jtool = J6 + Jadd;

//    //

//    RMatrix T0_7(4,4), T_tool(4,4), Rn(3,3), Rn0(3,3), m_zeros(3,3), sk(3,3), C(3,3);
//    RVector on(3);
//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            T_tool.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
//        }
//        T_tool.value[i][3] = RobotControl::m_toolPosition[i];
//    }
//    T_tool.value[3][3] = 1;

//    T0_7 = T6*T_tool;

//    Rn = RMatrix::subRMatrix(T0_7,0,2,0,2);
//    on = RMatrix::subRVector(T0_7,0,2,3,"Column");
//    sk =  RVector::Skew(on);
//    Rn0 = RMatrix::RTranspose(Rn);

//    C = sk*Rn;
//    RMatrix::catRMatrix(A,0,2,0,2,Rn);
//    RMatrix::catRMatrix(A,0,2,3,5,C);
//    RMatrix::catRMatrix(A,3,5,0,2,m_zeros);
//    RMatrix::catRMatrix(A,3,5,3,5,Rn);

//    if(index == 1)
//    {
//        RMatrix Trans(6,6), Jn(6,6);
//        RMatrix::catRMatrix(Trans,0,2,0,2,Rn0);
//        RMatrix::catRMatrix(Trans,0,2,3,5,m_zeros);
//        RMatrix::catRMatrix(Trans,3,5,0,2,m_zeros);
//        RMatrix::catRMatrix(Trans,3,5,3,5,Rn0);
//        Jn = Trans*Jtool;
//        Jtool = Jtooln;
//    }
//}


//void Kinematics::obtainConstraintForce(RVector q0, RVector& F_constraint, RMatrix& Jt_inv_current)
//{
//    bool enable_constraints = 1;
//    RMatrix Jt_inv(6,3);// Jt_inv_current(6,3);
//    RVector Aq(3), F_v(3);
//    double wth = 0.025, wcr = 0.01, lamda = 100;
//    double kw = 0;

//    double w_q0 = getPerformanceIndex(q0, Jt_inv_current);
//    if(w_q0 > wth)
//    {
//        kw = 0;
//    }
//    else
//    {
//        kw = lamda*(1.0/(w_q0 - wcr) + 1.0/(wth - wcr));

//        for(int i = 0; i < 3; i++)
//        {

//            double v_unit = 1, dt = 0.005, dw = 0;
//            int direct_index = 0;
//            double direct[] = {1.0,-1.0}, dwt[2];
//            for(int j = 0; j < 2; j++)
//            {
//                RVector x_dot(3), q_next(6);
//                x_dot.value[i]= v_unit*direct[j];
//                q_next = q0 + Jt_inv_current * x_dot * dt;

//                double w_next = getPerformanceIndex(q_next, Jt_inv);
//                if(w_next > w_q0)
//                {
//                    dwt[j] =  w_next - w_q0;
//                }
//                else
//                    dwt[j]=0;
//            }
//            if(dwt[0] > dwt[1])
//            {
//                dw = dwt[0];
//                direct_index = 0;
//            }
//            else
//            {
//                dw = dwt[1];
//                direct_index = 1;
//            }
//            Aq.value[i] = dw*direct[direct_index];
//        }
//    }
//    if (enable_constraints)
//    {
//        F_constraint = Aq * kw  ;
//    }
//    //    for(int i = 0; i <3; i++)
//    //    {
//    //        F_constraint.[i] = F_v.value[i];
//    //    }
//}

//double Kinematics::getPerformanceIndex(RVector q, RMatrix& Jt_inv, RMatrix& Jt_)
//{
//    RMatrix Jn(6,6), ones(6,6);
//    RMatrix Jt(3,6);
//    RMatrix Jr(3,6), Jr_inv(6,6);
//    double wqt = 0;

//    getJacobianofTool(Jn, q, 1);
//    Jt = RMatrix::subRMatrix(Jn,0,2,0,5);
//    Jr = RMatrix::subRMatrix(Jn,3,5,0,5);
//    //obtain Right inverse of Jr;
//    Jr_inv = obtainRightInverse(Jr);
//    //
//    Jt_ = Jt * (ones - Jr_inv * Jr);
//    Jt_inv = obtainRightInverse(Jt_);

//    wqt = RMatrix::normRMatrix(Jt_,6);

//    return wqt;
//}

//bool Kinematics::getAngleVelocity(double* q, double *dq)
//{
//    double Md = 2, Cd = 50, Ts = 0.005;
//    RVector F_constraint(3), x_dot(3), q_prev(6), v_prev(3), q_dot(6);
//    RMatrix Jt_inv_current(6,3);

//    for(int i = 0; i < 6; i++)
//    {
//        q_prev.value[i] = q[i];
//    }

//    obtainConstraintForce(q_prev, F_constraint, Jt_inv_current);

//    double T = 1/(Md/Ts + Cd);
//    x_dot = T *(v_prev * Md/Ts + force_of_end_ + F_constraint);

//    q_dot = Jt_inv_current * x_dot;
//    //    q_next = q0 + q_dot * dt;

//    v_prev = x_dot;

//    for(int i = 0; i < 6; i++)
//    {
//        dq[i] = q_dot.value[i];
//    }
//}

//bool Kinematics::getAngleVelocity(const double* ds, double* q, double *dq)
//{
//    bool flag;
//    RVector dDis(6), qq(6), dDelta(6);
//    RMatrix J(6,6), A(6,6), T(6,6);

//    for(int i = 0; i<1; i++)
//    {
//        for(int i = 0; i < 6; i++)
//        {
//            dDis.value[i] = ds[i];              //the increment in sensor coordinate
//            qq.value[i] = q[i];
//        }
//        getJacobianofTool(J, A, qq, 0);

//        flag = RMatrix::RMatrixInv(J, T);

//        //        std::cout<<"1:"<<T.value[0][0]<<","<<T.value[0][1]<<","<<T.value[0][2]<<","<<T.value[0][3]<<","<<T.value[0][4]<<","<<T.value[0][5]<<std::endl;
//        //        std::cout<<"2:"<<T.value[1][0]<<","<<T.value[1][1]<<","<<T.value[1][2]<<","<<T.value[1][3]<<","<<T.value[1][4]<<","<<T.value[1][5]<<std::endl;
//        //        std::cout<<"3:"<<T.value[2][0]<<","<<T.value[2][1]<<","<<T.value[2][2]<<","<<T.value[2][3]<<","<<T.value[2][4]<<","<<T.value[2][5]<<std::endl;
//        //        std::cout<<"4:"<<T.value[3][0]<<","<<T.value[3][1]<<","<<T.value[3][2]<<","<<T.value[3][3]<<","<<T.value[3][4]<<","<<T.value[3][5]<<std::endl;
//        //        std::cout<<"5:"<<T.value[4][0]<<","<<T.value[4][1]<<","<<T.value[4][2]<<","<<T.value[4][3]<<","<<T.value[4][4]<<","<<T.value[4][5]<<std::endl;
//        //        std::cout<<"6:"<<T.value[5][0]<<","<<T.value[5][1]<<","<<T.value[5][2]<<","<<T.value[5][3]<<","<<T.value[5][4]<<","<<T.value[5][5]<<std::endl;
//        //        int a = 10;
//    }

//    if(flag/*!isnan(dDelta.value[0]) && !isnan(dDelta.value[1]) && !isnan(dDelta.value[2]) && !isnan(dDelta.value[3]) && !isnan(dDelta.value[4]) && !isnan(dDelta.value[5])*/)
//    {
//        RMatrix J_A = T * A;
//        dDelta = J_A * dDis; //the increment of joint angle
//        memcpy(dq, dDelta.value, 6*sizeof(double));
//        return true;
//    }
//    else
//    {
//        dq[0] = 0;dq[1] = 0;dq[2] = 0;dq[3] = 0;dq[4] = 0;dq[5] = 0;
//        return false;
//    }
//}

RMatrix Kinematics::obtainRightInverse(RMatrix J)
{
    bool flag;
    RMatrix J_trans(6,3), Jt(6,6), Jt_inv(3,3), J_inv(6,3);
    J_trans = RMatrix::RTranspose(J);
    Jt = J*J_trans;
    flag = RMatrix::RMatrixInv(Jt ,Jt_inv);
    J_inv = J_trans * Jt_inv ;

    return J_inv;
}
