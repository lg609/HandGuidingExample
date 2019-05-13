#include "robotcontrol.h"

int RobotControl::s_control_period = 5;//unit s
bool RobotControl::s_start_handguiding = false;
bool RobotControl::s_thread_handguiding = true;
double RobotControl::s_accumulate_time = 0.0;
double RobotControl::s_tool_pose[CARTESIAN_FREEDOM] = {0};           //TOOL POSE relative to the robot end
double RobotControl::m_toolPosition[3] = {0};
double RobotControl::m_toolOrientation[9] = {0};
double RobotControl::s_threshold[SENSOR_DIMENSION] = {0};
double RobotControl::s_limit[SENSOR_DIMENSION] = {0};
double RobotControl::force_of_end_[CARTESIAN_FREEDOM] = {0};
double RobotControl::tool_mass = 0.0;
double RobotControl::center_mass[3] = {0};

double RobotControl::s_pos_wth = 0.025;
double RobotControl::s_pos_wcr = 0.01;
double RobotControl::s_pos_lambda = 100;

double RobotControl::s_ori_wth = 0.025;
double RobotControl::s_ori_wcr = 0.015;
double RobotControl::s_ori_lambda = 100;

bool RobotControl::enable_constraints = false;

static RobotControl *s_instance = 0;



double pose1_jointangle[6] = {-15,15,75,60,80,0};
double pose2_jointangle[6] = {-36,27,95,-27,0,0};
double pose3_jointangle[6] = {-36,21,100,-6,90,0};

RobotControl *RobotControl::instance()
{
    return s_instance;
}

RobotControl::RobotControl():tcp2canMode_(false)
{
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        joint_max_acc_.jointPara[i] = MAX_ACCELERATION;
        joint_max_velc_.jointPara[i] = MAX_VELOCITY;
    }
    kine_ = new Kinematics(5);
}

RobotControl::~RobotControl()
{
    if(tcp2canMode_)
        robotServiceSend.robotServiceLeaveTcp2CanbusMode();
    //log out
    robotServiceSend.robotServiceRobotShutdown();
    robotServiceSend.robotServiceLogout();
}
bool RobotControl::updateRobotStatus()
{
    /** Get the intial value of theoretical waypoint**/
    int ret = robotServiceReceive.robotServiceGetCurrentWaypointInfo(theoretical_way_point_);

    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        average_sensor_data_[i] = 0;
        last_acclerations_[i] = 0;
        current_acclerations_[i] = 0;
        current_velocities_[i] = 0.0;
        last_velocities_[i] = 0.0;
        relative_position_[i] = 0.0;
        last_send_joints_[i] = theoretical_way_point_.jointpos[i];
        //penult_send_joints_[i] = last_send_joints_[i];
    }

    //first ensure the right teach mode
    enterTcp2CANMode(false);

    double R1[9];
    Util::quaternionToOriMatrix(theoretical_way_point_.orientation, R1);
    Util::hMatrixMultiply(R1, theoretical_way_point_.cartPos.positionVector, m_toolOrientation, m_toolPosition, &initial_way_point_);

    //set the tool's parameters
    setToolProperty();
    return !(bool)ret;
}

bool RobotControl::initRobotService()
{
    bool flag = false;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** login  ***/
    ret = robotServiceSend.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    ret = robotServiceReceive.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cout<<"login sucess."<<std::endl;
    else
    {
        std::cerr<<"login failed."<<std::endl;
        flag = false;
    }

    /** Initialize the real robot　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //tool dynamical parameters
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotServiceSend.rootServiceRobotStartup(toolDynamicsParam/**tool dynamical parameters**/,
                                                   0        /*collision class*/,
                                                   true     /*Allow read pose, true*/,
                                                   true,    /*Default true */
                                                   1000,    /*defaault 1000 */
                                                   result); /*initial result*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        std::cout<<"initial sucess."<<std::endl;
    else
        std::cerr<<"initial failed."<<std::endl;

    /** Initialize move profile ***/
    ret = robotServiceSend.robotServiceInitGlobalMoveProfile();
    /** Set max ACC ***/
    ret = robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(joint_max_acc_);
    /** Set max Velocity ***/
    ret = robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(joint_max_velc_);
    flag = updateRobotStatus();

    return flag;
}

void RobotControl::startHandGuiding()
{
    QTime current_time;
    int ret, msec1 = 0, msec2 = 0, addSize, rib_buffer_size_ = 0;
    double ts;
    float R[SENSOR_DIMENSION], Q[SENSOR_DIMENSION];
    if(FTSensorDataProcess::s_controlModel == CONTROL_MODE::ACCLERATION)
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.00241, 0.0023, 0.0077, 0.0090, 0.0020, 0.00241}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-7,1e-7,1e-7}, sizeof(float)*SENSOR_DIMENSION);
    }
    else
    {
        memcpy(R, new float[SENSOR_DIMENSION]{0.0241, 0.023, 0.077, 0.090, 0.090, 0.0241}, sizeof(float)*SENSOR_DIMENSION);
        memcpy(Q, new float[SENSOR_DIMENSION]{1e-5,1e-5,1e-5,1e-5,1e-5,1e-5}, sizeof(float)*SENSOR_DIMENSION);
    }
    float P[] = {1,1,1,1,1,1};
    float pp[6] = {0};
    float K[6] = {0};
    aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
    std::vector<aubo_robot_namespace::wayPoint_S> wayPointVector;
    //initial the move profile
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        last_acclerations_[i] = 0;
        current_acclerations_[i] = 0;
        current_velocities_[i] = 0.0;
        last_velocities_[i] = 0.0;
        relative_position_[i] = 0.0;
    }
    s_accumulate_time = 0.0;


    while(s_thread_handguiding)
    {
        current_time = QTime::currentTime();
        msec1 = current_time.msec();
        if(s_start_handguiding)
        {
            ret = robotServiceReceive.robotServiceGetRobotDiagnosisInfo(robot_diagnosis_info_);  //Get the current way point, but there is a delay.
            rib_buffer_size_ = robot_diagnosis_info_.macTargetPosDataSize;
            addSize = (FTSensorDataProcess::s_bufferSizeLimit - rib_buffer_size_) / aubo_robot_namespace::ARM_DOF;
            if(addSize > 0)
            {
                wayPointVector.clear();
                aubo_robot_namespace::wayPoint_S wp;
                addSize = 2;
                for(int bufferCount = 1; bufferCount <= addSize; bufferCount++)
                {
                    externalForceOnToolEnd(last_send_joints_);     //obtain the force of end;
                    double dd[] = {0,1,0,0,0,0};
                    memcpy(force_of_end_, dd, sizeof(double)*6);
                    if (enable_constraints)
                    {
                        getAngleVelocity(last_send_joints_);
                    }
                    //use the model to get the pose increament
                    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
                    {
                        //First-order lag filtering
                        average_sensor_data_[i] = force_of_end_[i] * 0.4 + average_sensor_data_[i] * 0.6;

                        if(FTSensorDataProcess::s_controlModel == CONTROL_MODE::ACCLERATION)
                        {
                            current_acclerations_[i] = 1.0/FTSensorDataProcess::s_sensitivity[i] * (average_sensor_data_[i] - FTSensorDataProcess::s_damp[i] * last_velocities_[i] - FTSensorDataProcess::s_stiffness[i] * relative_position_[i]);   //a = k*force   Newton
                            current_velocities_[i] = last_velocities_[i] +  s_control_period * (current_acclerations_[i] + last_acclerations_[i]) / 2000.0;
                        }
                        else
                            //                            current_velocities_[i] = FTSensorDataProcess::s_sensitivity[i] * average_sensor_data_[i];
                            current_velocities_[i] = (average_sensor_data_[i] + CONTROL_CYCLE * FTSensorDataProcess::s_sensitivity[i] * last_velocities_[i]) / (FTSensorDataProcess::s_sensitivity[i] * CONTROL_CYCLE + FTSensorDataProcess::s_damp[i]);
                    }
                    std::cout<<"current_velocities_  "<<current_velocities_[0]<<";"<<current_velocities_[1]<<";"<<current_velocities_[2]<<";"<<current_velocities_[3]<<";"<<current_velocities_[4]<<";"<<current_velocities_[5]<<";"<<std::endl;

                    for(int i = 0; i < 3; i++)
                    {
                        relative_pose_.relativePosition[i] = s_control_period * (last_velocities_[i] + current_velocities_[i]) / 2000.0;
                        relative_axis_angles_[i] = s_control_period * (last_velocities_[i+3] + current_velocities_[i+3]) / 2000.0;
                        relative_position_[i] += relative_pose_.relativePosition[i];
                        relative_position_[i+3] += relative_axis_angles_[i];

                    }

                    //                     std::cout<<"relative_position_"<<relative_position_[0]<<","<<relative_position_[1]<<","<<relative_position_[2];

                    if(!calculateTheoreticalWaypoint())
                    {
                        std::cout<<"calculation error!";
                        emit signal_handduiding_failed("calculation error!");
                    }
                    if(Util::checkForSafe(theoretical_way_point_.jointpos, last_send_joints_, aubo_robot_namespace::ARM_DOF))
                    {
                        for(int ks = 0; ks < aubo_robot_namespace::ARM_DOF; ks++)
                        {
                            wp.jointpos[ks] = theoretical_way_point_.jointpos[ks];
                            if(FTSensorDataProcess::s_filter2 != 0)
                            {
                                pp[ks] = P[ks] + Q[ks];
                                K[ks] = pp[ks] / (pp[ks] + R[ks]);
                                wp.jointpos[ks] = last_send_joints_[ks] + K[ks] * (wp.jointpos[ks] - last_send_joints_[ks]);
                                P[ks] = (1 - K[ks]) * pp[ks];
                            }
                            last_send_joints_[ks] = wp.jointpos[ks];// the theoretical joint after calman filter;
                        }
                        wayPointVector.push_back(wp);
                        //                        std::cout<<theoretical_way_point_.jointpos[0]<<","<<theoretical_way_point_.jointpos[1]<<","<<theoretical_way_point_.jointpos[2]<<","<<theoretical_way_point_.jointpos[3]<<","<<theoretical_way_point_.jointpos[4]<<","<<theoretical_way_point_.jointpos[5]<<","<<
                        //                                                                                                                                                                 last_send_joints_[0]<<","<<last_send_joints_[1]<<","<<last_send_joints_[2]<<","<<last_send_joints_[3]<<","<<last_send_joints_[4]<<","<<last_send_joints_[5];
                        //update the last state
                        for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
                        {
                            last_acclerations_[i] = current_acclerations_[i];
                            last_velocities_[i] = current_velocities_[i];

                            theoretical_way_point_.jointpos[i] = last_send_joints_[i]; //update the command joint positions
                            intermediate_way_point_.jointpos[i] = theoretical_way_point_.jointpos[i];
                        }
                    }
                    else
                    {
                        std::cout<<"robot OverSpeed!";
                        emit signal_handduiding_failed("robot OverSpeed!");
                    }
                }
                ret = robotServiceSend.robotServiceSetRobotPosData2Canbus(wayPointVector);    //
                //                for(int i = 0; i < wayPointVector.size(); i++)
                //  l                  qDebug()<<i<<" "<<wayPointVector[i].jointpos[0]<<","<<wayPointVector[i].jointpos[1]<<","<<wayPointVector[i].jointpos[2]<<","<<wayPointVector[i].jointpos[3]<<","<<wayPointVector[i].jointpos[4]<<","<<wayPointVector[i].jointpos[5];
                if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
                {
                    std::cout<<"Set data error;"<<ret;
                    emit signal_handduiding_failed("TCP2CAN error");
                }
            }
            else
            {
                //            continue;
            }
        }

        current_time =QTime::currentTime();
        msec2 = current_time.msec();
        ts = msec2 - msec1;
        if(ts < 0) ts += 1000;
        s_accumulate_time += ts;
        if(s_accumulate_time < s_control_period && s_accumulate_time >= 0)
        {
            usleep((s_control_period - s_accumulate_time) * 1000);         //guarantee a stable feed
        }
        s_accumulate_time -= s_control_period;
        if(s_accumulate_time < 0) s_accumulate_time = 0;
    }
}

bool RobotControl::calculateTheoreticalWaypoint()
{
    double relTran[9], R1[9], R2[9], relPos[3];
    aubo_robot_namespace::wayPoint_S temp_way_point;
    if(FTSensorDataProcess::s_calculateMethod == CALCULATE_METHOD::JACOBIAN)
    {
        double dq[6] = {0, 0, 0, 0, 0, 0};
        double ds[] = {relative_pose_.relativePosition[0], relative_pose_.relativePosition[1], relative_pose_.relativePosition[2],
                       relative_axis_angles_[0], relative_axis_angles_[1], relative_axis_angles_[2]};
        if(FTSensorDataProcess::s_dragMode == DRAG_MODE::POSITION)
        {
            ds[3] = 0;
            ds[4] = 0;
            ds[5] = 0;
        }
        else if(FTSensorDataProcess::s_dragMode == DRAG_MODE::ORI)
        {
            ds[0] = 0;
            ds[1] = 0;
            ds[2] = 0;
        }
        bool flag = getAngleVelocity(ds, last_send_joints_, dq);
        //bool flag = Util::getAngleVelocity(ds, current_way_point_.jointpos, dq);

        if(!flag)
        {
            std::cout<<"nan"<<last_send_joints_[0]<<","<<last_send_joints_[1]<<","<<last_send_joints_[2]<<","<<last_send_joints_[3]<<","<<last_send_joints_[4]<<","<<last_send_joints_[5]<<","
                    <<ds[0]<<","<<ds[1]<<","<<ds[2]<<","<<ds[3]<<","<<ds[4]<<","<<ds[5];
            return false;
        }

        //        std::cout<<"dqqq"<<dq[0]<<","<<dq[1]<<","<<dq[2]<<","<<dq[3]<<","<<dq[4]<<","<<dq[5];
        for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
        {
            theoretical_way_point_.jointpos[i] = last_send_joints_[i] + dq[i];
            //theoretical_way_point_.jointpos[i] = 0.7 * theoretical_way_point_.jointpos[i] + 0.2 * last_send_joints_[i] + 0.1 * penult_send_joints_[i];
            //theoretical_way_point_.jointpos[i] = current_way_point_.jointpos[i] + dq[i];
        }
    }
    else
    {
        relative_pose_.ena = true;
        intermediate_way_point_ = theoretical_way_point_;
        double m_wNorm = sqrt(relative_axis_angles_[0] * relative_axis_angles_[0] + relative_axis_angles_[1] * relative_axis_angles_[1] + relative_axis_angles_[2] * relative_axis_angles_[2]);
        if(m_wNorm == 0 || FTSensorDataProcess::s_dragMode == DRAG_MODE::POSITION)
        {
            relative_pose_.relativeOri.w = 1;
            relative_pose_.relativeOri.x = 0;
            relative_pose_.relativeOri.y = 0;
            relative_pose_.relativeOri.z = 0;
            relTran[0] = 1.0;relTran[1] = 0.0;relTran[2] = 0.0;
            relTran[3] = 0.0;relTran[4] = 1.0;relTran[5] = 0.0;
            relTran[6] = 0.0;relTran[7] = 0.0;relTran[8] = 1.0;
        }
        else
        {
            if(FTSensorDataProcess::s_dragMode == DRAG_MODE::ORI)
            {
                for(int i = 0; i < 3; i++)
                    relative_pose_.relativePosition[i] = 0;
            }

            for(int i = 0; i < 3; i++)
            {
                relative_axis_angles_[i] = relative_axis_angles_[i] / m_wNorm;  //normalize w
            }
            Util::angleAxisToTran(m_wNorm, relative_axis_angles_, relTran);
            if(Util::tranToQuaternion(relTran, ori))
            {
                relative_pose_.relativeOri.w = ori.w;
                relative_pose_.relativeOri.x = ori.x;
                relative_pose_.relativeOri.y = ori.y;
                relative_pose_.relativeOri.z = ori.z;
            }
            else
            {
                std::cout<<"Orientation computation error!";
                return false;
            }
        }

        for (int i = 0; i < 3; i++)
            relPos[i] = relative_pose_.relativePosition[i];
        Util::hMatrixMultiply(m_toolOrientation, m_toolPosition, relTran, relPos, &temp_way_point);

        Util::quaternionToOriMatrix(temp_way_point.orientation, R2);
        Util::quaternionToOriMatrix(intermediate_way_point_.orientation, R1);
        Util::hMatrixMultiply(R1, intermediate_way_point_.cartPos.positionVector, R2, temp_way_point.cartPos.positionVector, &intermediate_way_point_);

        //intermediate_way_point_ tool_end_pose_ describled in base coordinate
        //start
        aubo_robot_namespace::Ori tcpOriInEnd, tcpOri, endOri;
        tcpOri.w = intermediate_way_point_.orientation.w;
        tcpOri.x = intermediate_way_point_.orientation.x;
        tcpOri.y = intermediate_way_point_.orientation.y;
        tcpOri.z = intermediate_way_point_.orientation.z;

        tcpOriInEnd.w = userCoord.toolDesc.toolInEndOrientation.w;
        tcpOriInEnd.x = userCoord.toolDesc.toolInEndOrientation.x;
        tcpOriInEnd.y = userCoord.toolDesc.toolInEndOrientation.y;
        tcpOriInEnd.z = userCoord.toolDesc.toolInEndOrientation.z;

        robotServiceSend.toolOrientation2EndOrientation(tcpOriInEnd, tcpOri, endOri);

        intermediate_way_point_.orientation.w = endOri.w;
        intermediate_way_point_.orientation.x = endOri.x;
        intermediate_way_point_.orientation.y = endOri.y;
        intermediate_way_point_.orientation.z = endOri.z;

        Util::quaternionToOriMatrix(intermediate_way_point_.orientation, R1);
        double toolRelMove[3] = {0};

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                //incre pos: R * incTrans
                if (j==0)
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.x;
                else if (j==1)
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.y;
                else
                    toolRelMove[i] += R1[i*3+j] * userCoord.toolDesc.toolInEndPosition.z;
            }
        }
        for (int i=0;i<3;i++)
            intermediate_way_point_.cartPos.positionVector[i] -= toolRelMove[i];
        //end
        int ret = robotServiceReceive.robotServiceRobotIk(intermediate_way_point_.jointpos, intermediate_way_point_.cartPos.position, intermediate_way_point_.orientation, theoretical_way_point_); //less than 1 ms
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cout<<"robot IK error;"<<ret;
            return false;
        }
    }
    return true;
}

void RobotControl::setToolProperty()
{
    userCoord.coordType = aubo_robot_namespace::EndCoordinate;
    userCoord.toolDesc.toolInEndPosition.x = s_tool_pose[0];//s_tool_pose input from ui;
    userCoord.toolDesc.toolInEndPosition.y = s_tool_pose[1];
    userCoord.toolDesc.toolInEndPosition.z = s_tool_pose[2];
    m_toolPosition[0] = userCoord.toolDesc.toolInEndPosition.x;
    m_toolPosition[1] = userCoord.toolDesc.toolInEndPosition.y;
    m_toolPosition[2] = userCoord.toolDesc.toolInEndPosition.z;

    //    s_tool_pose[5] = -M_PI/2;

    double ori[4];
    Util::EulerAngleToQuaternion(new double[3]{s_tool_pose[3],s_tool_pose[4],s_tool_pose[5]}, ori);
    userCoord.toolDesc.toolInEndOrientation.w = ori[0];
    userCoord.toolDesc.toolInEndOrientation.x = ori[1];
    userCoord.toolDesc.toolInEndOrientation.y = ori[2];
    userCoord.toolDesc.toolInEndOrientation.z = ori[3];
    Util::quaternionToOriMatrix(userCoord.toolDesc.toolInEndOrientation, m_toolOrientation);
}

int RobotControl::enterTcp2CANMode(bool flag)
{
    int ret;
    if(flag)
    {
        // enter Tcp2Canbus Mode
        if(updateRobotStatus())
        {
            ret = robotServiceSend.robotServiceEnterTcp2CanbusMode();
            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
                std::cout<<"Enter TCP2CAN mode failed!";
            else
            {
                tcp2canMode_ = true;
                s_start_handguiding = true;
            }
        }
        else
            emit signal_handduiding_failed("update robot status failed.");
    }
    else
    {
        //        if(tcp2canMode_)
        ret = robotServiceSend.robotServiceLeaveTcp2CanbusMode();
        s_start_handguiding = false;
        memset(force_of_end_, 0, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    }
    return ret;
}
//ObtainCenterofMass in sensor coordinate
bool RobotControl::ObtainCenterofMass()
{
    //obtain pose1,2,3
    //move to pose 1,,wait for sensor stable ,,store the joint angle and sensor data;;
    //move to pose 2,,,,,,,
    //    s_tool_pose[3] = 0;
    //    s_tool_pose[4] = 0;
    //    s_tool_pose[5] = -M_PI/2;
    //    setToolProperty();
    //        float pose1_sensordata[SENSOR_DIMENSION] = {-26.6504,23.0249,1.98346,0.781332,0.296835,-2.71919};
    //        float pose2_sensordata[SENSOR_DIMENSION] = {-31.6855,27.5977,1.96821,0.671184,0.170454,-2.71761};
    //        float pose3_sensordata[SENSOR_DIMENSION] = {-31.0513,23.029,4.62102,0.779261,0.189995,-2.7181};

    //    float pose1_jointangle[6] = {-15,15,75,60,50,0};
    //    float pose2_jointangle[6] = {-36,27,95,-27,10,0};
    //    float pose3_jointangle[6] = {-36,21,100,-6,90,0};

    //    float FTSensorDataProcess::calibrationMessurement_[POSE_NUM][SENSOR_DIMENSION] = {0};
    double pose_jointangle[POSE_NUM][6] = {0};
    float p_k[3] = {0};

    for(int i = 0; i < SENSOR_DIMENSION; i++)
    {
        //        FTSensorDataProcess::calibrationMessurement_[0][i] = pose1_sensordata[i];
        //        FTSensorDataProcess::calibrationMessurement_[1][i] = pose2_sensordata[i];
        //        FTSensorDataProcess::calibrationMessurement_[2][i] = pose3_sensordata[i];

        pose_jointangle[0][i] = pose1_jointangle[i]*M_PI/180;
        pose_jointangle[1][i] = pose2_jointangle[i]*M_PI/180;
        pose_jointangle[2][i] = pose3_jointangle[i]*M_PI/180;
    }


    RMatrix pose_f(9,6);
    RMatrix pose_md(POSE_NUM*3,1);
    RMatrix pose_fd(POSE_NUM*3,1);

    RMatrix pose_ft(6,POSE_NUM*3);
    RMatrix p_center(6,1);

    for(int i = 0; i < 3; i++)
    {
        pose_f.value[3*i][0] = 0;
        pose_f.value[3*i][1] = FTSensorDataProcess::calibrationMessurement_[i][2];
        pose_f.value[3*i][2] = -FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_f.value[3*i][3] = 1;

        pose_f.value[3*i+1][0] = -FTSensorDataProcess::calibrationMessurement_[i][2];
        pose_f.value[3*i+1][1] = 0;
        pose_f.value[3*i+1][2] = FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_f.value[3*i+1][4] = 1;

        pose_f.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_f.value[3*i+2][1] = -FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_f.value[3*i+2][2] = 0;
        pose_f.value[3*i+2][5] = 1;

        pose_md.value[3*i][0] = FTSensorDataProcess::calibrationMessurement_[i][3];
        pose_md.value[3*i+1][0] = FTSensorDataProcess::calibrationMessurement_[i][4];
        pose_md.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][5];

        pose_fd.value[3*i][0] = FTSensorDataProcess::calibrationMessurement_[i][0];
        pose_fd.value[3*i+1][0] = FTSensorDataProcess::calibrationMessurement_[i][1];
        pose_fd.value[3*i+2][0] = FTSensorDataProcess::calibrationMessurement_[i][2];
    }


    pose_ft = RMatrix::RTranspose(pose_f);

    RMatrix ftf_inv(6,6);

    RMatrix ftf = pose_ft * pose_f;

    RMatrix::RMatrixInv(ftf, ftf_inv);

    p_center = ftf_inv*pose_ft*pose_md;

    for(int i = 0; i < 3; i++)
    {
        center_mass[i] = p_center.value[i][0];
        p_k[i] = p_center.value[i+3][0];
    }


    RMatrix pose_RR(9,6);
    RMatrix pose_RR_trans(6,9);
    RMatrix l_offset(6,1);

    for(int k = 0; k < 3; k++)
    {
        double pose_jointangle1[6] = {0};
        for(int m = 0; m < 6; m++)
        {
            pose_jointangle1[m] = pose_jointangle[k][m];
        }

        RMatrix pose_sensortobase(3,3);
        pose_sensortobase = sensorToBase(pose_jointangle1);

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                pose_RR.value[i+3*k][j] = pose_sensortobase.value[j][i];//transpose
            }
        }

        pose_RR.value[0+3*k][3] = 1;
        pose_RR.value[1+3*k][4] = 1;
        pose_RR.value[2+3*k][5] = 1;

    }

    pose_RR_trans = RMatrix::RTranspose(pose_RR);

    RMatrix RR_inv(6,6);

    RMatrix::RMatrixInv(pose_RR_trans*pose_RR, RR_inv);

    l_offset = RR_inv*pose_RR_trans*pose_fd;

    //FTSensorDataProcess::s_sensor_offset[6] = [fx0,fy0,fz0,mx0,my0,mz0];
    //center_mass[3] = [cx,cy,cz]

    //    float FTSensorDataProcess::s_sensor_offset[6] = {0};
    float l_l[3] = {0};

    float base_angle_offset[2];

    for(int i = 0; i < 3; i++)
    {
        FTSensorDataProcess::s_sensor_offset[i] = l_offset.value[i+3][0];
        l_l[i] = l_offset.value[i][0];
    }

    FTSensorDataProcess::s_sensor_offset[3] = p_k[0] - FTSensorDataProcess::s_sensor_offset[1]*center_mass[2] + FTSensorDataProcess::s_sensor_offset[2]*center_mass[1];
    FTSensorDataProcess::s_sensor_offset[4] = p_k[1] - FTSensorDataProcess::s_sensor_offset[2]*center_mass[0] + FTSensorDataProcess::s_sensor_offset[0]*center_mass[2];
    FTSensorDataProcess::s_sensor_offset[5] = p_k[2] - FTSensorDataProcess::s_sensor_offset[0]*center_mass[1] + FTSensorDataProcess::s_sensor_offset[1]*center_mass[0];

    tool_mass = sqrt(l_l[0]*l_l[0] + l_l[1]*l_l[1] + l_l[2]*l_l[2]);//the gravity of tool in base;

    base_angle_offset[0] = asin(-l_l[1]/tool_mass);//U
    base_angle_offset[1] = atan(-l_l[0]/l_l[2]);//V

    std::cout<<"center of mass"<<center_mass[0]<<","<<center_mass[1]<<","<<center_mass[2]<<std::endl;
    std::cout<<"tool_gravity"<<tool_mass<<std::endl;
    std::cout<<"sensor_offset"<<FTSensorDataProcess::s_sensor_offset[0]<<","<<FTSensorDataProcess::s_sensor_offset[1]<<","<<FTSensorDataProcess::s_sensor_offset[2]
            <<","<<FTSensorDataProcess::s_sensor_offset[3]<<","<<FTSensorDataProcess::s_sensor_offset[4]<<","<<FTSensorDataProcess::s_sensor_offset[5]<<std::endl;
    //    std::flush;
    return true;
}

int RobotControl::moveToTargetPose(int index)
{
    //move to target pose
    robotServiceSend.robotServiceInitGlobalMoveProfile();
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc, jointMaxVelc;
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = 100.0/180.0*M_PI;
        jointMaxVelc.jointPara[i] = 50.0/180.0*M_PI;

    }
    robotServiceSend.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robotServiceSend.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
    double jointAngle[6];
    if(index == 1)
        memcpy(jointAngle, pose1_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    else if(index == 2)
        memcpy(jointAngle, pose2_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    else
        memcpy(jointAngle, pose3_jointangle, sizeof(double)*aubo_robot_namespace::ARM_DOF);
    for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++)
        jointAngle[i] = jointAngle[i] / 180 * M_PI;
    return robotServiceSend.robotServiceJointMove(jointAngle, true);  // Start to move to the selected pose,if success, return 0;
}
// subtract gravity of tool;


//input current joints ,G of tool in base;
//output the Component of force and torque in sensor coordinate due to the gravity in sensor coordinate;
void RobotControl::getGravityOfToolInSensor(double current_joints[])
{
    RMatrix current_sensortobase(3,3);
    RMatrix g_component(3,1);
    RMatrix g_base(3,1);//[0,0,-1]

    g_base.value[2][0] = -1;
    current_sensortobase = sensorToBase(current_joints);
    g_component = RMatrix::RTranspose(current_sensortobase)*g_base;
    for(int i = 0; i < 3; i++)
    {
        gravity_component[i] = g_component.value[i][0]*tool_mass;//GX GY GZ MGx MGy MGz IN SENSOR
    }
    gravity_component[3] = gravity_component[2]*center_mass[1] - gravity_component[1]*center_mass[2];
    gravity_component[4] = gravity_component[0]*center_mass[2] - gravity_component[2]*center_mass[0];
    gravity_component[5] = gravity_component[1]*center_mass[0] - gravity_component[0]*center_mass[1];
}

//input current joints ;output the ori matrix from sensor to base ;in RMatrix form;
RMatrix RobotControl::sensorToBase(double current_joints[])
{
    aubo_robot_namespace::wayPoint_S current_way_point;
    double current_flangetobase[9] = {0};

    //    std::cout<<"center current_joints mass"<<current_joints[0]<<","<<current_joints[1]<<","<<current_joints[2]<<","<<current_joints[3]<<","<<current_joints[4]<<","<<current_joints[5]<<std::endl;

    int ret = robotServiceReceive.robotServiceRobotFk(current_joints, 6, current_way_point);
    Util::quaternionToOriMatrix(current_way_point.orientation, current_flangetobase);

    RMatrix c_flg_base(3,3);
    RMatrix c_sensor_flg(3,3);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            c_flg_base.value[i][j] = current_flangetobase[3*i+j];
            c_sensor_flg.value[i][j] = m_toolOrientation[3*i+j];
        }
    }
    RMatrix current_sensor_base(3,3);
    current_sensor_base = c_flg_base * c_sensor_flg;

    return current_sensor_base;
}

void RobotControl::externalForceOnToolEnd(double current_joints[])
{
    //    for(int i = 0; i < 6; i++)
    //    {
    //        current_joints[i] = pose1_jointangle[i]*M_PI/180;//GX GY GZ MGx MGy MGz IN SENSOR
    //    }
    //raw_sensor_data_ == the measured sensor data substract the sensor offset;
    memcpy(raw_sensor_data_,  FTSensorDataProcess::s_sensor_data, sizeof(double)*SENSOR_DIMENSION);

    //subtract gravity component(forcr and torque) from sensor data ;
    getGravityOfToolInSensor(current_joints);
    //translate to end of tool
    for(int i = 0; i < SENSOR_DIMENSION; i++)
    {
        raw_sensor_data_[i] -= gravity_component[i];
        if(i < 3)
            force_of_end_[i] = raw_sensor_data_[i];
    }

    double toolendtosensor[3] = {0, 0, -0.135};      // force acting point;;same ori;offset z = -0.12
    force_of_end_[3] = raw_sensor_data_[2]*toolendtosensor[1] - raw_sensor_data_[1]*toolendtosensor[2] + raw_sensor_data_[3];
    force_of_end_[4] = raw_sensor_data_[0]*toolendtosensor[2] - raw_sensor_data_[2]*toolendtosensor[0] + raw_sensor_data_[4];
    force_of_end_[5] = raw_sensor_data_[1]*toolendtosensor[0] - raw_sensor_data_[0]*toolendtosensor[1] + raw_sensor_data_[5];
    //set the threshold value
    for(int i = 0; i < SENSOR_DIMENSION; i++)
    {
        //threshold + max limit
        if(abs(force_of_end_[i]) < s_threshold[i])
            force_of_end_[i] = 0;
        else if (force_of_end_[i] > 0)
        {
            force_of_end_[i] -= s_threshold[i];
            if(force_of_end_[i] > s_limit[i])
                force_of_end_[i] = s_limit[i];
        }
        else
        {
            force_of_end_[i] += s_threshold[i];
            if(force_of_end_[i] < -s_limit[i])
                force_of_end_[i] = -s_limit[i];
        }
    }
}
//verified;
void RobotControl::getJacobianofTool(RMatrix& Jtool,RMatrix& A, RVector q, int index)
{
    RMatrix J6(6,6), Jadd(6,6), T6(4,4), JJ2(3,6);//tool in flange T67
    RVector tool_position(3),zi(3), Jaddi(3), v_zero(3);
    RMatrix R0_6(3,3);

    kine_->getJacobian(J6, T6, JJ2, q, 0);
    R0_6 = RMatrix::subRMatrix(T6,0,2,0,2);

    for(int i = 0; i < 3; i++)
    {
        tool_position.value[i] = m_toolPosition[i];
    }
    for(int i = 0; i < 5; i++)
    {
        zi = RMatrix::subRVector(JJ2,0,2,i,"Column");
        Jaddi = RVector::Skew(zi) * R0_6 * tool_position;
        RMatrix::catRMatrix(Jadd,0,2,i,Jaddi);
        RMatrix::catRMatrix(Jadd,3,5,i,v_zero);
    }

    Jtool = J6 + Jadd;

    //

    RMatrix T0_7(4,4), T_tool(4,4), Rn(3,3), Rn0(3,3), m_zeros(3,3), sk(3,3), C(3,3);
    RVector on(3);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            T_tool.value[i][j] = RobotControl::m_toolOrientation[3*i+j];
        }
        T_tool.value[i][3] = RobotControl::m_toolPosition[i];
    }
    T_tool.value[3][3] = 1;

    T0_7 = T6*T_tool;

    Rn = RMatrix::subRMatrix(T0_7,0,2,0,2);
    on = RMatrix::subRVector(T0_7,0,2,3,"Column");
    sk =  RVector::Skew(on);
    Rn0 = RMatrix::RTranspose(Rn);

    C = sk*Rn;
    RMatrix::catRMatrix(A,0,2,0,2,Rn);
    RMatrix::catRMatrix(A,0,2,3,5,C);
    RMatrix::catRMatrix(A,3,5,0,2,m_zeros);
    RMatrix::catRMatrix(A,3,5,3,5,Rn);

    if(index == 1)
    {
        RMatrix Trans(6,6), Jn(6,6);
        RMatrix::catRMatrix(Trans,0,2,0,2,Rn0);
        RMatrix::catRMatrix(Trans,0,2,3,5,m_zeros);
        RMatrix::catRMatrix(Trans,3,5,0,2,m_zeros);
        RMatrix::catRMatrix(Trans,3,5,3,5,Rn0);
        Jn = Trans*Jtool;
        Jtool = Jn;
    }
}

void RobotControl::obtainConstraintForce(const RVector q0, RVector& F_constraint)
{
    RMatrix J_inv(6,3), J_inv_current(6,3);
    double wq_0[2] = {0}, wq_next[2] = {0};
    double kw = 0;

    getPerformanceIndex(q0, J_inv_current, wq_0);
    std::cout<<"wqt_0  "<<wq_0[0]<<";   "<<"wqr_0  "<<wq_0[1]<<std::endl;

    double s_wth = 0;
    double s_wcr = 0;
    double s_lambda = 0;

    for(int k = 0; k < 2; k++)
    {
        if(k == 0)
        {
            s_wth = s_pos_wth;
            s_wcr= s_pos_wcr;
            s_lambda = s_pos_lambda;
        }
        else
        {
            s_wth = s_ori_wth;
            s_wcr= s_ori_wcr;
            s_lambda = s_ori_lambda;
        }
        if(wq_0[k] > s_wth)
        {
            kw = 0;
        }
        else
        {
            kw = s_lambda * (1.0/(wq_0[k] - s_wcr) - 1.0/(s_wth - s_wcr));

            for(int i = 0; i < 3; i++)
            {
                double v_unit = 1, dt = 1.0 / CONTROL_CYCLE, dw = 0;
                int direct_index = 0;
                double direct[] = {1.0,-1.0}, dw_direct[2];
                for(int j = 0; j < 2; j++)
                {
                    RVector x_dot(3), q_next(6);
                    x_dot.value[i]= v_unit*direct[j];
                    q_next = q0 + J_inv_current * x_dot * dt;

                    getPerformanceIndex(q_next, J_inv, wq_next);
                    if(wq_next[k] > wq_0[k])
                    {
                        dw_direct[j] =  wq_next[k] - wq_0[k];
                    }
                    else
                        dw_direct[j]=0;
                }
                if(dw_direct[0] > dw_direct[1])
                {
                    dw = dw_direct[0];
                    direct_index = 0;
                }
                else
                {
                    dw = dw_direct[1];
                    direct_index = 1;
                }
                if (enable_constraints)
                {
                    F_constraint.value[2*k + i] = dw * direct[direct_index] * kw;
                }
            }
        }
    }

    std::cout<<"F_constraint  "<<F_constraint.value[0]<<","<<F_constraint.value[1]<<","<<F_constraint.value[2]<<","<<F_constraint.value[3]<<","<<F_constraint.value[4]<<","<<F_constraint.value[5]<<std::endl;
}

void RobotControl::getPerformanceIndex(const RVector q, RMatrix& J_inv, double wq[])
{
    RMatrix Jn(6,6), ones(6), A(6,6);
    RMatrix Jt(3,6), Jt_t(6,3), Jt_norm(3,3);
    RMatrix Jr(3,6), Jr_inv(6,3);
    double wqt = 0;
    getJacobianofTool(Jn, A, q, 1);
    Jt = RMatrix::subRMatrix(Jn,0,2,0,5);
    Jr = RMatrix::subRMatrix(Jn,3,5,0,5);
    //obtain Right inverse of Jr;
    Jr_inv = kine_->obtainRightInverse(Jr);
    //
    RMatrix Jt_ = Jt * (ones - Jr_inv * Jr);
    Jt_t = RMatrix::RTranspose(Jt_);
    Jt_norm = Jt_*Jt_t;
    wqt = sqrt(RMatrix::detRMatrix(Jt_norm));

    RMatrix Jt_inv = kine_->obtainRightInverse(Jt);
    RMatrix Jr_ = Jr * (ones - Jt_inv * Jt);
    RMatrix Jr_t = RMatrix::RTranspose(Jr_);
    RMatrix Jr_norm = Jr_*Jr_t;
    double wqr = sqrt(RMatrix::detRMatrix(Jr_norm));
    wq[0] = wqt;
    wq[1] = wqr;

    RMatrix::catRMatrix(J_inv,0,5,0,2,Jt_inv);
    RMatrix::catRMatrix(J_inv,0,5,3,5,Jr_inv);

}

bool RobotControl::getAngleVelocity(double* q)
{
    //    double Md = 2, Cd = 50, Ts = 0.005;
    //    RVector F_constraint(3), x_dot(3), q_prev(6), v_prev(3), q_dot(6), force_end(3);
    //    RMatrix Jt_inv_current(6,3);

    //    for(int i = 0; i < 6; i++)
    //    {
    //        q_prev.value[i] = q[i];
    //    }
    //    for(int i = 0; i < 3; i++)
    //    {
    //        force_end.value[i] = force_of_end_[i];
    //    }

    //    obtainConstraintForce(q_prev, F_constraint, Jt_inv_current);

    //    double T = 1/(Md/Ts + Cd);
    //    x_dot = (v_prev * Md/Ts + force_end + F_constraint) * T;

    //    q_dot = Jt_inv_current * x_dot;
    //    //    q_next = q0 + q_dot * dt;

    //    v_prev = x_dot;

    //    for(int i = 0; i < 6; i++)
    //    {
    //        dq[i] = q_dot.value[i];
    //    }
    RVector F_constraint(6), q_prev(6);

    memcpy(&q_prev.value[0], q, CARTESIAN_FREEDOM*sizeof(double));
    //    QTime current_time = QTime::currentTime();
    //    int msec1 = current_time.msec();
    obtainConstraintForce(q_prev, F_constraint);
    //    current_time = QTime::currentTime();
    //    int msec2 = current_time.msec();
    //    std::cout<<"timmmmmmmmmmmmmmmmmmmmmmmmmmmmm_"<<msec2-msec1<<std::endl;
    for(int i = 0; i < 6; i++)
    {
        force_of_end_[i] += F_constraint.value[i];
    }


}

bool RobotControl::getAngleVelocity(const double* ds, double* q, double *dq)
{
    bool flag;
    RVector dDis(6), qq(6), dDelta(6);
    RMatrix J(6,6), A(6,6), T(6,6);


    for(int i = 0; i < 6; i++)
    {
        dDis.value[i] = ds[i];              //the increment of end in end coordinate
        qq.value[i] = q[i];
    }
    getJacobianofTool(J, A, qq, 0);

    flag = RMatrix::RMatrixInv(J, T);//ds_b = A * ds_t; J*q_dot = A * ds_t;

    //        std::cout<<"1:"<<T.value[0][0]<<","<<T.value[0][1]<<","<<T.value[0][2]<<","<<T.value[0][3]<<","<<T.value[0][4]<<","<<T.value[0][5]<<std::endl;
    //        std::cout<<"2:"<<T.value[1][0]<<","<<T.value[1][1]<<","<<T.value[1][2]<<","<<T.value[1][3]<<","<<T.value[1][4]<<","<<T.value[1][5]<<std::endl;
    //        std::cout<<"3:"<<T.value[2][0]<<","<<T.value[2][1]<<","<<T.value[2][2]<<","<<T.value[2][3]<<","<<T.value[2][4]<<","<<T.value[2][5]<<std::endl;
    //        std::cout<<"4:"<<T.value[3][0]<<","<<T.value[3][1]<<","<<T.value[3][2]<<","<<T.value[3][3]<<","<<T.value[3][4]<<","<<T.value[3][5]<<std::endl;
    //        std::cout<<"5:"<<T.value[4][0]<<","<<T.value[4][1]<<","<<T.value[4][2]<<","<<T.value[4][3]<<","<<T.value[4][4]<<","<<T.value[4][5]<<std::endl;
    //        std::cout<<"6:"<<T.value[5][0]<<","<<T.value[5][1]<<","<<T.value[5][2]<<","<<T.value[5][3]<<","<<T.value[5][4]<<","<<T.value[5][5]<<std::endl;
    //        int a = 10;


    if(flag/*!isnan(dDelta.value[0]) && !isnan(dDelta.value[1]) && !isnan(dDelta.value[2]) && !isnan(dDelta.value[3]) && !isnan(dDelta.value[4]) && !isnan(dDelta.value[5])*/)
    {
        RMatrix J_A = T * A;
        dDelta = J_A * dDis; //the increment of joint angle
        memcpy(dq, &dDelta.value[0], 6*sizeof(double));

        std::cout<<"ds"<<ds[0]<<","<<ds[1]<<","<<ds[2]<<","<<ds[3]<<","<<ds[4]<<","<<ds[5]<<std::endl;
        std::cout<<"dq"<<dq[0]<<","<<dq[1]<<","<<dq[2]<<","<<dq[3]<<","<<dq[4]<<","<<dq[5]<<std::endl;
        return true;
    }
    else
    {
        dq[0] = 0;dq[1] = 0;dq[2] = 0;dq[3] = 0;dq[4] = 0;dq[5] = 0;
        return false;
    }


}



