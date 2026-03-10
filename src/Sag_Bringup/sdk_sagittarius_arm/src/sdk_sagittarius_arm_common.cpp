/*
 *  Copyright (c) 2023, NXROBO Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common.h>

#include <cstdio>
#include <cstring>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h> 
namespace sdk_sagittarius_arm
{

    CSDarmCommon::CSDarmCommon()
    {
        /*Initialize receive buffer*/
        memset(mRecvBuffer, RECV_BUFFER_SIZE, 0);
        mDataLength = 0;



    //    motionPlan_pub = mNodeHandler.advertise<sensor_msgs::JointState>("sagittarius_joint_states",10);
    }



    int CSDarmCommon::StopArmLock()
    {
        int result = 0;

        return result;
    }

    bool CSDarmCommon::RebootDevice()
    {
        return true;
    }

    CSDarmCommon::~CSDarmCommon()
    {

        printf("sdk_sagittarius_arm drvier exiting.\n");
    }

    int CSDarmCommon::Init()
    {
        int result = InitDevice();
        if(0 != result)
        {
            printf("Failed to init device\n");
            return result;
        }

        result = InitArm();
        if(0 != result)
        {
            printf("Failed to init arm\n");
        }

        return result;
    }

    unsigned char CSDarmCommon::CheckSum(unsigned char *buf)
    {
        int i;
        unsigned char sum = 0;
        for(i=0; i<buf[2]; i++)
        {
            sum += buf[3+i];
        }
        return sum;
    }

    int CSDarmCommon::InitArm()
    {
        SendArmLockOrFree(1);
        return ExitSuccess;
    }
#define DEBUG 1
    void CSDarmCommon::print_hex(unsigned char *buf, int len)
    {
    #if DEBUG
        int i;
        for(i=0; i<len; i++)
        {
            printf("%02x ",buf[i]);
        }
        printf("\n");
    #endif
    }

    // void CSDarmCommon::StartReceiveSerail()
    // {
    //     if(mThrcv == NULL)
    //         mThrcv = new boost::thread(boost::bind(&CSDarmCommon::LoopRcv, this));
    // }
    // void CSDarmCommon::LoopRcv()
    // {
    //     while(ros::ok())
    //     {
    //         LoopOnce();
    //     }

    // }

    /// @brief  ROS Publishers JointState
    // void CSDarmCommon::PublishJointStates(unsigned char *buf)
    // {
    //     sensor_msgs::JointState 		joint_state;
    //     struct  timeval    tv;
    //     struct  timezone   tz;
    //     gettimeofday(&tv,&tz);
    //     static long long ct,lt;
    //     joint_state.header.stamp = ros::Time::now();
    //     joint_state.name.resize(8);
    //     joint_state.position.resize(8);
    //     joint_state.name[0] = "joint1";
    //     joint_state.position[0] = (short(buf[0]|buf[1]<<8))/1800.0*PI;
    //     joint_state.name[1] = "joint2";
    //     joint_state.position[1] = (short(buf[2]|buf[3]<<8))/1800.0*PI;
    //     joint_state.name[2] = "joint3";
    //     joint_state.position[2] = (short(buf[4]|buf[5]<<8))/1800.0*PI;         
    //     joint_state.name[3] = "joint4";
    //     joint_state.position[3] = (short(buf[6]|buf[7]<<8))/1800.0*PI;
    //     joint_state.name[4] = "joint5";
    //     joint_state.position[4] = (short(buf[8]|buf[9]<<8))/1800.0*PI;
    //     joint_state.name[5] = "joint6";
    //     joint_state.position[5] = (short(buf[10]|buf[11]<<8))/1800.0*PI;
    //     joint_state.name[6] = "joint_gripper_left";
    //     joint_state.position[6] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
    //     joint_state.name[7] = "joint_gripper_right";
    //     joint_state.position[7] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
    //     //	printf("[[[[%f,%f]]]]\n",joint_state.position[6],joint_state.position[7]);
    //     /*	joint_state.name[6] = "nx07";
    //                 joint_state.position[6] = 0;*/
    //     ct = tv.tv_sec*800000+tv.tv_usec;
    //     if(1)//ct>=lt)
    //     {
    //         //ROS_WARN("ARM->ROS:[%f,%f,%f,%f,%f,%f]", joint_state.position[0],joint_state.position[1],joint_state.position[2],joint_state.position[3],joint_state.position[4],joint_state.position[5]);
    //         joint_states = joint_state;
    //         motionPlan_pub.publish(joint_state);
    //         lt = ct+100000;
    //     }
    // }




    int CSDarmCommon::LoopOnce()
    {

        int dataLength = 0;
        int result = GetDataGram(mRecvBuffer, RECV_BUFFER_SIZE, &dataLength);
        if(result != ExitSuccess)
        {
            /*ROS_ERROR("sdk_sagittarius_arm - Read Error when getting datagram: %d", result);
            mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                                   "sdk_sagittarius_arm - Read Error when getting datagram.");*/
            return ExitError;
        }
        else
        {
            if(dataLength>0)
            {
                if((dataLength<255)&&(mDataLength<255))
                {
                    //print_hex(mRecvBuffer, dataLength);
                    memcpy(mFrameBuffer+mDataLength, mRecvBuffer, dataLength);
                    mDataLength = mDataLength+dataLength;
                    if(mFrameBuffer[0] != 0x55)
                        mDataLength = 0;
                }
                else
                    mDataLength = 0;
                if((mDataLength>3)&&(mDataLength >= (mFrameBuffer[2]+5))) //检测包的大小
                {
                    if((mFrameBuffer[0]==0x55)&&(mFrameBuffer[1]==0xAA)&&(mFrameBuffer[mFrameBuffer[2]+4]==0x7D)&&(CheckSum(mFrameBuffer)==(unsigned char)mFrameBuffer[mFrameBuffer[2]+3]))  //校验和与头尾比较
                    {
                        //printf("receive data:");
                        //print_hex(mFrameBuffer, mDataLength);
                        if(mFrameBuffer[4]==0x0A)  //升级相关的命令
                        {
                            printf("升级的命令\n");
                        }
                        else if(mFrameBuffer[4]==0x09)  //version的命令
                        {
                            if(mFrameBuffer[3]==0x02)
                                printf("version is %s\n",mFrameBuffer+5);
                        }
                        else if(mFrameBuffer[4]==0x06)  
                        {
                            if(mFrameBuffer[3]==0x01)
                            {
                                //PublishJointStates(mFrameBuffer+5);
                                return ExitSuccess; 
                                //ROS_INFO("ARM->ROS:length=%d [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", mDataLength, (float(short(mFrameBuffer[5]|(mFrameBuffer[6]<<8))))/10,(float(short(mFrameBuffer[7]|(mFrameBuffer[8]<<8))))/10,(float(short(mFrameBuffer[9]|(mFrameBuffer[10]<<8))))/10,(float(short(mFrameBuffer[11]|(mFrameBuffer[12]<<8))))/10,(float(short(mFrameBuffer[13]|(mFrameBuffer[14]<<8))))/10,(float(short(mFrameBuffer[15]|(mFrameBuffer[16]<<8))))/10,(float(short(mFrameBuffer[17]|(mFrameBuffer[18]<<8))))/10);

                                //printf("ARM->ROS: %f,%f,%f,%f,%f,%f\n",float(mFrameBuffer[5]|mFrameBuffer[6]<<8)/10,float(mFrameBuffer[7]|mFrameBuffer[8]<<8)/10,float(mFrameBuffer[9]|mFrameBuffer[10]<<8)/10,float(mFrameBuffer[11]|mFrameBuffer[12]<<8)/10,float(mFrameBuffer[13]|mFrameBuffer[14]<<8)/10,float(mFrameBuffer[15]|mFrameBuffer[16]<<8)/10);
                            }
                        }
                        else if(mFrameBuffer[4] == CMD_GET_SERVO_RT_INFO)  //舵机状态反馈
                        {
                            if(mFrameBuffer[3]==0x02)
                            {
                                printf("servo is respone CMD_GET_SERVO_RT_INFO\n");
                                servo_state.servo_id = mFrameBuffer[5];
                                servo_state.speed = mFrameBuffer[6]|(mFrameBuffer[7]<<8);
                                servo_state.payload = mFrameBuffer[8]|(mFrameBuffer[9]<<8);
                                servo_state.voltage = mFrameBuffer[10];
                                servo_state.current = mFrameBuffer[11]|(mFrameBuffer[12]<<8);
                                servo_state.flag = 1;
                                
                            }    
                        }
                        else
                        {
                            printf("其它的命令\n");
                        }
                    }
                    else
                    {
                        printf("帧数据出错!\n");
                        print_hex(mFrameBuffer, mDataLength);

                    }
                    mDataLength = 0;
                }
            }
        }
        return ExitSuccess; // return success to continue
    }


} // sdk_sagittarius_arm
