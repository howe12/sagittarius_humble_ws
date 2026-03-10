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
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>

namespace sdk_sagittarius_arm
{
    CSDarmCommonSerial::CSDarmCommonSerial(const std::string &serialname, const std::string &baudrate, int &timelimit, bool free_torque):
        CSDarmCommon(),
        mSerialName(serialname),
        mBaudrate(atoi(baudrate.c_str())),
        mTimeLimit(timelimit),
        mExitFreeTorque(free_torque)

    {

    }

    CSDarmCommonSerial::~CSDarmCommonSerial()
    {
        if(mExitFreeTorque)
            SendArmLockOrFree(0);
        CloseDevice();
    }

    int CSDarmCommonSerial::InitDevice()
    {
        size_t i;
        int speed_arr[] = { B1500000, B1000000, B460800, B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
        int name_arr[]  = {  1500000,  1000000, 460800,  230400,  115200,  19200,  9600,  4800,  2400,  1200,  300};
        mFd = -1;
        mFd = open(mSerialName.c_str(), O_RDWR | O_NOCTTY);
        if (mFd == -1)
        {
            printf("open serial failed :%s fail\n", mSerialName.c_str());
            return ExitError;
        }

        struct termios options;
        bzero(&options, sizeof(options));
        cfmakeraw(&options);//ÉèÖÃÎªRawÄ£Ê½
        //设置串口输入波特率和输出波特率
        for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
        {
            if  (mBaudrate == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
                printf("使用波特率：%d\n",mBaudrate);
                break;
            }
        }
        if(i==sizeof(speed_arr) / sizeof(int))
        {
            printf("do not support the baudrate：%d\n",mBaudrate);
            close(mFd);
            return ExitError;
        }

        options.c_cflag |= CLOCAL;//¿ØÖÆÄ£Ê½£¬±£Ö¤³ÌÐò²»»á³ÉÎª¶Ë¿ÚµÄÕ¼ÓÐÕß
        //  options.c_cflag &= ~CLOCAL;//¿ØÖÆÄ£Ê½£¬±£Ö¤³ÌÐò²»»á³ÉÎª¶Ë¿ÚµÄÕ¼ÓÐÕß
        options.c_cflag |= CREAD;//¿ØÖÆÄ£Ê½£¬Ê¹ÄÜ¶Ë¿Ú¶ÁÈ¡ÊäÈëµÄÊý¾Ý
        options.c_cflag &= ~CRTSCTS;//ÎÞÁ÷¿ØÖÆ
        options.c_cflag &= ~CSIZE;//¿ØÖÆÄ£Ê½£¬ÆÁ±Î×Ö·û´óÐ¡Î»
        options.c_cflag |= CS8;//8Î»
        options.c_cflag &= ~PARENB;//ÎÞÐ£Ñé
        options.c_cflag &= ~CSTOPB;//1Í£Ö¹Î»
        options.c_oflag &= ~OPOST;//Êä³öÄ£Ê½£¬Ô­Ê¼Êý¾ÝÊä³ö
        options.c_cc[VMIN]  = 0;//¿ØÖÆ×Ö·û, ËùÒª¶ÁÈ¡×Ö·ûµÄ×îÐ¡ÊýÁ¿
        options.c_cc[VTIME] = 0;//¿ØÖÆ×Ö·û, ¶ÁÈ¡µÚÒ»¸ö×Ö·ûµÄµÈ´ýÊ±¼ä£¬µ¥Î»Îª0.1s

        tcflush(mFd, TCIFLUSH);//Òç³öµÄÊý¾Ý¿ÉÒÔ½ÓÊÕ£¬µ«²»¶Á

        //ÉèÖÃÐÂÊôÐÔ£¬TCSANOW£ºËùÓÐ¸Ä±äÁ¢¼´ÉúÐ§
        if (tcsetattr(mFd, TCSANOW, &options) != 0)
        {
            printf("set device error\n");
            return ExitError;
        }
        printf("open serial: %s successful!\n",mSerialName.c_str());
        return ExitSuccess;
    }


    int CSDarmCommonSerial::CloseDevice()
    {
        if(mFd!=-1)
        {
            close(mFd);
            mFd = -1;
            printf("close serial and Close Device");
        }
        return ExitSuccess;
    }

    int CSDarmCommonSerial::SendSerialData2Arm(char *buf, int length)
    {
        static std::mutex m_mutex;
        int n = -1;
        if(mFd > 0)
        {
            m_mutex.lock();
            n = write(mFd, buf, length);
            m_mutex.unlock();
        }
        return n;
    }

    unsigned char CSDarmCommonSerial::CheckSum(unsigned char *buf)
    {
        int i;
        unsigned char sum = 0;
        for(i=0; i<buf[2]; i++)
        {
            sum += buf[3+i];
        }
        return sum;
    }
    void hex_printf(unsigned char *buf)
    {
        (void)buf;
    #if DEBUG
        int i;
        for(i=0; i<buf[2]+5; i++)
        {
            printf("%02x ",buf[i]);
        }
        printf("\n");
    #endif
    }
    //发送末端命令
    int CSDarmCommonSerial::SendArmEndAction(unsigned char onoff, short value)
    {
        unsigned char buf[30];
        if(mFd == -1)
        {
            /*ROS_ERROR("SendArmLockOrFree: Serial is not open");
            mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                                   "sdk_sagittarius_arm - SendArmLockOrFree: Serial is not open!");*/

            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 5;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_CONTROL_END_ACTION; 	//发送末端命令
        buf[5] = onoff; 					//0:关； 1：开
        buf[6] = value&0xFF;
        buf[7] = value>>8;
        buf[8] = CheckSum(buf);				//校验和
        buf[9] = 0x7D; 						//结束位
        hex_printf(buf);
        if(SendSerialData2Arm((char *)buf, 10) != 10)
        {
            printf("Write error for req command\n");
            return ExitError;
        }
        return ExitSuccess;
    }
    //发送机械臂舵机释放或者锁住命令
    int CSDarmCommonSerial::SendArmLockOrFree(unsigned char onoff)
    {
        unsigned char buf[30];
        if(mFd == -1)
        {

            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 3;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_CONTROL_LOCK_OR_FREE; 	//舵机释放or锁住
        buf[5] = onoff; 					//0:释放； 1：锁住
        buf[6] = CheckSum(buf);				//校验和
        buf[7] = 0x7D; 						//结束位 

        if(SendSerialData2Arm((char *)buf, 8) != 8)
        {
            printf("sdk_sagittarius_arm - SendArmLockOrFree: Write command failed\n");
            return ExitError;
        }
        return ExitSuccess;
    }
    //发送读取舵机实时信息的命令
    int CSDarmCommonSerial::SendGetServoRealTimeInfo(unsigned char id)
    {
        unsigned char buf[30];
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 3;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_GET_SERVO_RT_INFO; 	//读取舵机实时信息的命令
        buf[5] = id; 					    //id号
        buf[6] = CheckSum(buf);				//校验和
        buf[7] = 0x7D; 						//结束位 

        if(SendSerialData2Arm((char *)buf, 8) != 8)
        {
            printf("sdk_sagittarius_arm - SendGetServoRealTimeInfo: Write command failed!\n");
            return ExitError;
        }
        return ExitSuccess;
    }
    //设置舵机的插补速度
    int CSDarmCommonSerial::SetArmVel(unsigned short vel)
    {
        unsigned char buf[30];
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 4;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_SET_SERVO_VEL; 	//设置舵机的插补速度
        buf[5] = vel; 					//速度低位
        buf[6] = vel>>8; 					//速度高位
        buf[7] = CheckSum(buf);				//校验和
        buf[8] = 0x7D; 						//结束位

        if(SendSerialData2Arm((char *)buf, 9) != 9)
        {
            printf("sdk_sagittarius_arm - SetArmVel: Write command failed!\n");
            return ExitError;
        }
        printf("SetArmVel: %d\n",vel);
        return ExitSuccess;
    }
    //设置舵机的加速度
    int CSDarmCommonSerial::SetArmAcc(unsigned char acc)
    {
        unsigned char buf[30];
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 3;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_SET_SERVO_ACC; 	    //设置舵机的加速度
        buf[5] = acc; 					    //加速度
        buf[6] = CheckSum(buf);				//校验和
        buf[7] = 0x7D; 						//结束位

        if(SendSerialData2Arm((char *)buf, 8) != 8)
        {
            printf("sdk_sagittarius_arm - SetArmAcc: Write command failed!\n");
            return ExitError;
        }
        printf("SetArmAcc: %d\n",acc);
        return ExitSuccess;
    }
    //设置舵机的扭矩大小
    int CSDarmCommonSerial::SetArmTorque(int torque[])
    {
        unsigned char buf[30];
        int i;
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 2*7+2;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_SET_SERVO_TORQUE; 	    //设置舵机的扭矩大小
        for(i=0;i<7;i++)
        {
            buf[5+2*i] = torque[i];
            buf[6+2*i] = torque[i]>>8;
                    printf("SetArmTorque: %d",torque[i]);

        }    
        buf[5+2*i] = CheckSum(buf);				//校验和
        buf[6+2*i] = 0x7D; 						//结束位

        if(SendSerialData2Arm((char *)buf, 7+2*i) != 7+2*i)
        {
            printf("sdk_sagittarius_arm - SetArmTorque: Write command failed!\n");
            return ExitError;
        }
        return ExitSuccess;
    }
    int CSDarmCommonSerial::SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 16;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE_AND_DIFF_TIME; 	//多个舵机控制和时间差


        buf[5] = difftime; 					//角度低位
        buf[6] = difftime>>8;
        //    v1 = -v1;
        degree = 1800/PI*v1;				//v1转成角度x10
        buf[7] = degree; 					//角度低位
        buf[8] = degree>>8; 				//角度高位
        degree = 1800/PI*v2;				//v2转成角度x10
        buf[9] = degree; 					//角度低位
        buf[10] = degree>>8; 				//角度高位
        degree = 1800/PI*v3;				//v3转成角度x10  德晟此处为负
        buf[11] = degree; 					//角度低位
        buf[12] = degree>>8; 				//角度高位
        degree = 1800/PI*v4;				//v4转成角度x10
        buf[13] = degree; 					//角度低位
        buf[14] = degree>>8; 				//角度高位
        degree = 1800/PI*v5;				//v5转成角度x10
        buf[15] = degree; 					//角度低位
        buf[16] = degree>>8; 				//角度高位
        degree = 1800/PI*v6;				//v6转成角度x10
        buf[17] = degree; 					//角度低位
        buf[18] = degree>>8; 				//角度高位
        buf[19] = CheckSum(buf);			//校验和
        buf[20] = 0x7D; 					//结束位
        //    ROS_INFO("[%d,%d,%d,%d,%d,%d]", short(buf[5]|(buf[6]<<8)),short(buf[7]|(buf[8]<<8)),short(buf[9]|(buf[10]<<8)),short(buf[11]|(buf[12]<<8)),short(buf[13]|(buf[14]<<8)),short(buf[15]|(buf[16]<<8)));
        //--- ROS_INFO("time:%dms [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", difftime, (float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10,(float(short(buf[17]|(buf[18]<<8))))/10);
        if(memcmp(buf,lastbuf,19)!=0)
        {
            memcpy(lastbuf, buf, 21);
            if(SendSerialData2Arm((char *)buf, 21) != 21)
            {
                printf("sdk_sagittarius_arm - SendSerialData2Arm: Write command failed!\n");
                return ExitError;
            }
        }
        return ExitSuccess;
    }


    //插补控制命令
    int CSDarmCommonSerial::SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 14;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE_CB; 	//插补控制命令
        //    v1 = -v1;
        degree = 1800/PI*v1;				//v1转成角度x10
        buf[5] = degree; 					//角度低位
        buf[6] = degree>>8; 				//角度高位
        degree = 1800/PI*v2;				//v2转成角度x10
        buf[7] = degree; 					//角度低位
        buf[8] = degree>>8; 				//角度高位
        degree = 1800/PI*v3;				//v3转成角度x10  德晟此处为负
        buf[9] = degree; 					//角度低位
        buf[10] = degree>>8; 				//角度高位
        degree = 1800/PI*v4;				//v4转成角度x10
        buf[11] = degree; 					//角度低位
        buf[12] = degree>>8; 				//角度高位
        degree = 1800/PI*v5;				//v5转成角度x10
        buf[13] = degree; 					//角度低位
        buf[14] = degree>>8; 				//角度高位
        degree = 1800/PI*v6;				//v6转成角度x10
        buf[15] = degree; 					//角度低位
        buf[16] = degree>>8; 				//角度高位
        buf[17] = CheckSum(buf);			//校验和
        buf[18] = 0x7D; 					//结束位
        //    ROS_INFO("[%d,%d,%d,%d,%d,%d]", short(buf[5]|(buf[6]<<8)),short(buf[7]|(buf[8]<<8)),short(buf[9]|(buf[10]<<8)),short(buf[11]|(buf[12]<<8)),short(buf[13]|(buf[14]<<8)),short(buf[15]|(buf[16]<<8)));
        //printf("[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]\n", (float(short(buf[5]|(buf[6]<<8))))/10,(float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10);
        if(memcmp(buf,lastbuf,19)!=0)
        {
            memcpy(lastbuf, buf, 19);
            if(SendSerialData2Arm((char *)buf, 19) != 19)
            {
                printf("sdk_sagittarius_arm - SendSerialData2Arm: Write command failed!\n");
                return ExitError;
            }
        }
        return ExitSuccess;
    }


    int CSDarmCommonSerial::SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6)
    {
        unsigned char buf[30];
        static unsigned char lastbuf[30];
        short degree;
        if(mFd == -1)
        {
            return ExitError;
        }
        buf[0] = 0x55;						//帧头1 55
        buf[1] = 0xAA;						//帧头2 AA
        buf[2] = 14;  						//字节数 数据数＋2
        buf[3] = TYPE_REQUEST_MESSAGE;  	//请求信息
        buf[4] = CMD_CONTROL_ALL_DEGREE; 	//多个舵机控制
        degree = 1800/PI*v1;				//v1转成角度x10
        buf[5] = degree; 					//角度低位
        buf[6] = degree>>8; 				//角度高位
        degree = 1800/PI*v2;				//v2转成角度x10
        buf[7] = degree; 					//角度低位
        buf[8] = degree>>8; 				//角度高位
        degree = -1800/PI*v3;				//v3转成角度x10  德晟此处为负
        buf[9] = degree; 					//角度低位
        buf[10] = degree>>8; 				//角度高位
        degree = 1800/PI*v4;				//v4转成角度x10
        buf[11] = degree; 					//角度低位
        buf[12] = degree>>8; 				//角度高位
        degree = 1800/PI*v5;				//v5转成角度x10
        buf[13] = degree; 					//角度低位
        buf[14] = degree>>8; 				//角度高位
        degree = 1800/PI*v6;				//v6转成角度x10
        buf[15] = degree; 					//角度低位
        buf[16] = degree>>8; 				//角度高位
        buf[17] = CheckSum(buf);			//校验和
        buf[18] = 0x7D; 					//结束位
        //    ROS_INFO("[%d,%d,%d,%d,%d,%d]", short(buf[5]|(buf[6]<<8)),short(buf[7]|(buf[8]<<8)),short(buf[9]|(buf[10]<<8)),short(buf[11]|(buf[12]<<8)),short(buf[13]|(buf[14]<<8)),short(buf[15]|(buf[16]<<8)));
        printf("[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]\n", (float(short(buf[5]|(buf[6]<<8))))/10,(float(short(buf[7]|(buf[8]<<8))))/10,(float(short(buf[9]|(buf[10]<<8))))/10,(float(short(buf[11]|(buf[12]<<8))))/10,(float(short(buf[13]|(buf[14]<<8))))/10,(float(short(buf[15]|(buf[16]<<8))))/10);
        if(memcmp(buf,lastbuf,19)!=0)
        {
            memcpy(lastbuf, buf, 19);
            if(SendSerialData2Arm((char *)buf, 19) != 19)
            {
                printf("sdk_sagittarius_arm - SendSerialData2Arm: Write command failed!\n");

                return ExitError;
            }
        }
        return ExitSuccess;
    }


    int CSDarmCommonSerial::GetDataGram(unsigned char* receiveBuffer, int bufferSize, int *length)
    {
        int fs_sel;
        fd_set fs_read;
        static int first_bit=1;
        struct timeval time;
        if(mFd == -1)
        {
            if(first_bit)
            {
                first_bit = 0;
                printf("sdk_sagittarius_arm - GetDataGram:  Serial is not open!\n");
            }

            return ExitError;
        }

        if(1)//(!stream_stopped_)
        {
            FD_ZERO(&fs_read);
            FD_SET(mFd,&fs_read);
            time.tv_sec = 1;//mTimeLimit;
            time.tv_usec = 0;
            //使用select实现串口的多路通信
            fs_sel = select(mFd+1,&fs_read,NULL,NULL,&time);
            if(fs_sel>0)
            {
                if(FD_ISSET(mFd,&fs_read))
                {
                    *length = read(mFd,receiveBuffer,bufferSize);
                }
            }
            else
            {
                printf("sdk_sagittarius_arm - GetDataGram: No full response for read after 5s.\n");
                printf("GetDataGram timeout for %ds", mTimeLimit);
                return ExitError;
            }
        }

        return ExitSuccess;
    }
} /*sdk_sagittarius_arm*/
