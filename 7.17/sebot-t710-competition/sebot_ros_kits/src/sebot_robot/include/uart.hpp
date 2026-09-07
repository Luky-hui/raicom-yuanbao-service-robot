#pragma once
/**
 * @file uart.hpp
 * @author HC
 * @brief 机器人上下位机串口通信
 * @version 0.1
 * @date 2023-09-13
 *
 * @copyright Copyright (c) 2023
 * @note 具体功能模块：
 *                  1.接收来自传感器的数据并将数据储存在内部的结构体。
 *                  2.向下位机发送机器人的速度控制、心跳信号以及复位信号。
 */
#include <libserial/SerialPort.h> // 串口通信
#include "util.hpp"               // 公共类
#include <iostream>               // 输入输出类
#include <math.h>                 // 数学函数类
#include <stdint.h>               // 整型数据类
#include <thread>
#include <string.h>

using namespace LibSerial;
using namespace std;

// USB通信地址
#define USB_ADDR_SPEEDCTRL 1 // 速度控制
#define USB_ADDR_BATTERY 2   // 电量数据     :1s
#define USB_ADDR_ULTRA 3     // 超声波数据   :100ms
#define USB_ADDR_ODOM 4      // 里程计数据    :30ms
#define USB_ADDR_IMU 5       // IMU数据      :30ms
#define USB_ADDR_MOVECTRL 6  // 运动控制
#define USB_ADDR_BEEP 7      // 蜂鸣器控制
#define USB_ADDR_LIDAR 9     // 激光雷达使能

#define USB_ADDR_HART 55  // 心跳信号
#define USB_ADDR_RESET 66 // 系统复位

// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 36 // USB通信帧最长字节长度

/**
 * @brief 串口通信结构体
 *
 */
typedef struct
{
    bool start;                           // 开始接收标志
    uint8_t index;                        // 接收序列
    uint8_t buffRead[USB_FRAME_LENMAX];   // 临时缓冲数据
    uint8_t buffFinish[USB_FRAME_LENMAX]; // 校验成功数据
    uint8_t counterHart;                  // 发送心跳计数器
} SerialStruct;

/**
 * @brief 里程计数据
 *
 */
typedef struct
{
    float moveX;      // X轴线性位移
    float moveY;      // Y轴线性位移
    float angleRad;   // 转角（rad/弧度）
    float speedLineX; // X轴线速度m/s
    float speedLineY; // Y轴线速度m/s
    float speedAngle; // 角速度rad/s
} OdometryStruct;

/**
 * @brief Imu惯性里程计数据
 *
 */
typedef struct
{
    short accx; // 线加速度
    short accy;
    short accz;
    short gyrox; // 角速度
    short gyroy;
    short gyroz;
    float yaw; // 航向角: rad
} ImuStruct;

/**
 * @brief 超声波距离数据：m
 *
 */
typedef struct
{
    float ultraLF; // 左前超声波
    float ultraMF; // 中前超声波
    float ultraRF; // 右前超声波
    float ultraMB; // 中后超声波
} UltraStruct;

class Uart
{
private:
    std::unique_ptr<std::thread> threadRec; // 串口接收子线程
    std::shared_ptr<SerialPort> serialPort = nullptr;
    std::string portName; // 端口名字
    BaudRate baud;        // 波特率
    bool isOpen = false;

    /**
     * @brief 串口接收字节数据
     *
     * @param charBuffer
     * @param msTimeout
     * @return int
     */
    int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0)
    {
        /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
        如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
        try
        {
            /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
            如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
            serialPort->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
        }
        catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
        {
            // std::cerr << "The ReadByte() call has timed out." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        return 0;
    };

    /**
     * @brief
     *
     * @param data
     * @return int
     */
    int transmitByte(unsigned char data)
    {

        // try检测语句块有没有异常
        try
        {
            serialPort->WriteByte(data); // 写数据到串口
        }
        catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "The Write() runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Port Not Open ..." << std::endl;
            return -1;
        }
        serialPort->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
        return 0;
    }

public:
    // 定义构造函数
    Uart(const std::string &port, BaudRate bps) : portName(port), baud(bps){};
    // 定义析构函数
    ~Uart() { close(); };

public:
    SerialStruct serialStr; // 串口通信数据结构体
    UltraStruct ultraStr;   // 超声波距离数据：m
    OdometryStruct odomStr; // 里程计数据结构体
    ImuStruct imuStr;       // Imu惯性里程计数据结构体

    /**
     * @brief 启动串口通信
     *
     * @param port 串口号
     * @param baud 波特率
     * @return int
     */
    int open(void)
    {
        serialPort = std::make_shared<SerialPort>();
        if (serialPort == nullptr)
        {
            std::cerr << "Serial Create Failed ." << std::endl;
            return -1;
        }
        // try检测语句块有没有异常
        try
        {
            serialPort->Open(portName);                                 // 打开串口
            serialPort->SetBaudRate(baud);                              // 设置波特率
            serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
            serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
            serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
            serialPort->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位
        }
        catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << "open failed ..." << std::endl;
            isOpen = false;
            return -2;
        }
        catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << "open failed ..." << std::endl;
            isOpen = false;
            return -3;
        }
        catch (...) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << portName << " recv exception ..." << std::endl;
            isOpen = false;
            return -4;
        }

        serialStr.start = false;
        serialStr.index = 0;
        serialStr.counterHart = 0;
        isOpen = true;
        return 0;
    }

    /**
     * @brief 开启串口接收子线程
     *
     */
    void receiveThread(void)
    {
        if (!isOpen)
            return;

        threadRec = std::make_unique<std::thread>([this]()
                                                  {
        while (1) 
        {
            receiveCheck();//串口接收校验
        } }); // 串口通信数据接收处理线程
    }

    /**
     * @brief 关闭串口通信
     *
     */
    void close(void)
    {
        threadRec->join();
        if (serialPort != nullptr)
        {
            serialPort->Close();
            serialPort = nullptr;
        }
    }

    /**
     * @brief 串口接收校验
     *
     */
    void receiveCheck(void)
    {
        if (!isOpen) // 串口是否正常打开
            return;

        uint8_t resByte = 0;
        int ret = receiveBytes(resByte, 0);
        if (ret == 0)
        {
            if (resByte == USB_FRAME_HEAD && !serialStr.start) // 监听帧头
            {
                serialStr.start = true;                   // 开始接收数据
                serialStr.buffRead[0] = resByte;          // 获取帧头
                serialStr.buffRead[2] = USB_FRAME_LENMIN; // 初始化帧长
                serialStr.index = 1;
            }
            else if (serialStr.index == 2) // 接收帧的长度
            {
                serialStr.buffRead[serialStr.index] = resByte;
                serialStr.index++;
                if (resByte > USB_FRAME_LENMAX || resByte < USB_FRAME_LENMIN) // 帧长错误
                {
                    serialStr.buffRead[2] = USB_FRAME_LENMIN; // 重置帧长
                    serialStr.index = 0;
                    serialStr.start = false; // 重新监听帧长
                }
            }
            else if (serialStr.start && serialStr.index < USB_FRAME_LENMAX) // 开始接收数据
            {
                serialStr.buffRead[serialStr.index] = resByte; // 读取数据
                serialStr.index++;                             // 索引下移
            }

            // 帧长接收完毕
            if ((serialStr.index >= USB_FRAME_LENMAX || serialStr.index >= serialStr.buffRead[2]) &&
                serialStr.index > USB_FRAME_LENMIN) // 检测是否接收完数据
            {
                uint8_t check = 0; // 初始化校验和
                uint8_t length = USB_FRAME_LENMIN;
                length = serialStr.buffRead[2]; // 读取本次数据的长度
                for (int i = 0; i < length - 1; i++)
                    check += serialStr.buffRead[i]; // 累加校验和

                if (check == serialStr.buffRead[length - 1]) // 校验和相等
                {
                    memcpy(serialStr.buffFinish, serialStr.buffRead, USB_FRAME_LENMAX); // 储存接收的数据
                    dataTransform();
                }

                serialStr.index = 0;     // 重新开始下一轮数据接收
                serialStr.start = false; // 重新监听帧头
            }
        }
    }

    /**
     * @brief 串口通信协议数据转换
     */
    void dataTransform(void)
    {
        if (!isOpen)
            return;

        Bit32Union bit32Union;
        switch (serialStr.buffFinish[1])
        {
        case USB_ADDR_ODOM: // 里程计数据
            for (int i = 0; i < 4; i++)
                bit32Union.buff[i] = serialStr.buffFinish[i + 3];
            odomStr.moveX = bit32Union.float32;

            for (int i = 0; i < 4; i++) // Y轴线性移动
                bit32Union.buff[i] = serialStr.buffFinish[i + 7];
            odomStr.moveY = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 转角（rad/弧度）
                bit32Union.buff[i] = serialStr.buffFinish[i + 11];
            odomStr.angleRad = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 线速度m/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 15];
            odomStr.speedLineX = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 角速度rad/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 19];
            odomStr.speedAngle = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 角速度rad/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 23];
            odomStr.speedAngle = bit32Union.float32;
            break;

        case USB_ADDR_IMU:              // 陀螺仪数据
            for (int i = 0; i < 4; i++) // 线加速度m/s2
                bit32Union.buff[i] = serialStr.buffFinish[i + 3];
            imuStr.accx = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 线加速度m/s2
                bit32Union.buff[i] = serialStr.buffFinish[i + 7];
            imuStr.accy = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 线加速度m/s2
                bit32Union.buff[i] = serialStr.buffFinish[i + 11];
            imuStr.accz = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 角速度rad/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 15];
            imuStr.gyrox = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 角速度rad/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 19];
            imuStr.gyroy = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 角速度rad/s
                bit32Union.buff[i] = serialStr.buffFinish[i + 23];
            imuStr.gyroz = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 转角（rad/弧度）
                bit32Union.buff[i] = serialStr.buffFinish[i + 27];
            imuStr.yaw = bit32Union.float32;
            break;

        case USB_ADDR_ULTRA:            // 超声波数据
            for (int i = 0; i < 4; i++) // 左前
                bit32Union.buff[i] = serialStr.buffFinish[i + 3];
            ultraStr.ultraLF = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 中前
                bit32Union.buff[i] = serialStr.buffFinish[i + 7];
            ultraStr.ultraMF = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 右前
                bit32Union.buff[i] = serialStr.buffFinish[i + 11];
            ultraStr.ultraRF = bit32Union.float32;

            for (int i = 0; i < 4; i++) // 中后
                bit32Union.buff[i] = serialStr.buffFinish[i + 15];
            ultraStr.ultraMB = bit32Union.float32;
            break;
        default:
            break;
        }
    }

    /**

     * @brief 机器人速度控制
     *
     * @param speedLineX 线速度
     * @param speedLineY 线速度
     * @param speedAngle 角速度
     */
    void speedControl(float speedLineX, float speedLineY, float speedAngle)
    {
        if (!isOpen)
            return;

        uint8_t buff[18];  // 多发送一个字节
        uint8_t check = 0; // 校验位
        Bit32Union bit32Union;

        buff[0] = USB_FRAME_HEAD;     // 通信帧头
        buff[1] = USB_ADDR_SPEEDCTRL; // 地址
        buff[2] = 16;                 // 帧长

        bit32Union.float32 = speedLineX; // X轴线速度
        for (int i = 0; i < 4; i++)
            buff[i + 3] = bit32Union.buff[i];

        bit32Union.float32 = speedLineY; // Y轴线速度
        for (int i = 0; i < 4; i++)
            buff[i + 7] = bit32Union.buff[i];

        bit32Union.float32 = speedAngle; // 角速度
        for (int i = 0; i < 4; i++)
            buff[i + 11] = bit32Union.buff[i];

        for (int i = 0; i < 15; i++)
            check += buff[i];
        buff[15] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 18; i++)
            transmitByte(buff[i]);

        serialStr.counterHart = 0; // 心跳发送重计时
    }

    /**
     * @brief 串口发送心跳信号
     *
     */
    void transmitHart(void)
    {
        if (!isOpen)
            return;
        if (serialStr.counterHart > 10)
        {
            uint8_t buff[5];          // 多发送一个字节
            uint8_t check = 0;        // 校验位
            buff[0] = USB_FRAME_HEAD; // 通信帧头
            buff[1] = USB_ADDR_HART;  // 地址
            buff[2] = 4;              // 帧长

            for (int i = 0; i < 3; i++)
                check += buff[i];
            buff[3] = check; // 校验和

            // 循环发送数据
            for (size_t i = 0; i < 5; i++)
                transmitByte(buff[i]);
            serialStr.counterHart = 0; // 重置心跳计时器
        }
        else
            serialStr.counterHart++;
    }

    /**
     * @brief 发送系统复位信号
     */
    void transmitReset(void)
    {
        if (!isOpen)
            return;
        uint8_t buff[5];          // 多发送一个字节
        uint8_t check = 0;        // 校验位
        buff[0] = USB_FRAME_HEAD; // 通信帧头
        buff[1] = USB_ADDR_RESET; // 帧头
        buff[2] = 4;              // 帧长

        for (int i = 0; i < 3; i++)
            check += buff[i];
        buff[3] = check;

        // 循环发送数据
        for (size_t i = 0; i < 5; i++)
            transmitByte(buff[i]);

        serialStr.counterHart = 0; // 重置心跳计时器
    }

    /**
     * @brief 激光雷达使能
     *
     * @param enable true/fasle
     */
    void lidarEnable(bool enable)
    {
        if (!isOpen)
            return;
        uint8_t buff[6];          // 多发送一个字节
        uint8_t check = 0;        // 校验位
        buff[0] = USB_FRAME_HEAD; // 通信帧头
        buff[1] = USB_ADDR_LIDAR; // 地址
        buff[2] = 5;              // 帧长

        if (enable)
            buff[3] = 1;
        else
            buff[3] = 2;

        for (int i = 0; i < 4; i++)
            check += buff[i];
        buff[4] = check;

        // 循环发送数据
        for (size_t i = 0; i < 6; i++)
            transmitByte(buff[i]);

        serialStr.counterHart = 0; // 重置心跳计时器
    }
};