/**
 * @file arm.hpp
 * @author Leo (sasu@saishukeji.com)
 * @brief 机械臂通信驱动
 * @version 0.1
 * @date 2024-03-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <libserial/SerialPort.h> // 串口通信
#include <iostream>               // 输入输出类
#include <math.h>                 // 数学函数类
#include <stdint.h>               // 整型数据类
#include <thread>
#include <string.h>
#include <unistd.h>

using namespace LibSerial;
using namespace std;

// USB通信地址
#define USB_ADDR_POS_CTRL 1     // 机械臂运动学姿态控制
#define USB_ADDR_JOINTS_ANGLE 2 // 机械臂电机角度控制
#define USB_ADDR_JOINT_CTRL 3   // 机械臂单电机角度控制
#define USB_ADDR_JOINT_CAL 4    // 关节标定
#define USB_ADDR_JOINT_TH_MIN 5 // 关节下限阈值
#define USB_ADDR_JOINT_TH_MAX 6 // 关节阈值
#define USB_ADDR_JOINT_ENABE 7  // 关节使能
#define USB_ADDR_JOINT_TEACH 8  // 机械臂轨迹示教
#define USB_ADDR_JOINT_ACTION 9 // 机械臂动作组
#define USB_ADDR_CLAW_MOTION 10 // 机械爪运动控制
#define USB_ADDR_CLAW_CAL 11    // 机械爪位置标定（复位）

#define USB_ADDR_HART 55  // 心跳信号
#define USB_ADDR_RESET 66 // 系统复位

// USB通信帧
#define USB_FRAME_HEAD 0x42 // USB通信帧头
#define USB_FRAME_LENMIN 4  // USB通信帧最短字节长度
#define USB_FRAME_LENMAX 30 // USB通信帧最长字节长度

class Talon
{
private:
    std::unique_ptr<std::thread> threadRec; // 串口接收子线程
    std::shared_ptr<SerialPort> serialPort = nullptr;

    /**
     * @brief 32位数据内存对齐/联合体
     *
     */
    typedef union
    {
        uint8_t buff[4];
        float float32;
        int int32;
    } Bit32Union;

    /**
     * @brief 16位数据内存对齐/联合体
     *
     */
    typedef union
    {
        uint8_t buff[2];
        int int16;
        uint16_t uint16;
    } Bit16Union;

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
    Talon()
    {
        std::string port = "/dev/talon";
        serialPort = std::make_shared<SerialPort>();
        if (serialPort == nullptr)
        {
            std::cerr << "Serial Create Failed ." << std::endl;
            return;
        }
        // try检测语句块有没有异常
        try
        {
            serialPort->Open(port);                                     // 打开串口
            serialPort->SetBaudRate(BaudRate::BAUD_115200);             // 设置波特率
            serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
            serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
            serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
            serialPort->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位
        }
        catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << port << " open failed ..." << std::endl;
            isOpen = false;
            return;
        }
        catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << port << " open failed ..." << std::endl;
            isOpen = false;
            return;
        }
        catch (...) // catch捕获并处理 try 检测到的异常。
        {
            std::cerr << "Serial port: " << port << " recv exception ..." << std::endl;
            isOpen = false;
            return;
        }

        serialStr.start = false;
        serialStr.index = 0;
        serialStr.counterHart = 0;
        isOpen = true;
        // threadReceive(); // 开启数据接收子线程
    };

    // 定义析构函数
    ~Talon(){};
    SerialStruct serialStr; // 串口通信数据结构体
    bool isOpen = false;
    bool actionOver[4] = {false}; // 动作执行结束标志：复位/伸展/收缩/其它
    bool actionEnable = false;    // 动作库执行反馈标志

    /**
     * @brief    机械臂关节编号
     **/
    typedef enum
    {
        ARM_JOINT_1 = 0,
        ARM_JOINT_2,
        ARM_JOINT_3,
        ARM_JOINT_4,
        ARM_JOINT_5,
        ARM_JOINT_6,
        ARM_JOINT_ALL, // 所有关节
    } ArmJoint;

    /**
     * @brief 机械臂动作库类型
     *
     */
    typedef enum
    {
        ACTION_RES = 0, // 机械臂复位
        ACTION_EXT,     // 机械臂伸展
        ACTION_CUR,     // 机械臂收缩
        ACTION_PUT,     // 机械臂放置
        ACTION_DIY      // 自定义动作
    } Action;

    /**
     * @brief 开启串口接收子线程
     *
     */
    void threadReceive(void)
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
        // threadRec->join();
        // if (serialPort != nullptr)
        // {
        //     serialPort->Close();
        //     serialPort = nullptr;
        // }
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

        switch (serialStr.buffFinish[1])
        {
        case USB_ADDR_JOINT_ACTION:
            if (serialStr.buffFinish[2] == 5) // 动作库执行完成
            {
                if (serialStr.buffFinish[3] > 0 && serialStr.buffFinish[3] <= 4)
                    actionOver[serialStr.buffFinish[3] - 1] = true;
            }
            else if (serialStr.buffFinish[2] == 4) // 动作库执行反馈
            {
                actionEnable = true;
            }
            break;

        default:
            break;
        }
    }

    /**
     * @brief 控制关节电机使能/失能
     *
     * @param joint 电机编号
     * @param enable 开关标志
     */
    void setJointEnable(ArmJoint joint, bool enable)
    {
        if (!isOpen)
            return;

        uint8_t buff[7];                // 多发送一个字节
        uint8_t check = 0;              // 校验位
        buff[0] = USB_FRAME_HEAD;       // 通信帧头
        buff[1] = USB_ADDR_JOINT_ENABE; // 地址
        buff[2] = 6;                    // 帧长

        buff[3] = (uint8_t)(joint + 1); // 关节电机编号
        if (enable)                     // 机械臂是否使能
            buff[4] = 1;
        else
            buff[4] = 2;

        for (int i = 0; i < buff[2] - 1; i++)
            check += buff[i];
        buff[buff[2] - 1] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 7; i++)
            transmitByte(buff[i]);
    }

    /**
     * @brief 机械臂运动学姿态控制
     *
     * @param x 三维坐标系：x
     * @param y 三维坐标系：x
     * @param z 三维坐标系：x
     * @param pitch 俯仰角
     * @param yaw 航向角
     * @param roll 翻滚角
     */
    void setArmPose(float x, float y, float z, float pitch, float yaw, float roll)
    {
        if (!isOpen)
            return;

        uint8_t buff[29];            // 多发送一个字节
        uint8_t check = 0;           // 校验位
        buff[0] = USB_FRAME_HEAD;    // 通信帧头
        buff[1] = USB_ADDR_POS_CTRL; // 地址
        buff[2] = 28;                // 帧长

        Bit32Union bit32Union;
        bit32Union.float32 = x; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 3] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = y; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 7] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = z; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 11] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = pitch; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 15] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = roll; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 19] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = yaw; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 23] = bit32Union.buff[i]; // 写数据

        for (int i = 0; i < 27; i++)
            check += buff[i];
        buff[27] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 29; i++)
            transmitByte(buff[i]);
    }

    /**
     * @brief 机械臂单关节控制
     *
     * @param joint 关节电机编号
     * @param position 关节电机位置：-2Π~2Π（rad）
     */
    void setJointPosition(ArmJoint joint, float position)
    {
        if (!isOpen)
            return;

        if (position < 0)
            position = 0;
        else if (position > 10)
            position = 10;

        uint8_t buff[10];              // 多发送一个字节
        uint8_t check = 0;             // 校验位
        buff[0] = USB_FRAME_HEAD;      // 通信帧头
        buff[1] = USB_ADDR_JOINT_CTRL; // 地址
        buff[2] = 9;                   // 帧长

        buff[3] = (uint8_t)(joint + 1); // 关节电机编号

        Bit32Union bit32Union;
        bit32Union.float32 = position; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 4] = bit32Union.buff[i]; // 写数据

        for (int i = 0; i < 8; i++)
            check += buff[i];
        buff[8] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 10; i++)
            transmitByte(buff[i]);
    }

    /**
     * @brief 机械臂(所有)关节控制
     *
     * @param pos1 关节电机位置：-2Π~2Π（rad）
     * @param pos2
     * @param pos3
     * @param pos4
     * @param pos5
     * @param range 夹爪距离：0~10cm
     */
    void setJointsPosition(float pos1, float pos2, float pos3, float pos4, float pos5, float range)
    {
        if (!isOpen)
            return;

        uint8_t buff[29];                // 多发送一个字节
        uint8_t check = 0;               // 校验位
        buff[0] = USB_FRAME_HEAD;        // 通信帧头
        buff[1] = USB_ADDR_JOINTS_ANGLE; // 地址
        buff[2] = 28;                    // 帧长

        Bit32Union bit32Union;
        bit32Union.float32 = pos1; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 3] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = pos2; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 7] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = pos3; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 11] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = pos4; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 15] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = pos5; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 19] = bit32Union.buff[i]; // 写数据

        bit32Union.float32 = range; // 声明联合体对齐数据位
        for (int i = 0; i < 4; i++)
            buff[i + 23] = bit32Union.buff[i]; // 写数据

        for (int i = 0; i < 27; i++)
            check += buff[i];
        buff[27] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 29; i++)
            transmitByte(buff[i]);
    }

    /**
     * @brief 设置机械臂动作组
     *
     * @param action 动作类型
     */
    void setActions(Action action)
    {
        if (!isOpen)
            return;

        actionEnable = false;

        uint8_t buff[6];                 // 多发送一个字节
        uint8_t check = 0;               // 校验位
        buff[0] = USB_FRAME_HEAD;        // 通信帧头
        buff[1] = USB_ADDR_JOINT_ACTION; // 地址
        buff[2] = 5;                     // 帧长

        buff[3] = (uint8_t)(action + 1); // 动作类型

        for (int i = 0; i < 4; i++)
            check += buff[i];
        buff[4] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 6; i++)
            transmitByte(buff[i]);

        actionOver[action] = false; // 状态刷新
    }

    /**
     * @brief 机械爪运动控制
     *
     * @param x X轴方向角度控制
     * @param y Y轴方向角度控制
     */
    void setClawMotion(float x, float y)
    {
        if (!isOpen)
            return;

        uint8_t buff[13];               // 多发送一个字节
        uint8_t check = 0;              // 校验位
        buff[0] = USB_FRAME_HEAD;       // 通信帧头
        buff[1] = USB_ADDR_CLAW_MOTION; // 地址
        buff[2] = 12;                   // 帧长

        Bit32Union bit32U;
        bit32U.float32 = x;
        for (int i = 0; i < 4; i++)
            buff[3 + i] = bit32U.buff[i];
        bit32U.float32 = y;
        for (int i = 0; i < 4; i++)
            buff[7 + i] = bit32U.buff[i];

        for (int i = 0; i < 11; i++)
            check += buff[i];
        buff[11] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 13; i++)
            transmitByte(buff[i]);
    }

    /**
     * @brief 初始化（标定）机械爪
     *
     * @param action
     */
    void setClawInitial()
    {
        if (!isOpen)
            return;

        uint8_t buff[5];             // 多发送一个字节
        uint8_t check = 0;           // 校验位
        buff[0] = USB_FRAME_HEAD;    // 通信帧头
        buff[1] = USB_ADDR_CLAW_CAL; // 地址
        buff[2] = 4;                 // 帧长

        for (int i = 0; i < 3; i++)
            check += buff[i];
        buff[3] = check; // 校验位

        // 循环发送数据
        for (size_t i = 0; i < 5; i++)
            transmitByte(buff[i]);
    }
};