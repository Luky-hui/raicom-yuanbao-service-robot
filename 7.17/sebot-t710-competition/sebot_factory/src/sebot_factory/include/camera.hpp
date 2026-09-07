/**
 * @file camera.hpp
 * @author Leo (leo@saishukeji.com)
 * @brief 获取设备相机video编号
 * @version 0.1
 * @date 2025-02-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

enum VideoIndex
{
    ASTRA_DEPTH = 0, // 深度相机：深度图像
    ASTRA_RGB,       // 深度相机：彩色图像
    ARM_RGB,         // 机械臂相机：彩色图像
};

/**
 * @brief 获取相机图像类型
 *
 * @param device
 */
std::string getFormats(const std::string &device)
{
    std::string format = "";
    int fd = open(device.c_str(), O_RDONLY);
    if (fd == -1)
    {
        std::cerr << "Failed to open " << device << std::endl;
        return format;
    }

    struct v4l2_fmtdesc fmt;
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // 仅检查视频流
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0)
    {
        char fourcc[5] = {
            (char)(fmt.pixelformat & 0xFF),
            (char)((fmt.pixelformat >> 8) & 0xFF),
            (char)((fmt.pixelformat >> 16) & 0xFF),
            (char)((fmt.pixelformat >> 24) & 0xFF),
            '\0'};

        format += std::string(fourcc) + " ";
        std::cout << "  " << fmt.description << " ("
                  << (char)(fmt.pixelformat & 0xFF)
                  << (char)((fmt.pixelformat >> 8) & 0xFF)
                  << (char)((fmt.pixelformat >> 16) & 0xFF)
                  << (char)((fmt.pixelformat >> 24) & 0xFF)
                  << ")" << std::endl;
        fmt.index++;
    }

    close(fd);

    return format;
}

/**
 * @brief 获取相机驱动端口号（/dev/video*）
 *
 * @return bool
 */
bool getVideoDevice(VideoIndex camera, std::string &device)
{
    for (int i = 0; i < 30; ++i)
    {
        std::string dev_name = "/dev/video" + std::to_string(i);
        int fd = open(dev_name.c_str(), O_RDONLY);
        if (fd == -1)
            continue;

        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
        {
            // std::cout << "Device: " << dev_name
            //           << " Driver: " << cap.driver
            //           << " Card: " << cap.card
            //           << " Bus Info: " << cap.bus_info
            //           << std::endl;
            switch (camera)
            {
                // case VideoIndex::ASTRA_DEPTH: // 彩色相机：D435i相机（video4）
                //     if (strcmp((const char *)cap.card, "Intel(R) RealSense(TM) Depth Ca") == 0 &&
                //         strcmp((const char *)cap.bus_info, "usb-0000:00:14.0-2.3") == 0)
                //     {
                //         if (getFormats(dev_name).find("YUYV") != std::string::npos)
                //         {
                //             close(fd);
                //             device = dev_name;
                //             return true;
                //         }
                //     }
                //     break;

                // 乐视
                // Device: /dev/video0 Driver: uvcvideo Card: Astra Pro FHD Camera: Astra Pro Bus Info: usb-xhci-hcd.0.auto-1.1.3.2
                // Device: /dev/video1 Driver: uvcvideo Card: PC Camera A4: PC Camera A4 Bus Info: usb-xhci-hcd.0.auto-1.2.2.2
                // Astro-Pro
                // Device: /dev/video0 Driver: uvcvideo Card: USB 2.0 Camera: USB Camera Bus Info: usb-xhci-hcd.0.auto-1.1.3.2
                // Device: /dev/video1 Driver: uvcvideo Card: PC Camera A4: PC Camera A4 Bus Info: usb-xhci-hcd.0.auto-1.2.2.2

            case VideoIndex::ASTRA_RGB: // 深度相机：彩色图像（video0）
                if (strcmp((const char *)cap.card, "Astra Pro FHD Camera: Astra Pro") == 0)
                {
                    if (getFormats(dev_name).find("MJPG") != std::string::npos)
                    {
                        close(fd);
                        device = dev_name;
                        return true;
                    }
                }
                if (strcmp((const char *)cap.card, "USB 2.0 Camera: USB Camera") == 0)
                {
                    if (getFormats(dev_name).find("MJPG") != std::string::npos)
                    {
                        close(fd);
                        device = dev_name;
                        return true;
                    }
                }
                break;

            case VideoIndex::ARM_RGB: // 机械臂相机：彩色图像（video1）
                if (strcmp((const char *)cap.card, "PC Camera A4: PC Camera A4") == 0)
                {
                    if (getFormats(dev_name).find("MJPG") == std::string::npos)
                    {
                        close(fd);
                        device = dev_name;
                        return true;
                    }
                }
                break;

            default:
                break;
            }
        }
        close(fd);
    }

    return false;
}
