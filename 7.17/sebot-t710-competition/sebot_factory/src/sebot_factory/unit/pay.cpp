/**
 * @file pay.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 智能结算
 * @version 0.1
 * @date 2024/03/15 10:07:03
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

#include "../src/summary.cpp"

int main(int argc, char **argv)
{
    int sizeOrder = 1; // 订单数量
    ros::init(argc, argv, "sebot_summary");
    ros::param::get("/sebot_summary/sizeOrder", sizeOrder);

    Summary summary; // 实例化需求确认类
    vector<Order> orders;

    Order order1;
    order1.tableId = 1;
    order1.part.push_back(LABEL_AI_NUT);

    Order order2;
    order2.tableId = 2;
    order2.part.push_back(LABEL_AI_NUT);
    order2.part.push_back("screw");

    Order order3;
    order3.tableId = 3;
    order3.part.push_back(LABEL_AI_NUT);
    order3.part.push_back("screw");
    order3.part.push_back("block");

    Order order4;
    order4.tableId = 4;
    order4.part.push_back(LABEL_AI_NUT);
    order4.part.push_back("screw");
    order4.part.push_back("block");

    switch (sizeOrder)
    {
    case 0:
        break;
    case 1:
        orders.push_back(order1);
        break;
    case 2:
        orders.push_back(order1);
        orders.push_back(order2);
        break;
    case 3:
        orders.push_back(order1);
        orders.push_back(order2);
        orders.push_back(order3);
        break;
    case 4:
        orders.push_back(order1);
        orders.push_back(order2);
        orders.push_back(order3);
        orders.push_back(order4);
        break;
    case 5:
        orders.push_back(order1);
        orders.push_back(order2);
        orders.push_back(order3);
        orders.push_back(order4);
        orders.push_back(order4);
        break;
    case 6:
        orders.push_back(order1);
        orders.push_back(order1);
        orders.push_back(order2);
        orders.push_back(order2);
        orders.push_back(order3);
        orders.push_back(order3);
        break;
    default:
        break;
    }

    bool run = false;
    while (ros::ok())
    {
        if (!run)
            run = summary.showOrder(orders);
        ros::spinOnce(); // 回调话题通信
    }
    return 0;
}
