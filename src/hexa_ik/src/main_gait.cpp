#include "main_gait.hpp"
#include "tripod_gait.hpp"
#include "hexa_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "sensor_msgs/msg/joy.hpp"
#include <nav_msgs/msg/odometry.hpp>  // Добавляем сообщение одометрии


class TripodGaitNode : public rclcpp::Node, public std::enable_shared_from_this<TripodGaitNode> {
public:
    TripodGaitNode()
        : Node("main_gait_node"),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
          tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "MAIN Gait Node started");

        // Публикация углов суставов
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);


          // Подписка на топик /joy
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
         "/joy", 20, 
         std::bind(&TripodGaitNode::joy_callback, this, std::placeholders::_1));
  

        
        // Подписка на cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20,
            std::bind(&TripodGaitNode::cmd_vel_callback, this, std::placeholders::_1));
        
        // Паблишер одометрии
        // odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // Таймер для выполнения обработки
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TripodGaitNode::update_tripod_gait, this));


    }

private:
    // TF2 компоненты
    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;

    // Публикация состояний суставов
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    
    // Подписка на /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Подписка на /joy
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;


    // Публикация одометрии
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Хранилище текущего состояния суставов
    JointState joint_state_;

    // Глобальная переменная для геометрии гексапода
    const HexapodGeometry HEXAPOD_GEOMETRY;


    // Массивы для хранения линейных и угловых скоростей
    std::array<double, 3> linear_vector = {0.0, 0.0, 0.0};
    std::array<double, 3> angular_vector = {0.0, 0.0, 0.0};
    // double linear_vector[3];  // [x, y, z] линейные скорости
    // double angular_vector[3]; // [x, y, z] угловые скорости

    double z_offset = 0.07;
    double x_offset = 0.18;
    double max_step_len = 0.0;
    double step_speed= 1.875;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // Вывод значений осей

        // float axis_value = msg->axes[0];
        // RCLCPP_INFO(this->get_logger(), "Axis value: %f", static_cast<double>(axis_value));

        linear_vector[0] = msg->axes[3]*max_step_len*2; //  X hasee
        // linear_vector[0] = msg->axes[4]*max_step_len*2; //  X r-pi
        linear_vector[1] = msg->axes[2]*max_step_len*2; //  Y  hasee
        // linear_vector[1] = msg->axes[3]*max_step_len*2; //  Y  r-pi
        linear_vector[2] = 0.0; // линейная скорость по оси Z

        angular_vector[0] = 0.0; // угловая скорость по оси X
        angular_vector[1] = 0.0; // угловая скорость по оси Y
        angular_vector[2] = msg->axes[0]*0.05; // Z  hasee

        step_speed = 0.75 + (msg->axes[4] + 1) * (3.0 - 0.75) / 2.0; // L2 hasse
        // step_speed = 0.75 + (msg->axes[2] + 1) * (3.0 - 0.75) / 2.0; // L2 r-pi

        // Buttons z UP
        if (msg->buttons[4] == 1) {  // hasee
        // if (msg->buttons[2] == 1) {  // r-pi
            // Преобразуем builtin_interfaces::msg::Time в rclcpp::Time
            rclcpp::Time current_time(msg->header.stamp);
            static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
            if(z_offset <=0.025){
                z_offset = 0.025; 
            }
            if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
                z_offset -= 0.005;
                last_press_time = current_time; // Обновляем время
            }
        }

        // z DOWN
        if (msg->buttons[0] == 1) { // hasee
        // if (msg->buttons[0] == 1) {  // r-pi
            // Преобразуем builtin_interfaces::msg::Time в rclcpp::Time
            rclcpp::Time current_time(msg->header.stamp);
            static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
            
            if(z_offset >= 0.15){
                z_offset = 0.15; 
            }
            if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
                z_offset += 0.005;
                last_press_time = current_time; // Обновляем время
            }

        }

        // right 
        if (msg->buttons[1] == 1) {  // hasee
        // if (msg->buttons[1] == 1) {  // r-pi
            // Преобразуем builtin_interfaces::msg::Time в rclcpp::Time
            rclcpp::Time current_time(msg->header.stamp);
            static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
            if(x_offset >= 0.20){
                x_offset = 0.20; 
            }

            if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
                x_offset += 0.005;
                last_press_time = current_time; // Обновляем время
            }
        }
        // left 
        if (msg->buttons[3] == 1) { // hasee
        // if (msg->buttons[3] == 1) {  // r-pi 
            // Преобразуем builtin_interfaces::msg::Time в rclcpp::Time
            rclcpp::Time current_time(msg->header.stamp);
            static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
            if(x_offset <= 0.01){
                x_offset = 0.01; 
            }
            if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
                x_offset -= 0.005;
                last_press_time = current_time; // Обновляем время
            }

        }

        // // треугольник
        // if (msg->buttons[3] == 1) {
        //     rclcpp::Time current_time(msg->header.stamp);
        //     static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
        //     if(step_speed >= 3.0){
        //         step_speed = 3.0; 
        //     }
        //     if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
        //         step_speed += 0.25;
        //         last_press_time = current_time; // Обновляем время
        //     }

        // }

        // // крест
        // if (msg->buttons[0] == 1) {  
        //     rclcpp::Time current_time(msg->header.stamp);
        //     static rclcpp::Time last_press_time(0, 0, RCL_ROS_TIME); // Инициализация времени
        //     if(step_speed <= 0.75){
        //         step_speed = 0.75; 
        //     }
        //     if ((current_time - last_press_time).seconds() > 0.2) { // Задержка 100 мс
        //         step_speed -= 0.25;
        //         last_press_time = current_time; // Обновляем время
        //     }

        // }

        // RCLCPP_INFO(this->get_logger(), "Axes: ");
        // for (size_t i = 0; i < msg->axes.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "  Axis %zu: %f", i, msg->axes[i]);
        // }

        // Вывод значений кнопок
        // RCLCPP_INFO(this->get_logger(), "Buttons: ");
        // for (size_t i = 0; i < msg->buttons.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "  Button %zu: %d", i, msg->buttons[i]);
        // }
    }

        
    // Callback для обработки команд скорости
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Обновление линейных и угловых скоростей из полученного сообщения
        // linear_vector[0] = msg->linear.x; // линейная скорость по оси X
        // linear_vector[1] = msg->linear.y; // линейная скорость по оси Y
        // linear_vector[2] = msg->linear.z; // линейная скорость по оси Z

        // angular_vector[0] = msg->angular.x; // угловая скорость по оси X
        // angular_vector[1] = msg->angular.y; // угловая скорость по оси Y
        // angular_vector[2] = msg->angular.z; // угловая скорость по оси Z

        // Логирование полученных значений
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear[x=%.2f, y=%.2f, z=%.2f], angular[x=%.2f, y=%.2f, z=%.2f]",
        //             linear_vector[0], linear_vector[1], linear_vector[2],
        //             angular_vector[0], angular_vector[1], angular_vector[2]);
    }

    double period  = 0.0;

   
    // Основной цикл обработки данных
    void update_tripod_gait() {
        
        double step_height = 0.15; // Высота шага
        std::array<double, 6> leg_phases = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5}; 

        if (std::abs(linear_vector[0])+ std::abs(linear_vector[1]+angular_vector[2]) > 0.03) {
            period += 1;
            if (period >= step_speed*50) { 
                period = 0;
            } 
        } else {
            period = 0.0;
        }

        // Вызов тактирующей функции
        double timing = generate_cubic_timing(period, step_speed*50);
        // Выводим в лог
        // RCLCPP_INFO(this->get_logger(),"Start Time: %f, Elapsed Time: %f", start_time_.seconds(), elapsed_time);
        // RCLCPP_INFO(this->get_logger(), " timing : %f, period: %f", timing, period);

        // Распределяем фазы между ногами
        leg_phases = gait_phases_tripod(timing);
        // Генерация траекторий для каждой ноги
        for (size_t leg_number = 0; leg_number < 6; ++leg_number) {

            std::array<double, 6> step = generate_arc_and_line(leg_phases[leg_number], step_height, linear_vector, angular_vector[2]);
            // RCLCPP_INFO(this->get_logger(), "Axis value: %f", step[3]);

            // leg number для какой ноги
            // step дуга сгенерированная для шага
            // LEG_ANGLES разворот ног относительно тела
            // X_OFFSET как далеко от центра тела генерируется дуга 
            // linear_vector вектор напрвления 
            std::array<double, 6> ik_pos = tripod_transform_line(leg_number, step, LEG_ANGLES, linear_vector, angular_vector[2], x_offset, z_offset);
    
            // Вычисление IK
            const auto& leg_params = HEXAPOD_GEOMETRY.legs[leg_number];

            IKResult ik_result = calculate_leg_ik(ik_pos[0], ik_pos[1], ik_pos[2], leg_params, x_offset, z_offset+0.025);

            if (ik_result.success) {
                joint_state_.positions[leg_number * 3] = ik_result.joint1_angle;
                joint_state_.positions[leg_number * 3 + 1] = ik_result.joint2_angle;
                joint_state_.positions[leg_number * 3 + 2] = ik_result.joint3_angle;
            } else {
                RCLCPP_WARN(this->get_logger(), "IK failed for leg %zu", leg_number + 1);
            }
            // RCLCPP_INFO(this->get_logger(), "max_step_length_xy: %f", ik_result.max_step_length_xy);
            max_step_len = ik_result.max_step_length_xy;

        }

        // publish_odometry();
        // Публикация состояния суставов
        publish_joint_states();
        
    }

    // Функция публикации состояний суставов
    void publish_joint_states() {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.name = JOINT_NAMES;
        joint_state_msg.position = joint_state_.positions;

        joint_state_pub_->publish(joint_state_msg);
    }


        // Публикация одометрии
    // void publish_odometry() {
    //     auto odom_msg = nav_msgs::msg::Odometry();
    //     odom_msg.header.stamp = this->get_clock()->now();
    //     odom_msg.header.frame_id = "odom";
    //     odom_msg.child_frame_id = "base_link";

    //     // Заполняем только скорость, позицию пока не трогаем
    //     odom_msg.twist.twist.linear.x = linear_vector[0];
    //     odom_msg.twist.twist.linear.y = linear_vector[1];
    //     odom_msg.twist.twist.linear.z = 0.0;

    //     odom_msg.twist.twist.angular.x = 0.0;
    //     odom_msg.twist.twist.angular.y = 0.0;
    //     odom_msg.twist.twist.angular.z = 0.0;

    //     odom_pub_->publish(odom_msg);
    // }




};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TripodGaitNode>());
    rclcpp::shutdown();
    return 0;
}
