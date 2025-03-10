#include "hexa_ik.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <iostream>





        // Функция для расчёта инверсной кинематики
IKResult calculate_leg_ik(double foot_x, double foot_y, double foot_z, const HexapodGeometry::LegGeometry &params, double x_offset, double z_offset) {
    IKResult result;

        // Максимальная досягаемость ноги
        double max_reach = params.link2_length + params.link3_length + params.link4_length;

        // Длина от конца link1 до цели
        double leg_offset = std::sqrt(x_offset * x_offset + z_offset * z_offset);

        // Оставшаяся возможная длина шага в XY-плоскости
        result.max_step_length_xy = max_reach - leg_offset;

        // Длина от основания (link1_length) до цели в плоскости XY
        double foot_xy = std::sqrt(foot_x * foot_x + foot_y * foot_y);
        double x_target = foot_xy - params.link2_length;


        // Длина от конца link1 до цели
        double len_eff = std::sqrt(x_target * x_target + foot_z * foot_z);
        // std::cout << "len_eff: " << len_eff << std::endl;
        // std::cout << "foot_z: " << foot_z << std::endl;

        // Расчёт угла для первого звена (joint1)
        result.joint1_angle = std::atan2(foot_y, foot_x);
        // std::cout << "joint1_angle: " << result.joint1_angle << std::endl;

        // Расчёт угла для третьего звена (joint3_angle)
        double cos_joint3 = std::acos((params. link4_sqr + params. link3_sqr - len_eff * len_eff) / 
                                (2 * params. link4_length * params. link3_length));
        // std::cout << "cos_joint3: " << cos_joint3 << std::endl;


        result.joint3_angle =(cos_joint3)- ((M_PI/2 )+ 0.31339);
        // std::cout << "joint3_angle: " << result.joint3_angle << std::endl;

        // угол от плоскости к стороне ik_sw
        double angle_to_xy = std::atan2(foot_z , x_target);
        // std::cout << "angle_to_xy: " << angle_to_xy << std::endl;

        double angle_3rd = std::acos((params. link3_sqr+ len_eff * len_eff - params. link4_sqr) /
                (2 * params. link3_length * len_eff));
        // std::cout << "angle_3rd: " << angle_3rd << std::endl;

        result.joint2_angle = angle_to_xy  + angle_3rd   ;
        // std::cout << "joint2_angle: " << result.joint2_angle << std::endl;

        result.success = true;
        return result;
}


// // Функция для расчёта инверсной кинематики
// IKResult calculate_leg_ik(double foot_x, double foot_y, double foot_z, const HexapodGeometry::LegGeometry &params) {
//     IKResult result;

//     // Длина от основания (link1_length) до цели в плоскости XY
//     double foot_xy = std::sqrt(foot_x * foot_x + foot_y * foot_y);
//     double x_target = foot_xy - params.link2_length;

//     // Длина от конца link1 до цели
//     double len_eff = std::sqrt(x_target * x_target + foot_z * foot_z);

//     // Максимальная досягаемость ноги
//     double max_reach = params.link2_length + params.link3_length + params.link4_length;

//     // Если заданная точка недостижима, корректируем её
//     if (len_eff > max_reach) {
//         double scale = max_reach / len_eff;
//         foot_x *= scale;
//         foot_y *= scale;
//         foot_z *= scale;
//         foot_xy *= scale;
//         x_target = foot_xy - params.link2_length;
//         len_eff = max_reach;  // Теперь нога дотянется ровно на максимум
//         result.reached_limit = true;
//     } else {
//         result.reached_limit = false;
//     }

//     // Оставшаяся возможная длина шага в XY-плоскости
//     result.max_step_length_xy = max_reach - len_eff;

//     // Расчёт угла для первого звена (joint1)
//     result.joint1_angle = std::atan2(foot_y, foot_x);

//     // Расчёт угла для третьего звена (joint3_angle)
//     double cos_joint3 = std::acos((params.link4_sqr + params.link3_sqr - len_eff * len_eff) / 
//                         (2 * params.link4_length * params.link3_length));
    
//     result.joint3_angle = (cos_joint3) - ((M_PI / 2) + 0.31339);

//     // Угол от плоскости к стороне ik_sw
//     double angle_to_xy = std::atan2(foot_z, x_target);

//     double angle_3rd = std::acos((params.link3_sqr + len_eff * len_eff - params.link4_sqr) /
//             (2 * params.link3_length * len_eff));

//     result.joint2_angle = angle_to_xy + angle_3rd;

//     result.success = true;
//     return result;
// }
