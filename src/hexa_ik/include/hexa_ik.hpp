#ifndef HEXA_IK_HPP
#define HEXA_IK_HPP

#include <string>
#include <vector>


// Константы: Геометрия гексапода
struct HexapodGeometry {
    struct LegGeometry {
        double link1_length;  // Длина первого звена
        double link2_length;  // Длина второго звена
        double link2_sqr;  // Квадрат длины второго звена
        double link3_length;  // Длина третьего звена
        double link3_sqr;  // Квадрат длины третьего звена
        double link4_length;  // Длина четвёртого звена
        double link4_sqr;  // Квадрат длины четвёртого звена
    };

    std::vector<LegGeometry> legs;

    HexapodGeometry() {
        // Инициализация геометрии ног по умолчанию
        legs = {
            {0.077942, 0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
            {0.077942,  0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
            {0.077942,  0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
            {0.077942,  0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
            {0.077942,  0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
            {0.077942,  0.07059, 0.00498, 0.0794, 0.0063, 0.1053, 0.011088},
        };
    }
};


// struct IKResult {
//     double joint1_angle; // Угол первого звена
//     double joint2_angle; // Угол второго звена
//     double joint3_angle; // Угол третьего звена
//     bool success;       // Флаг успешного расчёта IK

//     IKResult() : joint1_angle(0.0), joint2_angle(0.0), joint3_angle(0.0), success(false) {}
// };

// IKResult calculate_leg_ik(double foot_x, double foot_y, double foot_z, const HexapodGeometry::LegGeometry &params);

// Структура результата IK
struct IKResult {
    double joint1_angle; // Угол первого звена
    double joint2_angle; // Угол второго звена
    double joint3_angle; // Угол третьего звена
    double max_step_length_xy; // Оставшаяся возможная длина шага в XY-плоскости
    bool success;       // Флаг успешного расчёта IK
    bool reached_limit; // Флаг, показывающий, было ли ограничение по досягаемости

    IKResult() : joint1_angle(0.0), joint2_angle(0.0), joint3_angle(0.0), max_step_length_xy(0.0), success(false), reached_limit(false) {}
};

IKResult calculate_leg_ik(double foot_x, double foot_y, double foot_z, const HexapodGeometry::LegGeometry &params, double x_offset, double z_offset);


#endif // HEXA_IK_HPP


