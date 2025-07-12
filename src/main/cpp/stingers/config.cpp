/**
*   Configuration options for FRC #8193's FRC 2026 season chassis.
*
*   Copyright (C) 2025 Frederick Ziola, et al. (New Lothrop Robotics)
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*   Contact us: robotics@newlothrop.k12.mi.us
*/

#include <stingers/swerve.hpp>

// Base swerve configuration, excluding navigational stuff
const stingers::swerve::Configuration stingers::swerve::swerve_config = {
    // All the modules' settings are here
    .modules = {
        // Front left
        stingers::swerve::Configuration::Module{
            .turn_type = TALON_FX,
            .turn_id = 2, // motor type and CAN id
            .turn_ratio = 1.0 / 13.3714,
            .turn_kp = 24.0,
            .turn_ki = 4.0,
            .turn_kd = 0.25,

            .drive_type = TALON_FX,
            .drive_id = 1,
            .drive_ratio = 5.01,
            .drive_kp = 0.1,
            .drive_ki = 0.0,
            .drive_kd = 0.0,

            .frame_offset_x = -28.575_cm,
            .frame_offset_y =
                28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        // Front right
        stingers::swerve::Configuration::Module{
            .turn_type = TALON_FX,
            .turn_id = 5, // motor type and CAN id
            .turn_ratio = 1.0 / 13.3714,
            .turn_kp = 24.0,
            .turn_ki = 4.0,
            .turn_kd = 0.25,

            .drive_type = TALON_FX,
            .drive_id = 4,
            .drive_ratio = 5.01,
            .drive_kp = 0.1,
            .drive_ki = 0.0,
            .drive_kd = 0.0,

            .frame_offset_x = 28.575_cm,
            .frame_offset_y =
                28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        // Back left
        stingers::swerve::Configuration::Module{
            .turn_type = TALON_FX,
            .turn_id = 8, // motor type and CAN id
            .turn_ratio = 1.0 / 13.3714,
            .turn_kp = 24.0,
            .turn_ki = 4.0,
            .turn_kd = 0.25,

            .drive_type = TALON_FX,
            .drive_id = 7,
            .drive_ratio = 5.01,
            .drive_kp = 0.1,
            .drive_ki = 0.0,
            .drive_kd = 0.0,

            .frame_offset_x = -28.575_cm,
            .frame_offset_y =
                -28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        // Back right
        stingers::swerve::Configuration::Module{
            .turn_type = TALON_FX,
            .turn_id = 11, // motor type and CAN id
            .turn_ratio = 1.0 / 13.3714,
            .turn_kp = 24.0,
            .turn_ki = 4.0,
            .turn_kd = 0.25,

            .drive_type = TALON_FX,
            .drive_id = 10,
            .drive_ratio = 5.01,
            .drive_kp = 0.1,
            .drive_ki = 0.0,
            .drive_kd = 0.0,

            .frame_offset_x = 28.575_cm,
            .frame_offset_y =
                -28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
    }};
