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

#include <stingers/swerve/swerve.hpp>

static const float turn_ratio = 1.0 / 13.3714;
static const float turn_kp = 24.0 ;
static const float turn_ki =  4.0 ;
static const float turn_kd =  0.25;

static const float drive_ratio = 1.0 / 5.01;
static const float drive_kp = 0.2;
static const float drive_ki = 0.0;
static const float drive_kd = 0.0;

// Base swerve configuration, excluding navigational stuff
const stingers::swerve::Configuration stingers::swerve::swerve_config = {
    // All the modules' settings are here
    .modules = {
        stingers::swerve::Configuration::Module{
            .name = "Front Left",
            .turn_type = TALON_FX,
            .turn_id = 2, // motor type and CAN id
            .turn_ratio = turn_ratio,
            .turn_kp = turn_kp,
            .turn_ki = turn_ki,
            .turn_kd = turn_kd,

            .drive_type = TALON_FX,
            .drive_id = 1,
            .drive_ratio = drive_ratio,
            .drive_kp = drive_kp,
            .drive_ki = drive_ki,
            .drive_kd = drive_kd,

            .frame_offset_x = -28.575_cm,
            .frame_offset_y =  28.575_cm,
                  // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        stingers::swerve::Configuration::Module{
            .name = "Front Right",
            .turn_type = TALON_FX,
            .turn_id = 5, // motor type and CAN id
            .turn_ratio = turn_ratio,
            .turn_kp = turn_kp,
            .turn_ki = turn_ki,
            .turn_kd = turn_kd,

            .drive_type = TALON_FX,
            .drive_id = 4,
            .drive_ratio = drive_ratio,
            .drive_kp = drive_kp,
            .drive_ki = drive_ki,
            .drive_kd = drive_kd,


            .frame_offset_x = 28.575_cm,
            .frame_offset_y =
                28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        stingers::swerve::Configuration::Module{
            .name = "Back Left",
            .turn_type = TALON_FX,
            .turn_id = 8, // motor type and CAN id
            .turn_ratio = turn_ratio,
            .turn_kp = turn_kp,
            .turn_ki = turn_ki,
            .turn_kd = turn_kd,

            .drive_type = TALON_FX,
            .drive_id = 7,
            .drive_ratio = drive_ratio,
            .drive_kp = drive_kp,
            .drive_ki = drive_ki,
            .drive_kd = drive_kd,

            .frame_offset_x = -28.575_cm,
            .frame_offset_y =
                -28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
        stingers::swerve::Configuration::Module{
            .name = "Back Right",
            .turn_type = TALON_FX,
            .turn_id = 11, // motor type and CAN id
            .turn_ratio = turn_ratio,
            .turn_kp = turn_kp,
            .turn_ki = turn_ki,
            .turn_kd = turn_kd,

            .drive_type = TALON_FX,
            .drive_id = 10,
            .drive_ratio = drive_ratio,
            .drive_kp = drive_kp,
            .drive_ki = drive_ki,
            .drive_kd = drive_kd,

            .frame_offset_x = 28.575_cm,
            .frame_offset_y =
                -28.575_cm, // you can use other units like _in for inches or _m for meters
            .wheel_diameter = 8.75_cm},
    },
    .min_speed = 0.05_mps,
};
