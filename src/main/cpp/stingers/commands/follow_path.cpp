/**
*   Path following command declarations for FRC #8193's FRC 2026 season chassis.
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

#include <iostream>
#include <stingers/commands/follow_path.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <stingers/util.hpp>
#include <algorithm>
#include <vector>
#include <numbers>

namespace stingers {

static float cross(glm::vec2 a, glm::vec2 b) {
  return a.x * b.y - b.x * a.y;
}

float FollowPath::calc_vmax(float t) {
  static const float EPS = .001;
  
  glm::vec2 path_last    = this->path? this->path->sample_position(t-EPS) : glm::vec2(0);
  glm::vec2 path_current = this->path? this->path->sample_position(t) : glm::vec2(0);
  glm::vec2 path_next    = this->path? this->path->sample_position(t+EPS) : glm::vec2(0);

  glm::vec2 dpdt = (path_next-path_last) * glm::vec2(1.0 / (EPS * 2.0));
  glm::vec2 d2pdt2 = (path_next - 2.0f * path_current + path_last) / (EPS * EPS);

  float kappa = cross(dpdt, d2pdt2) / powf(glm::length(dpdt), 3.0f);
  float vmax = sqrt((float)robot_linear_accel / fmax(1e-4, fabsf(kappa)));

  return vmax;
}

float FollowPath::find_nearest_t() {
  const int MAX_ITER = 5;
  const float EPSILON = 1e-4f;
  const float DT = 0.01f; // small delta for finite difference

  float nearest_t = 0.0f;
  float nearest_dist2 = 1e+9;

  glm::vec2 R = this->navi.get_frame_position();    // robot current position

  // avoid excessive constructor calls
  glm::vec2 B_forward, B_backward, dB, B_current, ddB, diff;
  
  const int NUM_CHECKS = 20;
  for (int j = 0; j < NUM_CHECKS; ++j) {
    float t = (float)j / (float)NUM_CHECKS;
    for (int i = 0; i < MAX_ITER; ++i) {
      // finite differences for first derivative
      float t_forward = std::min(t + DT, 1.0f);
      float t_backward = std::max(t - DT, 0.0f);

      B_forward = this->path->sample_position(t_forward);
      B_backward = this->path->sample_position(t_backward);

      dB = (B_forward - B_backward) * (0.5f / DT);

      // finite differences for second derivative
      B_current = this->path->sample_position(t);
      ddB = (B_forward - B_current * 2.0f + B_backward) * (1.0f / (DT * DT));

      diff = B_current - R;

      float f_prime = 2.0f * (diff.x * dB.x + diff.y * dB.y);
      float f_double_prime = 2.0f * (dB.x*dB.x + dB.y*dB.y + diff.x*ddB.x + diff.y*ddB.y);

      if (std::abs(f_double_prime) < 1e-6f)
        break;

      float t_next = t - f_prime / f_double_prime;
      t_next = std::clamp(t_next, 0.0f, 1.0f);

      if (fabsf(t_next - t) < EPSILON) {
        break;
      }

      t = t_next;
    }
    float dist2 = diff.x * diff.x + diff.y * diff.y;
    if (dist2 < nearest_dist2) {
      nearest_t = t;
      nearest_dist2 = dist2;
    }
  }

  return nearest_t;
}

void FollowPath::Execute() {
  static const float dt = loop_time;
  static const float EPS = .001;
  
  this->t = this->find_nearest_t();

  //if (t > 1.0 - (2.0 * EPS)) {
  //  this->time = 0.0f;
  //}

  frc::SmartDashboard::PutBoolean("path finished", t > 0.9);

  glm::vec2 path_last    = this->path? this->path->sample_position(t-EPS) : glm::vec2(0);
  glm::vec2 path_current = this->path? this->path->sample_position(t) : glm::vec2(0);
  glm::vec2 path_next    = this->path? this->path->sample_position(t+EPS) : glm::vec2(0);

  glm::vec2 dpdt = (path_next-path_last) * glm::vec2(2.0 / EPS);

  if (glm::length(dpdt) < EPS) {
    //this->time = 0.0f;
    //return;
  }

  float cur_vel = glm::length(this->navi.get_frame_velocity_fieldspace());

  float stopping_distance = powf(cur_vel, 2.0f) / (2.0 * (float)robot_linear_accel);

  frc::SmartDashboard::PutNumber("stopping distance", stopping_distance);

  float t_stop = t+1.5 * stopping_distance/glm::length(dpdt); // assuming arc-length parameterization
  
  const int LOOKAHEAD_SAMPLES = 4;
  const float accel_safety = 1.0;
  const float vmax_safety = 1.0;

  float min_accel = (float)robot_linear_accel * accel_safety;
  float min_accel_vel = this->calc_vmax(t) * vmax_safety;
  float min_accel_dist = 0.0f;

  std::vector<frc::Pose2d> poses;
  poses.reserve(LOOKAHEAD_SAMPLES);


  for (int i = 1; i <= LOOKAHEAD_SAMPLES; ++i) {
    float t_samp = (float)i/(float)LOOKAHEAD_SAMPLES;
    float t_loc = (t_stop-t)*t_samp + t;
    if (t_loc >= 1.0f && !this->stop_at_end) break;

    float targ_vel = 0.0f;

    if (t_loc < 1.0f) {
      targ_vel = this->calc_vmax(t_loc) * vmax_safety;
    }

    float targ_accel = (targ_vel*targ_vel - cur_vel*cur_vel) / (t_samp * 2.0 * stopping_distance);

    if (targ_accel < min_accel) {
      min_accel = targ_accel;
      min_accel_vel = targ_vel;
      min_accel_dist = t_samp * stopping_distance;
    }

    glm::vec2 p = this->path->sample_position(fminf(t_loc, 1.0f));

    poses.emplace_back(
      units::meter_t{p.x},
      units::meter_t{p.y},
      frc::Rotation2d{}
    );
  }

  this->navi.get_field().GetObject("path_follow_lookaheads")->SetPoses(poses);

  float vtarg = sqrtf(fmax(0.0f, min_accel_vel*min_accel_vel - 2.0f * min_accel_dist * (float)robot_linear_accel));

  float timescale = vtarg / std::max(1e-2f, glm::length(dpdt));
  glm::vec2 gradient = glm::normalize(dpdt);
  
  glm::vec2 position_setpoint = path_current;

  glm::vec2 target = position_setpoint;
  float angle_setpoint = 0.0;

  glm::vec2 pos = this->navi.get_frame_position();
  float angle = 0.0; // TODO

  //this->position_pid.kD = 1.0 * glm::length(this->navi.get_frame_velocity_fieldspace());

  float ff = 1.0; // velocity feedforward
  // get back to the path if we're too far away
  if (glm::length(position_setpoint - this->navi.get_frame_position()) > 0.2) ff = 0.0f;
  glm::vec2 velocity_setpoint = this->position_pid.update(position_setpoint, pos, dt) + ff * gradient * fminf(vtarg, (float)robot_linear_max_vel * this->aggressiveness);

  frc::SmartDashboard::PutNumber("path follow t", t);
  frc::SmartDashboard::PutNumber("path follow vx", velocity_setpoint.x);
  //float angular_velocity_setpoint = this->angle_pid.update(angle_setpoint, angle, dt);

    // TODO: convert to fieldspace
  this->swerve.drive_framespace(units::meters_per_second_t(velocity_setpoint.y), units::meters_per_second_t(velocity_setpoint.x), units::radians_per_second_t(0.0));
  //this->robot.set_angular_velocity_setpoint(angular_velocity_setpoint);
}

}
