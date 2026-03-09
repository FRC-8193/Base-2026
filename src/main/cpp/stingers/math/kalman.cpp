/**
*   Kalman filter definitions for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/math/kalman.hpp>

namespace stingers {


KalmanFilter::KalmanFilter(glm::mat4x4 Q, std::optional<std::reference_wrapper<const KalmanSensor>> init_sensor) : Q(Q) {
  if (init_sensor.has_value()) {
    const KalmanSensor &sensor = init_sensor->get();
    this->state = glm::transpose(sensor.H()) * sensor.z();
    this->covariance[0][0] = sensor.R()[0][0];
    this->covariance[1][1] = sensor.R()[1][1]; // this assumes the init sensor provides position, which it absolutely should
  }
}

void KalmanFilter::update(std::vector<std::reference_wrapper<const KalmanSensor>> sensors, float dt) {
  glm::mat4x4 A = {
  { 1,  0, 0, 0 },
  { 0,  1, 0, 0 },
  { dt, 0, 1, 0 },
  { 0, dt, 0, 1 }, // this encodes the function position = position + velocity * dt
                   // more specifically,

                   // x = x + vx * dt
                   // y = y + vy * dt
                   // vx = vx
                   // vy = vy
  };

  // prediction step
  // multiply the state transition matrix to perform Eulerian integration as described above
  this->state = A * this->state;
  // A * P * Aᵀ is an identity that transforms the state's covariance P according to the state transform matrix A
  // + Q*dt increases variance for environmental uncertainty (jitter, wheelspin, direction changes)
  this->covariance = A * this->covariance * glm::transpose(A) + this->Q * dt;

  // sensor fusion
  for (const KalmanSensor &sensor : sensors) {
    // Observation matrix (H x̂ = ẑ where x̂ is state and ẑ is predicted measurement)
    auto H = sensor.H();
    auto H_T = glm::transpose(H);
    // Sensor covariance (essentially, how much don't we trust it)
    auto R = sensor.R();

    // Kalman gain (how much we'll use the sensor vs our prediction)
    auto K = this->covariance * H_T * glm::inverse(H * this->covariance * H_T + R);

    // Fancy matrix lerp
    // Sensor z is just the measured data
    this->state = this->state + K * (sensor.z() - H * this->state);

    // Update covariance in a similar way (magical Joseph form)
    // (I - K H) P (I - K H)ᵀ + K R Kᵀ
    auto I = glm::mat4(1.0f);

    auto KH = K * H;
    auto I_KH = I - KH;

    this->covariance =
        I_KH * this->covariance * glm::transpose(I_KH)
      + K * R * glm::transpose(K);
  }
}

} // namespace stingers
