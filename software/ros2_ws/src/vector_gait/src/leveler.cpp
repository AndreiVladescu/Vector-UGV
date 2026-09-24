#include "vector_gait/leveler.hpp"

#include <algorithm>

namespace vector
{

void Leveler::update(double roll, double pitch, double dt)
{
  roll_ = std::clamp(roll_ - gain_ * roll * dt, -max_, max_);
  pitch_ = std::clamp(pitch_ - gain_ * pitch * dt, -max_, max_);
}

}  // namespace vector
