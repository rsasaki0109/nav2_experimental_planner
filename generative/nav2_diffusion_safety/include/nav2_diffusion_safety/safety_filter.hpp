// Copyright 2026 Nav2PlannerBattle contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_DIFFUSION_SAFETY__SAFETY_FILTER_HPP_
#define NAV2_DIFFUSION_SAFETY__SAFETY_FILTER_HPP_

#include <string>

#include "nav2_diffusion_core/trajectory.hpp"

namespace nav2_diffusion_safety
{

/// Result of evaluating one trajectory candidate against a safety filter.
struct SafetyResult
{
  bool safe{true};
  std::string rejection_reason;  ///< empty when safe

  static SafetyResult accepted() {return SafetyResult{true, ""};}
  static SafetyResult rejected(const std::string & reason)
  {
    return SafetyResult{false, reason};
  }
};

/// Abstract hard-safety filter.
///
/// The learned planner is never the final authority (docs/safety.md section
/// 8.1): every candidate must pass the deterministic filters before it can be
/// executed. Implementations must be deterministic and GPU-independent so that
/// the robot stays safe even when the inference backend fails
/// (docs/deployment.md section 11.3).
class SafetyFilter
{
public:
  virtual ~SafetyFilter() = default;

  /// Short identifier used in diagnostics and rejection reasons.
  virtual std::string name() const = 0;

  /// Evaluate a single candidate trajectory.
  virtual SafetyResult check(const nav2_diffusion_core::Trajectory & trajectory) const = 0;
};

}  // namespace nav2_diffusion_safety

#endif  // NAV2_DIFFUSION_SAFETY__SAFETY_FILTER_HPP_
