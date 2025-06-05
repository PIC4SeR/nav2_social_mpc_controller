#Critics Overview

This document describes the custom critics used in the `nav2_social_mpc_controller` package. Critics are cost functions that guide the robot's behavior during navigation, ensuring safety, efficiency, and social compliance.

---

## Agent Angle Cost Function

**Purpose:**  
Enforces an angular speed for the robot in certain situations.  
**Behavior:**  
If an agent is on the left side of the robot, the critic encourages steering to the right to follow social norms.

---

## Distance Cost Function

**Purpose:**  
Pushes the robot to follow points on the path.

**Usage:**  
- **Path Align:** If all trajectory points for different time steps are provided, aligns the robot to the path.
- **Path Follow:** If only the final point is given, acts as a path-follow critic.

---

## Goal Align Cost Function

**Purpose:**  
Reduces the angular difference between the robot's orientation and the goal pose orientation.

---

## Obstacle Cost Function

**Purpose:**  
Keeps the robot away from high-cost zones.

**Details:**  
Uses a bicubic interpolator to estimate the cost of a robot's position in future time steps using the local costmap.

---

## Social Work Cost Function

**Purpose:**  
Uses the Social Force Model (SFM) to consider social work as a cost, aiming to minimize the social impact of the robot.

**Note:**  
Directly optimizing social work led to undesirable behavior (robot overlapping with agents). This was mitigated by using a `1/(social work)` cost, and further refined to smooth the forces.

---

## Velocity Cost Function

**Purpose:**  
Keeps the robot's velocity terms near the desired values.

---

## Velocity Feasibility Cost Function

**Purpose:**  
Prevents the optimizer from computing drastically different velocity terms in subsequent time steps.

---

## License

See the repository for license information.