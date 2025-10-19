# STANLEY CONTROLLER (Basic Level)
## Introduction & Math Based
### Stanley Controller Overview
The **Stanley Controller** is a lateral vehicle controller that computes the **steering angle** to follow a reference trajectory. It uses a **kinematic bicycle model** and takes into account the vehicle's **current velocity and direction**.
- **Heading Error (Qe):** Difference between the vehicle's yaw (`theta`) and the trajectory heading (`theta_p`):
  Qe = Q - Qp
  Qp = tan⁻¹((Y_last - Y_first) / (X_last - X_first))

![Formula Notation]("C:\Users\aktas\Pictures\Screenshots\Screenshot 2025-10-19 161949.png")

- **Cross Track Error (CTE):** Lateral distance from the **front axle** to the closest point on the trajectory. The steering correction increases as this error grows:
  CTE -> tan((ke * efa) / (kv + v))

- **Final Steering Angle:** The sum of heading and cross track corrections. Angles are **normalized between -π and +π** and limited to the **vehicle's steering range**:
  δ = Qe + CTE

-This controller ensures the vehicle follows the path smoothly while correcting both **yaw differences** and **lateral deviations**.
