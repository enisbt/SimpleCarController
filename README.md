# SimpleCarController
A simple Unity car controller based on Unity Standard Assets car controller

# Features

This controller extends the Unity Standard Asset car controller with:

- Better braking
- Antiroll
- Boost zones (Similar to Mario Kart speed pads)

# Usage

This controller requires a certain rigidbody and collider setup. All the colliders and the rigidbody should be in the root GameObject of the car.

### Accurate center of mass

For better stability, the car's center of mass needs to be accurate. You can achieve this by making a rough sketch of the car with multiple box colliders setup like this:

![](https://imgur.com/gw42pP3.png)

### Setting up the wheels

Wheel meshes and colliders need to be in a separate empty object like this:

![](https://imgur.com/mUsItyx.png)

More reading on how to set up wheel colliders: https://docs.unity3d.com/Manual/class-WheelCollider.html

### Setting up the script

![](https://imgur.com/sb7CZ0q.png)

- Max Steer Angle: Maximum allowed angle of wheels while turning. A higher steer angle means faster turning but will make the car unstable.
- Motor Force: The torque applied to wheels at every physics update. (For reference Nissan GT-R R35 has 635 Nm torque applied on 4 wheels)
- Brake Force: The brake torque applied to all wheels at every physics update.
- Top Speed: Currently not implemented
- Anti Roll: Anti-roll force applied to wheels to stabilize the car while turning. A good value for this is the damper value on wheel colliders. More reading: http://projects.edy.es/trac/edy_vehicle-physics/wiki/TheStabilizerBars
- Slip limit: The value to activate traction control. A higher slip limit means more sliding.
- Steering Assist: This enables steering assist. Turns the car around its up vector to achieve smoother steering.
- Boost Zone Multiplier: When a car enters a boost zone, the motor force is multiplied with this value.

Car controller expects wheel colliders and wheel meshes in this order:

- Front left
- Front right
- Back left
- Back right

### Boost zones

Call the `ActivateBoost()` on car controller to activate the speed boost, call `DeactivateBoost()` to cancel it.

# Roadmap

- [ ] More drivetrains (4WD, RWD, AWD)
- [ ] Support more than 4 wheel
- [ ] Better sound
- [ ] Improve gears and engine rpm
- [ ] Proper documentation and video tutorial
