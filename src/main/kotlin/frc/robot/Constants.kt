/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import kotlin.math.PI

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
  object DrivetrainConstants {
    const val LEFT_MASTER_ID = 0 // CAN ID
    const val LEFT_SLAVE_ID = 0 // CAN ID
    const val RIGHT_MASTER_ID = 0 // CAN ID
    const val RIGHT_SLAVE_ID = 0 // CAN ID

    const val DEADBAND = 0.1

    const val WHEEL_DIAMETER = 0.1524 // Meters
    const val WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI // Meters

    const val GEARBOX_RATIO = 58.0 / 14.0
    const val SHAFT_WHEEL_RATIO = 26.0 / 22.0
    const val DISTANCE_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / (GEARBOX_RATIO * SHAFT_WHEEL_RATIO) // Meters
    const val PULSES_PER_REVOLUTION = 42
    const val DISTANCE_PER_PULSE = DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION // Meters

    const val WHEEL_BASE_WIDTH = 0.0 // Meters

    const val MAX_SPEED = 0.0 // Meters per second
    const val MAX_ACCELERATION = 0.0 // Meters per second
    const val MAX_ANGULAR_VELOCITY = 0.0

    const val omegaCorrection = 0.0
  }

  object FlywheelConstants {
    // Put Constants inside the companion object to make them globally accessible.
    // ex. val motorPort: Int = 0
    const val LEFT_MOTOR_ID = 0
    const val RIGHT_MOTOR_ID = 0

    const val GEAR_RATIO = 1.0

    const val MAX_RPM = 5000.0
  }
}
