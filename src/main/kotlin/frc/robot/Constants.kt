/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.wpilibj.util.Units

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
  companion object DrivetrainConstants {
    // Put Constants inside the companion object to make them globally accessible.
    // ex. val motorPort: Int = 0
    const val LEFT_MASTER_ID = 0
    const val LEFT_SLAVE_ID = 0
    const val RIGHT_MASTER_ID = 0
    const val RIGHT_SLAVE_ID = 0

    const val DEADBAND = 0.1

    const val DISTANCE_PER_REVOLUTION = 0.1524
    const val PULSES_PER_REVOLUTION = 42
    const val DISTANCE_PER_PULSE = DISTANCE_PER_REVOLUTION / PULSES_PER_REVOLUTION

    val WHEEL_BASE_WIDTH: Double = 0.0
  }
}
