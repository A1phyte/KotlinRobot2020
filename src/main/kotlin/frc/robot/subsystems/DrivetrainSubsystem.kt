package frc.robot.subsystems

import com.analog.adis16470.frc.ADIS16470_IMU
import com.revrobotics.CANEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DrivetrainConstants
import frc.robot.util.applyDeadband
import frc.robot.util.constrain
import kotlin.math.abs

object DrivetrainSubsystem : SubsystemBase() {
  private val leftMasterMotor = CANSparkMax(DrivetrainConstants.LEFT_MASTER_ID, MotorType.kBrushless)
  private val leftSlaveMotor = CANSparkMax(DrivetrainConstants.LEFT_SLAVE_ID, MotorType.kBrushless)
  private val rightMasterMotor = CANSparkMax(DrivetrainConstants.RIGHT_MASTER_ID, MotorType.kBrushless)
  private val rightSlaveMotor = CANSparkMax(DrivetrainConstants.RIGHT_SLAVE_ID, MotorType.kBrushless)

  private val leftMasterEncoder = CANEncoder(leftMasterMotor)
  private val leftSlaveEncoder = CANEncoder(leftSlaveMotor)
  private val rightMasterEncoder = CANEncoder(rightMasterMotor)
  private val rightSlaveEncoder = CANEncoder(rightSlaveMotor)

  private val encoders = arrayOf(leftMasterEncoder, leftSlaveEncoder, rightMasterEncoder, rightSlaveEncoder)

  private val odometry = DifferentialDriveOdometry(Rotation2d())
  private val kinematics = DifferentialDriveKinematics(DrivetrainConstants.WHEEL_BASE_WIDTH)

  private val imu = ADIS16470_IMU()

  init {
    leftSlaveMotor.follow(leftMasterMotor)
    rightSlaveMotor.follow(rightMasterMotor)
    arrayOf(leftMasterEncoder, leftMasterEncoder, rightMasterEncoder, rightSlaveEncoder).forEach {
      it.velocityConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION
      it.positionConversionFactor = DrivetrainConstants.DISTANCE_PER_REVOLUTION
    }
  }

  /**
   *
   * @param throttle positive is forward.
   * @param twist positive is counterclockwise.
   * @param squareInputs square inputs for finer controls or not
   */
  fun arcadeDrive(throttle: Double, twist: Double, squareInputs: Boolean = true) {
    val leftOutput =
        if (squareInputs) {
          constrain(
              value = applyDeadband(value = throttle * abs(throttle), deadband = DrivetrainConstants.DEADBAND)
                  + applyDeadband(value = twist * abs(twist), deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12
        }
        else {
          constrain(
              value = applyDeadband(value = throttle, deadband = DrivetrainConstants.DEADBAND)
                  + applyDeadband(value = twist, deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12
        };
    val rightOutput =
        if (squareInputs) {
          constrain(
              value = applyDeadband(value = throttle * abs(throttle), deadband = DrivetrainConstants.DEADBAND)
                  - applyDeadband(value = twist * abs(twist), deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        }
        else {
          constrain(
              value = applyDeadband(value = throttle, deadband = DrivetrainConstants.DEADBAND)
                  - applyDeadband(value = twist, deadband = DrivetrainConstants.DEADBAND),
              max = 1.0) * 12.0
        };
    leftMasterMotor.setVoltage(leftOutput)
    rightMasterMotor.setVoltage(rightOutput)
  }

  @Override
  override fun periodic() {
    odometry.update(Rotation2d(getYaw()), getLeftPosition(), getRightPosition())
  }

  fun fastRecalibrateIMU() {
    imu.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._128ms)
    imu.calibrate()
    imu.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._4s)
  }

  /**
   * @return angular velocity in radians per second
   */
  fun getAngularVelocity(): Double {
    return Units.degreesToRadians(imu.rate)
  }

  /**
   * @return angle in radians. Will go past 2pi
   */
  fun getYaw(): Double {
    return Units.degreesToRadians(imu.angle)
  }

  /**
   * @return total left encoder position in meters
   */
  fun getLeftPosition(): Double {
    return arrayOf(leftMasterEncoder.position, leftSlaveEncoder.position).average()
  }

  /**
   * @return total right encoder position in meters
   */
  fun getRightPosition(): Double {
    return arrayOf(rightMasterEncoder.position, rightSlaveEncoder.position).average()
  }

  fun getPose(): Pose2d {
    return odometry.poseMeters
  }

  fun getRotation(): Rotation2d {
    return Rotation2d.fromDegrees(imu.angle)
  }

  fun resetOdometry(newPose: Pose2d, newAngle: Rotation2d) {
    return odometry.resetPosition(newPose, newAngle)
  }
}