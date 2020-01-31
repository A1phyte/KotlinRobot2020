package frc.robot.util

import kotlin.math.*

fun applyDeadband(value: Double, deadband: Double, default: Double = 0.0): Double {
  return when {
    abs(value) > deadband -> value
    else -> default
  }
}

fun constrain(value: Double, max: Double, min: Double = -max): Double {
  return when {
    value > max -> max
    value < min -> min
    else -> value
  }
}