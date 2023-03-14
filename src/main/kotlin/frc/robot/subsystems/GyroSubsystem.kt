package frc.robot.subsystems

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GyroSubsystem(private val gyro: AHRS): SubsystemBase() {
    fun reset() {
        gyro.reset()
    }

    fun getHeading(type: HeadingType = HeadingType.FUSED) : Float {
        if (type.equals(HeadingType.FUSED)) {
            return gyro.fusedHeading
        } else {
            return gyro.compassHeading
        }
    }



    enum class HeadingType {
        FUSED,
        COMPASS
    }
}