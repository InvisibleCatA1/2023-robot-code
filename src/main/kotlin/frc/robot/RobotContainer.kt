// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.kauailabs.navx.frc.AHRS
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.PathPoint
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.motorcontrol.Spark
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.commands.*
import frc.robot.constants.ArmConstants
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.IntakeConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.*
import java.nio.file.Path

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)
    val pickAndPlace = PickAndPlaceSubsystem()
    val swerveSubsystem = SwerveSubsystem()
    val trajectories = Trajectories()



    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()


    }
    
    private fun configureButtonBindings() {

//       PRIMARY CONTROLLER:
//       KeyBinds...
//
//       SECONDARY CONTROLLER:
//
//       Left Stick Up = Elbow Down
//       Left Stick Down = Elbow Up
//
//       Right Stick Up = Wrist Down
//       Right Stick Down = Wrist Up
//
//       Left Trigger = Elevator Down
//       Right Trigger = Elevator Up
//
//       Left Bumper = Intake In
//       Right Bumper = Intake Out


       swerveSubsystem.defaultCommand = StandardDrive(swerveSubsystem,
            { primaryController.leftY * -1.0 },
            { primaryController.leftX * -1.0 },
            { primaryController.rightX * -1.0},
            true,
            true)

//        JoystickButton(primaryController, XboxController.Button.kA.value).onTrue(
//            AlignToAprilTag(swerveSubsystem, primaryController)
//        )
//TODO: The alignment to apriltag needs to happen before the pick and place command in sequential command order
//TODO: The alignment to apriltag needs to change to pegs for certain cases, or that needs to be added in in a different way cause right now, it would only place for cube shelves

        //PRIMARY CONtROLLER:
        pickAndPlace.defaultCommand = Base(pickAndPlace)

        JoystickButton(primaryController, XboxController.Button.kX.value).whileTrue(
            RunCommand({
                swerveSubsystem.setX()
            })
        )
        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({swerveSubsystem.zeroGyroAndOdometry()})
        )

        JoystickButton(primaryController, XboxController.Button.kY.value).whileTrue(
            RunCommand({
                swerveSubsystem.zeroGyroAndOdometry()
            })

        )


        //SECONDARY CONTROLLER

        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            LowPickCube(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Button.kB.value).whileTrue(
            LowPickCone(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.kDPadLeft.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            ShelfPick(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Button.kDPadRight.value).whileTrue(
            ShootPick(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            MidPlace(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            HighPlace(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            MidPlace(pickAndPlace)
        )
        JoystickButton(secondaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
//            AlignToAprilTag(swerveSubsystem, secondaryController)
            HighPlace(pickAndPlace)
        )

//        JoystickButton(secondaryController, XboxController.Button.kX.value).whileTrue(
//            RunCommand({
//                swerveSubsystem.setX()
//            })
//        )
//
//        JoystickButton(secondaryController, XboxController.Button.kY.value).whileTrue(
//            RunCommand({
//                swerveSubsystem.zeroGyroAndOdometry()
//            })
//
//        )
//        JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { 4.0 })})
//        )
//
//        JoystickButton(secondaryController, XboxController.Button.kRightBumper.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { 0.0 }, { -4.0 })})
//        )
//
//        JoystickButton(secondaryController, XboxController.Axis.kLeftTrigger.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { primaryController.leftTriggerAxis * -2.0 }, { 0.0 }, { 0.0 }, { 0.0 })})
//        )
//
//        JoystickButton(secondaryController, XboxController.Axis.kRightTrigger.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { primaryController.rightTriggerAxis * 4.0 }, { 0.0 }, { 0.0 }, { 0.0 })})
//        )
//
//        JoystickButton(secondaryController, XboxController.Axis.kLeftY.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { 0.0 }, { primaryController.leftX * -4.0 }, { 0.0 }, { 0.0 })})
//        )
//
//        JoystickButton(secondaryController, XboxController.Axis.kRightY.value).whileTrue(
//            RunCommand({VoltageArm(pickAndPlace, { 0.0 }, { 0.0 }, { primaryController.rightY * -4.0 }, { 0.0 })})
//        )

    }
    val autonomousCommand: Command = RunCommand({trajectories.BlueTop1GetBalance()})
}
