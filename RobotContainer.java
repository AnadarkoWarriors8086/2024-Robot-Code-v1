// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.swervesubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LaunchNote;
import frc.robot.subsystems.PWMLauncher;
import frc.robot.subsystems.PWMClimber;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  swervesubsystem drivebase = new swervesubsystem();
  PWMLauncher m_launcher = new PWMLauncher();
  PWMClimber m_climber = new PWMClimber();
  CommandXboxController driverXbox = new CommandXboxController(0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
    //     () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
    //     () -> -MathUtil.applyDeadband(driverXbox.getRightX(), 0.1),
    //     () -> -MathUtil.applyDeadband(driverXbox.getRightY(), 0.1));

    Command driveFieldOrientedAnglulacrVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), 0.1),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.1),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(), 0.1));

        drivebase.setDefaultCommand(driveFieldOrientedAnglulacrVelocity);

    // Configure the trigger bindings
    configureBindings();
  }


  private void configureBindings() {
    //reset robot heading in a match by pressing start.
  driverXbox.start().whileTrue(drivebase.zeroGyro());

  //Start the launch sequence when pressing the right bumper.
  driverXbox.rightBumper()
  .whileTrue(
      new PrepareLaunch(m_launcher)
          .withTimeout(LauncherConstants.kLauncherDelay)
          .andThen(new LaunchNote(m_launcher))
          .handleInterrupt(() -> m_launcher.stop()));
  
  driverXbox.rightTrigger()
  .whileTrue(
      new PrepareLaunch(m_launcher)
          .withTimeout(LauncherConstants.kLauncherDelay)
          .andThen(new LaunchNote(m_launcher))
          .handleInterrupt(() -> m_launcher.stop()));

  //start the intake sequence when pressing the left bumper.
  driverXbox.leftBumper().whileTrue(m_launcher.getIntakeCommand());

  //start the climbing sequence when pressing the y button.
  driverXbox.y().whileTrue(m_climber.getClimbCommand());

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No Auto Command Configured");
  }
}
