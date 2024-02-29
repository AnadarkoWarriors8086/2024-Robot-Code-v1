// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public class swervesubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;


  /** Creates a new swervesubsystem. */
  public swervesubsystem() {
SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
  {
    double maximumSpeed = Units.feetToMeters(12.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "neo/swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }catch(
  Exception e)
  {
    throw new RuntimeException(e);
  }
}
  
//  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
//                               DoubleSupplier headingY)
//   {
//     return run(() -> {
//       double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
//       double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
//       // Make the robot move
//       swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
//                                                                       headingX.getAsDouble(),
//                                                                       headingY.getAsDouble(),
//                                                                       swerveDrive.getYaw().getRadians(),
//                                                                       swerveDrive.getMaximumVelocity()));
//     });
//   }
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    swerveDrive.setHeadingCorrection(true);
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  public Command zeroGyro() {
    return runOnce(() -> {
      this.swerveDrive.zeroGyro();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
