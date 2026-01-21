// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilLock extends Command {
  /** Creates a new FaceObject. */
  LimeLight limelight;
  Drivetrain drivetrain;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private PIDController pid;

  public AprilLock(LimeLight limeLight, Drivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.limelight = limeLight;
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.pid = new PIDController(0.9, 0.02, 0.05);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // apply deadzone
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.ControllerConstants.STICK_DEADBAND);
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.ControllerConstants.STICK_DEADBAND);
      boolean targetValidity = this.limelight.hasValidTarget();
      if (!targetValidity) {
        System.out.println("Target Not Detected");
        this.drivetrain.simpleDrive(new Translation2d(-translationVal, -strafeVal).times(Constants.DrivetrainConstants.MAX_SPEED), 0);
        // this.drivetrain.simpleDrive(new Translation2d(0, 0), 0);
        return;
      } else {
        System.out.println("Target Detected");
        // get offset in the x direction by 
        double xOffset = this.limelight.getXOffset() * 2 / Constants.VisionConstants.FOV_HORIZONTAL;
        double pidOutput = pid.calculate(xOffset);
        double clampPid = pidOutput > 1.0 ? 1.0 : pidOutput;
        this.drivetrain.simpleDrive(new Translation2d(-translationVal - 0.1 * Math.abs(strafeVal), -strafeVal).times(Constants.DrivetrainConstants.MAX_SPEED), 0.3 * clampPid * Constants.DrivetrainConstants.maxAngularVelocity);
      }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop drivetrain when command ends
    this.drivetrain.simpleDrive(new Translation2d(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
