// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants;

public class GoToPose2D extends Command {
    PoseEstimatorSubsystem poseEstimator;
    Drivetrain drivetrain;
    Pose2d desiredPose;
    Transform2d difference;
    double diffTransMagnitude;
    double diffRotMagnitude;
    PIDController transPid;
    PIDController rotPid;

  /** Creates a new GoToPoseTest. */
  public GoToPose2D(PoseEstimatorSubsystem poseEstimator, Drivetrain drivetrain, Pose2d desiredPose) {
    this.poseEstimator = poseEstimator;
    this.drivetrain = drivetrain;
    // the pose we want to end up at
    this.desiredPose = desiredPose;
    this.transPid = new PIDController(0.6, 0.01, 0.00);
    this.rotPid = new PIDController(0.01, 0.01, 0);
    addRequirements(poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // transform2d that represents difference between current estimated pose and desired pose
    difference = poseEstimator.getEstimatedPose().minus(desiredPose);
    // magnitude in meters of difference between current estimated pose of robot and the desired pose
    diffTransMagnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // transform2d that represents difference between current estimated pose and desired pose
    difference = desiredPose.minus(poseEstimator.getEstimatedPose());

    // distance to desired pose translation in meters
    diffTransMagnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));

    // Velocity translation and multiplier to be applied
    Translation2d driveTranslation = new Translation2d(Constants.DrivetrainConstants.MAX_SPEED, new Rotation2d(Math.atan2(difference.getY(),difference.getX())));
    double velocityMultiplier = transPid.calculate(-diffTransMagnitude);

    // Angular velocity rotation and multiplier to be applied
    diffRotMagnitude = Constants.DrivetrainConstants.maxAngularVelocity*difference.getRotation().getRadians();
    double angularVelocityMultiplier = rotPid.calculate(-diffRotMagnitude);
    
    drivetrain.simpleDrive(driveTranslation.times(velocityMultiplier), diffRotMagnitude*angularVelocityMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.simpleDrive(new Translation2d(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ends command if the robot is within 10 cm
    if (diffTransMagnitude > 0.02) {
        return false;
    } else {
        return true;
    }
  }
}