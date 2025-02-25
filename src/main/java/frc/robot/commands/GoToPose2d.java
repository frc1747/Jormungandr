// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class GoToPose2d extends Command {
    PoseEstimatorSubsystem poseEstimator;
    Drivetrain drivetrain;
    Pose2d desiredPose;
    Transform2d difference;
    double diffTransMagnitude;
    double diffAngle;
  
  /** Creates a new GoToPose2d. */
  public GoToPose2d(PoseEstimatorSubsystem poseEstimator, Drivetrain drivetrain, Pose2d desiredPose) {
    this.poseEstimator = poseEstimator;
    this.drivetrain = drivetrain;
    // the pose we want to end up at
    this.desiredPose = desiredPose;
    addRequirements(poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // transform2d that represents difference between current estimated pose and desired pose
    difference = poseEstimator.getEstimatedPose().minus(desiredPose);
    // magnitude in meters of difference between current estimated pose of robot and the desired pose
    diffTransMagnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));
    // distance to desired pose angle in radians
    diffAngle = difference.getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // transform2d that represents difference between current estimated pose and desired pose
    difference = poseEstimator.getEstimatedPose().minus(desiredPose);
    System.out.println(difference);
    
    // distance to desired pose translation in meters
    diffTransMagnitude = Math.sqrt(Math.pow(difference.getX(), 2) + Math.pow(difference.getY(), 2));

    // distance to desired pose angle in radians
    diffAngle = difference.getRotation().getRadians();
    
    // velocityMultiplier [0, 1] tells robot how quickly to move towards desired pose translation
    double velocityMultiplier = diffTransMagnitude / 3; // starts slowing down within 3 meters
    if (velocityMultiplier > 1) velocityMultiplier = 1;

    // angularVelocityMultiplier [-1, 1] tells robot how quickly to move towards desired pose rotation
    double angularVelocityMultiplier = diffAngle / (Math.PI / 4); // starts slowing down within pi/4 radians
    if (angularVelocityMultiplier > 1) angularVelocityMultiplier = 1;
    if (angularVelocityMultiplier < -1) angularVelocityMultiplier = -1;

    // full speed translation in the direction of the desired pose translation
    Translation2d driveTranslation;
    if (difference.getX() >= 0) {
      driveTranslation = new Translation2d(1, new Rotation2d(-Math.atan(difference.getY()/difference.getX())));
    } else {
      driveTranslation = new Translation2d(1, new Rotation2d(Math.atan(difference.getY()/difference.getX())));
    }

    drivetrain.simpleDrive(driveTranslation.times(velocityMultiplier), angularVelocityMultiplier);
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.simpleDrive(new Translation2d(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ends command if the robot is within 2 cm and 1 degree of desired pose 
    if (diffTransMagnitude > 0.02) {
        return false;
    } else if (Math.abs(diffAngle) > Math.PI / 180){
        return false;
    } else {
        return true;
    }
  }
}
