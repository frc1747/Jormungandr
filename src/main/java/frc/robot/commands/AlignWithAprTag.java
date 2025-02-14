// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;

public class AlignWithAprTag extends Command {
  private LimeLight limeLight;
  private Drivetrain drivetrain;
  private Joystick controller;
  private int direction;
  
  /** Creates a new LockOn. */
  public AlignWithAprTag(Drivetrain drivetrain, LimeLight limeLight, Joystick controller, int direction) {
    this.limeLight = limeLight;
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.direction = direction;
    addRequirements(drivetrain, limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /* curve such that an x value of 0.0
  * results in a y value of 0.0, and
  * as x goes towards infinity, 
  * y approaches 1.0.
  */

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Strafe Code

    // using both 4 and 7
    
    // final int tagIDchoice1 = 4;
    // int index1 = limeLight.getTagIndex(tagIDchoice1);

    // final int tagIDchoice2 = 7;
    // int index2 = LimeLight.getTagIndex(tagIDchoice2);

    // int index = -1;

    // if (index1 != -1) {
    //   index = index1;
    // } else if (index2 != -1) {
    //   index = index2;
    // }

    double translationY = Constants.VisionConstants.alignDistance*this.direction;
    double translationX = 0.0;

    Translation2d translation = new Translation2d(translationX,translationY);

    double rotation = 0.0;
    // double rotation = -controller.getRawAxis(XboxController.Axis.kRightX.value) * Constants.DrivetrainConstants.maxAngularVelocity;
    // double rotation = 0.0;

    drivetrain.simpleDrive(
      translation,
      rotation
    );

    // possibly use PID here eventually
    // if (index != -1) {
    //   System.out.println(index);
    //   rotation = 0.0;
      
      // double yaw = limeLight.getYaw(index);
      // if (yaw <= -yawTolerance) {
      //     rotation = 0.3 * Constants.DrivetrainConstants.maxAngularVelocity * (-yaw / 23.0);
      // } 
      // else if (yaw >= yawTolerance) {
      //     rotation = -0.3 * Constants.DrivetrainConstants.maxAngularVelocity * (yaw / 23.0);
      // } 

    //   drivetrain.simpleDrive(
    //     translation,
    //     rotation
    //   );

    // } else {
    //   drivetrain.simpleDrive(
    //     translation,
    //     rotation
    //   );
    //};

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.simpleDrive(new Translation2d(0.0,0.0), 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}