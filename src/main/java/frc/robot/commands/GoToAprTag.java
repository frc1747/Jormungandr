// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;

/** An example command that uses an example subsystem. */
public class GoToAprTag extends Command {
      private final int AprilTag = 0;
      @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
      private final LimeLight limelight;
      private final Drivetrain drivetrain;
      private final Joystick controller;
      private final List<Pose2d> poseList;

      /**
       * Creates a new ExampleCommand.
       *
       * @param limelight The subsystem used by this command.
       */
      public GoToAprTag(LimeLight limelight, Drivetrain drivetrain, Joystick controller) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.controller = controller;
      
        this.poseList = Constants.VisionConstants.poseList;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(limelight, drivetrain);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {}
    
      @Override
      public void execute() {
        System.out.println(drivetrain.getPose());

        // final double yawTolerance = 0.0;

        // Pose2d bot2d = drivetrain.getPose();
        // Pose2d nearest = bot2d.nearest(this.poseList);

        // drivetrain.simpleDrive(nearest.getTranslation(), nearest.getRotation().getDegrees());

      }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     LimeLightHelpers.PoseEstimate mt1 = LimeLightHelpers. getBotPoseEstimate_wpiBlue("limelight");

//       if (mt1.tagCount ==  1 && mt1.rawFiducials.length == 1) {
//         if (mt1.rawFiducials[0].ambiguity > .7) {
//           boolean doRejectUpdate = true;
//         }

//         if (mt1.rawFiducials[0].distToCamera > 3) {
//           boolean doRejectUpdate = true;
//         }
//       }

//       if (mt1.tagCount == 0) {
//         boolean doRejectUpdate = true;
//       }

//       boolean doRejectUpdate;
//             if(!doRejectUpdate) {
//         Object m_poseEstimator;
//                 m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 999999));
//                 m_poseEstimator.addVisionMeasurement (
//                   mt1.pose,
//                   mt1.timestampsSeconds
//                 );
//       }
//   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}