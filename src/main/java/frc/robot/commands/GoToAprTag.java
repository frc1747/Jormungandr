// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimeLightHelpers;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class GoToAprTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimeLight limelight;

  /**
   * Creates a new ExampleCommand.
   *
   * @param limelight The subsystem used by this command.
   */
  public GoToAprTag(LimeLight limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
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