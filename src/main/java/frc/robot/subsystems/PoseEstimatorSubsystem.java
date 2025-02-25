// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimeLightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight limeLight;
    private Pose2d currentEstimate;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight limeLight) {
        this.drivetrain = drivetrain;
        this.limeLight = limeLight;
        currentEstimate = new Pose2d();      // probably needs to be adjested later
    }

    public Pose2d getEstimatedPose() {
        return currentEstimate;
    }

    @Override
    public void periodic() {
        LimeLightHelpers.SetRobotOrientation(limeLight.getName(), drivetrain.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        LimeLightHelpers.PoseEstimate mt2 = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLight.getName());

        boolean rejectVisionUpdate = false;
        if (mt2 == null) {
            return;
        }

        if (Math.abs(drivetrain.gyro.getRate()) > 720) { 
            rejectVisionUpdate = true;
        } else if (mt2.tagCount == 0) {
            rejectVisionUpdate = true;
        } 
        if (!rejectVisionUpdate) {
            currentEstimate = Utilities.average(mt2.pose, drivetrain.getPose());
            drivetrain.setPose(currentEstimate);
        } else {
            currentEstimate = drivetrain.getPose();
        }

        RobotContainer.estimatedField.setRobotPose(getEstimatedPose());

        //System.out.println(getEstimatedPose());
    }
}
