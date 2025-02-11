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

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight limeLight;
    private LimeLightHelpers.PoseEstimate mt2;
    private SwerveDrivePoseEstimator poseEstimator;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight limeLight) {
        this.drivetrain = drivetrain;
        this.limeLight = limeLight;
        this.mt2 = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLight.getName());
        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.swerveKinematics, 
            drivetrain.getYaw(), 
            drivetrain.getModulePositions(),   
            new Pose2d());                     // neeeds to be adjusted later
    }

    public Pose2d getEstimatedPose() {
        if (mt2.tagCount == 0) {
            return poseEstimator.getEstimatedPosition();
        } else {
            return mt2.pose;
        }
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(System.currentTimeMillis()/1000, drivetrain.gyro.getRotation2d(), drivetrain.getModulePositions());
        LimeLightHelpers.SetRobotOrientation(limeLight.getName(), drivetrain.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        mt2 = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLight.getName());

        boolean rejectVisionUpdate = false;
        if (Math.abs(drivetrain.gyro.getRate()) > 720) { 
            rejectVisionUpdate = true;
        } else if (mt2.tagCount == 0) {
            rejectVisionUpdate = true;
        } 
        if (!rejectVisionUpdate) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            //SmartDashboard.putString("Bot Position MegaTag2", "" + mt2.pose);
            //SmartDashboard.putData(mt2.pose);
            // SmartDashboard.putString("Bot Position Combined", "" + this.getEstimatedPose());
            // drivetrain.setPose(mt2.pose);
            // RobotContainer.combined_field.setRobotPose(drivetrain.getPose());
        }

        RobotContainer.limelight_field.setRobotPose(mt2.pose);
        RobotContainer.combined_field.setRobotPose(this.getEstimatedPose());
    }
}
