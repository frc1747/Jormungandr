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
import frc.robot.LimeLightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight[] limeLights;
    private LimeLightHelpers.PoseEstimate[] mt2s;
    private SwerveDrivePoseEstimator poseEstimator;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight... limeLights) {
        this.drivetrain = drivetrain;
        this.limeLights = limeLights;
        mt2s = new LimeLightHelpers.PoseEstimate[limeLights.length];
        for (int i = 0; i < limeLights.length; i++) {
            mt2s[i] = LimeLightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLights[i].getName());
        }
        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.swerveKinematics, 
            drivetrain.getYaw(), 
            drivetrain.getModulePositions(),   
            new Pose2d());                     // neeeds to be adjusted later
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(System.currentTimeMillis()/1000,drivetrain.getYaw(),drivetrain.getModulePositions());
        
        for (LimeLight limeLight : limeLights) {
            LimeLightHelpers.SetRobotOrientation(limeLight.getName(), poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }
        System.out.println(this.getEstimatedPose());
        
        boolean rejectVisionUpdate = false;
        if (Math.abs(drivetrain.gyro.getRate()) > 720) { 
            rejectVisionUpdate = true;
        }
        
        if (!rejectVisionUpdate) {
            for (LimeLightHelpers.PoseEstimate mt2 : mt2s) {
                if (mt2.tagCount == 0) {
                    rejectVisionUpdate = true;
                } 
                if (!rejectVisionUpdate) {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                }
            }
        }
    }
}
