// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private SwerveDrivePoseEstimator poseEstimator;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.DrivetrainConstants.swerveKinematics, 
            drivetrain.getYaw(), 
            drivetrain.getModulePositions(),   
            new Pose2d());
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(System.currentTimeMillis()/1000,drivetrain.getYaw(),drivetrain.getModulePositions());
        System.out.println(this.getEstimatedPose());
    }
}
