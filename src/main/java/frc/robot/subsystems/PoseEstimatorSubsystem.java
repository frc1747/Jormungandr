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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimeLightHelpers;
import frc.robot.RobotContainer;
import frc.robot.util.VisionUtil;

public class PoseEstimatorSubsystem extends SubsystemBase {
    private Drivetrain drivetrain;
    private LimeLight limeLight;
    private Pose2d currentEstimate;

    public SwerveDrivePoseEstimator poseEstimator;

    private boolean hasCalibratedOnceWhenNear = false;

    /** Creates a new PoseEstimatorSubsystem. */
    public PoseEstimatorSubsystem(Drivetrain drivetrain, LimeLight limeLight) {
        this.drivetrain = drivetrain;
        this.limeLight = limeLight;
        currentEstimate = new Pose2d();      // probably needs to be adjested later
    }

    public Pose2d getEstimatedPose() {
        return currentEstimate;
    }

    public void updatePoseEstimation() {
        poseEstimator.update(drivetrain.getYaw(), drivetrain.getModulePositions());
        updateLimelightPoseEstimation();
    }

    public void updateLimelightPoseEstimation() {
        if (VisionUtil.Limelight.isMultiTag() && VisionUtil.Limelight.isTagClear()) {
            Pose2d megaTagPose = VisionUtil.Limelight.getMegaTagOnePose();
            poseEstimator.addVisionMeasurement(
                    megaTagPose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency(),
                    Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(VisionUtil.Limelight.getNearestTagDist())
            );
        }
    }

    public void configureLimelightOffset() {

        // offset the limelight pose from the current estimated robot pose
        VisionUtil.Limelight.setPose(poseEstimator.getEstimatedPosition());
    }

    // This sets the limelight pose relative to the robot pose
    public void updateLimelightPose() {
        // unsure what this does

        // if the tag distance is greater than threshold, signal that we need to re-calibrate
        if (VisionUtil.Limelight.getNearestTagDist() > Constants.VisionConstants.LimelightConstants.MIN_TAG_DIST_TO_BE_FAR) {
            hasCalibratedOnceWhenNear = false;
        }

        // if we need to recalibrate
        if (!hasCalibratedOnceWhenNear) {

            // check whether the estimation is good
            if (VisionUtil.Limelight.isTagClear()
                    // and translation and rotation speeds are 0 (or close to 0 as we should be doing)
                    && this.swerve.getState().Speeds.vxMetersPerSecond == 0  
                    && this.swerve.getState().Speeds.vyMetersPerSecond == 0
                    && Math.abs(this.swerve.getState().Speeds.omegaRadiansPerSecond) == 0) {

                // don't know what this does
                configureLimelightOffset();
                LimeLightHelpers.setCameraPose_RobotSpace();

                // say that we have successfully calibrated and don't need to again
                hasCalibratedOnceWhenNear = true;
            }
        }
    }

    public void updatePoses() {
        updatePoseEstimation();
        updateVisionPose();
    }


    @Override
    public void periodic() {
        // updatePoses








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
            currentEstimate = mt2.pose;
            drivetrain.setPose(currentEstimate);
        } else {
            currentEstimate = drivetrain.getPose();
        }

        RobotContainer.estimatedField.setRobotPose(getEstimatedPose());

        //System.out.println(limeLight.getPoseAmbiguity());
    }
}