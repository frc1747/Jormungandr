package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;


public final class VisionUtil {
    public static final class Limelight {
        private static final NetworkTable LIMELIGHT_NT = Constants.Robot.NT_INSTANCE.getTable(Constants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME);

        public static boolean tagExists() {
            return LIMELIGHT_NT.getEntry("tv").getDouble(0.0) == 1.0;
        }
        
        private static double[] getTargetPoseRobotSpaceValues() {
            return LIMELIGHT_NT.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        }

        private static double[] getMegaTagOneValues() {
            return LIMELIGHT_NT.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        }

        public static double getLatency() {
            return (LIMELIGHT_NT.getEntry("tl").getDouble(0.0) + LIMELIGHT_NT.getEntry("cl").getDouble(0.0)) / 1000.0;
        }

        public static boolean isMultiTag() {
            return getNumTags() >= 2;
        }
        
        public static int getNumTags() {
            double[] values = getMegaTagOneValues();

            // The 8th entry in the MegaTag network table is the number of tags the camera detects
            if (values.length < 8) {
                return 0;
            }
            return (int) values[7];
        }

        public static Pose2d getTargetPoseRobotSpace() {
            double[] values = getTargetPoseRobotSpaceValues();

            if (values.length < 6) {
                return new Pose2d();
            }
            return new Pose2d(
                    new Translation2d(values[0], values[2]),
                    new Rotation2d(Units.degreesToRadians(values[5]))
            );
        }
        
        public static double getNearestTagDist() {
            return getTargetPoseRobotSpace().getTranslation().getNorm();
        }

        public static boolean isTagClear() {
            return tagExists() && getNearestTagDist() < Constants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST;
        }
        
        public static Pose2d getMegaTagOnePose() {
            double[] values = getMegaTagOneValues();

            if (values.length < 6) {
                return new Pose2d();
            }
            return new Pose2d(
                    new Translation2d(values[0], values[1]),
                    new Rotation2d(Units.degreesToRadians(values[5]))
            );
        }

        public static void setPose(Pose2d robotPose) {
            Pose2d cameraPose = robotPose.plus(robotToCameraOffset);
            if (Limelight.get() != 98) {
                LimelightResetPose.set(new double[] {
                        cameraPose.getX(),
                        cameraPose.getY(),
                        cameraPose.getRotation().getDegrees()
                });
                Limelight.set(2);
            }
        }
        
        public static void configureRobotToCameraOffset() {
            LIMELIGHT_NT.getEntry("camerapose_robotspace_set").setDoubleArray(
                    new double[] {
                            Constants.VisionConstants.LimelightConstants.LL_FORWARD,
                            Constants.VisionConstants.LimelightConstants.LL_RIGHT,
                            Constants.VisionConstants.LimelightConstants.LL_UP,
                            Constants.VisionConstants.LimelightConstants.LL_ROLL,
                            Constants.VisionConstants.LimelightConstants.LL_PITCH,
                            Constants.VisionConstants.LimelightConstants.LL_YAW
                    }
            );
        }

    }


}