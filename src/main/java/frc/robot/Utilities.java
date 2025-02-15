package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Utilities {
    public static Pose2d average(Pose2d a, Pose2d b) {
        Rotation2d aRotation = a.getRotation();
        Rotation2d bRotation = b.getRotation();
        double x = (a.getX() + b.getX()) / 2;
        double y = (a.getY() + b.getY()) / 2;
        double rotationRadians = (aRotation.getRadians() + bRotation.getRadians()) / 2;
        return new Pose2d(x, y, new Rotation2d(rotationRadians));
    }
}
