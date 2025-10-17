
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLightHelpers;
import frc.robot.Constants.VisionConstants;

import javax.sound.midi.SysexMessage;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    String name;
    NetworkTable table;
    NetworkTableEntry xOffsetEntry;
    NetworkTableEntry yOffsetEntry;
    NetworkTableEntry areaEntry;
    NetworkTableEntry poseAmbiguityEntry;
    public LimeLight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        xOffsetEntry = table.getEntry("tx");
        yOffsetEntry = table.getEntry("ty");
        areaEntry = table.getEntry("ta");
        poseAmbiguityEntry = table.getEntry("pa");
    }

    public String getName() {
        return name;
    }

    public double getXOffset() {
        return xOffsetEntry.getDouble(361.0);
    }

    public double getYOffset() {
        return yOffsetEntry.getDouble(69.0);
    }

    public double getAreaOffset() {
        return areaEntry.getDouble(101.0);
    }

    public double getPoseAmbiguity() {
        return poseAmbiguityEntry.getDouble(-1);
    }
    
    public double[] getCrosshairHSV() {
        return LimeLightHelpers.getTargetColor(this.name);
    }

    public void robotInit() {
      for (int port = 5800; port <= 5809; port ++) {
        PortForwarder.add(port+10, "limelight.local", port);
      }
    }
    
    @Override
    public void periodic() {
        double[] currentHSV = getCrosshairHSV();
        SmartDashboard.putNumber("Crosshair Hue", currentHSV[0] * 180);
        SmartDashboard.putNumber("Crosshair Saturation", currentHSV[1] * 255);
        SmartDashboard.putNumber("Crosshair Value", currentHSV[2] * 255);
        if (currentHSV[0] * 180  >= VisionConstants.HSVRange[0][0] && currentHSV[0] * 180 <= VisionConstants.HSVRange[1][0] && currentHSV[1] * 255 >= VisionConstants.HSVRange[0][1] && currentHSV[1]  * 255 <= VisionConstants.HSVRange[1][1] && currentHSV[2] * 255 >= VisionConstants.HSVRange[0][2] && currentHSV[2] * 255 <= VisionConstants.HSVRange[1][2]) {
            System.out.println("YAYAY");
        }
    }    
}