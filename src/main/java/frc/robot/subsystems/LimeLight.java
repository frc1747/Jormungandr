
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public LimeLight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        xOffsetEntry = table.getEntry("tx");
        yOffsetEntry = table.getEntry("ty");
        areaEntry = table.getEntry("ta");
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

    public void robotInit() {
      for (int port = 5800; port <= 5809; port ++) {
        PortForwarder.add(port+10, "limelight.local", port);
      }
    }

    public void updateTargetsList() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateTargetsList'");
    }

    public static int getTagIndex(int apriltag) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTagIndex'");
    }

    public double getYaw(int index) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getYaw'");
    }
    
}