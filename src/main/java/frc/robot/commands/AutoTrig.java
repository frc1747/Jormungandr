package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoTrig extends Command {
    private double speakerheight;
    private double distanceInCentimeters; 
    private double angle; 
    public AutoTrig() { 
        speakerheight = 190.0; 
    }
    public double AutoTrigonometry(double distance){ 
        distanceInCentimeters = distance * 100; 
        angle = Math.atan(speakerheight / distanceInCentimeters);
        return angle;
    }   
}
