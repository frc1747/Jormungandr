package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  private TalonFX winch;
  boolean limitReversed = false;
  String label;
  double limRev = 1.0;

  /**
   * creates a climber object to control the climber on the robot
   * @author da bois
   */
  public Climber(int id, String label, boolean reversed) {
    this.label = label;
    limitReversed = reversed;

    if (reversed) {
      limRev = -1.0;
    }
    
    winch = new TalonFX(id);
    winch.setNeutralMode(NeutralModeValue.Brake);

    setPids(new double[] {0.4, 0, 0, 0}); 
    
    winch.configPeakOutputForward(0.75); 
    winch.configPeakOutputReverse(-0.75);

	  winch.configMotionCruiseVelocity(9000);
	  winch.configMotionAcceleration(3000);
    winch.configMotionSCurveStrength(0);
  }

  public void cruiseVelocity(int encoderPerSecond) {
    winch.configMotionCruiseVelocity(encoderPerSecond / 10);
  }

  public void acceleration(int encoderPerSecondSqr) {
    winch.configMotionAcceleration(encoderPerSecondSqr / 10);
  }

  public void setPids(double[] pidf) {
    winch.config_kP(0, pidf[0]);
    winch.config_kI(0, pidf[1]);
    winch.config_kD(0, pidf[2]);
    winch.config_kF(0, pidf[3]);
  }

  // sets the encoder to 0
  public void zeroEncoder() {
    winch.setSelectedSensorPosition(0);
  }

  public double getDistance() {
    return winch.getSelectedSensorPosition();
  }

  public double getPosition(){
    return winch.getSelectedSensorPosition();
  }

  public double getSpeed() {
    return winch.getSelectedSensorVelocity();
  }
  
  public double getCurrent() {
    return winch.getSupplyCurrent();
  }
  
  // /**
  //  * @param power The percentage of power being sent to the right winch motor
  //  * @author Foz
  //  */
  public void setPower(double power) {
    // Soft limit stop on reaching top
    if (Math.abs(winch.getSelectedSensorPosition()) > Constants.ClimberConstants.UP_LIMIT && power * limRev >= 0) {
      winch.set(ControlMode.PercentOutput, 0.0);
    
    // Slow down when reaching position when going down
    } else if (Math.abs(winch.getSelectedSensorPosition()) < Constants.ClimberConstants.SLOW_LIMIT && power * limRev <= 0) {
      winch.set(ControlMode.PercentOutput, power / 2);

    // Go full speed
    } else {
      winch.set(ControlMode.PercentOutput, power);
    }
  }

  /**
   * @param setPoint the positional setpoint for the right winch
   * @author Foz
   */
  public void setpoint(double setPoint) {
    winch.set(ControlMode.Position, setPoint);
  }

  public void setMotionMagicSetpoint(double setPoint) {
    winch.set(ControlMode.MotionMagic, setPoint);
  }

  public boolean switchPressed() {
    if (limitReversed) {      
      return winch.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
    }
    return winch.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  /**
   * Extends the large pistons
   * @author Cobob
   */
  public TalonFX[] gibMotors() {
    return new TalonFX[] {winch}; 
  }

  @Override
  public void periodic() {
    if (switchPressed()) {
      winch.setSelectedSensorPosition(0.0);
    }
    SmartDashboard.putNumber("Climber/" + this.label + " Encoder", this.getDistance());
    SmartDashboard.putNumber("Climber/" + this.label + " Current", this.getCurrent());
  }
}
