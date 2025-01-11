package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import org.ejml.ops.FConvertArrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooting;
  private TalonFX shooting2;

  /** Creates a new Shooter. */
  public Shooter() {
    shooting = new TalonFX(Constants.ShooterConstants.FRONT);
    shooting2 = new TalonFX(Constants.ShooterConstants.FRONT_TWO);
    shooting.setNeutralMode(NeutralModeValue.Brake);
    shooting2.setNeutralMode(NeutralModeValue.Brake);
    shooting.setselected
  }

  public void setShooterPower(double power) {
    shooting.set(power);
    shooting2.set(-power);
  }
  public double getPosition() {
    return shooting.getPosition().getValueAsDouble();
  }

  public double getSpeed() {
    return (shooting.getVelocity().getValueAsDouble()*600 ) / 4096;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", getSpeed());

    if (getSpeed() < Constants.ShooterConstants.FLYWHEEL_HIGH_SPEED) {
      SmartDashboard.putBoolean("Shooter Ready?", true);
    } else {
      SmartDashboard.putBoolean("Shooter Ready?", false);
    }
  }
}
