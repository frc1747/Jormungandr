// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotIntake extends SubsystemBase {
  private TalonFX hinge;

  /** Creates a new Intake. */
  public PivotIntake() {
    hinge = new TalonFX(Constants.IntakeConstants.HINGE);
    configPID();
    hinge.setNeutralMode(NeutralMode.Brake);
  }

  public void configPID() {
    double[] pidf = new double[] {0.05, 0.0, 0.0, 0};
    hinge.config_kP(0, pidf[0]);
    hinge.config_kI(0, pidf[1]);
    hinge.config_kD(0, pidf[2]);
    hinge.config_kF(0, pidf[3]);
  }

  public void setHingePower(double power) {
    if (getPosition() > Constants.IntakeConstants.DROPPED && power >= 0) {
      hinge.set(ControlMode.PercentOutput, 0.0);
    } else if (power < 0 && getPosition() <= Constants.IntakeConstants.DROPPED / 3) {
      hinge.set(ControlMode.PercentOutput, power * Constants.IntakeConstants.IN_SLOW_FACTOR);
    } else {
      hinge.set(ControlMode.PercentOutput, power);
    }
  }

  public void liftIntake() {
    hinge.set(ControlMode.Position, Constants.IntakeConstants.STOWED);
  }

  public void dropIntake() {
    hinge.set(ControlMode.Position, Constants.IntakeConstants.DROPPED);
  }

  public void setEncoderPos(double position) {
    hinge.setSelectedSensorPosition(position);
  }

  public double getPosition() {
    // System.out.println(hinge.getSelectedSensorPosition());
    return hinge.getSelectedSensorPosition();
  }

  public boolean switchPressed() {
    return hinge.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    /*
    boolean reverseLimitClosed = hinge.isRevLimitSwitchClosed() == 1;
    if (reverseLimitClosed) {
      setEncoderPos(0);
    }
    */
    // This method will be called once per scheduler run
    //System.out.println(this.getPosition());
    SmartDashboard.putNumber("Intake Pivot Encoder", getPosition());
    if (switchPressed()) {
      setEncoderPos(0.0);
    }
  }
}

