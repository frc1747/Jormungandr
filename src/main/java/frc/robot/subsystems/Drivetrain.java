// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;
  public Pigeon2 gyro;

  public SwerveModule[] swerveModules;
  //public ChassisSpeeds  speed;
  private SwerveModuleState[] swerveModuleStates;

  public Drivetrain() {

    // Create an instance of the gyro, config its parameters, and zero it out, 
    // making whichever direction the robot is facing when robot code is initialized 0
    gyro = new Pigeon2(Constants.DrivetrainConstants.PIGEON_ID);
    gyro.reset();
    zeroGyro();
    //speed = new ChassisSpeeds(0,0,0);


    // Define all swerve modules with the constants defined in Constants.java
    swerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.DrivetrainConstants.FRONT_LEFT.constants),
      new SwerveModule(1, Constants.DrivetrainConstants.FRONT_RIGHT.constants),
      new SwerveModule(2, Constants.DrivetrainConstants.REAR_LEFT.constants),
      new SwerveModule(3, Constants.DrivetrainConstants.REAR_RIGHT.constants)
    };
    // swerveModuleStates = new SwerveModuleState[] {
    //   new SwerveModuleState(),
    //    new SwerveModuleState(),
    //     new SwerveModuleState(),
    //      new SwerveModuleState()
    // };
    

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
    * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
    */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(Constants.DrivetrainConstants.swerveKinematics, getYaw(), getModulePositions());
    /*
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeChassisSpeeds,
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      Constants.AutoConstants.pathFollowerConfig,
      this::shouldFlipPath,
      this // Reference to this subsystem to set requirements
    );

    6.85782
    40,526.388852
    */
    RobotConfig ppConfig = new RobotConfig(
      55, 
      11.859622, 
      new ModuleConfig(
        0.0508, 
        0, 
        1.0, 
        DCMotor.getNEO(0),
        0, 
        1
      ), 
      Constants.DrivetrainConstants.CENTER_TO_WHEEL
    );
    try {
      ppConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeChassisSpeeds,
      this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(3.0, 0.0, 0.0), 
        new PIDConstants(1, 0.0, 0.01)
      ),
      ppConfig,
      this::shouldFlipPath,
      this // Reference to this subsystem to set requirements
    );
  }

  public boolean shouldFlipPath() {
    var result = DriverStation.getAlliance();
    if (result.isEmpty()) {
      DriverStation.reportWarning("Alliance was empty at auto start!", false);
      return false;
    }
    return result.get().equals(Alliance.Red);
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return Constants.DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] states = Constants.DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw())
    );
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DrivetrainConstants.MAX_SPEED);
    for(SwerveModule mod : swerveMods){
      mod.setDesiredState(states[mod.moduleNumber], true);
    }
  }

 public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
   // CODE WITHOUT REASON
    //speed = new ChassisSpeeds(
     //         translation.getX(), 
       //       translation.getY(), 
         //     rotation);
    SwerveModuleState[] swerveModuleStates =
      Constants.DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), 
              translation.getY(), 
              rotation, 
              getYaw()
          )
          : new ChassisSpeeds(
              translation.getX(), 
              translation.getY(), 
              rotation)
          );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DrivetrainConstants.MAX_SPEED);

        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
}        
}

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DrivetrainConstants.MAX_SPEED);

    for (SwerveModule mod : swerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, true);
  }
  

  public void zeroGyro() {
    gyro.setYaw(0.0);
  }

  public void simpleDrive(Translation2d translation, double rotation) {
    drive(translation, rotation, false, true);
  }

  public Rotation2d getYaw() {
    return (Constants.DrivetrainConstants.invertGyro) ? Rotation2d.fromDegrees(180-gyro.getYaw().getValueAsDouble()) : Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : swerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    for (SwerveModule mod : swerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }
  //public ChassisSpeeds getCurrentSpeeds(){
  //  return speed;
  //}

  @Override
  public void periodic() {
    // Update the odometry every robot cycle tick
    swerveOdometry.update(getYaw(), getModulePositions());
    SmartDashboard.putString("Robot Location: ", getPose().getTranslation().toString());
    SmartDashboard.putString("Yaw status", getYaw().toString());

    for (SwerveModule mod : swerveMods) {
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCancoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);  
    }

    //swerveMods[1].setMotorForTest();
  }












  public class SwerveModule {
    // This might go in its own class, but am leaving it here for now

    public int moduleNumber;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANcoder angleEncoder;

    private SparkClosedLoopController driveController;  // Deprecated from SparkMaxPIDController
    private SparkClosedLoopController angleController;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DrivetrainConstants.DRIVE_KS, Constants.DrivetrainConstants.DRIVE_KV, Constants.DrivetrainConstants.DRIVE_KA);


    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
      this.moduleNumber = moduleNumber;
      this.angleOffset = moduleConstants.angleOffset;

      // CANCoder
      angleEncoder = new CANcoder(moduleConstants.cancoderID);  // Fix this deprecation later
      configAngleEncoder();

      // Angle Motor
      angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
      integratedAngleEncoder = angleMotor.getEncoder();
      angleController = angleMotor.getClosedLoopController();
      configAngleMotor();

      // Drive Motor
      driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
      driveEncoder = driveMotor.getEncoder();
      driveController = driveMotor.getClosedLoopController();
      configDriveMotor();

      lastAngle = getState().angle;
    }

    private void configAngleEncoder() {
      angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
      CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
      angleEncoder.getConfigurator().apply(RobotContainer.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
      EncoderConfig encoderConfig = new EncoderConfig();
      encoderConfig.positionConversionFactor(Constants.DrivetrainConstants.angleConversionFactor);

      SparkMaxConfig config = new SparkMaxConfig();
      config
        .smartCurrentLimit(Constants.DrivetrainConstants.angleContinuousCurrentLimit)
        .idleMode(Constants.DrivetrainConstants.angleNeutralMode)
        .inverted(Constants.DrivetrainConstants.angleMotorInvert)
        .apply(encoderConfig)
        .closedLoop
          .pidf(
            Constants.DrivetrainConstants.ANGLE_KP, 
            Constants.DrivetrainConstants.ANGLE_KI, 
            Constants.DrivetrainConstants.ANGLE_KD, 
            Constants.DrivetrainConstants.ANGLE_KF
          );
      config
        .signals
          .primaryEncoderVelocityPeriodMs(500)
          .primaryEncoderPositionPeriodMs(20)
          .analogVoltagePeriodMs(500);

      angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      

      //CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
      resetToAbsolute();
    }

    public void setMotorForTest() {
      angleMotor.set(1.0);
    }

    private void configDriveMotor() {    
      EncoderConfig encoderConfig = new EncoderConfig();
      encoderConfig
        .positionConversionFactor(Constants.DrivetrainConstants.driveConversionPositionFactor)
        .velocityConversionFactor(Constants.DrivetrainConstants.driveConversionVelocityFactor);
      
      SparkMaxConfig config = new SparkMaxConfig();
      config
        .smartCurrentLimit(Constants.DrivetrainConstants.driveContinuousCurrentLimit)
        .idleMode(Constants.DrivetrainConstants.driveNeutralMode)
        .inverted(Constants.DrivetrainConstants.driveMotorInvert)
        .voltageCompensation(Constants.DrivetrainConstants.voltageComp)
        .apply(encoderConfig)
        .closedLoop
          .pidf(
            Constants.DrivetrainConstants.DRIVE_KP, 
            Constants.DrivetrainConstants.DRIVE_KI, 
            Constants.DrivetrainConstants.DRIVE_KD, 
            Constants.DrivetrainConstants.DRIVE_KF
          );
      config
        .signals
          .primaryEncoderVelocityPeriodMs(20)
          .primaryEncoderPositionPeriodMs(20)
          .analogVoltagePeriodMs(50);
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      driveEncoder.setPosition(0.0);

      //CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    }

    public void resetToAbsolute() {
      double absolutePosition = getCancoder().getDegrees() - angleOffset.getDegrees();
      angleMotor.getEncoder().setPosition(absolutePosition);
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
      desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
      setAngle(desiredState);
      setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
      if (isOpenLoop) {
        double percentOutput = desiredState.speedMetersPerSecond / Constants.DrivetrainConstants.MAX_SPEED;
        driveMotor.set(percentOutput);
      } else {        
        driveController.setReference(
          desiredState.speedMetersPerSecond, 
          ControlType.kVelocity, 
          ClosedLoopSlot.kSlot0, 
          feedforward.calculate(desiredState.speedMetersPerSecond));
      }
    }

    public void setAngle(SwerveModuleState desiredState) {
      // Prevent roating module if speed is less than 1%
      Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle;

      angleController.setReference(angle.getDegrees(), ControlType.kPosition);
      lastAngle = angle;
    }    

    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCancoder() {
      return Rotation2d.fromDegrees(angleEncoder.getPosition().getValue().in(Degrees));
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }
 


  }

  public class OnboardModuleState {

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
     * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
     * support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
      double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
      double targetSpeed = desiredState.speedMetersPerSecond;
      double delta = targetAngle - currentAngle.getDegrees();
      if (Math.abs(delta) > 90) {
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
      }
      return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }
  
    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      // scopeReference = scopeReference % 360;
      if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
      } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
        newAngle += 360;
      }
      while (newAngle > upperBound) {
        newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
        newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
      }
      return newAngle;
    }
  }
}
