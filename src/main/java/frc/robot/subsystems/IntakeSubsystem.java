
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final int INTAKE_CAN_ID = 13;
  private static final int TOP_SHOOTER_CAN_ID = 14;
  private static final int BOTTOM_SHOOTER_CAN_ID = 15;
  private static final double INTAKE_POWER = 1.0;
  private static final double SHOOTER_FREE_SPEED_RPM = 5676.0;
  private static final double SHOOTER_TARGET_MAX_RPM = 5000.0;
  private static final double SHOOTER_VELOCITY_KP = 0.0002;

  private final SparkMax intakeMotor = new SparkMax(INTAKE_CAN_ID, MotorType.kBrushless);
  private final SparkMax topShooterMotor = new SparkMax(TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
  private final SparkMax bottomShooterMotor = new SparkMax(BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topShooterMotor.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomShooterMotor.getEncoder();
  private final SparkClosedLoopController topController = topShooterMotor.getClosedLoopController();
  private final SparkClosedLoopController bottomController = bottomShooterMotor.getClosedLoopController();
  private double topClockwiseSpeed = 1.0;
  private double bottomClockwiseSpeed = 1.0;

  public IntakeSubsystem() {
    configureIntakeMotor();
    configureShooterVelocityControl();
    publishClockwiseSpeeds();
  }

  private void configureIntakeMotor() {
    REVLibError intakeError = intakeMotor.configure(new SparkMaxConfig(),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    if (intakeError != REVLibError.kOk) {
      DriverStation.reportWarning("Intake motor failed to configure.", false);
    }
  }

  private void configureShooterVelocityControl() {
    REVLibError topError = configureShooterMotor(topShooterMotor);
    REVLibError bottomError = configureShooterMotor(bottomShooterMotor);

    if (topError != REVLibError.kOk) {
      DriverStation.reportWarning("Top shooter motor failed to configure closed-loop velocity control.", false);
    }
    if (bottomError != REVLibError.kOk) {
      DriverStation.reportWarning("Bottom shooter motor failed to configure closed-loop velocity control.", false);
    }
  }

  private REVLibError configureShooterMotor(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(SHOOTER_VELOCITY_KP, 0.0, 0.0)
        .outputRange(-1.0, 1.0);
    config.closedLoop.feedForward.kV(12.0 / SHOOTER_FREE_SPEED_RPM);

    return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void publishClockwiseSpeeds() {
    SmartDashboard.putNumber("Shooter/Top Clockwise Speed", topClockwiseSpeed);
    SmartDashboard.putNumber("Shooter/Bottom Clockwise Speed", bottomClockwiseSpeed);
    SmartDashboard.putNumber("Shooter/Top Target RPM", topClockwiseSpeed * SHOOTER_TARGET_MAX_RPM);
    SmartDashboard.putNumber("Shooter/Bottom Target RPM", bottomClockwiseSpeed * SHOOTER_TARGET_MAX_RPM);
  }

  public void setShooterPower(double normalizedPower) {
    topClockwiseSpeed = MathUtil.clamp(normalizedPower, 0.0, 1.0);
    bottomClockwiseSpeed = topClockwiseSpeed;
    publishClockwiseSpeeds();
  }

  public void adjustClockwiseSpeeds(double speedDelta) {
    setShooterPower(topClockwiseSpeed + speedDelta);
  }

  private void setTopVelocity(double normalizedSpeed) {
    topController.setSetpoint(normalizedSpeed * SHOOTER_TARGET_MAX_RPM, ControlType.kVelocity);
  }

  private void setBottomVelocity(double normalizedSpeed) {
    bottomController.setSetpoint(normalizedSpeed * SHOOTER_TARGET_MAX_RPM, ControlType.kVelocity);
  }

  public void runIntakeClockwise() {
    intakeMotor.set(INTAKE_POWER);
  }

  public void runIntakeCounterClockwise() {
    intakeMotor.set(-INTAKE_POWER);
  }

  public void runTopClockwise() {
    setTopVelocity(topClockwiseSpeed);
  }

  public void runTopCounterClockwise() {
    setTopVelocity(-topClockwiseSpeed);
  }

  public void runTriggerIntake() {
    runIntakeCounterClockwise();
    setTopVelocity(topClockwiseSpeed);
    setBottomVelocity(bottomClockwiseSpeed);
  }

  public void runTriggerShoot() {
    runIntakeClockwise();
    setTopVelocity(topClockwiseSpeed);
    setBottomVelocity(bottomClockwiseSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void stopTop() {
    topShooterMotor.stopMotor();
  }

  public void stopBottom() {
    bottomShooterMotor.stopMotor();
  }

  public void stopAll() {
    intakeMotor.stopMotor();
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Actual RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/Bottom Actual RPM", bottomEncoder.getVelocity());
  }

  public Command intakeClockwiseCommand() {
    return runEnd(this::runIntakeClockwise, this::stopIntake);
  }

  public Command intakeCounterClockwiseCommand() {
    return runEnd(this::runIntakeCounterClockwise, this::stopIntake);
  }

  public Command topClockwiseCommand() {
    return runEnd(this::runTopClockwise, this::stopTop);
  }

  public Command triggerIntakeCommand() {
    return runEnd(this::runTriggerIntake, this::stopAll);
  }

  public Command triggerShooterCommand() {
    return runEnd(this::runTriggerShoot, this::stopAll);
  }
}
