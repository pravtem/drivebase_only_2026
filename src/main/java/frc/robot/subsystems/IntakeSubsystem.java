
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static final int TOP_INTAKE_CAN_ID = 14;
  private static final int BOTTOM_INTAKE_CAN_ID = 13;
  private static final double TOP_FREE_SPEED_RPM = 6784.0;
  private static final double BOTTOM_FREE_SPEED_RPM = 5676.0;
  private static final double TOP_TARGET_MAX_RPM = 5800.0;
  private static final double BOTTOM_TARGET_MAX_RPM = 5000.0;
  private static final double TOP_VELOCITY_KP = 0.0002;
  private static final double BOTTOM_VELOCITY_KP = 0.0002;

  private final SparkFlex topIntakeMotor = new SparkFlex(TOP_INTAKE_CAN_ID, MotorType.kBrushless);
  private final SparkMax bottomIntakeMotor = new SparkMax(BOTTOM_INTAKE_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topIntakeMotor.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomIntakeMotor.getEncoder();
  private final SparkClosedLoopController topController = topIntakeMotor.getClosedLoopController();
  private final SparkClosedLoopController bottomController = bottomIntakeMotor.getClosedLoopController();
  private double topClockwiseSpeed = 1.0;
  private double bottomClockwiseSpeed = -1.0;

  public IntakeSubsystem() {
    configureShooterVelocityControl();
    publishClockwiseSpeeds();
  }

  private void configureShooterVelocityControl() {
    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(TOP_VELOCITY_KP, 0.0, 0.0)
        .outputRange(-1.0, 1.0);
    topConfig.closedLoop.feedForward.kV(12.0 / TOP_FREE_SPEED_RPM);

    SparkMaxConfig bottomConfig = new SparkMaxConfig();
    bottomConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(BOTTOM_VELOCITY_KP, 0.0, 0.0)
        .outputRange(-1.0, 1.0);
    bottomConfig.closedLoop.feedForward.kV(12.0 / BOTTOM_FREE_SPEED_RPM);

    REVLibError topError = topIntakeMotor.configure(topConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    REVLibError bottomError = bottomIntakeMotor.configure(bottomConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    if (topError != REVLibError.kOk) {
      DriverStation.reportWarning("Top shooter motor failed to configure closed-loop velocity control.", false);
    }
    if (bottomError != REVLibError.kOk) {
      DriverStation.reportWarning("Bottom shooter motor failed to configure closed-loop velocity control.", false);
    }
  }

  private void publishClockwiseSpeeds() {
    SmartDashboard.putNumber("Shooter/Top Clockwise Speed", topClockwiseSpeed);
    SmartDashboard.putNumber("Shooter/Bottom Clockwise Speed", bottomClockwiseSpeed);
    SmartDashboard.putNumber("Shooter/Top Target RPM", topClockwiseSpeed * TOP_TARGET_MAX_RPM);
    SmartDashboard.putNumber("Shooter/Bottom Target RPM", bottomClockwiseSpeed * BOTTOM_TARGET_MAX_RPM);
  }

  public void setShooterPower(double normalizedPower) {
    topClockwiseSpeed = MathUtil.clamp(normalizedPower, 0.0, 1.0);
    bottomClockwiseSpeed = -topClockwiseSpeed;
    publishClockwiseSpeeds();
  }

  public void adjustClockwiseSpeeds(double speedDelta) {
    setShooterPower(topClockwiseSpeed + speedDelta);
  }

  private void setTopVelocity(double normalizedSpeed) {
    topController.setSetpoint(normalizedSpeed * TOP_TARGET_MAX_RPM, ControlType.kVelocity);
  }

  private void setBottomVelocity(double normalizedSpeed) {
    bottomController.setSetpoint(normalizedSpeed * BOTTOM_TARGET_MAX_RPM, ControlType.kVelocity);
  }

  public void runBottomClockwise() {
    setBottomVelocity(bottomClockwiseSpeed);
  }

  public void runBottomCounterClockwise() {
    setBottomVelocity(-bottomClockwiseSpeed);
  }

  public void runTopClockwise() {
    setTopVelocity(-topClockwiseSpeed);
  }

  public void runTopCounterClockwise() {
    setTopVelocity(topClockwiseSpeed);
  }

  public void runTriggerIntake() {
     setTopVelocity(topClockwiseSpeed);
     setBottomVelocity(bottomClockwiseSpeed);
  }

  public void runTriggerShoot() {
    setTopVelocity(-topClockwiseSpeed);
    setBottomVelocity(bottomClockwiseSpeed);
  }

  public void stopTop() {
    topIntakeMotor.stopMotor();
  }

  public void stopBottom() {
    bottomIntakeMotor.stopMotor();
  }

  public void stopAll() {
    topIntakeMotor.stopMotor();
    bottomIntakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Actual RPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/Bottom Actual RPM", bottomEncoder.getVelocity());
  }

  public Command bottomClockwiseCommand() {
    return runEnd(this::runBottomClockwise, this::stopBottom);
  }

  public Command bottomCounterClockwiseCommand() {
    return runEnd(this::runBottomCounterClockwise, this::stopBottom);
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
