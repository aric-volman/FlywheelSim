// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;

public class SetBangBang extends CommandBase {
  BangBangController controller = new BangBangController();

  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);

  private double setpoint;

  Flywheel flywheel;

  /** Creates a new SetRPMPIDF. */
  public SetBangBang(Flywheel fly, double setpoint) {
    this.setpoint = setpoint;
    flywheel = fly;
    controller.setTolerance(0.01);
    addRequirements(fly);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setFlywheelPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //flywheel.setFlywheelVoltage(controller.calculate(flywheel.getAngularVelocityRPM(), setpoint)*0.1 + 0.6*ff.calculate(RPMtoRadS(flywheel.getAngularVelocityRPM()), RPMtoRadS(setpoint), 0.02));
    // Some reminders for BangBang:
    // MathUtil.clamp - CLAMP THE OUTPUT TO 12 V!!!!!1!1!1!!!
    // Multiply bangbang output by 2 to get less action

    flywheel.setFlywheelVoltage(MathUtil.clamp(controller.calculate(flywheel.getAngularVelocityRPM(), setpoint)*2.0 + 0.6*ff.calculate(RPMtoRadS(flywheel.getAngularVelocityRPM()), RPMtoRadS(setpoint), 0.02), 0.0, 12.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.setFlywheelPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double RPMtoRadS(double RPM) {
    return Units.rotationsPerMinuteToRadiansPerSecond(RPM);
  }
}
