// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;

public class SetPID extends CommandBase {
  private PIDController pid = new PIDController(0.0001, 0, 0.000001);

  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);

  private double setpoint;

  Flywheel flywheel;

  /** Creates a new SetRPMPIDF. */
  public SetPID(Flywheel fly, double setpoint) {
    this.setpoint = setpoint;
    flywheel = fly;
    addRequirements(fly);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setFlywheelPower(MathUtil.clamp(pid.calculate(flywheel.getAngularVelocityRPM(), setpoint) + 0.7*ff.calculate(RPMtoRadS(flywheel.getAngularVelocityRPM()), RPMtoRadS(setpoint), 0.02)/12.0, 0.0, 1.0));
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

  // Helper methods to the rescue! The Units class is so useful!!
  public double RPMtoRadS(double RPM) {
    return Units.rotationsPerMinuteToRadiansPerSecond(RPM);
  }

}
