// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

  private ShuffleboardTab flyTab = Shuffleboard.getTab("Flywheel");
  GenericEntry nativeVel = flyTab.add("TickVelocity", 0.0).getEntry();
  GenericEntry nativePos = flyTab.add("TickPos", 0.0).getEntry();
  GenericEntry rpmEntry = flyTab.add("RPM", 0.0).getEntry();

  // Create system from kV and kA
  // DO NOT USE MOI (moment of inertia) TO CREATE SIM FLYWHEEL
  // This approach is unrealistic
  // ONLY USE REAL MEASUREMENTS/FEEDFORWARD
  private final LinearSystem<N1, N1, N1> flywheelPlant =
      LinearSystemId.identifyVelocitySystem(Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);

  private final WPI_TalonSRX flyWheel;
  private final TalonSRXSimCollection flyWheelSimMotor;
  private final FlywheelSim flyWheelSim;

  private static final double kFlywheelGearing = 1.0;

  private double motorVoltSim = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel() {

    // Assigns simulation and motor objects
    flyWheelSim = new FlywheelSim(flywheelPlant, DCMotor.getBag(0), kFlywheelGearing);
    
    flyWheel = new WPI_TalonSRX(Constants.OperatorConstants.flyWheelPort);
    flyWheelSimMotor = flyWheel.getSimCollection();

    flyWheelSimMotor.setQuadratureVelocity(1);

    // Motor settings
    flyWheel.setNeutralMode(NeutralMode.Coast);
    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();
    
    flyWheel.setExpiration(.02);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    nativeVel.setDouble(this.getNativeTicksPer100());
    nativePos.setDouble(this.getNativeTicks());
    rpmEntry.setDouble(flyWheelSim.getAngularVelocityRPM());
  }

  @Override
  public void simulationPeriodic() {
    // Set inputs of voltage
    flyWheelSim.setInput(motorVoltSim);
    // Update with dt of 0.02
    flyWheelSim.update(0.02);

    // Update Quadrature
    // Only velocity can be set - position can't be set

    flyWheelSimMotor.setQuadratureVelocity(
        velocityToNativeUnits(
            flyWheelSim.getAngularVelocityRPM()));
  }

  public void setFlywheelPower(double power) {
    flyWheel.set(power);
    motorVoltSim = power * 12.0;
  }

  public void setFlywheelVoltage(double power) {
    flyWheel.set(power/12.0);
    motorVoltSim = power;
  }


  /**
   * Resets the chassis encoders to 0 ticks.
   */
  public void resetEncoders() {
    flyWheel.setSelectedSensorPosition(0);
  }

  /**
   * Gets native units per 1000 ms
   */
  public double getNativeTicksPer100() {
    return flyWheel.getSelectedSensorVelocity();
  }

  /**
   * Gets native units in ticks
   */
  public double getNativeTicks() {
    return flyWheel.getSelectedSensorPosition();
  }

  /**
   * Gets RPM of flywheel
   * @return RPM of flywheel
   */
  public double getAngularVelocityRPM() {
    return 60.0*((flyWheel.getSelectedSensorVelocity()*10.0)/4096.0);
  }

  // CTRE SIM methods:
  
  private int velocityToNativeUnits(double velocityRPM) {
    double motorRotationsPerSecond = velocityRPM / 60.0;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10.0;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096.0);
    return sensorCountsPer100ms;
  }
}
