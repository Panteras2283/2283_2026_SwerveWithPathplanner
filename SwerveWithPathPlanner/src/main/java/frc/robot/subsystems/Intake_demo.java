// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;


public class Intake_demo extends SubsystemBase {
 /** Creates a new Intake_demo. */
  private TalonFX IntakeMotor = new TalonFX(Constants.intake.motorID);

  double IntakeMotorRPS = IntakeMotor.getVelocity().getValueAsDouble();
  double IntakeMotorRPM = IntakeMotorRPS*60;
  public Intake_demo() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake RPMs", IntakeMotorRPM);
  }

  public void feed(double speed){
    IntakeMotor.set(speed);
  } 

  public void stop(){
    IntakeMotor.set(0);
  }
}
