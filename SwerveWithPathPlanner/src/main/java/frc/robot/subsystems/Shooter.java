// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX JoshAllen = new TalonFX(Constants.shooter.flywheelID);
  private TalonFX Brandon_aubrey = new TalonFX(Constants.shooter.kickerID);

  
  public Shooter() {

    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = Constants.shooter.slot0P;
    slot0Configs.kI = Constants.shooter.slot0I;
    slot0Configs.kD = Constants.shooter.slot0D;
    slot0Configs.kS = Constants.shooter.slot0S;
    slot0Configs.kV = Constants.shooter.slot0V;
    slot0Configs.kA = Constants.shooter.slot0A;

    JoshAllen.getConfigurator().apply(slot0Configs);
    JoshAllen.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double speed){
    JoshAllen.set(speed);
  } 

  public void kick(double speed){
    Brandon_aubrey.set(speed);
  }

  public void stopShooter(){
    JoshAllen.set(0);
  }

  public void stopKicker(){
    Brandon_aubrey.set(0);
  }
}
