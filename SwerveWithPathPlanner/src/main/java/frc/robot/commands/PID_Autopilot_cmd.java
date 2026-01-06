// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class PID_Autopilot_cmd extends Command {
  private final CommandSwerveDrivetrain s_Swerve;
  private final Pose2d targetPose;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  // Use FieldCentric request so the robot drives relative to the field
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

  // Get max speeds from TunerConstants
  private final double MAX_SPEED = 4.7;
  // Using the value from RobotContainer
  private final double MAX_ANGULAR_SPEED = Units.rotationsToRadians(0.55); // Convert 0.55 rotations/sec to rad/sec

  // --- TUNING VALUES ---
  // Start with values like 2.5 for P and 0 for I and D, then tune
  private static final double kPX = 1.5; // Proportional gain for X
  private static final double kIX = 0.5; // Integral gain for X
  private static final double kDX = 0.0; // Derivative gain for X

  private static final double kPY = 1.5; // Proportional gain for Y
  private static final double kIY = 0.5; // Integral gain for Y
  private static final double kDY = 0.0; // Derivative gain for Y

  private static final double kPRot = 1.5; // Proportional gain for Rotation
  private static final double kIRot = 0.2; // Integral gain for Rotation
  private static final double kDRot = 0.0; // Derivative gain for Rotation

  // Tolerances for ending the command
  private static final double POSE_TOLERANCE_METERS = 0.05; // 5 cm
  private static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(2); // 2 degrees
  // --- END TUNING VALUES ---


  /** Creates a new PIDautopilotCommand. */
  public PID_Autopilot_cmd(CommandSwerveDrivetrain s_Swerve, Pose2d targetPose, CommandXboxController driver, CommandXboxController operator) {
    this.s_Swerve = s_Swerve;
    this.targetPose = targetPose;
    this.driverController = driver;
    this.operatorController = operator;

    xController = new PIDController(kPX, kIX, kDX);
    yController = new PIDController(kPY, kIY, kDY);
    rotationController = new PIDController(kPRot, kIRot, kDRot);

    
    // Enable continuous input for rotation controller to handle wrap-around (e.g., -Pi to +Pi)
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the tolerance for the PID controllers to match the isFinished() logic
    xController.setTolerance(POSE_TOLERANCE_METERS);
    yController.setTolerance(POSE_TOLERANCE_METERS);
    rotationController.setTolerance(ANGLE_TOLERANCE_RADIANS);


    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset controllers to clear any previous state
    xController.reset();
    yController.reset();
    rotationController.reset();

    SmartDashboard.putBoolean("AtTargetPose", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = s_Swerve.getState().Pose;

    // Calculate outputs from PID controllers
    // These are desired *velocities* in the field-relative frame
    // calculate(measurement, setpoint) => (current, target)
    double xOutput = xController.calculate(currentPose.getX(), targetPose.getX());
    double yOutput = yController.calculate(currentPose.getY(), targetPose.getY());
    
    // For rotation, we use the error between the current angle and the target angle
    // The PID controller will output a rotational velocity
    double rotationOutput = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    SmartDashboard.putNumber("X error",targetPose.getX()-currentPose.getX());
    SmartDashboard.putNumber("Y error", targetPose.getY()-currentPose.getY());
    SmartDashboard.putNumber("Rot Error", targetPose.getRotation().getDegrees()-currentPose.getRotation().getDegrees());

    // Clamp the outputs to the maximum robot speeds
    xOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, xOutput));
    yOutput = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, yOutput));
    rotationOutput = Math.max(-MAX_ANGULAR_SPEED, Math.min(MAX_ANGULAR_SPEED, rotationOutput));

    // Apply the calculated velocities to the swerve drive
    // The FieldCentric request will automatically convert these field-relative
    // velocities to robot-relative velocities based on the current gyro angle.
    s_Swerve.setControl(driveRequest
        .withVelocityX(xOutput)   // Field-relative X velocity (m/s)
        .withVelocityY(yOutput)   // Field-relative Y velocity (m/s)
        .withRotationalRate(rotationOutput) // Field-relative rotational velocity (rad/s)
    );

    if(xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint()) {
      SmartDashboard.putBoolean("AtTargetPose", true);
      driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.8);
      operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.8);
    } else {
      SmartDashboard.putBoolean("AtTargetPose", false);
      driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
      operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    s_Swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    SmartDashboard.putBoolean("AtTargetPose", false);
    driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if the robot is at the target pose and rotation within the defined tolerances
    return false;
  }
}