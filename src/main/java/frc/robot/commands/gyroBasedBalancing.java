// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;





public class gyroBasedBalancing extends CommandBase {
  /** Creates a new gyroBasedBalancing. */
  public Rotation2d yaw;
  public Rotation2d pitch;
  public Rotation2d roll;
  public Swerve swerve;
  public double prev = 0;

  public gyroBasedBalancing(Swerve swerveInput) {
    this.swerve = swerveInput;
    yaw = swerve.getYaw(); 
    pitch = swerve.getPitch();
    roll = swerve.getRoll();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curr = 
    if (Math.abs(curr - prev) > 0) {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
