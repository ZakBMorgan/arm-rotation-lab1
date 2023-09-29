// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Rotate extends CommandBase {
  public Arm arm;
  public double angle;

  /** Rotates the Arm to a certain angle using PIDF control
  * @param arm - Arm subsystem
  * @param angle - Desired angle in degrees
  */
  
  public Rotate(Arm arm, double angle) {
    this.arm = arm;
    this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets difference in angle in order to calculate kP
    // This is the difference between the current and target angle
    arm.setDifferenceInAngle(arm.getAngle() - angle);
    // Sets the override variable to false (meaning the arm is controlled via PID)
    arm.setOverride(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Constantly set the target angle of the PID
    arm.setTargetAngle(angle);
    SmartDashboard.putString("Current Command: ", "Rotate");
    //SmartDashboard.putNumber("Diff in arm angle: ", Math.abs(angle - arm.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setOverride(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop when the arm is within the setpoint by a certain margin
    if(angle == 0 && 5 >= Math.abs(angle - arm.getAngle())) return true;
    return Constants.Measurements.armPIDTolerance >= Math.abs(angle - arm.getAngle());
  }
}