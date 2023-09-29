// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Measurements;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {

  // Creates Arm and Extending talons
  TalonSRX rotTalon = new TalonSRX(Constants.OperatorConstants.armPort);
  TalonSRX extendingTalon = new TalonSRX(Constants.OperatorConstants.extPort);

  // Creates necessary Shuffleboard tab, visible on DriverStation as well
  private ShuffleboardTab pidTab = Shuffleboard.getTab("Arm PID");
  private GenericEntry armPID_P = pidTab.add("Arm PID P", Constants.PIDConstants.armPID_P).getEntry();
  private GenericEntry armPID_I = pidTab.add("Arm PID I", Constants.PIDConstants.armPID_I).getEntry();
  private GenericEntry armPID_D = pidTab.add("Arm PID D", Constants.PIDConstants.armPID_D).getEntry();
  private GenericEntry armPID_K = pidTab.add("Power Const", Constants.PIDConstants.armPID_K).getEntry();

  // Create data log entry for angle
  public DoubleLogEntry angleLog;

  // Creates a PID controller from GenericEntry NetworkTables entries
  public PIDController armPID = new PIDController(
  armPID_P.getDouble(Constants.PIDConstants.armPID_P), 
  armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
  armPID_D.getDouble(Constants.PIDConstants.armPID_D));

  public PIDController extendPID = new PIDController(Constants.PIDConstants.extPID_P, 0, 0);

  // Creates target angle variable
  private double targetAngle = 0.0;

  // Creates variable for local PID Power Constant (K)
  // This is used to calculate the kP from the difference in angle between the original and target positions
  // Example: K/difference = kP = 0.7/45.0 = 0.01555
  public double powerConstant = armPID_K.getDouble(Constants.PIDConstants.armPID_K);

  // Manual override
  // True =  Manual Arm control (default), False = Auto
  public static boolean override = true;

  // Data logging override
  // True = Logging, False = Not Logging (default)
  public boolean logOverride = false;

  // Default difference in angle in order to calculate kP
  // Its value is always changed in the subsystem, but shouldn't be 0!
  public double differenceInAngle = 10.0;

  /** Creates a new Arm. */
  public Arm() {
    // Reset encoders to either 0 or otherwise an arbitrary offset
    // resetEncoders();
    rotTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    extendingTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Sets max current limit for amperage
    rotTalon.configPeakCurrentLimit(15);

    // Important for future use on final mechanism - configures limit switches for extension
    extendingTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    extendingTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);


    rotTalon.setInverted(Constants.OperatorConstants.armInvert);

    // Very important - sets max output, so that PID doesn't output an output that it too high
    // Might be changed based upon Arm's torque and sensitivity to voltage
    rotTalon.configPeakOutputForward(1.0);
    rotTalon.configPeakOutputReverse(-1.0);

    // Sets limits for how far the talon can extend forwards and backwards
    rotTalon.configForwardSoftLimitThreshold(Constants.Measurements.upperAngleBound*Constants.Measurements.degreesToTicks);
    rotTalon.configReverseSoftLimitThreshold(Constants.Measurements.lowerAngleBound*Constants.Measurements.degreesToTicks);
    extendingTalon.configForwardSoftLimitThreshold(((Constants.Measurements.upperScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);
    extendingTalon.configReverseSoftLimitThreshold(((Constants.Measurements.lowerScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);

    // NOTICE: These lines of code are needed to enable the limits!
    rotTalon.configForwardSoftLimitEnable(true);
    rotTalon.configReverseSoftLimitEnable(true);
    extendingTalon.configForwardSoftLimitEnable(true);
    extendingTalon.configReverseSoftLimitEnable(true);

    

    // Reset Extending Talon
    extendingTalon.setSelectedSensorPosition(0);

    // Set tolerance of PID
    extendPID.setTolerance(0.2);

    // Tolerance from reaching the setpoint
    armPID.setTolerance(Constants.Measurements.armPIDTolerance);

    // Integration range, if we are using the integral term of PID
    // Meaning that the controller only takes the integral when it
    // is within a certain range of the setpoint
    armPID.setIntegratorRange(-5, 5);

    // Only use the datalogger in testing!

    // Starts datalogger
    DataLogManager.start();

    // Set up data log entries
    DataLog log = DataLogManager.getLog();
    angleLog = new DoubleLogEntry(log, "/my/double");
  }

  // Tested 1/21/2022
  /**
  * The testing arm holds with the power of the calculated feedforward, not the action (manual increment of 20%).
  * This is calculated with sine, where the max voltage is 1.12/12 = 9% voltage
  * We might need to increase the multiplier on the input, so that when operating manually, the pivot works better for the driver
  @param power - Percent input into the controller
  */
  public void rotateArm(double power) {
      if(power == 0)rotTalon.setNeutralMode(NeutralMode.Brake);
      else rotTalon.setNeutralMode(NeutralMode.Coast);

      rotTalon.set(ControlMode.PercentOutput, power*1.0+(Math.sin(getAngle())*(Constants.PIDConstants.armManualHoldingVoltage/12.0))); 
  }

  public void extendArm(double power) {
    extendingTalon.set(ControlMode.PercentOutput, power);
    //((extendingTalon.getSelectedSensorPosition(0)/4096.0)*Constants.Measurements.threadLength)/Constants.Measurements.gearRatio
  }

  /**
  The robot starts at 0 degrees, and drops down to 90 degrees for the horizontal position.
  Since sin(90) = 1, then we can multiply it by the max holding voltage.
  Max holding voltage for testing pivot: 1.12 volts = 0.09 power.
  */
  public void resetEncoders() {
    rotTalon.setSelectedSensorPosition(0, 0, 10);
  }

  /** 
  Returns the angle of the Arm (degrees)
  */
  public double getAngle() {
    double result = (rotTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  /** 
  Returns the position of the Leadscrew (inches)
  */
  public double getLeadScrewPos() {
    double result = ((extendingTalon.getSelectedSensorPosition(0)/4096.0)*Constants.Measurements.threadLength)/Constants.Measurements.gearRatio;
    return result;
  }

  /**
   * Returns the velocity of the extension
  @return velocity - The velocity of the extension in inches/sec
   */
  public double getLeadScrewVelocity() {
    double velocity = (((extendingTalon.getSelectedSensorVelocity(0)*10.0)/4096.0)*Constants.Measurements.threadLength)/Constants.Measurements.gearRatio;
    return velocity;
  }

  /**
   * Sets the setpoint of the PID controller
   */
  public void setExtendPIDSetpoint(double setpoint) {
    extendPID.setSetpoint(setpoint);
  }

  /**
  Updates the PID loop with a calculated feedforward value
   */
  public void updateRotatePID() {
    // Without two clamps: 1.12
    // With clamps: 1.2
    rotTalon.set(ControlMode.PercentOutput, armPID.calculate(getAngle())+(Math.sin(getAngle())*(Constants.PIDConstants.armHoldingVoltage/12.0)));
    SmartDashboard.putNumber("PID: ", armPID.calculate(getAngle())+(Math.sin(getAngle())*(Constants.PIDConstants.armHoldingVoltage/12.0)));
  }

  public void updateExtendPID() {
    if(RobotContainer.getJoy1().getPOV() == 0 && getLeadScrewPos() > Constants.Measurements.maxExtension - 1){
      extendingTalon.set(ControlMode.PercentOutput, Constants.Measurements.extLimitPower);
    }
    else if(RobotContainer.getJoy1().getPOV() == 180 && getLeadScrewPos() < 0.5){
      extendingTalon.set(ControlMode.PercentOutput, -2*Constants.Measurements.extLimitPower);
    }
    
    else if(RobotContainer.getJoy1().getPOV() == 0){
      extendingTalon.set(ControlMode.PercentOutput, extendPID.calculate(getLeadScrewVelocity()) + Constants.Measurements.baseExtendPower);
    }
    else if(RobotContainer.getJoy1().getPOV() == 180){
      extendingTalon.set(ControlMode.PercentOutput, extendPID.calculate(getLeadScrewVelocity()) - Constants.Measurements.baseExtendPower);
    }
    
  }

  public boolean fullyExtended(){
    return extendingTalon.isFwdLimitSwitchClosed() == 0.0;
  }

  public boolean fullyRetracted(){
    return extendingTalon.isRevLimitSwitchClosed() == 0.0;
  }

  public void configSoftLimits(boolean config){
      extendingTalon.configForwardSoftLimitEnable(config);
      extendingTalon.configReverseSoftLimitEnable(config);
      rotTalon.configForwardSoftLimitEnable(config);
      rotTalon.configReverseSoftLimitEnable(config);
  }

  public void configHardLimits(boolean config){
    if(config){
      extendingTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
      extendingTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
      extendingTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
      extendingTalon.configForwardSoftLimitThreshold(((Constants.Measurements.upperScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);
      extendingTalon.configReverseSoftLimitThreshold(((Constants.Measurements.lowerScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);
      //extendingTalon.configForwardSoftLimitEnable(true);
      //extendingTalon.configReverseSoftLimitEnable(false);
    }else{
      extendingTalon.configFactoryDefault();
      extendingTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }
  }

  /**
  Whether the PID controller is at the setpoint.
   */
  public boolean withinSetpoint() {
    return armPID.atSetpoint();
  }

  /** 
  Sets subsystem's internal angle
  @param angle - Angle to set the arm
   */
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /** 
  Sets subsystem's internal difference in angle, in order to calculate kP
  @param diff - Difference in angle
   */
  public void setDifferenceInAngle(double diff) {
    this.differenceInAngle = diff;
  }
  
  /** 
  Gets the status of control (manual is true, auto is false)
  */
  public static boolean getOverride() {
    return override;
  }
  /**
  Sets the status of control (manual is true, auto is false).
  Useful for auto commands.
  @param o - Boolean for overriding to manual mode
  */
  public void setOverride(boolean o) {
    override = o;
  }

  // Runs periodically  
  @Override
  public void periodic() {
    //extendingTalon.set(ControlMode.PercentOutput, RobotContainer.getJoy1().getX());
    // Button to toggle between manual and PID control
    if (RobotContainer.getJoy1().getRawButtonReleased(Constants.ButtonMap.manualOverride)) {
      override = !override;
    }

    // Button to toggle whether or not to log
    if (RobotContainer.getJoy1().getRawButtonReleased(Constants.ButtonMap.logButton)) {
      logOverride = !logOverride;
    }

    // power is a function of distance with max power at half extension
    //double power =  (Constants.Measurements.maxExtendPower * (1 - Math.pow(extendingTalon.getSelectedSensorPosition()*Constants.Measurements.ticksToInches - Constants.Measurements.maxExtension/2, 8)));
    //if(power < Constants.Measurements.minExtendPower) power = Constants.Measurements.minExtendPower;
    updateExtendPID();
    // SmartDashboard.putNumber("Extension Power", power);
    if (RobotContainer.getJoy1().getPOV() == 0) {
      //Max extension 16.67 inches
      // Move extension forward at POV pos of 0
      extendPID.setSetpoint(11.0);

      //extendArm(power);
    }  else if (RobotContainer.getJoy1().getPOV() == 180) {
      // Move extension backward at POV pos of 180
      extendPID.setSetpoint(-11.0);
      //extendArm(-1*power);
    }
    else {
      extendArm(0);
    }

    if (RobotContainer.getJoy1().getRawButton(Constants.ButtonMap.extLimitReset)) {
      configSoftLimits(false);
    }else{
      configSoftLimits(true);
    }

    if (RobotContainer.getJoy1().getRawButton(8)) {
      configHardLimits(false);
    } else {
      configHardLimits(true);
    }
    

    // If controlling manually
    if (override) {
      // We want to reset the target angle to the current angle
      rotateArm(-RobotContainer.getJoy1().getY());
      targetAngle = getAngle();
    // Else if not controlling manually
    } else if (!override) {
      armPID.setSetpoint(targetAngle);
      updateRotatePID();
    }

    // Override for logging button (REMOVE IN FINAL CODE)
    if (logOverride) {
      angleLog.append(getAngle());
    }

    // Repeatedly set new Power constant for finding kP
    powerConstant = armPID_K.getDouble(Constants.PIDConstants.armPID_K);

    // Repeatedly set new PID constants from DriverStation
    // If the Direction of the Arm is greater than 0, set certain PID constants
    if (differenceInAngle >= 0) {
    armPID.setPID(
      armPID_P.getDouble(powerConstant/Math.abs(differenceInAngle)), 
      armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
      armPID_D.getDouble(Constants.PIDConstants.armPID_D));
    }
    // If the Direction of the Arm is less than 0, then set certain PID constants
    else if (differenceInAngle <= 0) {
      armPID.setPID(
        armPID_P.getDouble(powerConstant/Math.abs(differenceInAngle)), 
        armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
        armPID_D.getDouble(Constants.PIDConstants.armPID_D));
    }
    
    // Increment arm angle if the joystick's y axis is at a positive value, the trigger button is pressed, and the target angle is below a certain bound
    if (!override && RobotContainer.getJoy1().getY() > 0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle <= Constants.Measurements.upperAngleBound) {
      SmartDashboard.putString("Status: ", "Incrementing");
      targetAngle += 10;
    // Decrement arm angle if the joystick's y axis is at a negative value, the trigger button is pressed, and the target angle is above a certain bound
    } else if (!override && RobotContainer.getJoy1().getY() < 0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle >= Constants.Measurements.lowerAngleBound) {
      SmartDashboard.putString("Status: ", "Decrementing");
      targetAngle -= 10;
    } else {
      SmartDashboard.putString("Status: ", "None");
    }

    // Enable/Disable Soft limits when the leadscrew extends a certain amount
    // This is so that it can stow safely, yet extend beyond the frame to pick up field elements
    if (getLeadScrewPos() >= Constants.Measurements.fullyExtendedLeadScrewThreshold && RobotContainer.getJoy1().getRawButton(Constants.ButtonMap.extLimitReset) == false) {
      rotTalon.configReverseSoftLimitThreshold(Constants.Measurements.bumperAngleBound*Constants.Measurements.degreesToTicks);
      rotTalon.configReverseSoftLimitEnable(true);
    } else if (getLeadScrewPos() <  Constants.Measurements.fullyExtendedLeadScrewThreshold && RobotContainer.getJoy1().getRawButton(Constants.ButtonMap.extLimitReset) == false) {
      rotTalon.configReverseSoftLimitThreshold(Constants.Measurements.lowerAngleBound*Constants.Measurements.degreesToTicks);
      rotTalon.configReverseSoftLimitEnable(true);
    }

    if (rotTalon.isRevLimitSwitchClosed() == 0.0) {
      rotTalon.setSelectedSensorPosition(0);
    }

    if (fullyRetracted()) {
      extendingTalon.setSelectedSensorPosition(0);
    }
    if(fullyExtended()) {
      extendingTalon.setSelectedSensorPosition(Constants.Measurements.maxExtension/Constants.Measurements.ticksToInches);
    }

    // Prints status of certain things to the DriverStation
    SmartDashboard.putNumber("kP: ", 0.7/differenceInAngle);
    SmartDashboard.putNumber("kI: ", armPID_I.getDouble(Constants.PIDConstants.armPID_I));
    SmartDashboard.putNumber("Lead Screw Raw Pos: ", extendingTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Extension Pos In Inches: ", getLeadScrewPos());
    SmartDashboard.putNumber("Extension Velocity: ", getLeadScrewVelocity());
    SmartDashboard.putNumber("POV Angle", RobotContainer.getJoy1().getPOV());
    SmartDashboard.putNumber("Arm Ticks: ", rotTalon.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Arm Voltage: ", rotTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Arm Angle: ", getAngle());
    SmartDashboard.putNumber("Joystick Y Position: ", RobotContainer.getJoy1().getY());
    SmartDashboard.putNumber("Current", rotTalon.getStatorCurrent());
    SmartDashboard.putNumber("Ext Current", extendingTalon.getStatorCurrent());

    SmartDashboard.putNumber("Rot Reverse", rotTalon.isRevLimitSwitchClosed());
    SmartDashboard.putNumber("Rot Forward", rotTalon.isFwdLimitSwitchClosed());

    SmartDashboard.putNumber("Ext Reverse", extendingTalon.isRevLimitSwitchClosed());
    SmartDashboard.putNumber("Ext Forward", extendingTalon.isFwdLimitSwitchClosed());
    
    SmartDashboard.putNumber("Extension Current", extendingTalon.getStatorCurrent());
    SmartDashboard.putNumber("Extension PID", extendPID.calculate(getLeadScrewPos()));

  }
}
