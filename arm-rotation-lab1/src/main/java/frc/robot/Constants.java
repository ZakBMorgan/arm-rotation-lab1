// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static int joyStick1 = 0;
  public static int joyStick2 = 1;
  public static int joyStick3 = 2;
  public static int joystick4 = 3;

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final int LeftDriveTalonPort = 2;
    public static final int RightDriveTalonPort = 3;

  }

  public static class PIDConstants {
    public static final double armPID_P = 0.1;
    public static final double armPID_I = 0.001;
    public static final double armPID_D = 0.0;
    public static final double armPID_K = 2.0; // Used to calculate kP, from the difference in angle
    public static final double extPID_P = 0.05; 
    public static final double armHoldingVoltage = 1.1; // Used to calculate Feedforward
    public static final double armManualHoldingVoltage = 1.9; // Manual holding voltage

  }

  public static class Measurements {
    public static final double threadLength = 0.375; // Inches per rotation of leadscrew
    public static final double gearRatio = 1.0/1.0; // 1 rotation of screw = 1 rotation of leadscrew motor
    
    public static final double baseExtendPower = 1; //Percent of full power/100
    public static final double extLimitPower = 0.4; //Percent of full power/100. Power when extention is close to to limit switch

    public static final double armPIDTolerance = 2;

  }

  public static class ButtonMap {

    // Trigger button is used for incrementing and decrementing angle of arm

    // Manual override button
    public static final int manualOverride = 1;

    // Buttons for Arm rotation
    public static final int stowPosition = 3;
    public static final int middlePosition = 4;
    public static final int topPosition = 6;
    public static final int groundPosition = 5;

  }

  /*
  public static final int leftPort = 1;
  public static final int rightPort = 2;

  public static int joy1;
  public static int joy2;
  */
}
