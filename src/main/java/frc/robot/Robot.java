// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;


/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
    // Create TalonFX motor controller with CAN ID 1
    private final TalonFX motor = new TalonFX(25);
    
    // Joystick on USB port 0
    private final Joystick joystick = new Joystick(0);

    // Duty cycle control object for setting motor output
    private final DutyCycleOut output = new DutyCycleOut(0);

    // Motor speed (0.5 = 50% output, adjust as needed)
    private final double motorSpeed = 0.5;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder",1);
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    // Button 1 starts the motor
    if (joystick.getRawButtonPressed(1)) {
        motor.setControl(new MotionMagicVoltage(5)); // Moves smoothly at 200 RPM

        // motor.setControl(output.withOutput(motorSpeed));
        System.out.println("Motor STARTED");
    }

    // Button 2 stops the motor
    if (joystick.getRawButtonPressed(2)) {
        motor.setControl(output.withOutput(0));
        System.out.println("Motor STOPPED");
    }
  }
}
