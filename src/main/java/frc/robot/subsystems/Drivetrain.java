// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private XRPMotor leftMotor = new XRPMotor(0);
  private XRPMotor rightMotor = new XRPMotor(1);

  private DifferentialDrive dDrive = new DifferentialDrive(leftMotor, rightMotor);
  
  private XboxController joy = new XboxController(0);

  // The XRP has onboard encoders that are hardcoded. To use the DIO pins 4/5 left and 6/7 for right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7); 
 
  /** Creates a new Drivetrain. */
  public Drivetrain() {

   m_leftEncoder.reset();
   m_rightEncoder.reset();

  }
  
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    dDrive.arcadeDrive(-joy.getLeftY(),-joy.getRightX());
    
  }

  public void drive(double x_speed, double z_rotation){
    dDrive.arcadeDrive(x_speed, z_rotation);
  }
}
