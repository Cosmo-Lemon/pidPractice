// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
  
     private XRPMotor leftMotor = new XRPMotor(0);
     private XRPMotor rightMotor = new XRPMotor(1);

     private DifferentialDrive dDrive = new DifferentialDrive(leftMotor, rightMotor);
  
     private XboxController joy = new XboxController(0);

     // The XRP has onboard encoders that are hardcoded
     // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);

    private final XRPRangefinder rangeFinder = new XRPRangefinder();
    private final XRPReflectanceSensor reflectSensor = new XRPReflectanceSensor();
     
    private final double kDriveTick2Inch = Math.PI * 2.3622/585;

    final double kP = 0.045;

    private double setpoint = 0;

    private double leftsensorPosition = 0;
    private double rightsensorPosition = 0;
    private double averagesensorPosition = 0;

    private double currentPoint = 0;

    private double lefterror = 0;
    private double righterror = 0;
    private double averageerror = 0;

    private double leftoutputSpeed = 0;
    private double rightoutputSpeed = 0;
    private double averageoutputSpeed = 0;



  public Robot() {
    rightMotor.setInverted(true);
  }



  @Override
  public void autonomousInit() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();

  
  }

  
  @Override
  public void autonomousPeriodic() {
    

      if (joy.getBButtonPressed()){
        setpoint = 0; 
      }
       else if  (joy.getAButtonPressed()){
        setpoint = 40;
      }

      if (rangeFinder.getDistanceInches() <= 5.1 && setpoint > 0) {
        setpoint = currentPoint;
      } 
    
     

   

    leftsensorPosition = m_leftEncoder.get() * kDriveTick2Inch;
    rightsensorPosition = m_rightEncoder.get() * kDriveTick2Inch;
    averagesensorPosition = (leftsensorPosition + rightsensorPosition)/2; 

    currentPoint = averagesensorPosition;

    lefterror = setpoint - leftsensorPosition;
    righterror = setpoint - rightsensorPosition;
    averageerror = (lefterror + righterror)/2;

    leftoutputSpeed = kP * lefterror;
    rightoutputSpeed = kP  * righterror;
    averageoutputSpeed = (leftoutputSpeed + rightoutputSpeed)/2;

    leftMotor.set(leftoutputSpeed);
    rightMotor.set(rightoutputSpeed);

    
  }

  @Override
  public void robotPeriodic(){
   SmartDashboard.putNumber("leftEncoder value", m_leftEncoder.get());
   SmartDashboard.putNumber("rightEncoder value", m_rightEncoder.get());

   SmartDashboard.putNumber("leftsensorPosition value", leftsensorPosition);
   SmartDashboard.putNumber("rightsensorPosition value", rightsensorPosition);
   SmartDashboard.putNumber("averagesensorPosition", averagesensorPosition);

   SmartDashboard.putNumber("currentPoint", currentPoint);
   SmartDashboard.putNumber("setpoint", setpoint);

   SmartDashboard.putNumber("lefterror value", lefterror);
   SmartDashboard.putNumber("righterror value", righterror);
   SmartDashboard.putNumber("averageerror", averageerror);

   SmartDashboard.putNumber("leftoutputSpeed value", leftoutputSpeed);
   SmartDashboard.putNumber("rightoutputSpeed value", rightoutputSpeed);
   SmartDashboard.putNumber("outputSpeed", averageoutputSpeed);

   SmartDashboard.putNumber("rangedistance", rangeFinder.getDistanceInches());
   
   
  }
  @Override
  public void teleopInit() {
    m_leftEncoder.reset();
    m_rightEncoder.reset(); 

  }

  @Override
  public void teleopPeriodic() {

  
   

    dDrive.arcadeDrive(-joy.getLeftY(),-joy.getRightX());

  

  }
  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
