// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//motor imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//pneumatics imports
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//Smartdashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Feedback/Feedforward imports
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmSubsystem extends SubsystemBase { 
  private TalonSRX m_Talonmotor2 = new TalonSRX(3);
  private TalonSRX m_Talonmotor = new TalonSRX(2);
  private DoubleSolenoid m_ArmLifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
  private DoubleSolenoid m_gripper = new DoubleSolenoid(PneumaticsModuleType.REVPH,2,3);

  // proportional, integral, and derivative speed constants
  // DANGER: when tuning PID constants, high/inappropriate values for kP, kI,
  // and kD may cause dangerous, uncontrollable, or undesired behavior!
  private static final double EkP = 0.3;
  private static final double EkI = 0;
  private static final double EkD = 0;
  private static final double Bkp = 0.02;
  private static final double BkI = 0;
  private static final double BkD = 0;

  private final ProfiledPIDController m_ExtenderPID = new ProfiledPIDController(EkP, EkI, EkD, 
    new TrapezoidProfile.Constraints(35, 30));
  private final ProfiledPIDController m_BenderPID = new ProfiledPIDController(Bkp, BkI, BkD, 
    new TrapezoidProfile.Constraints(65, 55));
  
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    //Set up arm configuration here? such as
    //m_Talonmotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,0,1000);
    m_Talonmotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        //Update the encoder values
        getArmAngle();
        getExtension();
        SmartDashboard.putNumber("Arm Angle", getArmAngle());
        SmartDashboard.putNumber("Arm Extension", getExtension());
  }

  /**
  * @return the length the arm is extending
  */
  public double getExtension() {
    return m_Talonmotor.getSelectedSensorPosition()/1848.72;
  }


  /**
  * @return the angle the arm is bending
  */
  public double getArmAngle() {
    return m_Talonmotor2.getSelectedSensorPosition()/59.68;
  }

  //Sets up what the arm can do
  public void LiftArm() {
    //raises the base of the arm
    m_ArmLifter.set(Value.kReverse);
  }
  public void LowerArm() {
    //lowers the base of the arm
    m_ArmLifter.set(Value.kForward);
  }
  public void BendArm(double TargetAngle){
    //bends the arm up and down
    m_Talonmotor2.set(ControlMode.PercentOutput, m_BenderPID.calculate(getArmAngle(), TargetAngle)); //maybe target angle times 59.68
  }
  public void ExtendArm(double ArmExtension) {
    //moves the arm in and out
    m_Talonmotor.set(ControlMode.PercentOutput, m_ExtenderPID.calculate(getExtension(), ArmExtension));  //maybe ArmExtension times 1848.72
  }

  //sets up gripper movement
  public void grip() {
    m_gripper.set(Value.kForward);
  }
  public void release() {
    m_gripper.set(Value.kReverse);
  }

  //Executes arm movement
  public void Home() {
    //puts the arm in the home position
    LowerArm();
    BendArm(0);
    ExtendArm(0);
    //m_Talonmotor.set(ControlMode.PercentOutput, 0);
    //m_Talonmotor2.set(ControlMode.PercentOutput, 0);
  }
  public void HighPlace() {
    //places game piece in the top row
    LiftArm();
    BendArm(85);
    ExtendArm(32);
    if (getArmAngle() > 80) {
      release();
    }
  }
  public void MidPlace() {
    //places the game piece in the middle row
    LowerArm();
    BendArm(15);
    ExtendArm(32);
  }
  public void LowPlace() {
    //places the game piece in the bottom row
    LowerArm();
    BendArm(0);
    ExtendArm(10);
  }
  public void SinglePick() {
    //picks the game piece from the single substation
    Home();
  }
  public void DoublePick(){
    //picks the game piece from the double substation
    LiftArm();
    BendArm(20);
    ExtendArm(10);
  }
  public void FloorPick() {
    //picks the game piece from the floor
    LiftArm();
    BendArm(0);
    ExtendArm(15);
  }
  public void ResetEncoders() {
    //Reset the encoders for home position
    m_Talonmotor.setSelectedSensorPosition(0);
    m_Talonmotor2.setSelectedSensorPosition(0);
  }

  //test functions
  public void LongerArm() {
    m_Talonmotor.set(ControlMode.PercentOutput, -0.5);
  }
  public void ShorterArm() {
    m_Talonmotor.set(ControlMode.PercentOutput, 0.5); 
  }
  public void RaiseArm() {
    m_Talonmotor2.set(ControlMode.PercentOutput, 0.3);
  }
  public void DropArm() {
    m_Talonmotor2.set(ControlMode.PercentOutput, -0.1);
  }

}