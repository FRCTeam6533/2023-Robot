// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class DriverControl extends SubsystemBase {
  private XboxController m_driverController = new XboxController(0);
  
  /** Creates a new DriverControl. */
  public DriverControl() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void a() {
    m_driverController.getRawButton(1);
  }
  public void b() {
    m_driverController.getRawButton(2);
  }
  public void x() {
    m_driverController.getRawButton(3);
  }
  public void y() {
    m_driverController.getRawButton(4);
  }
  public void l1() {
    m_driverController.getRawButton(5);
  }
  public void r1() {
    m_driverController.getRawButton(6);
  }
  public void sel() {
    m_driverController.getRawButton(7);
  }
  public void start(){
    m_driverController.getRawButton(8);
  }
  public void lstick(){
    m_driverController.getRawButton(9);
  }
  public void rstick(){
    m_driverController.getRawButton(10);
  }
  public void lxaxis(){
    m_driverController.getRawAxis(0);
  }
  public void lyaxis(){
    m_driverController.getRawAxis(1);
  }
  public void lbumper(){
    m_driverController.getRawAxis(2);
  }
  public void rbumper(){
    m_driverController.getRawAxis(3);
  }
  public void rxaxis(){
    m_driverController.getRawAxis(4);
  }
  public void ryaxis(){
    m_driverController.getRawAxis(5);
  }
  public void povup(){
    m_driverController.pov(0, null);
  }
  public void povrt(){
    m_driverController.pov(90, null);
  }
  public void povdn(){
    m_driverController.pov(180, null);
  }
  public void povlt(){
    m_driverController.pov(270, null);
  }
}
