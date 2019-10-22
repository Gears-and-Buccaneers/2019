/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.  
/*            
/*  Zane McMorris is who to blame for this horrible code.
*/
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI.*;
import edu.wpi.first.wpilibj.SerialPort.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;




/*
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  TalonSRX left1, left2, right1, right2, dongerDeployer, gate;
  Joystick pad;
  I2C wire;
  SerialPort rs2;
  double pneumaticTime, acceleration, brake, turn, speed, throttle, swapTime, dongerPistonTime, dongerTime, gateTime, current, time, dongerDeployedTime, dong, climbTime, fullYeet, forceGate, rampUp;
  int trgb, backward, gateOpen, dongerDeployed;
  boolean fullStop, reverse, swapButton, dongerPistonButton, rampButton, stopButton, dongerDeployedButton, gateButton, climbDown, climbButton, enabled, pressureSwitch, dongerPistonEngaged;
  byte[] reading, dataToSend;
  Compressor compressor;
  DoubleSolenoid dongerSol, rampSol, climbSol;
  boolean dongerEngaged = false;
  boolean rampEngaged = true;

  final ControlMode PO = ControlMode.PercentOutput;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  
  public void robotInit() {

    pad = new Joystick(0);
    left2 = new TalonSRX(0);
    right2 = new TalonSRX(2); // These Talon assignment MAY change, double check physical comps
    left1 = new TalonSRX(1);
    right1 = new TalonSRX(3);
    gate = new TalonSRX(7);
    dongerDeployer = new TalonSRX(6);
    compressor = new Compressor(0);
    dongerSol = new DoubleSolenoid(0,1);
    rampSol = new DoubleSolenoid(2, 3);
    climbSol = new DoubleSolenoid(5,4);

    //wire = new I2C(I2C.Port.kOnboard, 8);
    //reading =  new byte[1];
    //dataToSend = new byte[1];
    
    CameraServer.getInstance().startAutomaticCapture();
    //CameraServer.getInstance().startAutomaticCapture();
  
    left1.set(PO,0);
    right1.set(PO,0);
    right2.set(PO,0);
    left2.set(PO,0);

    backward = 1;
    gateOpen = 1;
    dongerDeployed = -1;
    dongerEngaged = false;
    rampEngaged = false;
    pneumaticTime = System.currentTimeMillis();
    climbDown = false;
    rampUp = 0;
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
 
  public void autonomousInit() {
    left1.set(PO,0);
    right1.set(PO,0);
    right2.set(PO,0);
    left2.set(PO,0);

  }

  /**
   * This function is called periodically during autonomous.
   */
  public void autonomousPeriodic() {
    //CONTROLS START

    throttle = pad.getRawAxis(1); // Left JoyStick Vert.
    turn = pad.getRawAxis(4);     // Right JoyStick Horiz.

    swapButton = pad.getRawButton(4); // Triangle / Y
    dongerPistonButton = pad.getRawButton(3); // Opens or Closes Dong Arms  : Square / X
    rampButton = pad.getRawButton(8); // Options ? / Start
    stopButton = pad.getRawButton(7); // Share ? / Back
    dongerDeployedButton = pad.getRawButton(6); // Deploys or Retracts dong   : R1 / RB
    gateButton = pad.getRawButton(5); // L1 / LB
    climbButton = pad.getRawButton(1); // X / A
    fullYeet = pad.getRawAxis(2);
    forceGate = pad.getRawAxis(3);
    time = System.currentTimeMillis();

    

    //enabled = compressor.enabled();
    //pressureSwitch = compressor.getPressureSwitchValue();
    //current = compressor.getCompressorCurrent();

    //SmartDashboard.putBoolean("compressor on: ", enabled);
    //SmartDashboard.putBoolean("pressure switch triggered?: ", pressureSwitch);
    //SmartDashboard.putNumber("compressor current draw: ", current);

    //dataToSend[0] = (byte) trgb;
    //wire.write(8,trgb);
    //SmartDashboard.putNumber("reading", (double) reading[0] );

    // SWAP TIMING / LOGIC
    if(time - swapTime > 1500){
      if(swapButton){
        backward *= -1;
        swapTime = time;      
      }
    }

// Stop and swap logic

// if(Math.abs(throttle) > .1){
//   rampUp += .05;
// }
// else{
//   rampUp = 0;
// }

if(fullYeet > .5)
{
  left1.set(PO, -1);
  right1.set(PO, 1);
  right2.set(PO, 1);
  left2.set(PO, -1);
}else{
    if(!stopButton){
        left1.set(PO, (backward*-throttle + .6*turn));
        right1.set(PO, (backward*throttle + .6*turn));
        right2.set(PO, (backward*throttle + .6*turn));
        left2.set(PO, (backward*-throttle + .6*turn));
    }
    else{
        left1.set(PO, 0);
        right1.set(PO, 0);
        right2.set(PO, 0);
        left2.set(PO, 0);
    }
  }  
    //CONTROLS END

    // GATE TIMING / OPENING
  //if(forceGate < .6)
  //{

  
    if(time - gateTime > 175){
      if(gateButton){
        gateTime = time;
        gate.set(PO, gateOpen);
        gateOpen *= -1;
      }
      else if(gateOpen > .5){
        gate.set(PO, -.1);
      }
      else{
        gate.set(PO, 0);
      }
    }
  //}
  //else
  //  gate.set(PO, -forceGate);
  


    // DONG TIMING / OPENING
    if(time - dongerDeployedTime > 240){
      if(dongerDeployedButton){
        dongerDeployedTime = time;
        dongerDeployer.set(PO, dongerDeployed);
        dongerDeployed *= -1;
      }
      else{
        dongerDeployer.set(PO, dongerDeployed*-.1);
      }
    }


   //DONG TIMING / DEPLOYMENT

    if (time - dongerPistonTime > 600){
      if (dongerPistonButton){
        if(dongerPistonEngaged) dongerSol.set(DoubleSolenoid.Value.kReverse);
        else dongerSol.set(DoubleSolenoid.Value.kForward);
        dongerPistonEngaged = !dongerPistonEngaged;
        dongerPistonTime = time;
      }
      else{
        dongerSol.set(DoubleSolenoid.Value.kOff);
      }
    }
    // Clinmbing timings and deployment logic
    if (time - climbTime > 600){
      if (climbButton){
        if(climbDown) climbSol.set(DoubleSolenoid.Value.kReverse);
        else climbSol.set(DoubleSolenoid.Value.kForward);
        climbDown = !climbDown;
        climbTime = time;
      }
      else{
        climbSol.set(DoubleSolenoid.Value.kOff);
      }
    }

    // RAMP TIMING / DEPLOYMENT
    if (time - pneumaticTime > 500){
      if (rampButton){
        if(rampEngaged) rampSol.set(DoubleSolenoid.Value.kReverse);
        else rampSol.set(DoubleSolenoid.Value.kForward);
        rampEngaged = !rampEngaged;
        pneumaticTime = time;
      }
      else{
        rampSol.set(DoubleSolenoid.Value.kOff);
      }
    }
  }
  

  /**
   * This function is called periodically during operator control.
   */
  public void teleopInit(){
    left1.set(PO,0);
    right1.set(PO,0);
    right2.set(PO,0);
    left2.set(PO,0);
  }

  public void teleopPeriodic() {
  //CONTROLS START

  throttle = pad.getRawAxis(1); // Left JoyStick Vert.
  turn = pad.getRawAxis(4);     // Right JoyStick Horiz.

  swapButton = pad.getRawButton(4); // Triangle / Y
  dongerPistonButton = pad.getRawButton(3); // Opens or Closes Dong Arms  : Square / X
  rampButton = pad.getRawButton(8); // Options ? / Start
  stopButton = pad.getRawButton(7); // Share ? / Back
  dongerDeployedButton = pad.getRawButton(6); // Deploys or Retracts dong   : R1 / RB
  gateButton = pad.getRawButton(5); // L1 / LB
  climbButton = pad.getRawButton(1); // X / A
  fullYeet = pad.getRawAxis(2);
  forceGate = pad.getRawAxis(3);
  time = System.currentTimeMillis();

  

  //enabled = compressor.enabled();
  //pressureSwitch = compressor.getPressureSwitchValue();
  //current = compressor.getCompressorCurrent();

  //SmartDashboard.putBoolean("compressor on: ", enabled);
  //SmartDashboard.putBoolean("pressure switch triggered?: ", pressureSwitch);
  //SmartDashboard.putNumber("compressor current draw: ", current);

  //dataToSend[0] = (byte) trgb;
  //wire.write(8,trgb);
  //SmartDashboard.putNumber("reading", (double) reading[0] );

  // SWAP TIMING / LOGIC
  if(time - swapTime > 1500){
    if(swapButton){
      backward *= -1;
      swapTime = time;      
    }
  }

// Stop and swap logic

// if(Math.abs(throttle) > .1){
//   rampUp += .05;
// }
// else{
//   rampUp = 0;
// }

if(fullYeet > .8) // Full yeet doesn't work because of horrible batts and browning out, Don't use.
{
left1.set(PO, -backward);
right1.set(PO, backward);
right2.set(PO, backward);
left2.set(PO, -backward);
}else{
  if(!stopButton){
      left1.set(PO, (backward*-.82*throttle + .82*turn));
      right1.set(PO, (backward*.82*throttle + .82*turn));
      right2.set(PO, (backward*.82*throttle + .82*turn));
      left2.set(PO, (backward*-.82*throttle + .82*turn));
  }
  else{
      left1.set(PO, 0);
      right1.set(PO, 0);
      right2.set(PO, 0);
      left2.set(PO, 0);
  }
}  
  //CONTROLS END

  // GATE TIMING / OPENING
//if(forceGate < .6)
//{


if(time - gateTime > 175){
  if(gateButton){
    gateTime = time;
    gate.set(PO, gateOpen);
    gateOpen *= -1;
  }
  else if(gateOpen > .5){
    gate.set(PO, -.1);
  }
  else{
    gate.set(PO, 0);
  }
}
//}
//else
//  gate.set(PO, -forceGate);



  // DONG TIMING / OPENING
  if(time - dongerDeployedTime > 240){
    if(dongerDeployedButton){
      dongerDeployedTime = time;
      dongerDeployer.set(PO, dongerDeployed);
      dongerDeployed *= -1;
    }
    else{
      dongerDeployer.set(PO, dongerDeployed*-.1);
    }
  }


 //DONG TIMING / DEPLOYMENT

  if (time - dongerPistonTime > 600){
    if (dongerPistonButton){
      if(dongerPistonEngaged) dongerSol.set(DoubleSolenoid.Value.kReverse);
      else dongerSol.set(DoubleSolenoid.Value.kForward);
      dongerPistonEngaged = !dongerPistonEngaged;
      dongerPistonTime = time;
    }
    else{
      dongerSol.set(DoubleSolenoid.Value.kOff);
    }
  }

  if (time - climbTime > 600){
    if (climbButton){
      if(climbDown) climbSol.set(DoubleSolenoid.Value.kReverse);
      else climbSol.set(DoubleSolenoid.Value.kForward);
      climbDown = !climbDown;
      climbTime = time;
    }
    else{
      climbSol.set(DoubleSolenoid.Value.kOff);
    }
  }

  // RAMP TIMING / DEPLOYMENT
  if (time - pneumaticTime > 500){
    if (rampButton){
      if(rampEngaged) rampSol.set(DoubleSolenoid.Value.kReverse);
      else rampSol.set(DoubleSolenoid.Value.kForward);
      rampEngaged = !rampEngaged;
      pneumaticTime = time;
    }
    else{
      rampSol.set(DoubleSolenoid.Value.kOff);
    }
  }
  //SmartDashboard.putNumber("t", System.currentTimeMillis()-time);
}
  public void testPeriodic() {
  }
}

