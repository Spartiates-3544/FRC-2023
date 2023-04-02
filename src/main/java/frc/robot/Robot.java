// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.playback.Reader;
import frc.robot.playback.Recorder;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean record = true, play = false;
  String recordingURL = "/home/lvuser/2piece2.json";

  long initialTime;
  Reader reader;
  Recorder recorder;
  int currentRecordingIndex;

  String[] playbackURLs = { "/home/lvuser/Trajectoire1_04_05_2022_ReculerC.json"};
  


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //CameraServer.startAutomaticCapture();
    m_robotContainer.getLEDs().setRGB(0, 255, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getLEDs().setRGB(0, 255, 0);

   /*  if (recorder != null) {
      recorder.close();
      recorder = null;
    }

    if (reader != null) {
      reader.close();
      reader = null;
    }*/
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    currentRecordingIndex = 0;

    if (play) {
      reader = initializeReader(playbackURLs[currentRecordingIndex]);
    }

    Shuffleboard.selectTab("Autonomous");
  }

  private Reader initializeReader(String playbackURL) {
    Reader reader;
    try {
      // System.out.println(playbackURL); //for testing purpose only
      reader = new Reader(playbackURL);
      initialTime = System.currentTimeMillis();
    } catch (Exception e) {
      // This segment will execute if the file is missing or has the wrong
      // permissions
      reader = null;
      e.printStackTrace();
    }
    return reader;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (record) {
      // This initializes the recorder. The former parameter is the keys,
      // and the latter is the defaults to use.
      recorder = new Recorder(new String[] { "v", "omega", "arm", "intake" }, new Object[] { 0.0, 0.0, 0.0, 0.0 },
          recordingURL);
    }

    m_robotContainer.getDrivetrain().setNeutralMode(NeutralMode.Coast);
    //Set to cone
    m_robotContainer.getLEDs().setRGB(255, 150, 5);

    Shuffleboard.selectTab("Teleop");
  }

  @Override
  public void teleopPeriodic() {
    if (recorder != null) {
      // xbox and js are two input controllers. These methods just return
      // joystick values (in the form of doubles)
      Object[] input = new Object[] { (-1) * m_robotContainer.getJoystick().getLeftY(),
          (1) * ((m_robotContainer.getJoystick().getRightTriggerAxis() - m_robotContainer.getJoystick().getLeftTriggerAxis() ) * 0.8), 0.0, 0.0 };
      // Do stuff to drive with the inputs.
      //m_robotContainer.drivetrain.arcadeDrive((Double) input[0], (Double) input[1]);
      // System.out.println((Double) input[0]);
      // arm.setRawSpeed((Double) input[2]);
      // conveyor.setSpeed((Double) input[3]);
      recorder.appendData(input);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
