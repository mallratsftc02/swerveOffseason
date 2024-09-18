// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Advancer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Doubles;

public class Robot extends TimedRobot {
	private Doubles m_doubkles;
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	private Intake m_intake;
	private Advancer m_advancer;
	private Shooter m_shooter;
  
	private static final String kDefaultAuto = "4 note Auto";
	private static final String k3note = "3 note auto";
	private static final String k2note = "2 note Auto";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	private final XboxController m_controller = new XboxController(1);
	// private final XboxController m_controller2 = new XboxController(1);
	private final Joystick m_drive2 = new Joystick(0);
	private XboxController m_opp;
	private XboxController m_clibe;
	private final Drivetrain m_swerve = new Drivetrain();

	// Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
	private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
	private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
	
	private boolean shoot = false;
	private boolean prevShooterReady = false;
	private boolean shootVariable = false;
	private boolean shooterIntake = false;
	private boolean alignToTop = false;
	private boolean floorIntake = false;
	private boolean bumberLPressed = false;
	private boolean YbuttonPressed = false;
	private boolean XbuttonPressed = false;
	private boolean HasNote = true;
	private int Sate = 0;
	private boolean intake = false;
	private boolean shooooot = true;
	private double top_shooter_save = 0;
	private double bottom_shooteer_save = 0;
	//limelight stuff
	boolean bumberL;
	boolean bumberR;

	double driveX, driveY, driveRot;
	double starttime = 0;
	double i = 0.1;
	double ii = 0;
	double iii = 0;
	double Top_shooteer = 0.0;
	double bottom_shooteer = 0.0;
	double x;
	double y;
	double area;
	
	 

	@Override
	public void robotInit() {
		SmartDashboard.putNumber("Top shooteer", Top_shooteer);
		SmartDashboard.putNumber("bottom shooter", bottom_shooteer);
    	driveX = 0;
		driveRot = 0;
		driveY = 0;
		m_chooser.setDefaultOption("4 note Auto", kDefaultAuto);
		m_chooser.addOption("3 note auto", k3note);
		m_chooser.addOption("2 note Auto", k2note);
		SmartDashboard.putData("Auto choices", m_chooser);
		m_opp = new XboxController(1);
		m_clibe = new XboxController(2);
		for (int port = 5800; port <= 5807; port++) {
			PortForwarder.add(port, "limelight.local", port);
		}

		CameraServer.startAutomaticCapture();

		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();
		m_shooter = new Shooter(15, 16);
		m_advancer = new Advancer (12, 0, 1);
		m_intake = new Intake(14);
		
		m_doubkles = new Doubles();
	}
  @Override
	public void robotPeriodic() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		NetworkTableEntry tl = table.getEntry("tl");
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		try {
			Top_shooteer = SmartDashboard.getNumber("Top shooteer", 1);
			bottom_shooteer = SmartDashboard.getNumber("bottom shooteer", 1);
		} catch (Exception e) {}
		if (RobotState.isAutonomous()) {
		driveWithJoystick(false);
		} else {
		driveWithJoystick(true);
		}
	
		if (m_controller.getAButton() && m_controller.getYButton()) {
		m_swerve.ResetDrives(); 
		}
	}
  
	private void driveWithJoystick(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		final var xSpeed =
			-m_xspeedLimiter.calculate(MathUtil.applyDeadband(driveY * -1, 0.3))
				* Drivetrain.kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		final var ySpeed =
			-m_yspeedLimiter.calculate(MathUtil.applyDeadband(driveX * -1, 0.3))
				* Drivetrain.kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		final var rot = /* FWF - removed a * -1 here to try to fix the park/rotate problem */
			m_rotLimiter.calculate(MathUtil.applyDeadband(driveRot * -0.5, 0.3))	// this was 0.2, Mick asked for a larger deadband
				* Drivetrain.kMaxAngularSpeed;

		m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);

		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		
	}
	@Override 
	public void disabledPeriodic() {
		driveX = 0;
		driveRot = 0;
		driveY = 0;
	}
	@Override
	public void testPeriodic() {
	}
	@Override
	public void autonomousInit() {
		Sate = 0;
		starttime = Timer.getFPGATimestamp();
		driveX = 0;
		driveRot = 0;
		driveY = 0;
		i = 0.1;
		ii = 0;
		iii = 0;
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
    	}
    	m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
  	}
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("AH", Sate);
		driveWithJoystick(false);
		m_swerve.updateOdometry();
		if (alignToTop) {
			if (m_advancer.AlignToTop()) {
				alignToTop = false;
				m_shooter.Stop();
				m_advancer.Stop();
			}

		}
		if (shoot) {
			if (m_shooter.Shoot()) {
				if (m_advancer.AdvanceToShoot()) {
					Sate ++;
					m_shooter.Stop();
					m_advancer.Stop();
				}
			}
		}
		if (shootVariable) {
			if (m_shooter.ChangeShoot(Top_shooteer, bottom_shooteer)) {
				if (m_advancer.AdvanceToShoot()) {
					shootVariable = false;
					m_shooter.Stop();
					m_advancer.Stop();
				}
			}
		}
		if (intake) {
			if (m_intake.Load()) {
				if (m_advancer.AdvanceToTop()) {
					m_intake.Stop();
					alignToTop = true;
				}
			}
		}
		switch (m_autoSelected) {
			
			case k2note:
				if (Sate == 0) {
					Top_shooteer = -0.5;
					bottom_shooteer = 0.5;
					shootVariable = true;
				} 
				if (Sate == 0 && m_shooter.ChangeShoot(Top_shooteer, bottom_shooteer)) {
					Sate = 1;
				}
				if (Sate == 1) {
					driveY = -.4;
					intake = true;
				} if (Sate == 1 && m_advancer.AlignToTop()) {
					driveY = 0;
					Sate = 2;
				}
				if (Sate == 2) {
					shoot = true;
				}
				break;

			case k3note:
				
				break;
		
			default:
				break;
		}
	}

	@Override
	public void teleopInit() {
		SmartDashboard.putNumber("Top Shooteer real", top_shooter_save);
		SmartDashboard.putNumber("bottom shooteer real", bottom_shooteer_save);
		driveX = 0;
		driveRot = 0;
		driveY = 0;
    
		shoot = false;
		shootVariable = false;
		shooterIntake = false;
		alignToTop = false;
		floorIntake = false;
		bumberLPressed = false;
		YbuttonPressed = false;
		XbuttonPressed = false;

		prevShooterReady = false;

		/*
			try{
			TimeUnit.SECONDS.sleep(10);
			} catch (Exception e) {}
		*/    
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
  	}
  	double deaddband (double inVal) {
    	double retVal = inVal;
    	if (retVal >= -0.3 && retVal <= 0.3) {
      		retVal = 0;
    	}
    	return retVal;
	}
	@Override
	public void teleopPeriodic() {
		Top_shooteer = SmartDashboard.getNumber("Top Shooteer real", top_shooter_save);
		bottom_shooteer = SmartDashboard.getNumber("bottom shooteer real", bottom_shooteer_save);
		top_shooter_save = Top_shooteer;
		bottom_shooteer_save = bottom_shooteer;
		bumberL = m_opp.getLeftBumper(); //|| m_driver.getLeftBumper();
		bumberR = m_opp.getRightBumper(); //|| m_driver.getRightBumper();

		/* buttons
		 * 
		 * a - shoot
		 * b - cancel
		 * y - intake on shooter
		 * left bump - intake
		 * right bump - outtake
		 * 
		 */
		if (bumberR) {bumberLPressed = false; shoot = false; XbuttonPressed = false;}
		if (m_opp.getAButtonPressed()) {
			if (shoot) {
				m_shooter.Stop();
				m_advancer.Stop();
			}
			shoot = !shoot;
			prevShooterReady = false;
		}
		if (m_opp.getXButtonPressed()) {
			if (shootVariable) {
				m_shooter.Stop();
				m_advancer.Stop();
			}
			shootVariable = !shootVariable;
		}
		
		if (m_opp.getYButtonPressed()) {shooterIntake = !shooterIntake;}
		if (bumberL) {floorIntake = true; bumberLPressed = true; shoot = false; XbuttonPressed = false;} 
		//if (m_opp.getXButtonPressed()) {alignToTop = true; XbuttonPressed = true;}
		//if (m_opp.getYButtonPressed()) {YbuttonPressed = true; shoot = false; XbuttonPressed = false;}
		
		if (m_opp.getBButton()) {
			m_intake.Stop();
			m_advancer.Stop();
			m_shooter.Stop();
			shoot = false;
			prevShooterReady = false;
			floorIntake = false;
			YbuttonPressed = false;
			XbuttonPressed = false;
		}

		if (alignToTop) {
			if (m_advancer.AlignToTop()) {
				alignToTop = false;
				m_shooter.Stop();
				m_advancer.Stop();
			}

		}
		if (shoot) {
			if (m_shooter.Shoot()) {
				if (prevShooterReady) {
					if (m_advancer.AdvanceToShoot()) {
						shoot = false;
						prevShooterReady = false;
						m_shooter.Stop();
						m_advancer.Stop();
					}
				} else {
					prevShooterReady = true;
				}
			}
		}
		if (shootVariable) {
			if (m_shooter.ChangeShoot(Top_shooteer, bottom_shooteer)) {
				if (m_advancer.AdvanceToShoot()) {
					shootVariable = false;
					m_shooter.Stop();
					m_advancer.Stop();
				}
			}
		}
		if (shooterIntake) {
			if (m_shooter.Intake()) {
				if (m_advancer.RetractToBottom()) {
//					if (m_advancer.alignToTop()) {
						shooterIntake = false;
						m_shooter.Stop();
						m_advancer.Stop();
						alignToTop = true;
//					}
				}
			}
		}
		if (floorIntake) {
			if (m_intake.Load()) {
				if (m_advancer.AdvanceToTop()) {
					floorIntake = false;
					m_intake.Stop();
					alignToTop = true;
				}
			}
		}
    
		if(m_drive2.getTrigger()) {
			m_swerve.ResetDrives();
		}
		
		double var = 0;
		if (var == 0) {
			driveX = 0;
			driveRot = 0;
			driveY = 0;
			var = 1;
		}
		
		/* comment this out and restart the robot with the wheels misaligned to see if the wheels try to align */
		if (m_drive2.getRawButtonPressed(7)) {
			SmartDashboard.putNumber("X", m_doubkles.X);
		} else {
			driveX =  m_drive2.getX();
		}

		
		driveY =  m_drive2.getY();
		driveRot = m_drive2.getTwist();

		/* junk lines just to clear the not used warnings */
		bumberLPressed = m_clibe.getLeftBumperPressed();
		
	}

		// this is run when the robot enters disabled
	@Override
	public void disabledInit() {
		m_intake.Stop();
		m_advancer.Stop();
		m_shooter.Stop();
		shoot = false;
		shootVariable = false;
		shooterIntake = false;
		alignToTop = false;
		floorIntake = false;
		bumberLPressed = false;
		YbuttonPressed = false;
		XbuttonPressed = false;
	}
}