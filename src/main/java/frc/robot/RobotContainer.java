package frc.robot;

import com.fasterxml.jackson.core.sym.Name;

//import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
*/
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
/*
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.Autos;
import frc.robot.subsystems.BeamBreaks;
import frc.robot.subsystems.Drivetrain;
*/
// import frc.robot.commands.Auto.RunIntake;
// import frc.robot.commands.Auto.RunShooter;
//import frc.robot.commands.Telop.Intake;
//import frc.robot.commands.Telop.Shoot;
//import frc.robot.commands.Auto.RunIntake;



public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
 
//  private Shoot m_Shoot;
//  private Intake m_intake;
//  private BeamBreaks b;
  

  public RobotContainer() {
    
    configureBindings();
    
   // m_Shoot = new Shoot(m_up.m_intake1Motor, m_up.m_advancerMotor, b.m_input, m_up.m_shooter1Motor, m_up.m_shooter2Motor);
    
    // NamedCommands.registerCommand("shoot", m_Shoot);
    // ...
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    
    
  }
  private void configureBindings() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // // Add a button to run pathfinding commands to SmartDashboard
    // SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   2.0
    // ));
    // SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
    //   new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
    //   new PathConstraints(
    //     4.0, 4.0, 
    //     Units.degreesToRadians(360), Units.degreesToRadians(540)
    //   ), 
    //   0, 
    //   0
    // ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
   
    
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}