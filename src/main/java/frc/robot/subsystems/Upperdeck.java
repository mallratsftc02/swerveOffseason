// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.RobotState;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.RunIntake;
// import frc.robot.commands.Shoot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.BeamBreaks;

// public class Upperdeck {
//     BeamBreaks b = new BeamBreaks();
//     private double shooter1Speed_in = 1;
// 	private double shooter2Speed_in = -1;
// 	private double servoAmpAngle = 0.0;
// 	private double ampPower = 0;
// 	private boolean turnComplete = false;
//  	private boolean doSequence = false;
//  	private long sequenceTime = 0;
// 	private double globaltime = 0;
// 	private double ggvarr;
// 	private boolean iiii = true;
// 	private boolean reverseB = true;
// 	private boolean AbuttonPressedC;
// 	private boolean YbuttonPressedC;
// 	private boolean BButtonPressedC;
// 	private double rampy = 0.0;
// 	private boolean shooter_up_to_speed = false;
// 	private double light = 0.0;

// 	private double climb = 0.0;
// 	//limelight stuff
//   Boolean shooter1Target;
//   Boolean shooter2Target;
//   boolean bumberL;
// 	boolean bumberR;
//   private XboxController m_opp = new XboxController(1);
//    double intakeSpeed = 0;
// 	 double advancerSpeed = 0;
// 	 double shooter1Speed = 0;
// 	 double shooter2Speed = 0;
	
	
// 	private boolean yn = true;
// 	private boolean AButtonPressed = false;
// 	private boolean bumberLPressed = false;
// 	private boolean YbuttonPressed = false;
// 	private boolean XbuttonPressed = false;
// 	private boolean xx = false;
    
	
	
	
// 	
      
	  
	
// 	  //MotorController m_shoulderintakeMotor = new CANSparkMax(9,MotorType.kBrushless);
// 	// private final MotorController m_shouldermoveMotor = new CANSparkMax(10,MotorType.kBrushless);
// 	// private final MotorController m_shoulderrotateMotor = new CANSparkMax(3,MotorType.kBrushless);
// 	// private final MotorController m_climbMotor = new CANSparkMax(12,MotorType.kBrushless);
// 	private Spark m_climb2Motor;
// 	private Spark m_climb1Motor;
// 	private Spark m_outRig1;
// 	private Spark m_outRig2;
    
//     public Upperdeck(){

    
        
    
//         m_shooter1Motor.setInverted(false);
//         m_shooter2Motor.setInverted(true);
//         m_intake1Motor.setInverted(true);
		
//       }

//     public void intake() {
        
		
//     // NamedCommands.registerCommand("Check For Ball", Autos.exampleAuto(null) );
    
//     // Build an auto chooser. This will use Commands.none() as the default option.
    
//     SmartDashboard.putBoolean("shooter 1 RPM target", shooter1Target);
// 		SmartDashboard.putBoolean("shooter 2 RPM target", shooter2Target);
//     SmartDashboard.putBoolean("DIO 1", b.m_input);

// 		bumberL = m_opp.getLeftBumper(); //|| //m_driver.getLeftBumper();
// 		bumberR = m_opp.getRightBumper(); //|| //m_driver.getRightBumper();
		
// 		if (bumberL) {bumberLPressed = true; AButtonPressed = false; XbuttonPressed = false;} 
// 		if (m_opp.getXButton()) {XbuttonPressed = true;}
// 		if (m_opp.getYButtonPressed()) {YbuttonPressed = true; AButtonPressed = false; XbuttonPressed = false;}
// 		if (!b.m_input && !b.m_input1) {YbuttonPressed = false; xx = false;}
//         if (m_opp.getAButtonPressed()) {AButtonPressed = true;}

// 		if (m_opp.getBButton()) {
// 			shooter1Speed = 0;
// 			shooter2Speed = 0;
// 			advancerSpeed = 0;
// 			AButtonPressed = false;
// 			YbuttonPressed = false;
// 			XbuttonPressed = false;
// 			shooter_up_to_speed = false;
// 		}
// 		if (xx && !b.m_input || !YbuttonPressed && !AButtonPressed && !XbuttonPressed) {
// 					shooter1Speed = 0;
// 					shooter2Speed = 0;
// 					advancerSpeed = 0;
// 					AButtonPressed = false;
// 					XbuttonPressed = false;
// 					shooter_up_to_speed = false;
// 					yn = true;
// 				}
		

// 		if (!b.m_input || bumberR) {
// 			intakeSpeed = 0;
// 			bumberLPressed = false;
// 		}
// 		// m_outRig1.set(outRigC);
// 		// m_outRig2.set(outRigC);
// 		if (bumberLPressed && b.m_input){
// 			intakeSpeed = .5;
// 			advancerSpeed = -.6;
// 			bumberL = false;

// 		} else if (bumberL && !b.m_input) {
// 			intakeSpeed = 0;
// 			advancerSpeed = 0;

// 		} else if (bumberR) {
// 			intakeSpeed = -.5;
// 			advancerSpeed = .6;
// 		} else {
// 			intakeSpeed = 0;
// 			advancerSpeed = 0;
// 		}
		
//     }
    
//     public void Shoot() {
//         shooter1Target = Math.abs(m_shooter1Motor.getEncoder().getVelocity() / 5300.0) >= Math.abs(0.9 * shooter1Speed_in);
// 		shooter2Target = Math.abs(m_shooter2Motor.getEncoder().getVelocity() / 5300.0) >= Math.abs(0.9 * shooter2Speed_in);
// 		if (bumberL) {bumberLPressed = true; AButtonPressed = false; XbuttonPressed = false;} 
// 		if (m_opp.getXButton()) {XbuttonPressed = true;}
// 		if (m_opp.getYButtonPressed()) {YbuttonPressed = true; AButtonPressed = false; XbuttonPressed = false;}
// 		if (!b.m_input && !b.m_input1) {YbuttonPressed = false; xx = false;}
//         if (m_opp.getAButtonPressed()) {AButtonPressed = true;}
//         if (AButtonPressed && !b.m_input){
// 			shooter1Speed = (shooter1Speed_in)*-1;
// 			shooter2Speed = shooter2Speed_in;
// 			yn = false;
// 			if(shooter1Target && shooter2Target || shooter_up_to_speed) {
// 				shooter_up_to_speed = true;
// 				advancerSpeed = -0.6;
// 			}
// 		} else if (YbuttonPressed && b.m_input1) {
// 			shooter1Speed = .5;
// 			shooter2Speed = -.5;
// 			advancerSpeed = .6;
// 			xx = true;
// 		} else if (m_opp.getBButton()) {
// 			shooter1Speed = -1;
// 			shooter2Speed = 0.8;
// 			if (yn) {advancerSpeed = -1;}
// 		} else if (XbuttonPressed && !b.m_input) {
// 			shooter1Speed = -.60;
// 			shooter2Speed = .60;
// 			if (shooter1Target && shooter2Target || shooter_up_to_speed) {
// 				shooter_up_to_speed = true;
// 				advancerSpeed = -.6;
// 			} 
// 		}  else {
// 			shooter1Speed = (m_opp.getRightTriggerAxis()*-1)* .5;
// 			shooter2Speed = m_opp.getRightTriggerAxis()* .5;
// 		}
//     if (RobotState.isEnabled()) {
//       m_intake1Motor.set(intakeSpeed);
// 		 m_advancerMotor.set(advancerSpeed);
// 		 m_shooter1Motor.set(shooter1Speed);
// 		 m_shooter2Motor.set(shooter2Speed);
//     } else {
//       m_intake1Motor.set(0);
// 		m_advancerMotor.set(0);
// 		m_shooter1Motor.set(0);
// 		m_shooter2Motor.set(0);
//     }
//     }
// }
