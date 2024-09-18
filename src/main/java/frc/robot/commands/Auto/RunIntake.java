// package frc.robot.commands.Auto;

// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.Telop.Shoot;
// import frc.robot.commands.Telop.Advance;
// import frc.robot.commands.Telop.Intake;
// import frc.robot.subsystems.BeamBreaks;

// public class RunIntake extends Command {
// 	Shoot s;
// 	BeamBreaks b;

//     public void execute() {
//         s = new Shoot();
// 		b = new BeamBreaks();
// 		if(b.m_input0) {
// 			s.IntakePower(true, 
// 			false, 
// 			true, 
// 			b.m_input0, 
// 			b.m_input1
// 			);
// 		} else {
// 			s.IntakePower(false, 
// 			false, 
// 			false, 
// 			b.m_input0, 
// 			b.m_input1
// 			);
// 		}
//     }

    
// }
