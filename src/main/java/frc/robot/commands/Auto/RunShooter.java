// package frc.robot.commands.Auto;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.Telop.Shoot;
// import frc.robot.subsystems.BeamBreaks;

// public class RunShooter extends Command{
//     Shoot s;
//     BeamBreaks b;
//     public void execute() {
//         s = new Shoot();
//         b = new BeamBreaks();
//         if(!b.m_input0){
//         s.ShooterPower(
//             1, 
//             1, 
//             false,
//             true, 
//             false, 
//             b.m_input0, 
//             b.m_input1 );
//         } else {
//             s.ShooterPower(
//             1, 
//             1, 
//             false,
//             false, 
//             false, 
//             b.m_input0, 
//             b.m_input1 );
//         }
//     }
// }
