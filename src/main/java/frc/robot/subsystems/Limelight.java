// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.proto.Pose2dProto;
// import edu.wpi.first.math.geometry.struct.Pose2dStruct;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.subsystems.Doubles;
// public class Limelight {
//     Doubles d;
    
//     //read values periodically
//     public double x = tx.getDouble(0.0);
//     public double y = ty.getDouble(0.0);
//     public double area = ta.getDouble(0.0);
//     double q;
//     double driveX = 0.0;

//     public Limelight() {
//         d = new Doubles();
//         //post to smart dashboard periodically
//         SmartDashboard.putNumber("LimelightX", x);
//         SmartDashboard.putNumber("LimelightY", y);
//         SmartDashboard.putNumber("LimelightArea", area);
//     }

    
    
//     public Doubles newDriver () {
//         driveX = 0.0;
//         if(x >= 1) {
//             driveX = 0.3;
//         } else if (x <= -1) {
//             driveX = -0.3;
//         }
//         d.X = driveX;

//         return d;
//     }
// }
