package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    private CANSparkMax upperShooter;
    private CANSparkMax lowerShooter;
    private double Time = 0.0;
    
    public Shooter (int upperAddress, int lowerAddress) {
        upperShooter = new CANSparkMax (upperAddress, MotorType.kBrushless);
        lowerShooter = new CANSparkMax (lowerAddress, MotorType.kBrushless);
    }

    private boolean RunShooter (double shooter1Speed_in, double shooter2Speed_in) {
        upperShooter.set(shooter1Speed_in);
        lowerShooter.set(shooter2Speed_in);
        

        return (Math.abs(upperShooter.getEncoder().getVelocity() / 5300.0) >= Math.abs(0.9 * shooter1Speed_in))
            && (Math.abs(lowerShooter.getEncoder().getVelocity() / 5300.0) >= Math.abs(0.9 * shooter2Speed_in)) ;
    }

    public boolean Shoot () {
        Time = 0.0;
//        return (RunShooter(-1.0, 1.0));
        return (RunShooter(-0.15, 1.0));
    }
    public boolean Intake () {
        RunShooter(0.2, -0.2);
        return true;
//        return (RunShooter(0.5, -0.5));
    }
    public void Stop () {
        RunShooter(0, 0);
    }
    public boolean ChangeShoot (double top_shooter, double bottom_shooter) {
        Time = 0.0;
        return (RunShooter(top_shooter, bottom_shooter));
    }
}
