package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    private CANSparkMax intake;

    public Intake (int intakeAddress) {
        intake = new CANSparkMax (intakeAddress, MotorType.kBrushless);
    }

    public boolean Load () {
        intake.set(-1);

        return (true);
    }
    public boolean Outtake () {
        intake.set(1);

        return (true);
    }
    public boolean Stop (){
        intake.set(0);

        return (true);
    }
}