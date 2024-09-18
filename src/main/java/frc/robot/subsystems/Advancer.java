package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Advancer {
    private CANSparkMax advancer;
    private DigitalInput UpperBeam;
    private DigitalInput LowerBeam;
    private int advancerStep = 0;

    public Advancer (int advancerAddress, int upperAddress, int lowerAddress) {
        advancer = new CANSparkMax (advancerAddress, MotorType.kBrushless);
        UpperBeam = new DigitalInput(upperAddress);
        LowerBeam = new DigitalInput(lowerAddress);
        advancerStep = 0;
    }
    
    public boolean AdvanceToTop() {
        if (UpperBeam.get()) {
            advancer.set(-1);
        } else {
            advancer.set(0);
            return true;
        }
        return false;
    }
    public boolean AlignToTop() {
        boolean currentBeam = UpperBeam.get();
        switch (advancerStep) {
        case 0:
            if (!currentBeam) {
                advancer.set(0.2);
            } else {
                advancer.set (0);
                advancerStep = 1;
            }
        break;
        case 1:
            if (currentBeam) {
                advancer.set(-0.3);
            } else {
                advancer.set (0);
                advancerStep = 0;
                return true;
            }
        }
        return (false);
    }

    public boolean AdvanceToShoot() {
        SmartDashboard.putBoolean("upper beam", UpperBeam.get());
//       advancer.set((UpperBeam.get()? 0 : 1));
        if (UpperBeam.get()) {
            advancer.set(0);
            return true;
        } else {
            advancer.set(-1);
            return (false);
        }
    }

    public boolean AdvanceToClear () {
        advancer.set ((!UpperBeam.get()? 1 : 0));
        return (!UpperBeam.get());
    }

     public boolean RetractToBottom () {
        advancer.set ((LowerBeam.get()? 1.0 : 0.0));
        return (!LowerBeam.get());
    }

     public boolean RetractToRelease () {
        advancer.set ((!LowerBeam.get()? -1 : 0));
        return (!LowerBeam.get());
    }

    public boolean RingHeld (){
        return (!LowerBeam.get() || !UpperBeam.get());
    }
    public boolean RingAtTop (){
        return (!UpperBeam.get());
    }
    public boolean RingAtBottom (){
        return (!LowerBeam.get());
    }
    public boolean Stop () {
        advancer.set(0);

        return (true);
    }   
}
