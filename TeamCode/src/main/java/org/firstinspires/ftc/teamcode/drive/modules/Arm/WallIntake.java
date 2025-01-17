package org.firstinspires.ftc.teamcode.drive.modules.Arm;

import java.util.Objects;

@Deprecated
public class WallIntake implements ArmState{

    public void move(ArmController armController){

        if (!Objects.equals(armController.getCurrentState(), new GroundIntake())){
            armController.extendSlideToTick(700, 0.5, true);
        }
        armController.rotateSlide(false, 0.5, false);
        armController.moveWristToAngle(0.446);
        armController.moveClaw(true);
        armController.twistClawToAngle(0.13);
    }
}
