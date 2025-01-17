package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class HighChamberDeposit implements ArmState {

    private static final int UNSTICK = 700;
    private static final int HIGH_CHAMBER_EXTENDER_ENCODER_TICK = 0;

    @Override
    public void move(ArmController armController) {
        //        TODO: Implement this method
        armController.extendSlideToTick(UNSTICK, 0.5,true);//Wait till unstuck
        armController.rotateSlide(true, .5,false);//Rotate if need be
        armController.extendSlideToTick(HIGH_CHAMBER_EXTENDER_ENCODER_TICK, 0.5,false);//Extend remaining
        armController.moveClaw(true);
        armController.twistClawToAngle(.13);
        armController.moveWristToAngle(0);

    }
}
