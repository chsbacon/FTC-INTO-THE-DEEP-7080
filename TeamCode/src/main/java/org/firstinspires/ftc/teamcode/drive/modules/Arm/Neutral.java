package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class Neutral implements ArmState {

    private static final int NEUTRAL_EXTENDER_ENCODER_TICK = 20;

    @Override
    public void move(ArmController armController) {
//        TODO: Implement this method
        armController.extendSlideToTick(NEUTRAL_EXTENDER_ENCODER_TICK,0);
        armController.rotateSlide(true,.5);
        armController.moveClaw(false);
        armController.moveWristToAngle(0); // TODO: 1/14/25 mesure

    }
}
