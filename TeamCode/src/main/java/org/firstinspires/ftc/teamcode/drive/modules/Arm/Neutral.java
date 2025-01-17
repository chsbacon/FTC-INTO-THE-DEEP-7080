package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class Neutral implements ArmState {

    private static final int NEUTRAL_EXTENDER_ENCODER_TICK = 700;

    @Override
    public void move(ArmController armController) {

        armController.extendSlideToTick(NEUTRAL_EXTENDER_ENCODER_TICK,0.5,true); //wait until extended
        armController.rotateSlide(true,.5,true); //rotate up
        armController.extendSlideToTick(20,0.5,false);
        armController.moveClaw(false);
        armController.twistClawToAngle(.13                                                                              );
        armController.moveWristToAngle(1); // TODO: 1/14/25 mesure

    }
}
