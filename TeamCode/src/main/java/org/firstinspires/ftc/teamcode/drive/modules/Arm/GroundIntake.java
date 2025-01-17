package org.firstinspires.ftc.teamcode.drive.modules.Arm;

public class GroundIntake implements ArmState {

    private static final int GROUND_EXTENDER_ENCODER_TICK = 0;
    private static final int WRIST_INTAKE_ANGLE = 0
    ;


    @Override
    public void move(ArmController armController) {
        armController.extendSlideToTick(GROUND_EXTENDER_ENCODER_TICK, .5);
        armController.rotateSlide(false, .5);
        armController.moveWristToAngle(WRIST_INTAKE_ANGLE);
        armController.moveClaw(true);
        armController.twistClawToAngle(0); // TODO: 1/14/25 mesure
    }
}
