package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Geronimo 1 Manual Control")
@Disabled
public class Geronimo_Manual_Control extends LinearOpMode {
    Geronimo theRobot;
    static final double SCALE = 0.001;
    public void runOpMode() {
        theRobot = new Geronimo(true, true, this);

        theRobot.Init(this.hardwareMap);
        theRobot.ShowTelemetry();

        telemetry.update();
        waitForStart();
        resetRuntime();


            while (opModeIsActive()) {
                theRobot.EndgameBuzzer();
                /* *************************************************
                 *************************************************
                 * Driver Controls (gamepad1)
                 *************************************************
                 *************************************************
                 */
                // Drive Controls uses left_stick_y, left_stick_x, and right_stick_x
                theRobot.driveControlsFieldCentric();
                if (gamepad1.a && !gamepad1.start) {
                   // theRobot.LaunchAirplane();
                }
                if (gamepad1.b && !gamepad1.start){
                    // Auto drives to the board using the arm distance sensor
                  //  theRobot.StopNearBoard();
                }
                if(gamepad1.y){
                    theRobot.imu.resetYaw();
                }
                if (gamepad1.left_trigger != 0) {
                /*    theRobot.SetScissorLiftPower(gamepad1.left_trigger);
                } else if (gamepad1.right_trigger != 0) {
                    theRobot.SetScissorLiftPower(-gamepad1.right_trigger);
                } else {
                    theRobot.SetScissorLiftPower(0); */
                }

                /* *************************************************
                 *************************************************
                 * Gunner / Arm Controls (gamepad2)
                 *
                 *************************************************
                 *************************************************
                 */
                if (gamepad2.right_bumper) { // intake in
                   // theRobot.EnableAutoIntake();
                }
                if (gamepad2.dpad_right) {
                  //  theRobot.ServoStop();
                  //  theRobot.DisableAutoIntake();
                }
                if (gamepad2.left_bumper) { // release pixels
                  //  theRobot.ReleaseRight();
                  //  theRobot.ReleaseLeft();
                }
                if (gamepad2.left_trigger != 0){
                  //  theRobot.ReleaseRight();
                }
                if (gamepad2.right_trigger != 0){
                 //   theRobot.ReleaseLeft();
                }
                // Slides HIGH
                if (gamepad2.y && !gamepad2.back) {
                   // theRobot.ServoStop();
                   // theRobot.SlidesHigh();
                   // theRobot.SpecialSleep(500);
                  //  theRobot.DeliverPixelToBoardPos();
                }
                // Slides MEDIUM
                if (gamepad2.x) {
                  //  theRobot.ServoStop();
                   // theRobot.SlidesMedium();
                  //  theRobot.SpecialSleep(500);
                  //  theRobot.DeliverPixelToBoardPos();
                }
                // Slides LOW
                if (gamepad2.a && !gamepad2.start) {
                //    theRobot.ServoStop();
                //    theRobot.SlidesToAuto();
                    //theRobot.SpecialSleep(500);
                  //  theRobot.DeliverPixelToBoardPos();
                }
                if (gamepad2.left_stick_button){
                    theRobot.SlidesRotating(0.1   );
                 //   theRobot.PickupOne();
                }
                // RESET Slides, ARM, and Wrist
                if (gamepad2.b && !gamepad2.start) {
                //    theRobot.ResetArm();
                }
                // Manual incremental control of the wrist
                if (gamepad2.right_stick_x > 0.1)
                {
                //    theRobot.ArmWrist(theRobot.getWristPosition() + 0.01);
                 //   theRobot.SpecialSleep(150);
                }
                else if (gamepad2.right_stick_x < -0.1)
                {
                 //   theRobot.ArmWrist(theRobot.getWristPosition() - 0.01);
                 //   theRobot.SpecialSleep(150);
                }
                // Manual incremental control to rotate the arm servo
                if (gamepad2.left_stick_y != 0) {
                /*    armRotationLeftPosition += -gamepad2.left_stick_y * SCALE;
                    armRotationRightPosition += -gamepad2.left_stick_y * SCALE;
                    if (armRotationLeftPosition >= MAX_POS) {
                        armRotationLeftPosition = MAX_POS;
                    }
                    if (armRotationLeftPosition <= MIN_POS_ARM) {

                        armRotationLeftPosition = MIN_POS_ARM;
                    }
                    if (armRotationRightPosition >= MAX_POS) {
                        armRotationRightPosition = MAX_POS;
                    }
                    if (armRotationRightPosition <= MIN_POS_ARM) {
                        armRotationRightPosition = MIN_POS_ARM;
                    }
                    theRobot.armRotLeft.setPosition(armRotationLeftPosition);
                    theRobot.armRotRight.setPosition(armRotationRightPosition);

                 */
                }
                // Manual control of the slides
                if (gamepad2.dpad_up)
                {
                    /*
                    // slowly raise the slides up
                    theRobot.leftSlide.setTargetPosition(-1800);
                    theRobot.rightSlide.setTargetPosition(-1800);
                    theRobot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    theRobot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    theRobot.SetSlidePower(.5);

                     */
                }
                if (gamepad2.dpad_down)
                {
                    // slowly lower the slides
                 //   theRobot.SlidesDown();
                }
                // special combo (left tiny button and top right button together)
                if (gamepad2.back && gamepad2.y) {
                   // theRobot.ResetWristGrabPixelPos();
                   // theRobot.SpecialSleep(150);
                }

                theRobot.ShowTelemetry();
                telemetry.update();
            } // end while (opModeIsActive())

            // Stop must have been pressed to get here, make sure the continuous rotation servos are stopped
           // theRobot.ServoStop();
          //  theRobot.DisableAutoIntake();
          //  sleep(150);

        }
}
