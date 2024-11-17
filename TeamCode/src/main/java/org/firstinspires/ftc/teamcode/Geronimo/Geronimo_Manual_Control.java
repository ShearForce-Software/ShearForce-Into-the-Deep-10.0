package org.firstinspires.ftc.teamcode.Geronimo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Geronimo 1 Manual Control")
//@Disabled
public class Geronimo_Manual_Control extends LinearOpMode {
    Geronimo theRobot;

    public void runOpMode() {
        theRobot = new Geronimo(true, false, this);

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
                //theRobot.driveControlsRobotCentric();

                if(gamepad1.y){
                    theRobot.imu.resetYaw();
                }

                /* *************************************************
                 *************************************************
                 * Gunner / Arm Controls (gamepad2)
                 *
                 *************************************************
                 *************************************************
                 */
                // SLIDE MOTOR CONTROL
                if ((gamepad2.left_stick_y > 0.1) || (gamepad2.left_stick_y <= -0.1)) {
                    theRobot.SetSlidesPower(gamepad2.left_stick_y);
                }
                else
                {
                    theRobot.SetSlidesPower(0.0);
                }

                // Rotater MOTOR CONTROL
                if ((gamepad2.right_stick_y > 0.1) || (gamepad2.right_stick_y <= -0.1)) {
                    theRobot.SetRotatorArmPower(gamepad2.right_stick_y);
                }
                else if (gamepad2.circle) {
                    theRobot.SetRotatorToPosition(250);
                }
                else if (gamepad2.triangle) {
                    theRobot.SetRotatorToPosition(450);
                }
                else if (gamepad2.square) {
                    theRobot.SetRotatorToPosition(820);
                }
                else if (gamepad2.cross) {
                    theRobot.SetRotatorToPosition(0);
                }
                else if (theRobot.GetRotatorArmRunningToPosition())
                {
                    int leftDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorLeftArmCurrentPosition());
                    int rightDifference = Math.abs(theRobot.GetRotatorArmTargetPosition() - theRobot.GetRotatorRightArmCurrentPosition());
                    if ((leftDifference <= 2) || (rightDifference <= 2))
                    {
                        theRobot.SetRotatorArmHoldPositon();
                    }
                }
                else
                {
                    theRobot.SetRotatorArmPower(0.0);
                }

                // Claw Control
                if (gamepad2.right_bumper) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MAX_POS);
                }
                else if (gamepad2.left_bumper) {
                    theRobot.SetClawPosition(Geronimo.CLAW_MIN_POS);
                }

                // intake Hanger Control
                if (gamepad2.dpad_up) {
                    theRobot.SetHangerMaxUp();
                }
                else if (gamepad2.dpad_down)
                {
                    theRobot.SetHangerMaxDown();
                }
                else if (gamepad2.dpad_left)
                {
                    theRobot.HangerIncrementUp();
                }
                else if (gamepad2.dpad_right)
                {
                    theRobot.HangerDecrementDown();
                }

                // intake Star Control
                if (gamepad2.right_trigger > 0.1)
                {
                    theRobot.SetIntakeStarPower(gamepad2.right_trigger);
                }
                else if (gamepad2.left_trigger < -0.1)
                {
                    theRobot.SetIntakeStarPower(gamepad2.left_trigger);
                }
                else {
                    theRobot.SetIntakeStarPower(0.0);
                }

                // intake star rotator control
                if (gamepad1.dpad_up)
                {
                    theRobot.SetIntakeStarRotatorPosition(Geronimo.INTAKE_ROTATOR_MAX_POS);
                }
                else if (gamepad1.dpad_down)
                {
                    theRobot.SetIntakeStarRotatorPosition(Geronimo.INTAKE_ROTATOR_MIN_POS);
                }


                theRobot.ShowTelemetry();
                telemetry.update();
            } // end while (opModeIsActive())

        }
}
