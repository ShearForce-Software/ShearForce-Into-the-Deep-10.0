/* This code was used during the Freight Frenzy season to test LG's arm servo positions.
It uses a SCALE factor to incrementally move the servo to allow for precise positioning.
Code was developed by Jacob Lemanski and previously list in the menu as "Jacob Servo Test"
*/

/* Code developed to test the servos on Geronimo.  Instead of using fixed step increments
which may not be as accurate as needed for precise mechanism placement, code uses joysticks and scaling
factor to define servo movement step.

Triangle + Left Stick Y = move both smallArmHangerServos
Triangle + Left Stick X = move intakeBoxRotaterServo (rotate urchin)
Square + Left Stick Y = move clawServo
Circle + Left Stick Y = move urchinServo
X + Left Stick Y = move both swiper servos

Note: Servos initialize to midpoint of servo travel
*/

package org.firstinspires.ftc.teamcode.Geronimo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Config // added for FTC dashboard usage
//@Disabled
@TeleOp(name = "Geronimo Intake Servo Test")
public class Geronimo_Intake_Servo_Test extends LinearOpMode {
    static final double SCALE   = 0.01;     // Joystick scaling for servo increment value
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo clawServo;
    Servo intakeBoxRotaterServo; // rotate urchin
    Servo smallArmHangerLeftServo;
    Servo smallArmHangerRightServo;
    Servo urchinServo; //open close urchin
    Servo swiperServo; //original swiper (right side)
    Servo swiper2; // added swiper (left side)

    double init_position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double claw_position = 0;
    double urchin_claw_position = 0;
    double urchin_rotate_position = 0;
    double swiper_position = 0;
    double small_rotator_position = 0;

    @Override
    public void runOpMode() {

        // Change the text in quotes to match any servo name on your robot.
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeBoxRotaterServo = hardwareMap.get(Servo.class, "intakeRotater");
        smallArmHangerLeftServo = hardwareMap.get(Servo.class, "intakeHangerLeft");
        smallArmHangerRightServo = hardwareMap.get(Servo.class, "intakeHangerRight");
        urchinServo = hardwareMap.get(Servo.class, "urchinServo");
        swiperServo = hardwareMap.get(Servo.class, "swiper");
        swiper2 = hardwareMap.get(Servo.class, "swiper2");

        // Set servo directions (useful for paired servos on opposite sides of arm)
        smallArmHangerLeftServo.setDirection(Servo.Direction.FORWARD);
        smallArmHangerRightServo.setDirection(Servo.Direction.REVERSE);
        swiperServo.setDirection(Servo.Direction.FORWARD);
        swiper2.setDirection(Servo.Direction.REVERSE);

        // Move servos to midpoint of servo travel
        clawServo.setPosition(claw_position); // init closed
        urchinServo.setPosition(urchin_claw_position); // init closed
        intakeBoxRotaterServo.setPosition(init_position);
        smallArmHangerLeftServo.setPosition(init_position);
        smallArmHangerRightServo.setPosition(init_position);
        swiperServo.setPosition(init_position);
        swiper2.setPosition(init_position);

        // Wait for the start button
        telemetry.addData(">", "Press Start to test servo movement.");
        telemetry.addData(">", "Triangle + Left Stick Y = rotate small Arms");
        telemetry.addData(">", "Triangle + Left Stick X = rotate urchin");
        telemetry.addData(">", "Square + Left Stick Y = move clawServo");
        telemetry.addData(">", "Circle + Left Stick Y = move urchinServo");
        telemetry.addData(">", "X + Left Stick Y = move both swiper servos");
        telemetry.update();
        waitForStart();


        // Test servo positions until stop pressed.
        while (opModeIsActive()) {

            // Rotate Small Arms -- Triangle + Left Stick Y
            if (gamepad1.left_stick_y != 0 && gamepad1.triangle) {
                // Keep stepping until servo reaches the min/max value.
                small_rotator_position += gamepad1.left_stick_y * SCALE;

                if (small_rotator_position >= MAX_POS) {
                    small_rotator_position = MAX_POS;
                }
                if (small_rotator_position <= MIN_POS) {
                    small_rotator_position = MIN_POS;
                }
            }
            smallArmHangerLeftServo.setPosition(small_rotator_position);
            smallArmHangerRightServo.setPosition(small_rotator_position);

            // Rotate Urchin -- Triangle + Left Stick X
            if (gamepad1.left_stick_x != 0 && gamepad1.triangle) {
                // Keep stepping until servo reaches the min/max value.
                urchin_rotate_position += gamepad1.left_stick_x * SCALE;

                if (urchin_rotate_position >= MAX_POS) {
                    urchin_rotate_position = MAX_POS;
                }
                if (urchin_rotate_position <= MIN_POS) {
                    urchin_rotate_position = MIN_POS;
                }
            }
            intakeBoxRotaterServo.setPosition(urchin_rotate_position);

            // Open/Close Claw -- Square + Left Stick Y
            if (gamepad1.left_stick_y != 0 && gamepad1.square) {
                // Keep stepping until servo reaches the min/max value.
                claw_position += gamepad1.left_stick_y * SCALE;

                if (claw_position >= MAX_POS) {
                    claw_position = MAX_POS;
                }
                if (claw_position <= MIN_POS) {
                    claw_position = MIN_POS;
                }
            }
            clawServo.setPosition(claw_position);

            // Open/Close Urchin -- Circle + Left Stick Y
            if (gamepad1.left_stick_y != 0 && gamepad1.circle) {
                // Keep stepping until servo reaches the min/max value.
                urchin_claw_position += gamepad1.left_stick_y * SCALE;

                if (urchin_claw_position >= MAX_POS) {
                    urchin_claw_position = MAX_POS;
                }
                if (urchin_claw_position <= MIN_POS) {
                    urchin_claw_position = MIN_POS;
                }
            }
            urchinServo.setPosition(urchin_claw_position);

            //Rotate Swiper Straws -- X + Left Stick Y
            if (gamepad1.left_stick_y != 0 && gamepad1.x) {
                // Keep stepping until servo reaches the min/max value.
                swiper_position += gamepad1.left_stick_y * SCALE;

                if (swiper_position >= MAX_POS) {
                    swiper_position = MAX_POS;
                }
                if (swiper_position <= MIN_POS) {
                    swiper_position = MIN_POS;
                }
            }
            swiperServo.setPosition(swiper_position);
            swiper2.setPosition(swiper_position);

            // Display the commanded servo value
            telemetry.addData("Small Arm Hanger Servo Positions", "%5.2f", small_rotator_position);
            telemetry.addData("Urchin Rotate Servo Position", "%5.2f", urchin_rotate_position);
            telemetry.addData("Claw Servo Position", "%5.2f", claw_position);
            telemetry.addData("Urchin Claw Servo Position", "%5.2f", urchin_claw_position);
            telemetry.addData("Swiper Servo Positions", "%5.2f", swiper_position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        }
    }
}