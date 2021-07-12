//Run code for the red left position (1 wobble goal)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "RedLeftMontwood", group = "Taus")
public class RedLeftMontwood extends AutonomousMethods {

    @Override
    public void runOpMode() throws InterruptedException {

        //initializing robot
        initializeRobot();
        //detect number of rings
        int numberOfRings = findNumRings(bmp);
        telemetry.addData("rings", numberOfRings);
        telemetry.update();
        bmp.recycle();


        stopAndResetEncoders();
        controlIndexServo(1);
        controlBlocker(.375);

        //pick up wobble goal
        controlClawServo(.25);//close
        controlArmServo(.25);//down
        //shoot
        robot.shooter.setVelocityPIDFCoefficients(p,i,d,f);
        robot.shooter.setVelocity(powerShotPower);

        switch (numberOfRings){
            case 0:
                //code
                forward(.5, 2, 14);//16
                shoot(0, (2190*28)/60.0);
                forward(.5, 0, 14);//12
                strafeRight(.5, 1, 0);
                toAngle(-175, 1);
                dropWobbleGoal();
                strafeRight(.5, 1, 0);
                toAngle(0, 1);
                break;
            case 1:
                //code
                forward(.5, 2, 14);//16
                shoot(0, (2190*28)/60.0);
                forward(.5, 1, 17);//15
                toAngle(-150, 1);
                dropWobbleGoal();
                strafeRight(.5, 0, 3);
                toAngle(-180, 1);
                forward(.5, 1, 9);
                toAngle(0, 1);
//                toAngle(270, 1);
//                setIntakePower(1);
//                backward(.5, 0, 12);
//                forward(.5, 0, 12);
//                toAngle(360, 1);
//                forward(.5, 0, 20);
//                shoot(2, (2195*28)/60.0);
//                toAngle(0, 1);
//                forward(.5, 0, 12);
                break;
            case 4:
                //code
                sleep(4000);
                setShooterPower((2264*28)/60.0);
                forward(.5, 1, 15);
                shoot(-5, (2264*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(1, 0, 7, 6, 2, 0);
                forward(1, 0, 7);
                shoot(-5, (2265*28)/60.0);
                setShooterPower((2190*28)/60.0);
                toAngle(-90, 1);
                setIntakePower(1);
                backward(.1, 0, 8);
                backward(.05, 0, 10);
                forward(.5, 0, 18);
                toAngle(0, 1);
                forward(.5, 0, 18);
                shoot(0, (2190*28)/60.0);
                toAngle(0, 1);
                forward(.5, 2, 12);
                strafeRight(.5, 1, 0);
                toAngle(155, 1);
                dropWobbleGoal();
//                strafeRight(.5, 0, 20);
//                toAngle(-180, 1);
//                forward(.5, 1, 6);
//                toAngle(0, 1);
                break;
        }

    }
}
