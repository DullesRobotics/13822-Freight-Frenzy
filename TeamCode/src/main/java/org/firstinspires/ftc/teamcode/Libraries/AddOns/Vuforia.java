package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;

import java.util.UUID;

public class Vuforia extends AddOn {

    private Robot r;
    private VuforiaLocalizer vuforia;

    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private UUID vuforiaThreadID;

    private VuforiaLocalizer.CameraDirection cameraDirection;
    private volatile boolean phoneIsPortrait;
    private final static String licenseKey = "AdtCIzH/////AAAAGVkbDFcppkWGkqpLjdBEavWJ2uW/CgCrc" +
            "Md74zFJYJgq1RfL+bjzIAzhefr6rvFBhvoSqYKp8FeNeJgnwNsnJ7qj/XTve5QijLoCzjf/qjXJ0N5wfzLY" +
            "45ycBm0X7awOau1NcBOrU2/XvQWmawD79QDnHWRlBivh84Qx72CdTHWRA/BoJhXMugJIMolVxQ7kcfJwL6S" +
            "uYgO5cCB8Vk4SFmNRpb0LwJiNs3ICBULjnLECi3VH4OWAsQGneogMn9I+Ngrq1cKl+ko3Wy2tav0MmD6KPd" +
            "ikSnhQRRyEK6vd93Npntt6p5+XoQ9P7kMx2ERvWVZJIljE6+OsrQgT7S7twchFljtlP4Ou5lRFKbb8gLZO";

    /**
     * @param cameraDirection The direction of the camera (front or back
     * @param phoneIsPortrait If the phone is portrait
     */
    public Vuforia (VuforiaLocalizer.CameraDirection cameraDirection, boolean phoneIsPortrait) {
        super(AddOnType.VUFORIA);
        this.cameraDirection = cameraDirection;
        this.phoneIsPortrait = phoneIsPortrait;
    }

    @Override
    protected void initAO(Robot r) {
        this.r = r;

        int cameraMonitorViewId = r.op().hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", r.op().hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        Vparameters.cameraDirection = cameraDirection;
        Vparameters.vuforiaLicenseKey = licenseKey;
        this.vuforia = ClassFactory.getInstance().createVuforia(Vparameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    @Override
    protected void startAO() {
        relicTrackables.activate();
        vuforiaThreadID = r.addThread(new Thread(() -> {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                r.getLogger().putData("VuMark", vuMark + " visible");

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                r.getLogger().putData("Pose", pose);

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
        }), true);
    }

    @Override
    protected void stopAO() {
        if(vuforiaThreadID != null && r.getThread(vuforiaThreadID) != null)
            r.getThread(vuforiaThreadID);
        relicTrackables.deactivate();
    }
}
