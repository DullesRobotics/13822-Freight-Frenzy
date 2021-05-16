package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.USBWebcam;
import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.UUID;

@Config
public class EasyOpenCV extends AddOn {

    private Robot r;
    private OpenCvCamera camera;
    private final Pipeline pipeline;
    private UUID threadUUID;
    private CameraType cameraType;
    private WebcamName webcamName;
    private OpenCvCameraRotation phoneOrientation;
    private OpenCvInternalCamera.CameraDirection cameraDirection;

    /** The width and height to set the camera's dimensions to. Should be smaller to make it faster */
    public static int VIEWPORT_WIDTH = 320, VIEWPORT_HEIGHT = 240;

    /**
     * The EasyOpenCV constructor for usb webcams
     * @param pipeline The {@link Pipeline} to use for the robot. Use any object implementing
     * @param webcam the robot's {@link USBWebcam} that OpenCV will use
     */
    public EasyOpenCV(Pipeline pipeline, @Nullable USBWebcam webcam, OpenCvCameraRotation phoneOrientation){
        super(AddOnType.EASY_OPEN_CV);
        this.cameraType = CameraType.USB_WEBCAM;
        this.pipeline = pipeline;
        this.webcamName = webcam == null ? null : webcam.get();
    }

    /**
     * The EasyOpenCV constructor for internal phone cameras
     * @param pipeline The {@link Pipeline} to use for the robot. Use any object implementing
     * @param cameraDirection Which camera to use (Forward or Backwards)
     * @param phoneOrientation What orientation the camera is in.
     */
    public EasyOpenCV(Pipeline pipeline, OpenCvInternalCamera.CameraDirection cameraDirection, OpenCvCameraRotation phoneOrientation){
        super(AddOnType.EASY_OPEN_CV);
        this.cameraType = CameraType.INTERNAL_PHONE;
        this.pipeline = pipeline;
        this.cameraDirection = cameraDirection;
        this.phoneOrientation = phoneOrientation;
    }

    @Override
    protected void initAO(Robot r) {
        /* If no webcam exists, return null */
        if(cameraType == CameraType.USB_WEBCAM && webcamName == null) return;

        this.r = r;
        /*
         * Initializes the viewing monitor on the driver station phone
         */
       // int cameraMonitorViewId = r.op().hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", r.op().hardwareMap.appContext.getPackageName());

        /*
         * Depending on the type of camera, we use a different method
         */
        switch(cameraType){
            case USB_WEBCAM: camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName/*, cameraMonitorViewId*/); break;
            case INTERNAL_PHONE: camera = OpenCvCameraFactory.getInstance().createInternalCamera(cameraDirection/*, cameraMonitorViewId*/); break;
        }

        camera.setPipeline((OpenCvPipeline) pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)

                camera.startStreaming(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, phoneOrientation);
                FtcDashboard.getInstance().startCameraStream(camera, 30);

            }
        });
    }

    @Override
    protected void startAO() {
        /* If no webcam exists and it's set to webcam, stop */
        if(cameraType == CameraType.USB_WEBCAM && webcamName == null){
            stop();
            return;
        }

        threadUUID = r.addThread(new Thread(() -> {
            while(r.op().opModeIsActive()){
                pipeline.updateLog(r.getLogger());
                r.op().sleep(50); //TODO is this okay?
            }
        }), true);
    }

    @Override
    protected void stopAO() {
        Thread t = r.getThread(threadUUID);
        if(t != null) t.interrupt();
    }

    /**
     * Sets the opencv pipeline
     * @param pipeline The new pipeline to use
     */
    public void setPipeline(@NotNull Pipeline pipeline) {
        if(camera != null)
            camera.setPipeline((OpenCvPipeline) pipeline);
    }

    private enum CameraType {
        INTERNAL_PHONE,
        USB_WEBCAM
    }

}
