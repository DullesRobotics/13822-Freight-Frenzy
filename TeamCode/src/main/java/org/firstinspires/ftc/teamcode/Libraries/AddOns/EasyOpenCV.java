package org.firstinspires.ftc.teamcode.Libraries.AddOns;

import org.firstinspires.ftc.teamcode.RobotManager.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.UUID;

public class EasyOpenCV extends AddOn {

    private Robot r;
    private OpenCvInternalCamera phoneCam;
    private final Pipeline pipeline;
    private UUID threadUUID;
    private final OpenCvCameraRotation phoneOrientation;
    private final OpenCvInternalCamera.CameraDirection cameraDirection;

    /**
     *
     * @param pipeline The {@link Pipeline} to use for the robot. Use any object implementing
     * @param cameraDirection Which camera to use (Forward or Backwards)
     * @param phoneOrientation What orientation the camera is in.
     */
    public EasyOpenCV(Pipeline pipeline, OpenCvInternalCamera.CameraDirection cameraDirection, OpenCvCameraRotation phoneOrientation){
        super(AddOnType.EASY_OPEN_CV);
        this.pipeline = pipeline;
        this.cameraDirection = cameraDirection;
        this.phoneOrientation = phoneOrientation;
    }

    @Override
    protected void initAO(Robot r) {
        this.r = r;
        int cameraMonitorViewId = r.op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", r.op.hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(cameraDirection, cameraMonitorViewId);

        phoneCam.setPipeline((OpenCvPipeline) pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                phoneCam.startStreaming(320,240, phoneOrientation);
            }
        });
    }

    @Override
    protected void startAO() {
        threadUUID = r.addThread(new Thread(() -> {
            while(r.op.opModeIsActive()){
                pipeline.updateLog(r.getLogger());
                r.op.sleep(50); //TODO is this okay?
            }
        }), true);
    }

    @Override
    protected void stopAO() {
        Thread t = r.getThread(threadUUID);
        if(t != null) t.interrupt();
    }

}
