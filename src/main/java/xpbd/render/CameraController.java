package xpbd.render;

import javafx.scene.PerspectiveCamera;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

public final class CameraController {
    private final Rotate rotateX = new Rotate(-30, Rotate.X_AXIS);
    private final Rotate rotateY = new Rotate(0, Rotate.Y_AXIS);
    private final Translate translate = new Translate(0, 0, -20);

    private double anchorX, anchorY;
    private double anchorAngleX, anchorAngleY;
    private double anchorTx, anchorTy;

    public CameraController(PerspectiveCamera camera) {
        camera.getTransforms().addAll(rotateY, rotateX, translate);
        camera.setNearClip(0.1);
        camera.setFarClip(1000);
    }

    public void onMousePressed(MouseEvent e) {
        anchorX = e.getSceneX();
        anchorY = e.getSceneY();
        anchorAngleX = rotateX.getAngle();
        anchorAngleY = rotateY.getAngle();
        anchorTx = translate.getX();
        anchorTy = translate.getY();
    }

    public void onMouseDragged(MouseEvent e) {
        if (e.isPrimaryButtonDown()) {
            double dx = e.getSceneX() - anchorX;
            double dy = e.getSceneY() - anchorY;
            rotateX.setAngle(clamp(anchorAngleX + dy * 0.3, -89, 89));
            rotateY.setAngle(anchorAngleY - dx * 0.3);
        } else if (e.isSecondaryButtonDown() || e.isMiddleButtonDown()) {
            double dx = e.getSceneX() - anchorX;
            double dy = e.getSceneY() - anchorY;
            translate.setX(anchorTx + dx * 0.05);
            translate.setY(anchorTy - dy * 0.05);
        }
    }

    public void onScroll(ScrollEvent e) {
        translate.setZ(translate.getZ() + e.getDeltaY() * 0.05);
    }

    public void reset() {
        rotateX.setAngle(30);
        rotateY.setAngle(0);
        translate.setX(0);
        translate.setY(0);
        translate.setZ(-50);
    }

    /** 设置与鼠标拖拽、滚轮操作相同的相机视图状态。 */
    public void setView(double pitchDegrees, double yawDegrees,
                        double panX, double panY, double distance) {
        if (!Double.isFinite(pitchDegrees) || !Double.isFinite(yawDegrees)
                || !Double.isFinite(panX) || !Double.isFinite(panY)
                || !Double.isFinite(distance)) {
            throw new IllegalArgumentException("camera view values must be finite");
        }
        rotateX.setAngle(clamp(pitchDegrees, -89, 89));
        rotateY.setAngle(yawDegrees);
        translate.setX(panX);
        translate.setY(panY);
        translate.setZ(distance);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
