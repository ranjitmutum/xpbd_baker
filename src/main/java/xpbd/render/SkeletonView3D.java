package xpbd.render;

import javafx.scene.*;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.Cylinder;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Scale;
import xpbd.baker.BedrockTransformResolver;
import xpbd.baker.BoneMapper;
import xpbd.baker.CubeGeometry;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;

import java.util.*;

public final class SkeletonView3D {
    private final Group root = new Group();
    private final SubScene subScene;
    private final CameraController cameraController;

    private final Group skeletonGroup = new Group();
    private final Group physicsGroup = new Group();
    private final Group referenceGroup = new Group();
    private final Group environmentGroup = new Group();

    private BedrockModelData.Geometry geometry;
    private BoneMapper boneMapper;

    private List<BedrockModelData.Bone> boneList = new ArrayList<>();
    private final Map<String, Sphere> boneSpheres = new HashMap<>();
    private final Map<String, double[]> boneWorldPivots = new HashMap<>();
    private final Map<String, Group> boneGroups = new HashMap<>();

    private static final double POINT_RADIUS = 0.1;
    private static final double BONE_RADIUS = 0.04;
    private static final PhongMaterial DEFAULT_MATERIAL = new PhongMaterial(Color.LIGHTGRAY);
    private static final PhongMaterial PHYSICS_MATERIAL = new PhongMaterial(Color.DODGERBLUE);
    private static final PhongMaterial FIXED_MATERIAL = new PhongMaterial(Color.CRIMSON);
    private static final PhongMaterial REFERENCE_MATERIAL = new PhongMaterial(Color.rgb(180, 180, 180, 0.5));
    private static final PhongMaterial GRID_MATERIAL = new PhongMaterial(Color.rgb(220, 220, 220));
    private static final PhongMaterial GROUND_MATERIAL =
            new PhongMaterial(Color.rgb(70, 78, 92, 0.45));
    private static final PhongMaterial CUBE_MATERIAL = new PhongMaterial(Color.rgb(180, 180, 200, 0.3));

    public SkeletonView3D(double width, double height) {
        PerspectiveCamera camera = new PerspectiveCamera(true);
        camera.setFieldOfView(45);
        cameraController = new CameraController(camera);
        cameraController.reset();

        subScene = new SubScene(root, width, height, true, SceneAntialiasing.BALANCED);
        subScene.setCamera(camera);
        subScene.setFill(Color.rgb(30, 30, 35));

        // Bedrock 使用 Y 轴向上坐标，JavaFX 使用 Y 轴向下坐标，因此翻转场景内容的 Y 轴。
        Group sceneContent = new Group(
                environmentGroup, referenceGroup, skeletonGroup, physicsGroup);
        sceneContent.getTransforms().add(new Scale(1, -1, 1));
        root.getChildren().add(sceneContent);
        buildGroundAndGrid();
        buildLights();
    }

    public SubScene getSubScene() {
        return subScene;
    }

    public CameraController getCameraController() {
        return cameraController;
    }

    public void setGeometry(BedrockModelData.Geometry geo) {
        physicsGroup.getChildren().clear();
        referenceGroup.getChildren().clear();
        this.geometry = geo;
        this.boneList = geo.bones;
        rebuildSkeleton();
    }

    public void setBoneMapper(BoneMapper mapper) {
        this.boneMapper = mapper;
        refreshColors();
    }

    public void refreshPhysicsSelection() {
        physicsGroup.getChildren().clear();
        referenceGroup.getChildren().clear();
        refreshColors();
    }

    public void updatePhysicsFrame(PhysicsBaker baker) {
        if (boneMapper == null || geometry == null) return;
        physicsGroup.getChildren().clear();

        for (String boneName : boneMapper.getPhysicsBones()) {
            double[] worldPos = baker.getCurrentWorldPosition(boneName);
            if (worldPos == null) continue;

            Sphere s = new Sphere(POINT_RADIUS * 1.2);
            s.setMaterial(isRootFixed(boneName) ? FIXED_MATERIAL : PHYSICS_MATERIAL);
            s.setTranslateX(worldPos[0]);
            s.setTranslateY(worldPos[1]);
            s.setTranslateZ(worldPos[2]);
            physicsGroup.getChildren().add(s);
        }

        for (String boneName : boneMapper.getPhysicsBones()) {
            BedrockModelData.Bone bone = ModelLoader.findBoneByName(boneList, boneName);
            if (bone == null || bone.parent == null) continue;
            if (!boneMapper.isPhysicsBone(bone.parent)) continue;

            double[] pA = baker.getCurrentWorldPosition(bone.parent);
            double[] pB = baker.getCurrentWorldPosition(boneName);
            if (pA == null || pB == null) continue;

            drawCylinderBetween(
                    pA[0], pA[1], pA[2],
                    pB[0], pB[1], pB[2],
                    BONE_RADIUS * 0.5,
                    Color.rgb(100, 180, 255, 0.7),
                    physicsGroup
            );
        }
    }

    public void updateReferenceFrame(PhysicsBaker baker) {
        referenceGroup.getChildren().clear();
        if (boneMapper == null) return;

        for (String boneName : boneMapper.getPhysicsBones()) {
            double[] worldPosition = baker.getCurrentReferenceWorldPosition(boneName);
            if (worldPosition == null) continue;

            Sphere s = new Sphere(POINT_RADIUS * 0.8);
            s.setMaterial(REFERENCE_MATERIAL);
            s.setTranslateX(worldPosition[0]);
            s.setTranslateY(worldPosition[1]);
            s.setTranslateZ(worldPosition[2]);
            referenceGroup.getChildren().add(s);
        }
    }

    public void applyAnimationPose(BedrockAnimationData.Animation animation, double time) {
        if (animation == null || boneGroups.isEmpty()) return;

        for (BedrockModelData.Bone bone : boneList) {
            Group bg = boneGroups.get(bone.name);
            if (bg == null) continue;

            BedrockAnimationData.BoneAnimation ba = animation.bones.get(bone.name);

            double rx = bone.rotation[0], ry = bone.rotation[1], rz = bone.rotation[2];
            double tx = 0, ty = 0, tz = 0;

            if (ba != null) {
                if (ba.rotation != null) {
                    double[] animRot = ba.rotation.evaluate(time);
                    rx += animRot[0];
                    ry += animRot[1];
                    rz += animRot[2];
                }
                if (ba.position != null) {
                    double[] animPos = ba.position.evaluate(time);
                    tx = animPos[0];
                    ty = animPos[1];
                    tz = animPos[2];
                }
            }

            setBoneLocalTransform(bg, bone, new double[]{tx, ty, tz},
                    new double[]{rx, ry, rz});
        }
    }

    public void resetPose() {
        if (boneGroups.isEmpty()) return;
        for (BedrockModelData.Bone bone : boneList) {
            Group bg = boneGroups.get(bone.name);
            if (bg == null) continue;
            setBoneLocalTransform(bg, bone, new double[3], bone.rotation);
        }
    }

    public void applyBakedFrame(PhysicsBaker.BakedFrame frame) {
        if (frame == null || boneGroups.isEmpty()) return;
        for (PhysicsBaker.BoneState bs : frame.boneStates) {
            Group bg = boneGroups.get(bs.boneName);
            if (bg == null) continue;

            BedrockModelData.Bone bone = ModelLoader.findBoneByName(boneList, bs.boneName);
            if (bone == null) continue;

            double tx = bs.position != null ? bs.position[0] : 0;
            double ty = bs.position != null ? bs.position[1] : 0;
            double tz = bs.position != null ? bs.position[2] : 0;

            double[] bakedRot = bs.rotation != null ? bs.rotation : new double[]{0, 0, 0};
            double rx = bone.rotation[0] + bakedRot[0];
            double ry = bone.rotation[1] + bakedRot[1];
            double rz = bone.rotation[2] + bakedRot[2];
            // 烘焙通道是最终动画增量，需要与绑定旋转组合。
            setBoneLocalTransform(bg, bone, new double[]{tx, ty, tz},
                    new double[]{rx, ry, rz});
        }
    }

    private void rebuildSkeleton() {
        skeletonGroup.getChildren().clear();
        boneSpheres.clear();
        boneWorldPivots.clear();
        boneGroups.clear();

        computeWorldTransforms();

        // 为立方体构建层级组，确保旋转能正确叠加。
        for (BedrockModelData.Bone bone : boneList) {
            Group boneGroup = new Group();
            setBoneLocalTransform(boneGroup, bone, new double[3], bone.rotation);
            boneGroups.put(bone.name, boneGroup);
        }

        // 建立组的层级关系。
        for (BedrockModelData.Bone bone : boneList) {
            Group bg = boneGroups.get(bone.name);
            if (bone.parent != null && boneGroups.containsKey(bone.parent)) {
                boneGroups.get(bone.parent).getChildren().add(bg);
            } else {
                skeletonGroup.getChildren().add(bg);
            }
        }

        // 将立方体加入所属骨骼组。
        for (BedrockModelData.Bone bone : boneList) {
            Group bg = boneGroups.get(bone.name);
            for (BedrockModelData.Cube cube : bone.cubes) {
                double[] effectiveSize = CubeGeometry.effectiveSize(cube);
                Box box = new Box(effectiveSize[0], effectiveSize[1], effectiveSize[2]);
                box.setMaterial(CUBE_MATERIAL);
                box.getTransforms().add(toAffine(
                        BedrockTransformResolver.resolveCubeLocalMatrix(cube)));
                bg.getChildren().add(box);
            }
        }

        // 使用计算出的世界坐标绘制枢轴球和骨骼连线。
        for (BedrockModelData.Bone bone : boneList) {
            double[] wp = boneWorldPivots.get(bone.name);
            if (wp == null) continue;

            Sphere sphere = new Sphere(POINT_RADIUS);
            sphere.setMaterial(DEFAULT_MATERIAL);
            sphere.setTranslateX(wp[0]);
            sphere.setTranslateY(wp[1]);
            sphere.setTranslateZ(wp[2]);
            skeletonGroup.getChildren().add(sphere);
            boneSpheres.put(bone.name, sphere);

            if (bone.parent != null) {
                double[] pp = boneWorldPivots.get(bone.parent);
                if (pp != null) {
                    drawCylinderBetween(pp[0], pp[1], pp[2], wp[0], wp[1], wp[2],
                            BONE_RADIUS, Color.LIGHTGRAY, skeletonGroup);
                }
            }
        }
    }

    private void computeWorldTransforms() {
        Map<String, BedrockTransformResolver.Matrix4> worldMatrices =
                BedrockTransformResolver.resolveBoneWorldMatrices(boneList);
        for (BedrockModelData.Bone bone : boneList) {
            BedrockTransformResolver.Matrix4 matrix = worldMatrices.get(bone.name);
            double[] pivot = BedrockTransformResolver.convertBedrockVector(bone.pivot);
            boneWorldPivots.put(bone.name, matrix.transformPoint(
                    pivot[0], pivot[1], pivot[2]));
        }
    }

    private static void setBoneLocalTransform(Group group,
                                              BedrockModelData.Bone bone,
                                              double[] translation,
                                              double[] rotation) {
        group.getTransforms().setAll(toAffine(
                BedrockTransformResolver.resolveBoneLocalMatrix(
                        bone, translation, rotation)));
    }

    private static Affine toAffine(BedrockTransformResolver.Matrix4 matrix) {
        return new Affine(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(0, 3),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(1, 3),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2), matrix.get(2, 3));
    }

    private void refreshColors() {
        if (boneMapper == null) return;
        for (BedrockModelData.Bone bone : boneList) {
            Sphere sphere = boneSpheres.get(bone.name);
            if (sphere == null) continue;
            if (boneMapper.isPhysicsBone(bone.name)) {
                PhongMaterial mat = isRootFixed(bone.name)
                        ? FIXED_MATERIAL : PHYSICS_MATERIAL;
                sphere.setMaterial(mat);
            } else {
                sphere.setMaterial(DEFAULT_MATERIAL);
            }
        }
    }

    private boolean isRootFixed(String boneName) {
        return boneMapper != null && boneMapper.isFixedBone(boneName);
    }

    private void drawCylinderBetween(double x1, double y1, double z1,
                                     double x2, double y2, double z2,
                                     double radius, Color color, Group parent) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (length < 1e-6) return;

        double mx = (x1 + x2) / 2.0;
        double my = (y1 + y2) / 2.0;
        double mz = (z1 + z2) / 2.0;

        Cylinder cyl = new Cylinder(radius, length);
        cyl.setMaterial(new PhongMaterial(color));
        cyl.setTranslateX(mx);
        cyl.setTranslateY(my);
        cyl.setTranslateZ(mz);

        double nx = dx / length;
        double ny = dy / length;
        double nz = dz / length;

        double angle = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, ny))));
        if (Math.abs(ny) > 0.9999) {
            if (ny < 0) {
                cyl.setRotationAxis(javafx.geometry.Point3D.ZERO.add(1, 0, 0));
                cyl.setRotate(180);
            }
        } else {
            double ax = nz;
            double az = -nx;
            double axisLen = Math.sqrt(ax * ax + az * az);
            ax /= axisLen;
            az /= axisLen;
            cyl.setRotationAxis(new javafx.geometry.Point3D(ax, 0, az));
            cyl.setRotate(angle);
        }

        parent.getChildren().add(cyl);
    }

    private void buildGroundAndGrid() {
        int gridSize = 10;
        Box ground = new Box(gridSize * 2, 0.05, gridSize * 2);
        ground.setId("groundPlane");
        ground.setMaterial(GROUND_MATERIAL);
        ground.setTranslateY(-0.025);
        environmentGroup.getChildren().add(ground);

        for (int i = -gridSize; i <= gridSize; i++) {
            Cylinder xLine = new Cylinder(0.015, gridSize * 2);
            xLine.setMaterial(GRID_MATERIAL);
            xLine.setTranslateY(0.01);
            xLine.setTranslateZ(i);
            xLine.setRotationAxis(javafx.geometry.Point3D.ZERO.add(0, 0, 1));
            xLine.setRotate(90);
            environmentGroup.getChildren().add(xLine);

            Cylinder zLine = new Cylinder(0.015, gridSize * 2);
            zLine.setMaterial(GRID_MATERIAL);
            zLine.setTranslateX(i);
            zLine.setTranslateY(0.01);
            zLine.setRotationAxis(javafx.geometry.Point3D.ZERO.add(1, 0, 0));
            zLine.setRotate(90);
            environmentGroup.getChildren().add(zLine);
        }
    }

    private void buildLights() {
        PointLight light = new PointLight(Color.WHITE);
        light.setTranslateX(10);
        light.setTranslateY(20);
        light.setTranslateZ(-10);
        root.getChildren().add(light);

        AmbientLight ambient = new AmbientLight(Color.rgb(80, 80, 90));
        root.getChildren().add(ambient);
    }
}
