package xpbd.baker;

import xpbd.loader.BedrockModelData;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** 单次烘焙中选定 Bedrock 立方体的缓存，表示为运动的六平面凸体。 */
public final class BodyColliderCache {
    private static final int[][] FACE_VERTICES = {
            {0, 2, 6, 4}, // -X
            {1, 5, 7, 3}, // +X
            {0, 4, 5, 1}, // -Y
            {2, 3, 7, 6}, // +Y
            {0, 1, 3, 2}, // -Z
            {4, 6, 7, 5}  // +Z
    };

    private final List<Collider> colliders = new ArrayList<>();
    private final List<Collider> collidersView = Collections.unmodifiableList(colliders);
    private final double epsilon;
    private int degenerateCubeCount;
    private boolean sweepContinuous;

    public BodyColliderCache(List<BedrockModelData.Bone> allBones,
                             Collection<String> collisionBones) {
        Set<String> included = collisionBones == null
                ? Collections.emptySet() : Set.copyOf(collisionBones);
        double modelScale = estimateModelScale(allBones);
        epsilon = Math.max(1e-9, modelScale * 1e-10);
        if (allBones == null || included.isEmpty()) return;
        for (BedrockModelData.Bone bone : allBones) {
            if (bone == null || bone.name == null || !included.contains(bone.name)
                    || bone.cubes == null) continue;
            for (int cubeIndex = 0; cubeIndex < bone.cubes.size(); cubeIndex++) {
                BedrockModelData.Cube cube = bone.cubes.get(cubeIndex);
                if (cube == null) continue;
                double[] effectiveSize = CubeGeometry.effectiveSize(cube);
                if (effectiveSize[0] <= epsilon || effectiveSize[1] <= epsilon
                        || effectiveSize[2] <= epsilon) {
                    degenerateCubeCount++;
                    continue;
                }
                colliders.add(new Collider(
                        bone.name, CubeGeometry.bindVertices(cube), epsilon));
            }
        }
    }

    public void initialize(Map<String, BonePoseCalculator.Pose> poses) {
        update(poses, false, true);
    }

    public void advance(Map<String, BonePoseCalculator.Pose> poses,
                        boolean historyContinuous) {
        update(poses, historyContinuous, false);
    }

    /** 仅更新当前姿态，供最终输出审计使用。 */
    public void setAuditPose(Map<String, BonePoseCalculator.Pose> poses) {
        update(poses, false, true);
    }

    private void update(Map<String, BonePoseCalculator.Pose> poses,
                        boolean historyContinuous, boolean initializeHistory) {
        if (poses == null) throw new IllegalArgumentException("collider poses are required");
        for (Collider collider : colliders) {
            BonePoseCalculator.Pose pose = poses.get(collider.ownerBone);
            if (pose == null) {
                throw new IllegalArgumentException(
                        "missing pose for collision bone: " + collider.ownerBone);
            }
            collider.update(pose, historyContinuous && !initializeHistory);
        }
        sweepContinuous = historyContinuous && !initializeHistory;
    }

    public List<Collider> getColliders() {
        return collidersView;
    }

    public int getDegenerateCubeCount() {
        return degenerateCubeCount;
    }

    public double getEpsilon() {
        return epsilon;
    }

    public boolean isSweepContinuous() {
        return sweepContinuous;
    }

    public boolean containsCurrent(double x, double y, double z, double skin) {
        for (Collider collider : colliders) {
            if (collider.containsCurrent(x, y, z, skin)) return true;
        }
        return false;
    }

    public boolean containsPrevious(double x, double y, double z, double skin) {
        for (Collider collider : colliders) {
            if (collider.containsPrevious(x, y, z, skin)) return true;
        }
        return false;
    }

    private static double estimateModelScale(List<BedrockModelData.Bone> bones) {
        double scale = 1;
        if (bones == null) return scale;
        for (BedrockModelData.Bone bone : bones) {
            if (bone == null || bone.cubes == null) continue;
            for (BedrockModelData.Cube cube : bone.cubes) {
                if (cube == null || cube.origin == null || cube.size == null
                        || cube.origin.length < 3 || cube.size.length < 3) continue;
                for (int axis = 0; axis < 3; axis++) {
                    if (Double.isFinite(cube.origin[axis])) {
                        scale = Math.max(scale, Math.abs(cube.origin[axis]));
                    }
                    if (Double.isFinite(cube.size[axis])) {
                        scale = Math.max(scale, Math.abs(cube.size[axis]));
                    }
                }
                if (Double.isFinite(cube.inflate)) {
                    scale = Math.max(scale, Math.abs(cube.inflate));
                }
            }
        }
        return scale;
    }

    public static final class Collider {
        private final String ownerBone;
        private final double[] bindVertices;
        private final double[] bindNormals = new double[18];
        private final double[] bindConstants = new double[6];
        private final double[] previousRotation = new double[]{0, 0, 0, 1};
        private final double[] currentRotation = new double[]{0, 0, 0, 1};
        private final double[] previousTranslation = new double[3];
        private final double[] currentTranslation = new double[3];
        private final double[] previousNormals = new double[18];
        private final double[] currentNormals = new double[18];
        private final double[] previousConstants = new double[6];
        private final double[] currentConstants = new double[6];
        private final double[] previousAabb = new double[6];
        private final double[] currentAabb = new double[6];
        private final double[] currentVertices = new double[24];
        private final double[] rotatedScratch = new double[3];
        private boolean initialized;

        private Collider(String ownerBone, double[] bindVertices,
                         double epsilon) {
            this.ownerBone = ownerBone;
            this.bindVertices = bindVertices;
            buildBindPlanes(epsilon);
        }

        private void buildBindPlanes(double epsilon) {
            double centerX = 0;
            double centerY = 0;
            double centerZ = 0;
            for (int vertex = 0; vertex < 8; vertex++) {
                centerX += bindVertices[vertex * 3];
                centerY += bindVertices[vertex * 3 + 1];
                centerZ += bindVertices[vertex * 3 + 2];
            }
            centerX /= 8;
            centerY /= 8;
            centerZ /= 8;
            for (int face = 0; face < 6; face++) {
                int a = FACE_VERTICES[face][0] * 3;
                int b = FACE_VERTICES[face][1] * 3;
                int c = FACE_VERTICES[face][2] * 3;
                double abX = bindVertices[b] - bindVertices[a];
                double abY = bindVertices[b + 1] - bindVertices[a + 1];
                double abZ = bindVertices[b + 2] - bindVertices[a + 2];
                double acX = bindVertices[c] - bindVertices[a];
                double acY = bindVertices[c + 1] - bindVertices[a + 1];
                double acZ = bindVertices[c + 2] - bindVertices[a + 2];
                double nx = abY * acZ - abZ * acY;
                double ny = abZ * acX - abX * acZ;
                double nz = abX * acY - abY * acX;
                double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
                if (!Double.isFinite(length) || length <= epsilon) {
                    throw new IllegalArgumentException("degenerate cube face");
                }
                nx /= length;
                ny /= length;
                nz /= length;
                double centerSide = nx * (centerX - bindVertices[a])
                        + ny * (centerY - bindVertices[a + 1])
                        + nz * (centerZ - bindVertices[a + 2]);
                if (centerSide > 0) {
                    nx = -nx;
                    ny = -ny;
                    nz = -nz;
                }
                bindNormals[face * 3] = nx;
                bindNormals[face * 3 + 1] = ny;
                bindNormals[face * 3 + 2] = nz;
                bindConstants[face] = nx * bindVertices[a]
                        + ny * bindVertices[a + 1] + nz * bindVertices[a + 2];
            }
        }

        private void update(BonePoseCalculator.Pose pose, boolean keepHistory) {
            requireFinite3(pose.worldTranslation, "collider translation");
            if (initialized && keepHistory) {
                System.arraycopy(currentRotation, 0, previousRotation, 0, 4);
                System.arraycopy(currentTranslation, 0, previousTranslation, 0, 3);
                System.arraycopy(currentNormals, 0, previousNormals, 0, 18);
                System.arraycopy(currentConstants, 0, previousConstants, 0, 6);
                System.arraycopy(currentAabb, 0, previousAabb, 0, 6);
            }
            normalizeQuaternion(pose.worldRotation, currentRotation);
            System.arraycopy(pose.worldTranslation, 0, currentTranslation, 0, 3);
            double[] rotated = rotatedScratch;
            currentAabb[0] = currentAabb[2] = currentAabb[4] = Double.POSITIVE_INFINITY;
            currentAabb[1] = currentAabb[3] = currentAabb[5] = Double.NEGATIVE_INFINITY;
            for (int vertex = 0; vertex < 8; vertex++) {
                int offset = vertex * 3;
                RotationUtil.rotateVector(currentRotation, bindVertices[offset],
                        bindVertices[offset + 1], bindVertices[offset + 2], rotated);
                double x = rotated[0] + currentTranslation[0];
                double y = rotated[1] + currentTranslation[1];
                double z = rotated[2] + currentTranslation[2];
                currentVertices[offset] = x;
                currentVertices[offset + 1] = y;
                currentVertices[offset + 2] = z;
                currentAabb[0] = Math.min(currentAabb[0], x);
                currentAabb[1] = Math.max(currentAabb[1], x);
                currentAabb[2] = Math.min(currentAabb[2], y);
                currentAabb[3] = Math.max(currentAabb[3], y);
                currentAabb[4] = Math.min(currentAabb[4], z);
                currentAabb[5] = Math.max(currentAabb[5], z);
            }
            for (int face = 0; face < 6; face++) {
                int offset = face * 3;
                RotationUtil.rotateVector(currentRotation, bindNormals[offset],
                        bindNormals[offset + 1], bindNormals[offset + 2], rotated);
                currentNormals[offset] = rotated[0];
                currentNormals[offset + 1] = rotated[1];
                currentNormals[offset + 2] = rotated[2];
                int vertex = FACE_VERTICES[face][0] * 3;
                currentConstants[face] = rotated[0] * currentVertices[vertex]
                        + rotated[1] * currentVertices[vertex + 1]
                        + rotated[2] * currentVertices[vertex + 2];
            }
            if (!initialized || !keepHistory) {
                System.arraycopy(currentRotation, 0, previousRotation, 0, 4);
                System.arraycopy(currentTranslation, 0, previousTranslation, 0, 3);
                System.arraycopy(currentNormals, 0, previousNormals, 0, 18);
                System.arraycopy(currentConstants, 0, previousConstants, 0, 6);
                System.arraycopy(currentAabb, 0, previousAabb, 0, 6);
            }
            initialized = true;
        }

        public double getBindNormal(int face, int axis) {
            return bindNormals[face * 3 + axis];
        }

        public double getBindConstant(int face) {
            return bindConstants[face];
        }

        public double getCurrentNormal(int face, int axis) {
            return currentNormals[face * 3 + axis];
        }

        public double signedDistanceCurrent(int face, double x, double y, double z) {
            int offset = face * 3;
            return currentNormals[offset] * x + currentNormals[offset + 1] * y
                    + currentNormals[offset + 2] * z - currentConstants[face];
        }

        public double signedDistancePrevious(int face, double x, double y, double z) {
            int offset = face * 3;
            return previousNormals[offset] * x + previousNormals[offset + 1] * y
                    + previousNormals[offset + 2] * z - previousConstants[face];
        }

        public boolean containsCurrent(double x, double y, double z, double skin) {
            if (!pointInAabb(currentAabb, x, y, z, skin)) return false;
            for (int face = 0; face < 6; face++) {
                if (signedDistanceCurrent(face, x, y, z) > skin) return false;
            }
            return true;
        }

        public boolean containsPrevious(double x, double y, double z, double skin) {
            if (!pointInAabb(previousAabb, x, y, z, skin)) return false;
            for (int face = 0; face < 6; face++) {
                if (signedDistancePrevious(face, x, y, z) > skin) return false;
            }
            return true;
        }

        public void toCurrentBind(double x, double y, double z, double[] result) {
            inverseTransform(currentRotation, currentTranslation, x, y, z, result);
        }

        public void toPreviousBind(double x, double y, double z, double[] result) {
            inverseTransform(previousRotation, previousTranslation, x, y, z, result);
        }

        public void fromCurrentBind(double x, double y, double z, double[] result) {
            forwardTransform(currentRotation, currentTranslation, x, y, z, result);
        }

        public void fromPreviousBind(double x, double y, double z, double[] result) {
            forwardTransform(previousRotation, previousTranslation, x, y, z, result);
        }

        private static boolean pointInAabb(double[] aabb, double x, double y,
                                           double z, double margin) {
            return x >= aabb[0] - margin && x <= aabb[1] + margin
                    && y >= aabb[2] - margin && y <= aabb[3] + margin
                    && z >= aabb[4] - margin && z <= aabb[5] + margin;
        }

        private static void inverseTransform(double[] rotation, double[] translation,
                                             double x, double y, double z,
                                             double[] result) {
            double px = x - translation[0];
            double py = y - translation[1];
            double pz = z - translation[2];
            rotateVector(-rotation[0], -rotation[1], -rotation[2], rotation[3],
                    px, py, pz, result);
        }

        private static void forwardTransform(double[] rotation, double[] translation,
                                             double x, double y, double z,
                                             double[] result) {
            RotationUtil.rotateVector(rotation, x, y, z, result);
            result[0] += translation[0];
            result[1] += translation[1];
            result[2] += translation[2];
        }

        private static void rotateVector(double qx, double qy, double qz, double qw,
                                         double x, double y, double z,
                                         double[] result) {
            double tx = 2 * (qy * z - qz * y);
            double ty = 2 * (qz * x - qx * z);
            double tz = 2 * (qx * y - qy * x);
            result[0] = x + qw * tx + (qy * tz - qz * ty);
            result[1] = y + qw * ty + (qz * tx - qx * tz);
            result[2] = z + qw * tz + (qx * ty - qy * tx);
        }

        private static void normalizeQuaternion(double[] quaternion, double[] result) {
            if (quaternion == null || quaternion.length < 4) {
                throw new IllegalArgumentException("collider rotation is missing");
            }
            double normSquared = 0;
            for (int i = 0; i < 4; i++) {
                if (!Double.isFinite(quaternion[i])) {
                    throw new IllegalArgumentException("collider rotation must be finite");
                }
                normSquared += quaternion[i] * quaternion[i];
            }
            if (!Double.isFinite(normSquared) || normSquared <= 1e-20) {
                throw new IllegalArgumentException("collider rotation is not invertible");
            }
            double inverseLength = 1.0 / Math.sqrt(normSquared);
            result[0] = quaternion[0] * inverseLength;
            result[1] = quaternion[1] * inverseLength;
            result[2] = quaternion[2] * inverseLength;
            result[3] = quaternion[3] * inverseLength;
        }

        private static void requireFinite3(double[] value, String label) {
            if (value == null || value.length < 3 || !Double.isFinite(value[0])
                    || !Double.isFinite(value[1]) || !Double.isFinite(value[2])) {
                throw new IllegalArgumentException(label + " must be finite");
            }
        }
    }
}
