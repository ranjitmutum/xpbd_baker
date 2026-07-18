package xpbd.baker;

import xpbd.loader.BedrockModelData;

/** Bedrock 立方体的统一几何规则，供碰撞、预览和测试共用。 */
public final class CubeGeometry {
    private CubeGeometry() {
    }

    public static double[] effectivePivot(BedrockModelData.Cube cube) {
        requireCube(cube);
        if (cube.pivot != null) return cube.pivot.clone();
        return new double[]{
                cube.origin[0] + cube.size[0] * 0.5,
                cube.origin[1] + cube.size[1] * 0.5,
                cube.origin[2] + cube.size[2] * 0.5
        };
    }

    /** Bedrock 的 inflate 会按该标量向外扩展每个面。 */
    public static double[] effectiveOrigin(BedrockModelData.Cube cube) {
        requireCube(cube);
        return new double[]{
                Math.min(cube.origin[0], cube.origin[0] + cube.size[0]) - cube.inflate,
                Math.min(cube.origin[1], cube.origin[1] + cube.size[1]) - cube.inflate,
                Math.min(cube.origin[2], cube.origin[2] + cube.size[2]) - cube.inflate
        };
    }

    /** 应用对称 inflate 后得到的规范化非负尺寸。 */
    public static double[] effectiveSize(BedrockModelData.Cube cube) {
        requireCube(cube);
        return new double[]{
                Math.abs(cube.size[0]) + cube.inflate * 2.0,
                Math.abs(cube.size[1]) + cube.inflate * 2.0,
                Math.abs(cube.size[2]) + cube.inflate * 2.0
        };
    }

    /** 返回立方体绕自身枢轴旋转后的八个绑定模型顶点。 */
    public static double[] bindVertices(BedrockModelData.Cube cube) {
        requireCube(cube);
        double[] result = new double[24];
        double[] size = effectiveSize(cube);
        BedrockTransformResolver.Matrix4 matrix =
                BedrockTransformResolver.resolveCubeLocalMatrix(cube);
        // 以固定索引顺序生成八个顶点，方便调用方复用拓扑。
        for (int z = 0; z < 2; z++) {
            for (int y = 0; y < 2; y++) {
                for (int x = 0; x < 2; x++) {
                    int index = x + y * 2 + z * 4;
                    double[] transformed = matrix.transformPoint(
                            (x - 0.5) * size[0],
                            (y - 0.5) * size[1],
                            (z - 0.5) * size[2]);
                    result[index * 3] = transformed[0];
                    result[index * 3 + 1] = transformed[1];
                    result[index * 3 + 2] = transformed[2];
                }
            }
        }
        return result;
    }

    public static void transformPoint(BonePoseCalculator.Pose pose,
                                      double x, double y, double z,
                                      double[] result) {
        transformPoint(pose, x, y, z, result, 0);
    }

    static void transformPoint(BonePoseCalculator.Pose pose,
                               double x, double y, double z,
                               double[] result, int offset) {
        if (pose == null || result == null || result.length < 3) {
            throw new IllegalArgumentException("pose and three-component result are required");
        }
        if (offset < 0 || result.length - offset < 3) {
            throw new IllegalArgumentException("transformed point result is too small");
        }
        double[] transformed = new double[3];
        RotationUtil.rotateVector(pose.worldRotation, x, y, z, transformed);
        result[offset] = transformed[0] + pose.worldTranslation[0];
        result[offset + 1] = transformed[1] + pose.worldTranslation[1];
        result[offset + 2] = transformed[2] + pose.worldTranslation[2];
        if (!Double.isFinite(result[offset]) || !Double.isFinite(result[offset + 1])
                || !Double.isFinite(result[offset + 2])) {
            throw new IllegalArgumentException("transformed model point must be finite");
        }
    }

    private static void requireCube(BedrockModelData.Cube cube) {
        if (cube == null || cube.origin == null || cube.origin.length < 3
                || cube.size == null || cube.size.length < 3) {
            throw new IllegalArgumentException("cube origin and size are required");
        }
        requireFinite(cube.origin, "cube origin");
        requireFinite(cube.size, "cube size");
        if (!Double.isFinite(cube.inflate)) {
            throw new IllegalArgumentException("cube inflate must be finite");
        }
        for (int axis = 0; axis < 3; axis++) {
            if (Math.abs(cube.size[axis]) + cube.inflate * 2.0 < 0) {
                throw new IllegalArgumentException(
                        "cube inflate shrinks an effective size below zero");
            }
        }
        if (cube.pivot != null) requireFinite(cube.pivot, "cube pivot");
        if (cube.rotation != null) requireFinite(cube.rotation, "cube rotation");
    }

    private static void requireFinite(double[] value, String label) {
        if (value.length < 3 || !Double.isFinite(value[0])
                || !Double.isFinite(value[1]) || !Double.isFinite(value[2])) {
            throw new IllegalArgumentException(label + " must contain three finite values");
        }
    }
}
