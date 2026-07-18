package xpbd.baker;

import xpbd.loader.BedrockModelData;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * 统一的 Bedrock 绑定姿态变换规则。
 *
 * <p>原始 JSON 保持不变。世界空间计算遵循 Blockbench 的 Bedrock 导入规则：
 * 模型向量镜像 X 轴，欧拉角映射为 {@code (-x, -y, +z)}，并通过
 * {@link RotationUtil} 按 ZYX 顺序组合。JavaFX 视口会在场景根节点单独完成 Y 轴向上到向下的显示转换。</p>
 */
public final class BedrockTransformResolver {
    private BedrockTransformResolver() {
    }

    public static double[] convertBedrockVector(double[] vector) {
        requireVector(vector, "Bedrock vector");
        return new double[]{-vector[0], vector[1], vector[2]};
    }

    public static Matrix4 resolveBoneLocalMatrix(BedrockModelData.Bone bone) {
        requireBone(bone);
        return resolveBoneLocalMatrix(bone, new double[3], bone.rotation);
    }

    public static Matrix4 resolveBoneLocalMatrix(BedrockModelData.Bone bone,
                                                  double[] translation,
                                                  double[] rotation) {
        requireBone(bone);
        requireVector(translation, "bone translation");
        requireVector(rotation, "bone rotation");
        double[] mappedTranslation = convertBedrockVector(translation);
        Matrix4 pivotRotation = rotationAroundPivot(bone.pivot, rotation);
        return Matrix4.translation(mappedTranslation[0], mappedTranslation[1],
                        mappedTranslation[2])
                .multiply(pivotRotation);
    }

    private static Matrix4 resolveBoneWorldMatrix(BedrockModelData.Bone bone,
                                                   Matrix4 parentWorld) {
        Matrix4 local = resolveBoneLocalMatrix(bone);
        return parentWorld == null ? local : parentWorld.multiply(local);
    }

    public static Map<String, Matrix4> resolveBoneWorldMatrices(
            List<BedrockModelData.Bone> bones) {
        if (bones == null) throw new IllegalArgumentException("bone list is required");
        Map<String, BedrockModelData.Bone> byName = new LinkedHashMap<>();
        for (BedrockModelData.Bone bone : bones) {
            requireBone(bone);
            byName.put(bone.name, bone);
        }
        Map<String, Matrix4> result = new LinkedHashMap<>();
        Set<String> visiting = new HashSet<>();
        for (BedrockModelData.Bone bone : bones) {
            resolveBoneWorldMatrix(bone, byName, result, visiting);
        }
        return result;
    }

    /** 将以原点为中心、尺寸为正的盒体映射到模型空间的矩阵。 */
    public static Matrix4 resolveCubeLocalMatrix(BedrockModelData.Cube cube) {
        requireCube(cube);
        double[] rawCenter = new double[]{
                cube.origin[0] + cube.size[0] * 0.5,
                cube.origin[1] + cube.size[1] * 0.5,
                cube.origin[2] + cube.size[2] * 0.5
        };
        double[] center = convertBedrockVector(rawCenter);
        double[] pivot = cube.pivot == null ? rawCenter : cube.pivot;
        double[] rotation = cube.rotation == null ? new double[3] : cube.rotation;
        return rotationAroundPivot(pivot, rotation)
                .multiply(Matrix4.translation(center[0], center[1], center[2]));
    }

    private static Matrix4 resolveBoneWorldMatrix(
            BedrockModelData.Bone bone,
            Map<String, BedrockModelData.Bone> byName,
            Map<String, Matrix4> cache,
            Set<String> visiting) {
        Matrix4 cached = cache.get(bone.name);
        if (cached != null) return cached;
        if (!visiting.add(bone.name)) {
            throw new IllegalArgumentException(
                    "Bone hierarchy contains a cycle at " + bone.name);
        }
        Matrix4 parentWorld = null;
        if (bone.parent != null) {
            BedrockModelData.Bone parent = byName.get(bone.parent);
            if (parent == null) {
                throw new IllegalArgumentException(
                        "Missing parent '" + bone.parent + "' for bone " + bone.name);
            }
            parentWorld = resolveBoneWorldMatrix(parent, byName, cache, visiting);
        }
        Matrix4 world = resolveBoneWorldMatrix(bone, parentWorld);
        cache.put(bone.name, world);
        visiting.remove(bone.name);
        return world;
    }

    private static Matrix4 rotationAroundPivot(double[] pivot, double[] rotation) {
        double[] mappedPivot = convertBedrockVector(pivot);
        double[] quaternion = RotationUtil.quaternionFromBedrockEuler(
                rotation[0], rotation[1], rotation[2]);
        return Matrix4.translation(mappedPivot[0], mappedPivot[1], mappedPivot[2])
                .multiply(Matrix4.rotation(quaternion))
                .multiply(Matrix4.translation(
                        -mappedPivot[0], -mappedPivot[1], -mappedPivot[2]));
    }

    private static void requireBone(BedrockModelData.Bone bone) {
        if (bone == null || bone.name == null || bone.name.isBlank()) {
            throw new IllegalArgumentException("named bone is required");
        }
        requireVector(bone.pivot, "bone pivot");
        requireVector(bone.rotation, "bone rotation");
    }

    private static void requireCube(BedrockModelData.Cube cube) {
        if (cube == null) throw new IllegalArgumentException("cube is required");
        requireVector(cube.origin, "cube origin");
        requireVector(cube.size, "cube size");
        if (cube.pivot != null) requireVector(cube.pivot, "cube pivot");
        if (cube.rotation != null) requireVector(cube.rotation, "cube rotation");
    }

    private static void requireVector(double[] vector, String label) {
        if (vector == null || vector.length < 3
                || !Double.isFinite(vector[0])
                || !Double.isFinite(vector[1])
                || !Double.isFinite(vector[2])) {
            throw new IllegalArgumentException(label + " must contain three finite values");
        }
    }

    /** 使用列向量相乘的不可变行主序仿射 4×4 矩阵。 */
    public static final class Matrix4 {
        private final double[] values;

        private Matrix4(double[] values) {
            this.values = values;
        }

        private static Matrix4 identity() {
            return new Matrix4(new double[]{
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1
            });
        }

        private static Matrix4 translation(double x, double y, double z) {
            Matrix4 result = identity();
            result.values[3] = x;
            result.values[7] = y;
            result.values[11] = z;
            return result;
        }

        private static Matrix4 rotation(double[] quaternion) {
            if (quaternion == null || quaternion.length < 4) {
                throw new IllegalArgumentException("four-component quaternion is required");
            }
            double x = quaternion[0], y = quaternion[1];
            double z = quaternion[2], w = quaternion[3];
            double length = Math.sqrt(x * x + y * y + z * z + w * w);
            if (!(length > 1e-20) || !Double.isFinite(length)) {
                throw new IllegalArgumentException("finite non-zero quaternion is required");
            }
            x /= length;
            y /= length;
            z /= length;
            w /= length;
            return new Matrix4(new double[]{
                    1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                    2 * (x * z + y * w), 0,
                    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                    2 * (y * z - x * w), 0,
                    2 * (x * z - y * w), 2 * (y * z + x * w),
                    1 - 2 * (x * x + y * y), 0,
                    0, 0, 0, 1
            });
        }

        private Matrix4 multiply(Matrix4 right) {
            if (right == null) throw new IllegalArgumentException("right matrix is required");
            double[] result = new double[16];
            for (int row = 0; row < 4; row++) {
                for (int column = 0; column < 4; column++) {
                    for (int inner = 0; inner < 4; inner++) {
                        result[row * 4 + column] += values[row * 4 + inner]
                                * right.values[inner * 4 + column];
                    }
                }
            }
            return new Matrix4(result);
        }

        public double[] transformPoint(double x, double y, double z) {
            return new double[]{
                    values[0] * x + values[1] * y + values[2] * z + values[3],
                    values[4] * x + values[5] * y + values[6] * z + values[7],
                    values[8] * x + values[9] * y + values[10] * z + values[11]
            };
        }

        public double get(int row, int column) {
            if (row < 0 || row > 3 || column < 0 || column > 3) {
                throw new IndexOutOfBoundsException("matrix index must be in [0, 3]");
            }
            return values[row * 4 + column];
        }

    }
}
