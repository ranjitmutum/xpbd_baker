package xpbd.baker;

/** 四元数与欧拉角转换、旋转及插值的数学工具。 */
public final class RotationUtil {
    private RotationUtil() {
    }

    private static double[] quaternionFromTwoVectors(double[] from, double[] to) {
        double fx = from[0], fy = from[1], fz = from[2];
        double tx = to[0], ty = to[1], tz = to[2];

        double dot = fx * tx + fy * ty + fz * tz;

        if (dot > 0.999999) {
            return new double[]{0, 0, 0, 1};
        }
        if (dot < -0.999999) {
            double[] axis = crossWithFallback(fx, fy, fz);
            double len = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
            axis[0] /= len; axis[1] /= len; axis[2] /= len;
            return new double[]{axis[0], axis[1], axis[2], 0};
        }

        double cx = fy * tz - fz * ty;
        double cy = fz * tx - fx * tz;
        double cz = fx * ty - fy * tx;
        double w = 1.0 + dot;

        double len = Math.sqrt(cx * cx + cy * cy + cz * cz + w * w);
        return new double[]{cx / len, cy / len, cz / len, w / len};
    }

    /**
     * 将旋转拟合到一组或多组对应方向。第一对确定摆动；第二对非共线方向确定绕该方向的扭转。
     */
    public static double[] quaternionFromDirectionPairs(double[][] from, double[][] to) {
        if (from == null || to == null || from.length == 0 || from.length != to.length) {
            return new double[]{0, 0, 0, 1};
        }
        double[] firstFrom = normalize3(from[0]);
        double[] firstTo = normalize3(to[0]);
        double[] swing = quaternionFromTwoVectors(firstFrom, firstTo);
        if (from.length == 1) return swing;

        for (int i = 1; i < from.length; i++) {
            double[] swung = rotateVector(swing, normalize3(from[i]));
            double[] target = normalize3(to[i]);
            double[] projectedSwung = projectOntoPlane(swung, firstTo);
            double[] projectedTarget = projectOntoPlane(target, firstTo);
            double swungLength = Math.sqrt(dot3(projectedSwung, projectedSwung));
            double targetLength = Math.sqrt(dot3(projectedTarget, projectedTarget));
            if (swungLength < 1e-8 || targetLength < 1e-8) continue;
            for (int axis = 0; axis < 3; axis++) {
                projectedSwung[axis] /= swungLength;
                projectedTarget[axis] /= targetLength;
            }
            double[] cross = new double[]{
                    projectedSwung[1] * projectedTarget[2]
                            - projectedSwung[2] * projectedTarget[1],
                    projectedSwung[2] * projectedTarget[0]
                            - projectedSwung[0] * projectedTarget[2],
                    projectedSwung[0] * projectedTarget[1]
                            - projectedSwung[1] * projectedTarget[0]
            };
            double angle = Math.atan2(dot3(firstTo, cross),
                    Math.max(-1, Math.min(1, dot3(projectedSwung, projectedTarget))));
            double half = angle * 0.5;
            double sin = Math.sin(half);
            double[] twist = new double[]{firstTo[0] * sin, firstTo[1] * sin,
                    firstTo[2] * sin, Math.cos(half)};
            return quaternionMultiply(twist, swing);
        }
        return swing;
    }

    private static double[] projectOntoPlane(double[] value, double[] normal) {
        double projection = dot3(value, normal);
        return new double[]{value[0] - normal[0] * projection,
                value[1] - normal[1] * projection,
                value[2] - normal[2] * projection};
    }

    private static double dot3(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double[] crossWithFallback(double fx, double fy, double fz) {
        if (Math.abs(fx) < 0.9) {
            return new double[]{0, -fz, fy};
        } else {
            return new double[]{fz, 0, -fx};
        }
    }

    public static double[] quaternionMultiply(double[] a, double[] b) {
        double[] result = new double[4];
        quaternionMultiply(a, b, result);
        return result;
    }

    public static void quaternionMultiply(double[] a, double[] b, double[] result) {
        double ax = a[0], ay = a[1], az = a[2], aw = a[3];
        double bx = b[0], by = b[1], bz = b[2], bw = b[3];
        result[0] = aw * bx + ax * bw + ay * bz - az * by;
        result[1] = aw * by - ax * bz + ay * bw + az * bx;
        result[2] = aw * bz + ax * by - ay * bx + az * bw;
        result[3] = aw * bw - ax * bx - ay * by - az * bz;
    }

    public static double[] quaternionInverse(double[] q) {
        return new double[]{-q[0], -q[1], -q[2], q[3]};
    }

    /** 四元数表示的最短轴旋转向量（弧度）。 */
    public static double[] rotationVectorFromQuaternion(double[] quaternion) {
        double length = Math.sqrt(quaternion[0] * quaternion[0]
                + quaternion[1] * quaternion[1]
                + quaternion[2] * quaternion[2]
                + quaternion[3] * quaternion[3]);
        if (!(length > 1e-20)) return new double[]{0, 0, 0};
        double sign = quaternion[3] < 0 ? -1 : 1;
        double x = quaternion[0] * sign / length;
        double y = quaternion[1] * sign / length;
        double z = quaternion[2] * sign / length;
        double w = Math.max(-1, Math.min(1, quaternion[3] * sign / length));
        double sine = Math.sqrt(x * x + y * y + z * z);
        if (sine < 1e-12) return new double[]{0, 0, 0};
        double angle = 2.0 * Math.atan2(sine, w);
        double scale = angle / sine;
        return new double[]{x * scale, y * scale, z * scale};
    }

    /** 旋转向量（弧度）的四元数指数映射。 */
    public static double[] quaternionFromRotationVector(double[] vector) {
        double angle = Math.sqrt(vector[0] * vector[0]
                + vector[1] * vector[1] + vector[2] * vector[2]);
        if (angle < 1e-12) return new double[]{0, 0, 0, 1};
        double half = angle * 0.5;
        double scale = Math.sin(half) / angle;
        return new double[]{vector[0] * scale, vector[1] * scale,
                vector[2] * scale, Math.cos(half)};
    }

    public static double[] eulerFromQuaternion(double[] q) {
        double x = q[0], y = q[1], z = q[2], w = q[3];

        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double rx = Math.atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (w * y - z * x);
        double ry;
        if (Math.abs(sinp) >= 1.0) {
            ry = Math.copySign(Math.PI / 2, sinp);
        } else {
            ry = Math.asin(sinp);
        }

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double rz = Math.atan2(siny_cosp, cosy_cosp);

        return new double[]{
                Math.toDegrees(rx),
                Math.toDegrees(ry),
                Math.toDegrees(rz)
        };
    }

    /** 将 Blockbench 空间四元数转换回原始 Bedrock 欧拉角。 */
    public static double[] bedrockEulerFromQuaternion(double[] q) {
        double[] internal = eulerFromQuaternion(q);
        return new double[]{-internal[0], -internal[1], internal[2]};
    }

    /** 选择按分量与前一帧最接近的等价欧拉角表示。 */
    public static double[] unwrapEuler(double[] previous, double[] current) {
        if (previous == null || current == null || previous.length < 3
                || current.length < 3) {
            throw new IllegalArgumentException("two three-component Euler values are required");
        }
        double[] result = new double[3];
        for (int axis = 0; axis < 3; axis++) {
            if (!Double.isFinite(previous[axis]) || !Double.isFinite(current[axis])) {
                throw new IllegalArgumentException("Euler values must be finite");
            }
            result[axis] = current[axis]
                    + 360.0 * Math.rint((previous[axis] - current[axis]) / 360.0);
        }
        return result;
    }

    /** 应用 Blockbench 的 Bedrock 导入映射：{@code (-x, -y, +z)}。 */
    public static double[] quaternionFromBedrockEuler(double rx, double ry, double rz) {
        return quaternionFromEuler(-rx, -ry, rz);
    }

    public static double[] quaternionFromEuler(double rx, double ry, double rz) {
        double[] result = new double[4];
        quaternionFromEuler(rx, ry, rz, result);
        return result;
    }

    public static void quaternionFromEuler(double rx, double ry, double rz,
                                           double[] result) {
        rx = Math.toRadians(rx);
        ry = Math.toRadians(ry);
        rz = Math.toRadians(rz);

        double cx = Math.cos(rx * 0.5), sx = Math.sin(rx * 0.5);
        double cy = Math.cos(ry * 0.5), sy = Math.sin(ry * 0.5);
        double cz = Math.cos(rz * 0.5), sz = Math.sin(rz * 0.5);

        result[0] = sx * cy * cz - cx * sy * sz;
        result[1] = cx * sy * cz + sx * cy * sz;
        result[2] = cx * cy * sz - sx * sy * cz;
        result[3] = cx * cy * cz + sx * sy * sz;
    }

    private static double[] normalize3(double[] v) {
        double len = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len < 1e-10) return new double[]{0, 1, 0};
        return new double[]{v[0] / len, v[1] / len, v[2] / len};
    }

    public static double[] rotateVector(double[] q, double[] v) {
        double[] result = new double[3];
        rotateVector(q, v[0], v[1], v[2], result);
        return result;
    }

    public static void rotateVector(double[] q, double x, double y, double z,
                                    double[] result) {
        double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
        double vx = x, vy = y, vz = z;

        double tx = 2 * (qy * vz - qz * vy);
        double ty = 2 * (qz * vx - qx * vz);
        double tz = 2 * (qx * vy - qy * vx);

        result[0] = vx + qw * tx + (qy * tz - qz * ty);
        result[1] = vy + qw * ty + (qz * tx - qx * tz);
        result[2] = vz + qw * tz + (qx * ty - qy * tx);
    }
}
