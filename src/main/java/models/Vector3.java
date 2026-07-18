package models;

public final class Vector3 {
    public double x, y, z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3() { this(0, 0, 0); }

    // 拷贝构造函数
    public Vector3(Vector3 v) { this(v.x, v.y, v.z); }

    public Vector3 set(double x, double y, double z) {
        this.x = x; this.y = y; this.z = z;
        return this;
    }

    public Vector3 set(Vector3 v) {
        return set(v.x, v.y, v.z);
    }

    public Vector3 sub(Vector3 v) {
        this.x -= v.x; this.y -= v.y; this.z -= v.z;
        return this;
    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    // 返回当前向量的副本
    public Vector3 copy() {
        return new Vector3(this);
    }

    @Override
    public String toString() {
        return String.format("(%.4f, %.4f, %.4f)", x, y, z);
    }
}
