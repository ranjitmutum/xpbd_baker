package xpbd.baker;

/** 指定距离约束在静载荷下的安全范围计算器。 */
/** 根据逐帧误差序列判定物理模拟是否已进入稳定状态。 */
public final class StabilityCalculator {
    private StabilityCalculator() {
    }

    public record Result(
            double currentLoad,
            double forceBudget,
            double maxCompliance,
            double maxGravityMagnitude,
            double maxWindSpeed,
            double maxTurbulence,
            double maxUniformMassScale,
            boolean safe) {
    }

    /**
     * 每个系数表示参数每单位产生的力，并在选定骨骼及其下游物理骨骼上累积。
     * 计算单项安全上限时，其他参数保持当前值。
     */
    public static Result calculate(double permittedExtension, double compliance,
                                   double gravityCoefficient, double windCoefficient,
                                   double turbulenceCoefficient,
                                   double gravityMagnitude, double windSpeed,
                                   double turbulence) {
        double extension = finiteNonNegative(permittedExtension);
        double currentCompliance = finiteNonNegative(compliance);
        double gCoeff = finiteNonNegative(gravityCoefficient);
        double wCoeff = finiteNonNegative(windCoefficient);
        double tCoeff = finiteNonNegative(turbulenceCoefficient);
        double gravity = finiteNonNegative(gravityMagnitude);
        double wind = finiteNonNegative(windSpeed);
        double gust = finiteNonNegative(turbulence);

        double gravityLoad = gCoeff * gravity;
        double windLoad = wCoeff * wind;
        double turbulenceLoad = tCoeff * gust;
        double currentLoad = gravityLoad + windLoad + turbulenceLoad;
        double forceBudget = currentCompliance > 0
                ? extension / currentCompliance : Double.POSITIVE_INFINITY;
        double maxCompliance = currentLoad > 0
                ? extension / currentLoad : Double.POSITIVE_INFINITY;

        double maxGravity = maxParameter(forceBudget - windLoad - turbulenceLoad, gCoeff);
        double maxWind = maxParameter(forceBudget - gravityLoad - turbulenceLoad, wCoeff);
        double maxTurbulence = maxParameter(forceBudget - gravityLoad - windLoad, tCoeff);
        double maxMassScale = currentLoad > 0
                ? Math.max(0, forceBudget / currentLoad) : Double.POSITIVE_INFINITY;
        boolean safe = currentCompliance <= maxCompliance;

        return new Result(currentLoad, forceBudget, maxCompliance, maxGravity,
                maxWind, maxTurbulence, maxMassScale, safe);
    }

    private static double maxParameter(double remainingBudget, double coefficient) {
        if (Double.isInfinite(remainingBudget)) return Double.POSITIVE_INFINITY;
        if (coefficient <= 0) return Double.POSITIVE_INFINITY;
        return Math.max(0, remainingBudget / coefficient);
    }

    private static double finiteNonNegative(double value) {
        return Double.isFinite(value) ? Math.max(0, value) : 0;
    }
}
