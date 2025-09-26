// FDPNextgen

package net.ccbluex.liquidbounce.features.module.modules.legit

import net.ccbluex.liquidbounce.event.EventTarget
import net.ccbluex.liquidbounce.event.StrafeEvent
import net.ccbluex.liquidbounce.event.UpdateEvent
import net.ccbluex.liquidbounce.features.module.Module
import net.ccbluex.liquidbounce.features.module.ModuleCategory
import net.ccbluex.liquidbounce.features.module.ModuleInfo
import net.ccbluex.liquidbounce.utils.EntityUtils
import net.ccbluex.liquidbounce.utils.Rotation
import net.ccbluex.liquidbounce.utils.RotationUtils
import net.ccbluex.liquidbounce.utils.extensions.getDistanceToEntityBox
import net.ccbluex.liquidbounce.utils.timer.MSTimer
import net.ccbluex.liquidbounce.features.value.*
import net.minecraft.entity.Entity
import net.minecraft.entity.EntityLivingBase
import kotlin.math.*
import java.util.Random

@ModuleInfo(name = "AimAssist", category = ModuleCategory.LEGIT)
class AimAssist : Module() {

    // Core Settings
    private val range = FloatValue("Range", 4.4f, 1f, 10f)
    private val fov = FloatValue("FOV", 90f, 1f, 180f)
    private val priority = ListValue("Priority", arrayOf("Closest", "FovCenter", "Health"), "Closest")
    private val targetTeam = BoolValue("TargetTeam", false)
    private val maxEntities = IntegerValue("MaxEntities", 5, 1, 10)
    
    // Activation Settings（已移除ActivationKey）
    private val triggerMode = ListValue("TriggerMode", arrayOf("Always", "OnClick", "HoldClick"), "OnClick")
    private val clickDuration = IntegerValue("ClickDuration", 500, 100, 2000).displayable { triggerMode.equals("OnClick") }
    private val onlyVisible = BoolValue("OnlyVisible", true)
    private val wallCheckRange = FloatValue("WallCheckRange", 1.5f, 0f, 5f).displayable { !onlyVisible.get() }

    // Algorithm Enable Settings - 算法独立启用开关
    private val usePID = BoolValue("UsePID", true)
    private val useBezier = BoolValue("UseBezier", false)
    private val useLinear = BoolValue("UseLinear", false)
    private val useCatmullRom = BoolValue("UseCatmullRom", false)
    
    // Algorithm Weight Settings - 算法权重设置
    private val pidWeight = FloatValue("PIDWeight", 1.0f, 0f, 1f).displayable { usePID.get() }
    private val bezierWeight = FloatValue("BezierWeight", 1.0f, 0f, 1f).displayable { useBezier.get() }
    private val linearWeight = FloatValue("LinearWeight", 1.0f, 0f, 1f).displayable { useLinear.get() }
    private val catmullRomWeight = FloatValue("CatmullRomWeight", 1.0f, 0f, 1f).displayable { useCatmullRom.get() }
    
    // Common smoothing parameters
    private val pathResolution = IntegerValue("PathResolution", 15, 5, 30)
    private val accelerationPhase = FloatValue("AccelerationPhase", 0.2f, 0.05f, 0.5f)
    private val decelerationPhase = FloatValue("DecelerationPhase", 0.2f, 0.05f, 0.5f)
    
    // PID Controller Parameters
    private val pidP = FloatValue("PID-P", 2.0f, 0.1f, 5.0f).displayable { usePID.get() }
    private val pidI = FloatValue("PID-I", 0.1f, 0.0f, 2.0f).displayable { usePID.get() }
    private val pidD = FloatValue("PID-D", 0.8f, 0.0f, 2.0f).displayable { usePID.get() }
    private val pidIntegralLimit = FloatValue("PID-IntegralLimit", 100f, 10f, 500f).displayable { usePID.get() }
    private val pidDerivativeSmoothing = FloatValue("PID-DerivativeSmoothing", 0.2f, 0f, 1f).displayable { usePID.get() }
    
    // Bezier Curve Parameters
    private val bezierControlPoints = IntegerValue("Bezier-Points", 5, 3, 15).displayable { useBezier.get() }
    private val bezierCurvature = FloatValue("Bezier-Curvature", 0.3f, 0.05f, 0.95f).displayable { useBezier.get() }
    private val bezierRandomness = FloatValue("Bezier-Randomness", 0.2f, 0f, 1f).displayable { useBezier.get() }
    
    // Catmull-Rom Spline Parameters
    private val catmullTension = FloatValue("Catmull-Tension", 0.5f, 0f, 1f).displayable { useCatmullRom.get() }
    private val catmullBias = FloatValue("Catmull-Bias", 0f, -1f, 1f).displayable { useCatmullRom.get() }

    // Speed & Adjustment Settings（增强转头力度）
    private val minSpeed = FloatValue("MinSpeed", 8f, 1f, 20f)  // 提高最小速度
    private val maxSpeed = FloatValue("MaxSpeed", 30f, 5f, 50f)
    private val speedTransition = FloatValue("SpeedTransition", 0.8f, 0.1f, 2f)  // 加快速度过渡
    private val maxAdjustPerTick = FloatValue("MaxAdjustPerTick", 5f, 0.5f, 10f)  // 提高单次调整上限
    private val smoothness = FloatValue("Smoothness", 2.0f, 0.5f, 10f)  // 降低平滑度，增强响应
    private val distanceScaling = BoolValue("DistanceScaling", true)
    private val scalingFactor = FloatValue("ScalingFactor", 0.8f, 0.1f, 2f).displayable { distanceScaling.get() }

    // Perturbation Settings
    private val perturbationEnabled = BoolValue("PerturbationEnabled", true)
    private val perturbationAlgorithm = ListValue("PerturbationAlgorithm", 
        arrayOf("LevyFlight", "Pulse", "Fatigue", "Combined"), "Combined").displayable { perturbationEnabled.get() }
    private val perturbationIntensity = FloatValue("PerturbationIntensity", 1f, 0f, 2f).displayable { perturbationEnabled.get() }
    private val yawPerturbationScale = FloatValue("YawPerturbScale", 1f, 0f, 2f).displayable { perturbationEnabled.get() }
    private val pitchPerturbationScale = FloatValue("PitchPerturbScale", 0.8f, 0f, 2f).displayable { perturbationEnabled.get() }
    
    // Levy Flight Parameters
    private val levyScale = FloatValue("Levy-Scale", 0.8f, 0.1f, 3f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("LevyFlight") || perturbationAlgorithm.equals("Combined")) }
    private val levyExponent = FloatValue("Levy-Exponent", 1.5f, 1.1f, 2.0f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("LevyFlight") || perturbationAlgorithm.equals("Combined")) }
    private val levyStepLimit = FloatValue("Levy-StepLimit", 3f, 0.5f, 10f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("LevyFlight") || perturbationAlgorithm.equals("Combined")) }
    
    // Pulse Perturbation Parameters
    private val pulseFrequency = FloatValue("Pulse-Frequency", 2.0f, 0.5f, 10f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Pulse") || perturbationAlgorithm.equals("Combined")) }
    private val pulseAmplitude = FloatValue("Pulse-Amplitude", 0.6f, 0.1f, 2f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Pulse") || perturbationAlgorithm.equals("Combined")) }
    private val pulsePhaseShift = FloatValue("Pulse-PhaseShift", 0f, 0f, 6.28f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Pulse") || perturbationAlgorithm.equals("Combined")) } // 0 to 2π
    
    // Fatigue Algorithm Parameters
    private val fatigueRate = FloatValue("Fatigue-Rate", 0.01f, 0.001f, 0.1f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Fatigue") || perturbationAlgorithm.equals("Combined")) }
    private val fatigueRecovery = FloatValue("Fatigue-Recovery", 0.02f, 0.001f, 0.2f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Fatigue") || perturbationAlgorithm.equals("Combined")) }
    private val maxFatigue = FloatValue("Fatigue-Max", 2.0f, 0.5f, 5f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Fatigue") || perturbationAlgorithm.equals("Combined")) }
    private val fatigueThreshold = FloatValue("Fatigue-Threshold", 0.5f, 0f, 2f).displayable { perturbationEnabled.get() && 
        (perturbationAlgorithm.equals("Fatigue") || perturbationAlgorithm.equals("Combined")) }

    // Prediction & Behavior Settings
    private val predictionEnabled = BoolValue("PredictionEnabled", true)
    private val predictionStrength = FloatValue("Prediction-Strength", 0.2f, 0f, 1f).displayable { predictionEnabled.get() }
    private val predictionType = ListValue("Prediction-Type", arrayOf("Linear", "Adaptive"), "Adaptive").displayable { predictionEnabled.get() }
    private val trackingDelay = IntegerValue("TrackingDelay", 1, 0, 15)  // 降低跟踪延迟
    private val fadeOut = BoolValue("FadeOut", true)
    private val fadeOutSpeed = FloatValue("FadeOutSpeed", 0.1f, 0.01f, 0.5f).displayable { fadeOut.get() }
    private val targetSwitchThreshold = FloatValue("TargetSwitchThreshold", 15f, 5f, 45f)
    private val retainTargetTime = IntegerValue("RetainTargetTime", 1000, 0, 5000)

    // Runtime State
    private val clickTimer = MSTimer()
    private val activationTimer = MSTimer()
    private val targetRetentionTimer = MSTimer()
    private var currentPaths = mutableMapOf<String, List<Rotation>>() // 存储每种算法的路径
    private var currentStep = 0
    private var lastTarget: Entity? = null
    private var pathUpdateTimer = 0
    private var targetChangeTimer = 0
    private var fadeProgress = 0f
    private var currentSpeed = 0f
    
    // PID Controller State
    private var yawErrorSum = 0f
    private var pitchErrorSum = 0f
    private var lastYawError = 0f
    private var lastPitchError = 0f
    private var smoothedYawDerivative = 0f
    private var smoothedPitchDerivative = 0f
    
    // Prediction State
    private val lastTargetPositions = mutableListOf<Pair<Double, Double>>() // x, z positions
    private val positionHistorySize = 5
    
    // Fatigue State
    private var fatigueLevel = 0f
    
    // Random instance for all random operations
    private val random = Random()

    /**
     * 检查是否至少启用了一种算法
     */
    private fun isAnyAlgorithmEnabled(): Boolean {
        return usePID.get() || useBezier.get() || useLinear.get() || useCatmullRom.get()
    }
    
    /**
     * 计算总权重用于归一化
     */
    private fun getTotalWeight(): Float {
        var total = 0f
        if (usePID.get()) total += pidWeight.get()
        if (useBezier.get()) total += bezierWeight.get()
        if (useLinear.get()) total += linearWeight.get()
        if (useCatmullRom.get()) total += catmullRomWeight.get()
        return if (total == 0f) 1f else total // 避免除以零
    }

    /**
     * 生成所有启用算法的路径并存储
     */
    private fun generateAllPaths(start: Rotation, target: Rotation) {
        currentPaths.clear()
        
        val yawDiff = abs(angleDiff(start.yaw, target.yaw))
        val pitchDiff = abs(angleDiff(start.pitch, target.pitch))
        val maxDiff = maxOf(yawDiff, pitchDiff)
        
        // 计算基础速度（增强转头力度）
        val baseSpeed = (minSpeed.get() + maxSpeed.get()) / 2f * 1.2f  // 额外增加20%基础速度
        val speed = if (distanceScaling.get() && lastTarget != null) {
            val currentTarget = lastTarget ?: return
            val distance = mc.thePlayer.getDistanceToEntityBox(currentTarget)
            // 调整距离缩放公式，确保远距离也有足够速度
            baseSpeed * (1f - (min(distance.toFloat(), 5f) / 5f) * (0.7f - scalingFactor.get() * 0.5f))
        } else {
            baseSpeed
        }
        
        // 根据角度差异计算步数（减少步数，加快转头）
        val steps = (maxDiff / (speed / smoothness.get())).toInt()
            .coerceIn(pathResolution.get() / 2, pathResolution.get())  // 缩短最大步数范围
        
        // 为每种启用的算法生成路径
        if (usePID.get()) {
            currentPaths["PID"] = generatePIDPath(start, target, steps)
        }
        if (useBezier.get()) {
            currentPaths["Bezier"] = generateBezierPath(start, target, steps)
        }
        if (useLinear.get()) {
            currentPaths["Linear"] = generateLinearPath(start, target, steps)
        }
        if (useCatmullRom.get()) {
            currentPaths["CatmullRom"] = generateCatmullRomPath(start, target, steps)
        }
    }

    /**
     * 混合多种算法的路径点
     */
    private fun getMixedRotation(step: Int): Rotation? {
        if (currentPaths.isEmpty() || step >= getMaxPathLength()) {
            return null
        }
        
        val totalWeight = getTotalWeight()
        var mixedYaw = 0f
        var mixedPitch = 0f
        
        // 混合所有启用算法的当前步长旋转值
        if (usePID.get() && "PID" in currentPaths) {
            val pidRotation = currentPaths["PID"]?.getOrNull(step) ?: return null
            val weight = pidWeight.get() / totalWeight
            mixedYaw += pidRotation.yaw * weight
            mixedPitch += pidRotation.pitch * weight
        }
        
        if (useBezier.get() && "Bezier" in currentPaths) {
            val bezierRotation = currentPaths["Bezier"]?.getOrNull(step) ?: return null
            val weight = bezierWeight.get() / totalWeight
            mixedYaw += bezierRotation.yaw * weight
            mixedPitch += bezierRotation.pitch * weight
        }
        
        if (useLinear.get() && "Linear" in currentPaths) {
            val linearRotation = currentPaths["Linear"]?.getOrNull(step) ?: return null
            val weight = linearWeight.get() / totalWeight
            mixedYaw += linearRotation.yaw * weight
            mixedPitch += linearRotation.pitch * weight
        }
        
        if (useCatmullRom.get() && "CatmullRom" in currentPaths) {
            val catmullRotation = currentPaths["CatmullRom"]?.getOrNull(step) ?: return null
            val weight = catmullRomWeight.get() / totalWeight
            mixedYaw += catmullRotation.yaw * weight
            mixedPitch += catmullRotation.pitch * weight
        }
        
        return Rotation(mixedYaw, mixedPitch)
    }
    
    /**
     * 获取最长路径的长度，确保混合时不会超出范围
     */
    private fun getMaxPathLength(): Int {
        return currentPaths.values.maxOfOrNull { it.size } ?: 0
    }

    /**
     * Generate path using Bezier curve algorithm
     */
    private fun generateBezierPath(start: Rotation, target: Rotation, steps: Int): List<Rotation> {
        val path = mutableListOf<Rotation>()
        
        // Create control points
        val controlPoints = mutableListOf<Rotation>()
        controlPoints.add(start)
        
        // Generate intermediate control points
        for (i in 1 until bezierControlPoints.get()) {
            val t = i.toFloat() / bezierControlPoints.get()
            val baseYaw = start.yaw + angleDiff(start.yaw, target.yaw) * t
            val basePitch = start.pitch + angleDiff(start.pitch, target.pitch) * t
            
            // Add controlled randomness to control points
            val randomYaw = (random.nextFloat() * 2 - 1) * bezierCurvature.get() * 10f * (1 - t)
            val randomPitch = (random.nextFloat() * 2 - 1) * bezierCurvature.get() * 10f * (1 - t)
            
            controlPoints.add(Rotation(
                baseYaw + randomYaw * bezierRandomness.get(),
                basePitch + randomPitch * bezierRandomness.get()
            ))
        }
        
        controlPoints.add(target)
        
        // Generate points along the Bezier curve
        for (i in 0..steps) {
            val t = i.toFloat() / steps
            val point = calculateBezierPoint(controlPoints, t)
            path.add(point)
        }
        
        return path
    }

    /**
     * Calculate point on Bezier curve
     */
    private fun calculateBezierPoint(points: List<Rotation>, t: Float): Rotation {
        val n = points.size - 1
        var yaw = 0f
        var pitch = 0f
        
        for (i in 0..n) {
            val binomial = binomialCoefficient(n, i).toFloat()
            val term = binomial * (1 - t).pow(n - i) * t.pow(i)
            yaw += points[i].yaw * term
            pitch += points[i].pitch * term
        }
        
        return Rotation(yaw, pitch)
    }

    /**
     * Calculate binomial coefficient
     */
    private fun binomialCoefficient(n: Int, k: Int): Long {
        if (k < 0 || k > n) return 0
        if (k == 0 || k == n) return 1
        
        val k = min(k, n - k)
        var result = 1L
        
        for (i in 0 until k) {
            result = result * (n - i) / (i + 1)
        }
        
        return result
    }

    /**
     * Generate path using linear interpolation with acceleration phases
     */
    private fun generateLinearPath(start: Rotation, target: Rotation, steps: Int): List<Rotation> {
        val path = mutableListOf<Rotation>()
        
        for (i in 0..steps) {
            val t = i.toFloat() / steps
            val smoothT = calculateSmoothProgress(t)
            
            val yaw = start.yaw + angleDiff(start.yaw, target.yaw) * smoothT
            val pitch = start.pitch + angleDiff(start.pitch, target.pitch) * smoothT
            
            path.add(Rotation(yaw, pitch))
        }
        
        return path
    }

    /**
     * Generate path using Catmull-Rom splines
     */
    private fun generateCatmullRomPath(start: Rotation, target: Rotation, steps: Int): List<Rotation> {
        val path = mutableListOf<Rotation>()
        
        // Create control points with virtual points for better curve behavior
        val tension = catmullTension.get()
        val bias = catmullBias.get()
        
        val preStartYaw = start.yaw - angleDiff(start.yaw, target.yaw) * 0.2f
        val preStartPitch = start.pitch - angleDiff(start.pitch, target.pitch) * 0.2f
        val postTargetYaw = target.yaw + angleDiff(start.yaw, target.yaw) * 0.2f
        val postTargetPitch = target.pitch + angleDiff(start.pitch, target.pitch) * 0.2f
        
        val points = listOf(
            Rotation(preStartYaw, preStartPitch),
            start,
            target,
            Rotation(postTargetYaw, postTargetPitch)
        )
        
        // Generate points along the spline
        for (i in 0..steps) {
            val t = i.toFloat() / steps
            val point = calculateCatmullRomPoint(points, 1, t, tension, bias)
            path.add(point)
        }
        
        return path
    }

    /**
     * Calculate point on Catmull-Rom spline
     */
    private fun calculateCatmullRomPoint(points: List<Rotation>, index: Int, t: Float, tension: Float, bias: Float): Rotation {
        val t2 = t * t
        val t3 = t2 * t
        
        // Calculate tangents
        fun tangent(y0: Float, y1: Float, y2: Float, y3: Float): Float {
            return 0.5f * (1 - tension) * 
                ((y2 - y0) * (1 + bias) + (y3 - y1) * (1 - bias))
        }
        
        val y0 = points[index - 1].yaw
        val y1 = points[index].yaw
        val y2 = points[index + 1].yaw
        val y3 = points[index + 2].yaw
        
        val p0 = points[index - 1].pitch
        val p1 = points[index].pitch
        val p2 = points[index + 1].pitch
        val p3 = points[index + 2].pitch
        
        // Calculate Catmull-Rom coefficients
        val yaw = (2f * y1 +
                (-y0 + y2) * t +
                (2f * y0 - 5f * y1 + 4f * y2 - y3) * t2 +
                (-y0 + 3f * y1 - 3f * y2 + y3) * t3) * 0.5f
        
        val pitch = (2f * p1 +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t3) * 0.5f
        
        return Rotation(yaw, pitch)
    }

    /**
     * Generate path using PID control algorithm
     */
    private fun generatePIDPath(start: Rotation, target: Rotation, steps: Int): List<Rotation> {
        val path = mutableListOf<Rotation>()
        var current = start
        
        // Reset PID state
        yawErrorSum = 0f
        pitchErrorSum = 0f
        lastYawError = 0f
        lastPitchError = 0f
        smoothedYawDerivative = 0f
        smoothedPitchDerivative = 0f
        
        for (i in 0..steps) {
            val targetYaw = target.yaw
            val targetPitch = target.pitch
            
            // Calculate errors
            val yawError = angleDiff(current.yaw, targetYaw)
            val pitchError = angleDiff(current.pitch, targetPitch)
            
            // Integral term with anti-windup
            yawErrorSum = (yawErrorSum + yawError).coerceIn(-pidIntegralLimit.get(), pidIntegralLimit.get())
            pitchErrorSum = (pitchErrorSum + pitchError).coerceIn(-pidIntegralLimit.get(), pidIntegralLimit.get())
            
            // Derivative term with smoothing
            val yawDerivative = yawError - lastYawError
            val pitchDerivative = pitchError - lastPitchError
            smoothedYawDerivative = smoothedYawDerivative * (1 - pidDerivativeSmoothing.get()) + 
                                   yawDerivative * pidDerivativeSmoothing.get()
            smoothedPitchDerivative = smoothedPitchDerivative * (1 - pidDerivativeSmoothing.get()) + 
                                     pitchDerivative * pidDerivativeSmoothing.get()
            
            // PID calculation（增强比例项权重，提高响应速度）
            val yawAdjust = pidP.get() * 1.1f * yawError +  // 比例项增加10%
                           pidI.get() * yawErrorSum + 
                           pidD.get() * smoothedYawDerivative
            val pitchAdjust = pidP.get() * 1.1f * pitchError +  // 比例项增加10%
                             pidI.get() * pitchErrorSum + 
                             pidD.get() * smoothedPitchDerivative
            
            // Apply adjustment with smoothing
            val newYaw = current.yaw + yawAdjust / smoothness.get()
            val newPitch = current.pitch + pitchAdjust / smoothness.get()
            
            current = Rotation(newYaw, newPitch)
            path.add(current)
            
            // Save current error for next derivative calculation
            lastYawError = yawError
            lastPitchError = pitchError
        }
        
        return path
    }

    /**
     * Calculate smooth progress with configurable acceleration and deceleration phases
     */
    private fun calculateSmoothProgress(t: Float): Float {
        val accel = accelerationPhase.get()
        val decel = decelerationPhase.get()
        
        return when {
            t < accel -> 0.5f * (t / accel) // Acceleration phase
            t > 1 - decel -> 1f - 0.5f * ((1f - t) / decel) // Deceleration phase
            else -> accel * 0.5f + (t - accel) / (1 - accel - decel) * (1 - accel) // Constant speed phase
        }
    }

    /**
     * Generate perturbation based on selected algorithm
     */
    private fun generatePerturbation(): Pair<Float, Float> {
        if (!perturbationEnabled.get()) return Pair(0f, 0f)
        
        var yawPerturb = 0f
        var pitchPerturb = 0f
        
        when (perturbationAlgorithm.get()) {
            "LevyFlight" -> {
                val (y, p) = levyFlightPerturbation()
                yawPerturb = y
                pitchPerturb = p
            }
            "Pulse" -> {
                val (y, p) = pulsePerturbation()
                yawPerturb = y
                pitchPerturb = p
            }
            "Fatigue" -> {
                val (y, p) = fatiguePerturbation()
                yawPerturb = y
                pitchPerturb = p
            }
            "Combined" -> {
                // Combine all perturbation types with weighting
                val (y1, p1) = levyFlightPerturbation()
                val (y2, p2) = pulsePerturbation()
                val (y3, p3) = fatiguePerturbation()
                
                yawPerturb = (y1 * 0.4f + y2 * 0.3f + y3 * 0.3f)
                pitchPerturb = (p1 * 0.4f + p2 * 0.3f + p3 * 0.3f)
            }
        }
        
        // Apply global intensity scaling
        return Pair(
            yawPerturb * perturbationIntensity.get() * yawPerturbationScale.get(),
            pitchPerturb * perturbationIntensity.get() * pitchPerturbationScale.get()
        )
    }

    /**
     * Levy flight perturbation - simulates natural human small adjustments
     */
    private fun levyFlightPerturbation(): Pair<Float, Float> {
        val scale = levyScale.get()
        val exponent = levyExponent.get()
        
        // Generate Levy-distributed random steps
        fun levyStep(): Float {
            val u = random.nextGaussian().toFloat()
            val v = random.nextGaussian().toFloat()
            val s = sign(u)
            var step = s * scale * abs(u / v).pow(1f / exponent)
            // Apply step limit
            return step.coerceIn(-levyStepLimit.get(), levyStepLimit.get())
        }
        
        return Pair(levyStep(), levyStep())
    }

    /**
     * Pulse perturbation - simulates muscle twitches
     */
    private fun pulsePerturbation(): Pair<Float, Float> {
        val frequency = pulseFrequency.get()
        val amplitude = pulseAmplitude.get()
        val phase = pulsePhaseShift.get()
        
        // Generate periodic pulse based on time
        val time = activationTimer.time.toDouble() / 1000.0
        val pulse = sin(2 * Math.PI * frequency.toDouble() * time + phase.toDouble()).toFloat() * amplitude        
        val randomVal = (random.nextFloat() * 2 - 1) * amplitude * 0.5f
        
        return Pair(pulse + randomVal, pulse * 0.8f + randomVal)
    }

    /**
    * Fatigue perturbation - increases with prolonged use
    */
    private fun fatiguePerturbation(): Pair<Float, Float> {
        // Update fatigue level based on activity
        if (this.state && currentPaths.isNotEmpty()) {
            // Increase fatigue when active
            if (fatigueLevel < fatigueThreshold.get()) {
                fatigueLevel += fatigueRate.get() * 0.1f
            } else {
                val newValue: Float = fatigueLevel + fatigueRate.get().toFloat()
                val maxValue: Float = maxFatigue.get().toFloat()
                fatigueLevel = min(newValue, maxValue)
            }
        } else {
            fatigueLevel = max(fatigueLevel - fatigueRecovery.get().toFloat(), 0f)
        }
        
        // Generate perturbation based on current fatigue
        val perturb = (random.nextFloat() * 2 - 1) * fatigueLevel
        return Pair(perturb, perturb * 0.9f)
    }

    /**
     * Find best target based on priority settings
     */
    private fun findBestTarget(): Entity? {
        val candidates = mc.theWorld.loadedEntityList
            .filter { isValidTarget(it) }
            .take(maxEntities.get())
        
        if (candidates.isEmpty()) return null
        
        // Sort by priority
        return when (priority.get()) {
            "Closest" -> candidates.minByOrNull { mc.thePlayer.getDistanceToEntityBox(it) }
            "FovCenter" -> candidates.minByOrNull { RotationUtils.getRotationDifference(it) }
            "Health" -> candidates.filterIsInstance<EntityLivingBase>()
                .minByOrNull { it.health }
                ?: candidates.minByOrNull { mc.thePlayer.getDistanceToEntityBox(it) }
            else -> candidates.minByOrNull { mc.thePlayer.getDistanceToEntityBox(it) }
        }
    }

    /**
     * Check if entity is a valid target
     */
    private fun isValidTarget(entity: Entity): Boolean {
        if (!EntityUtils.isSelected(entity, true)) return false
        
        // 保留队友判断，使用名称颜色替代isTeammate方法
        if (targetTeam.get() && entity is EntityLivingBase) {
            // 通过名称前缀颜色判断是否为队友
            val playerName = mc.thePlayer.displayName.formattedText
            val entityName = entity.displayName.formattedText
            if (playerName.isNotEmpty() && entityName.isNotEmpty() && 
                playerName[0] == entityName[0]) { // 比较名称前缀颜色
                return false
            }
        }
        
        // 其他原有判断逻辑保持不变
        if (mc.thePlayer.getDistanceToEntityBox(entity) > range.get()) return false
        if (RotationUtils.getRotationDifference(entity) > fov.get()) return false
        
        if (onlyVisible.get()) {
            val canSee = mc.thePlayer.canEntityBeSeen(entity)
            if (!canSee && mc.thePlayer.getDistanceToEntityBox(entity) > wallCheckRange.get()) {
                return false
            }
        }
        
        return true
    }

    /**
     * Predict target movement
     */
    private fun predictTargetMovement(entity: Entity, baseRotation: Rotation): Rotation {
        if (!predictionEnabled.get() || predictionStrength.get() <= 0) return baseRotation
        
        // Store recent positions for prediction
        lastTargetPositions.add(Pair(entity.posX, entity.posZ))
        if (lastTargetPositions.size > positionHistorySize) {
            lastTargetPositions.removeFirst()
        }
        
        return when (predictionType.get()) {
            "Adaptive" -> predictAdaptive(entity, baseRotation)
            else -> predictLinear(entity, baseRotation)
        }
    }

    /**
     * Linear movement prediction
     */
    private fun predictLinear(entity: Entity, baseRotation: Rotation): Rotation {
        val strength = predictionStrength.get()
        return Rotation(
            baseRotation.yaw + (entity.motionX * strength * 8).toFloat(),
            baseRotation.pitch + (entity.motionY * strength * 8).toFloat()
        )
    }

    /**
     * Adaptive movement prediction using historical data
     */
    private fun predictAdaptive(entity: Entity, baseRotation: Rotation): Rotation {
        if (lastTargetPositions.size < 3) return predictLinear(entity, baseRotation)
        
        // Calculate velocity from history
        val dx1 = lastTargetPositions[lastTargetPositions.size - 1].first - 
                  lastTargetPositions[lastTargetPositions.size - 2].first
        val dz1 = lastTargetPositions[lastTargetPositions.size - 1].second - 
                  lastTargetPositions[lastTargetPositions.size - 2].second
                  
        val dx2 = lastTargetPositions[lastTargetPositions.size - 2].first - 
                  lastTargetPositions[lastTargetPositions.size - 3].first
        val dz2 = lastTargetPositions[lastTargetPositions.size - 2].second - 
                  lastTargetPositions[lastTargetPositions.size - 3].second
                  
        // Calculate acceleration
        val ax = dx1 - dx2
        val az = dz1 - dz2
        
        // Apply prediction with adaptive strength
        val strength = predictionStrength.get()
        return Rotation(
            baseRotation.yaw + ((entity.motionX + ax) * strength * 6).toFloat(),
            baseRotation.pitch + (entity.motionY * strength * 8).toFloat()
        )
    }

    @EventTarget
    fun onStrafe(event: StrafeEvent) {
        // 检查是否启用了至少一种算法
        if (!isAnyAlgorithmEnabled()) {
            handleFadeOut()
            return
        }
        
        // Check activation conditions（移除ActivationKey相关判断）
        val isActive = when (triggerMode.get()) {
            "Always" -> true
            "OnClick" -> !clickTimer.hasTimePassed(clickDuration.get().toLong())
            "HoldClick" -> mc.gameSettings.keyBindAttack.isKeyDown
            else -> false
        }
        
        // Update activation timer
        if (isActive && activationTimer.hasTimePassed(100)) {
            activationTimer.reset()
        }
        
        if (!isActive) {
            handleFadeOut()
            return
        }

        // Find best target
        var entity = findBestTarget()
        
        // Handle target retention
        if (entity == null) {
            if (lastTarget != null && !targetRetentionTimer.hasTimePassed(retainTargetTime.get().toLong())) {
                // Keep last target for a while
                entity = lastTarget
            } else {
                handleTargetLoss()
                return
            }
        } else {
            targetRetentionTimer.reset()
        }

        // Check if we should switch targets
        val shouldSwitch = lastTarget == null || entity != lastTarget && 
            RotationUtils.getRotationDifference(entity) < targetSwitchThreshold.get()

        if (shouldSwitch) {
            val currentEntity = entity ?: return // 处理null情况
            // Calculate target rotation with prediction
            val baseRotation = RotationUtils.toRotation(RotationUtils.getCenter(currentEntity.entityBoundingBox), true)
            val predictedRotation = predictTargetMovement(currentEntity, baseRotation)

            // Update path periodically or when target changes
            if (currentEntity != lastTarget || pathUpdateTimer++ >= trackingDelay.get() + 1) {
                generateAllPaths(
                    Rotation(mc.thePlayer.rotationYaw, mc.thePlayer.rotationPitch),
                    predictedRotation
                )
                currentStep = 0
                lastTarget = currentEntity
                pathUpdateTimer = 0
                targetChangeTimer = 0
                fadeProgress = 0f
            }
        }
    }

    @EventTarget
    fun onUpdate(event: UpdateEvent) {
        // 检查是否有可用路径和有效步骤
        if (currentPaths.isEmpty() || currentStep >= getMaxPathLength()) return
        
        // 获取混合后的目标旋转角度
        val targetRotation = getMixedRotation(currentStep) ?: return
        val currentRotation = Rotation(mc.thePlayer.rotationYaw, mc.thePlayer.rotationPitch)
        
        // 计算角度差异（使用更直接的计算方式）
        var deltaYaw = angleDiff(currentRotation.yaw, targetRotation.yaw)
        var deltaPitch = angleDiff(currentRotation.pitch, targetRotation.pitch)
        val angleDiff = max(abs(deltaYaw), abs(deltaPitch))
        
        // 动态速度调整（增强小角度下的响应）
        val targetSpeed = if (angleDiff < 5f) {
            minSpeed.get() * 1.3f  // 小角度时额外加速
        } else if (angleDiff > 30f) {
            maxSpeed.get()
        } else {
            minSpeed.get() + (maxSpeed.get() - minSpeed.get()) * 
            (angleDiff / 30f).pow(speedTransition.get())
        }
        
        // 平滑速度变化（加快速度响应）
        currentSpeed = if (currentSpeed < targetSpeed) {
            min(currentSpeed + 1.0f, targetSpeed)  // 提高加速幅度
        } else {
            max(currentSpeed - 0.8f, targetSpeed)  // 减缓减速幅度
        }
        
        // 应用淡出效果
        if (fadeProgress > 0) {
            val fadeFactor = 1f - fadeProgress
            deltaYaw *= fadeFactor
            deltaPitch *= fadeFactor
            currentSpeed *= fadeFactor
        }
        
        // 应用最大调整限制（适当放宽限制）
        val adjustLimit = maxAdjustPerTick.get() * 1.1f  // 提高10%的调整上限
        deltaYaw = deltaYaw.coerceIn(-adjustLimit, adjustLimit)
        deltaPitch = deltaPitch.coerceIn(-adjustLimit, adjustLimit)
        
        // 应用平滑旋转（减少过度平滑）
        val speedFactor = currentSpeed / smoothness.get() * 1.1f  // 额外提高10%的速度因子
        var finalYaw = deltaYaw * speedFactor / 10f
        var finalPitch = deltaPitch * speedFactor / 10f
        
        // 应用扰动（不影响基础转头力度）
        val (yawPerturb, pitchPerturb) = generatePerturbation()
        finalYaw += yawPerturb
        finalPitch += pitchPerturb
        
        // 直接应用旋转（修复转头核心逻辑）
        // 移除可能限制旋转的中间变量，直接累加计算后的角度
        mc.thePlayer.rotationYaw += finalYaw
        mc.thePlayer.rotationPitch = (mc.thePlayer.rotationPitch + finalPitch)
            .coerceIn(-90f, 90f)  // 仅限制俯仰角范围，不限制旋转幅度
        
        currentStep++
    }
    
    /**
     * Handle smooth fade out when deactivated
     */
    private fun handleFadeOut() {
        if (!fadeOut.get() || currentPaths.isEmpty()) {
            currentPaths.clear()
            currentSpeed = 0f
            return
        }
        
        fadeProgress += fadeOutSpeed.get()
        if (fadeProgress >= 1f) {
            currentPaths.clear()
            currentSpeed = 0f
            fadeProgress = 0f
        }
    }
    
    /**
     * Handle target loss with retention
     */
    private fun handleTargetLoss() {
        if (targetChangeTimer++ > trackingDelay.get() * 2) {
            if (fadeOut.get() && !currentPaths.isEmpty()) {
                handleFadeOut()
            } else {
                if (lastTarget != null && retainTargetTime.get() > 0) {
                    targetRetentionTimer.reset()
                } else {
                    lastTarget = null
                    currentPaths.clear()
                    currentSpeed = 0f
                    targetChangeTimer = 0
                    lastTargetPositions.clear()
                }
            }
        }
    }
    
    /**
     * Calculate shortest angle difference（优化角度差计算，确保取最短路径）
     */
    private fun angleDiff(a: Float, b: Float): Float {
        var diff = (b - a) % 360f
        if (diff > 180f) diff -= 360f
        if (diff < -180f) diff += 360f
        return diff
    }

    override fun onEnable() {
        activationTimer.reset()
        targetRetentionTimer.reset()
        fatigueLevel = 0f
        lastTargetPositions.clear()
    }

    override fun onDisable() {
        currentPaths.clear()
        currentStep = 0
        lastTarget = null
        pathUpdateTimer = 0
        targetChangeTimer = 0
        fadeProgress = 0f
        currentSpeed = 0f
        fatigueLevel = 0f
        lastTargetPositions.clear()
        
        // Reset PID state
        yawErrorSum = 0f
        pitchErrorSum = 0f
        lastYawError = 0f
        lastPitchError = 0f
        smoothedYawDerivative = 0f
        smoothedPitchDerivative = 0f
    }
}