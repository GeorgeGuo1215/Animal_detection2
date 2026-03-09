/**
 * 姿态解算模块 - Attitude Solver
 * 支持 Madgwick 滤波器和互补滤波器
 */

class AttitudeSolver {
    constructor() {
        // 四元数 [w, x, y, z]
        this.q = [1, 0, 0, 0];

        // 欧拉角 (弧度)
        this.pitch = 0;  // 俯仰角
        this.roll = 0;   // 横滚角
        this.yaw = 0;    // 偏航角

        // 角速度 (rad/s)
        this.gx = 0;
        this.gy = 0;
        this.gz = 0;

        // 算法选择: 'madgwick' 或 'complementary'
        this.algorithm = 'madgwick';

        // Madgwick 滤波器参数 (增大 beta 提高响应速度)
        this.beta = 0.5;  // 增益参数 (0.1-1.0, 越大越灵敏但可能不稳定)

        // 互补滤波器参数
        this.alpha = 0.96;  // 陀螺仪权重 (降低以增加加速度计影响)

        // 采样时间 (秒)
        this.dt = 0.01;  // 默认 100Hz

        // 上次更新时间
        this.lastUpdateTime = null;
    }

    /**
     * 设置算法类型
     */
    setAlgorithm(algorithm) {
        if (algorithm === 'madgwick' || algorithm === 'complementary') {
            this.algorithm = algorithm;
            console.log(`✅ 姿态解算算法切换为: ${algorithm}`);
        }
    }

    /**
     * 设置采样频率
     */
    setSampleRate(fs) {
        this.dt = 1.0 / fs;
    }

    /**
     * 更新姿态 - 主入口
     * @param {number} gx - 陀螺仪 X 轴 (deg/s)
     * @param {number} gy - 陀螺仪 Y 轴 (deg/s)
     * @param {number} gz - 陀螺仪 Z 轴 (deg/s)
     * @param {number} ax - 加速度计 X 轴 (g)
     * @param {number} ay - 加速度计 Y 轴 (g)
     * @param {number} az - 加速度计 Z 轴 (g)
     */
    update(gx, gy, gz, ax, ay, az) {
        // 计算实际采样时间
        const now = Date.now();
        if (this.lastUpdateTime) {
            this.dt = (now - this.lastUpdateTime) / 1000.0;
            // 限制 dt 范围，避免异常值
            this.dt = Math.max(0.001, Math.min(this.dt, 0.1));
        } else {
            this.dt = 0.01; // 默认 100Hz
        }
        this.lastUpdateTime = now;

        // 保存角速度 (转换为 rad/s)
        this.gx = gx * Math.PI / 180;
        this.gy = gy * Math.PI / 180;
        this.gz = gz * Math.PI / 180;

        // 根据算法选择更新四元数
        if (this.algorithm === 'madgwick') {
            this.updateMadgwick(this.gx, this.gy, this.gz, ax, ay, az);
        } else {
            this.updateComplementary(this.gx, this.gy, this.gz, ax, ay, az);
        }

        // 从四元数计算欧拉角
        this.quaternionToEuler();
    }

    /**
     * Madgwick 滤波器更新
     */
    updateMadgwick(gx, gy, gz, ax, ay, az) {
        let [q0, q1, q2, q3] = this.q;

        // 归一化加速度计数据
        const norm = Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm === 0) return;
        ax /= norm;
        ay /= norm;
        az /= norm;

        // 目标函数的梯度
        const s0 = -2 * q2 * (2 * q1 * q3 - 2 * q0 * q2 - ax) +
                   -2 * q1 * (2 * q0 * q1 + 2 * q2 * q3 - ay) +
                   0;
        const s1 = 2 * q3 * (2 * q1 * q3 - 2 * q0 * q2 - ax) +
                   2 * q0 * (2 * q0 * q1 + 2 * q2 * q3 - ay) +
                   -4 * q1 * (1 - 2 * q1 * q1 - 2 * q2 * q2 - az);
        const s2 = -2 * q0 * (2 * q1 * q3 - 2 * q0 * q2 - ax) +
                   2 * q3 * (2 * q0 * q1 + 2 * q2 * q3 - ay) +
                   -4 * q2 * (1 - 2 * q1 * q1 - 2 * q2 * q2 - az);
        const s3 = 2 * q1 * (2 * q1 * q3 - 2 * q0 * q2 - ax) +
                   2 * q2 * (2 * q0 * q1 + 2 * q2 * q3 - ay) +
                   0;

        // 归一化梯度
        const sNorm = Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        const qs0 = s0 / sNorm;
        const qs1 = s1 / sNorm;
        const qs2 = s2 / sNorm;
        const qs3 = s3 / sNorm;

        // 四元数导数
        const qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - this.beta * qs0;
        const qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - this.beta * qs1;
        const qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - this.beta * qs2;
        const qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - this.beta * qs3;

        // 积分四元数
        q0 += qDot0 * this.dt;
        q1 += qDot1 * this.dt;
        q2 += qDot2 * this.dt;
        q3 += qDot3 * this.dt;

        // 归一化四元数
        const qNorm = Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        this.q = [q0 / qNorm, q1 / qNorm, q2 / qNorm, q3 / qNorm];
    }

    /**
     * 互补滤波器更新
     */
    updateComplementary(gx, gy, gz, ax, ay, az) {
        // 归一化加速度计数据
        const norm = Math.sqrt(ax * ax + ay * ay + az * az);
        if (norm === 0) return;
        ax /= norm;
        ay /= norm;
        az /= norm;

        // 从加速度计计算俯仰角和横滚角
        const pitchAcc = Math.atan2(ax, Math.sqrt(ay * ay + az * az));
        const rollAcc = Math.atan2(ay, az);

        // 陀螺仪积分
        const pitchGyro = this.pitch + gy * this.dt;
        const rollGyro = this.roll + gx * this.dt;
        const yawGyro = this.yaw + gz * this.dt;

        // 互补滤波
        this.pitch = this.alpha * pitchGyro + (1 - this.alpha) * pitchAcc;
        this.roll = this.alpha * rollGyro + (1 - this.alpha) * rollAcc;
        this.yaw = yawGyro;  // 偏航角只能用陀螺仪

        // 从欧拉角转换为四元数
        this.eulerToQuaternion();
    }

    /**
     * 四元数转欧拉角
     */
    quaternionToEuler() {
        const [q0, q1, q2, q3] = this.q;

        // Roll (x-axis rotation)
        const sinr_cosp = 2 * (q0 * q1 + q2 * q3);
        const cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        this.roll = Math.atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        const sinp = 2 * (q0 * q2 - q3 * q1);
        if (Math.abs(sinp) >= 1) {
            this.pitch = Math.sign(sinp) * Math.PI / 2;
        } else {
            this.pitch = Math.asin(sinp);
        }

        // Yaw (z-axis rotation)
        const siny_cosp = 2 * (q0 * q3 + q1 * q2);
        const cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        this.yaw = Math.atan2(siny_cosp, cosy_cosp);
    }

    /**
     * 欧拉角转四元数
     */
    eulerToQuaternion() {
        const cy = Math.cos(this.yaw * 0.5);
        const sy = Math.sin(this.yaw * 0.5);
        const cp = Math.cos(this.pitch * 0.5);
        const sp = Math.sin(this.pitch * 0.5);
        const cr = Math.cos(this.roll * 0.5);
        const sr = Math.sin(this.roll * 0.5);

        this.q[0] = cr * cp * cy + sr * sp * sy;
        this.q[1] = sr * cp * cy - cr * sp * sy;
        this.q[2] = cr * sp * cy + sr * cp * sy;
        this.q[3] = cr * cp * sy - sr * sp * cy;
    }

    /**
     * 获取欧拉角 (度)
     */
    getEulerAngles() {
        return {
            pitch: this.pitch * 180 / Math.PI,
            roll: this.roll * 180 / Math.PI,
            yaw: this.yaw * 180 / Math.PI
        };
    }

    /**
     * 获取四元数
     */
    getQuaternion() {
        return {
            w: this.q[0],
            x: this.q[1],
            y: this.q[2],
            z: this.q[3]
        };
    }

    /**
     * 获取角速度 (deg/s)
     */
    getAngularVelocity() {
        return {
            gx: this.gx * 180 / Math.PI,
            gy: this.gy * 180 / Math.PI,
            gz: this.gz * 180 / Math.PI
        };
    }

    /**
     * 重置姿态
     */
    reset() {
        this.q = [1, 0, 0, 0];
        this.pitch = 0;
        this.roll = 0;
        this.yaw = 0;
        this.lastUpdateTime = null;
        this._lastAcc = { ax: 0, ay: 0, az: 1 };
        this._accHistory = [];
        // 重置状态转换记忆
        this._lastPosture = null;
        this._lastPostureTime = 0;
        this._highMotionStartTime = 0;
    }

    /**
     * 判断动物姿态
     * 在每次 update() 后调用，需传入原始加速度（g）
     * @param {number} ax - 加速度 X (g)
     * @param {number} ay - 加速度 Y (g)
     * @param {number} az - 加速度 Z (g)
     * @returns {{ posture, label, confidence, accMag, gyrMag, isMoving }}
     */
    classifyPosture(ax, ay, az) {
        const pitchDeg = this.pitch * 180 / Math.PI;
        const rollDeg  = this.roll  * 180 / Math.PI;

        // 加速度幅值（g）
        const accMag = Math.sqrt(ax * ax + ay * ay + az * az);

        // 角速度幅值（deg/s）
        const gxDeg = this.gx * 180 / Math.PI;
        const gyDeg = this.gy * 180 / Math.PI;
        const gzDeg = this.gz * 180 / Math.PI;
        const gyrMag = Math.sqrt(gxDeg * gxDeg + gyDeg * gyDeg + gzDeg * gzDeg);

        // 滑动窗口：保留最近 20 帧加速度幅值，用于判断持续运动
        if (!this._accHistory) this._accHistory = [];
        this._accHistory.push(accMag);
        if (this._accHistory.length > 20) this._accHistory.shift();
        const accVar = this._accVariance(this._accHistory);

        // 基于真实数据的姿态模型（统计学方法）
        const postureModels = [
            {
                name: '低头进食',
                label: '🌿 低头进食',
                pitch_mean: -16.80,
                pitch_std: 5.94,
                roll_mean: 100.65,
                roll_std: 3.10,
                gyrMag_mean: 16.48,
                gyrMag_std: 5.84,
                accMag_mean: 1.0752,
                accMag_std: 0.1488
            },
            {
                name: '站立',
                label: '🐾 站立',
                pitch_mean: 16.08,
                pitch_std: 5.79,
                roll_mean: 113.30,
                roll_std: 2.16,
                gyrMag_mean: 8.26,
                gyrMag_std: 6.06,
                accMag_mean: 1.0623,
                accMag_std: 0.0563
            },
            {
                name: '右侧躺',
                label: '😴 右侧躺',
                pitch_mean: 7.04,
                pitch_std: 2.08,
                roll_mean: -165.70,
                roll_std: 3.35,
                gyrMag_mean: 5.08,
                gyrMag_std: 4.91,
                accMag_mean: 1.0246,
                accMag_std: 0.0114
            },
            {
                name: '行走',
                label: '🚶 行走',
                pitch_mean: 13.83,
                pitch_std: 9.24,
                roll_mean: 107.64,
                roll_std: 6.90,
                gyrMag_mean: 20.83,
                gyrMag_std: 7.17,
                accMag_mean: 1.0702,
                accMag_std: 0.1461
            },
            {
                name: '奔跑',
                label: '🏃 奔跑',
                pitch_mean: -7.23,
                pitch_std: 11.23,
                roll_mean: 100.60,
                roll_std: 6.01,
                gyrMag_mean: 23.30,
                gyrMag_std: 6.45,
                accMag_mean: 1.1685,
                accMag_std: 0.3288
            },
            {
                name: '仰卧',
                label: '🐾 仰卧',
                pitch_mean: 47.11,
                pitch_std: 10.28,
                roll_mean: 131.64,
                roll_std: 6.63,
                gyrMag_mean: 4.98,
                gyrMag_std: 4.67,
                accMag_mean: 1.0256,
                accMag_std: 0.0254
            },
            {
                name: '趴卧',
                label: '🐕 趴卧',
                pitch_mean: 10.21,
                pitch_std: 2.24,
                roll_mean: 113.38,
                roll_std: 1.20,
                gyrMag_mean: 3.49,
                gyrMag_std: 4.37,
                accMag_mean: 1.0674,
                accMag_std: 0.0154
            },
            {
                name: '左侧躺',
                label: '😴 左侧躺',
                pitch_mean: 12.58,
                pitch_std: 3.18,
                roll_mean: 21.99,
                roll_std: 2.27,
                gyrMag_mean: 5.08,
                gyrMag_std: 4.81,
                accMag_mean: 0.9644,
                accMag_std: 0.0120
            },
        ];

        // 判断运动状态（使用更保守的阈值避免误判）
        const runningModel = postureModels.find(m => m.name === '奔跑');
        const walkingModel = postureModels.find(m => m.name === '行走');

        // 奔跑：需要同时满足角速度和加速度的条件，或者角速度非常高
        // 使用更严格的阈值：角速度 > 23°/s 且加速度 > 1.1g
        let isRunning = (gyrMag > 23 && accMag > 1.1) ||
                         gyrMag > (runningModel.gyrMag_mean + runningModel.gyrMag_std);

        // 行走：角速度超过阈值，但不满足奔跑条件
        // 使用更严格的阈值：角速度 > 20°/s 且加速度方差 > 0.02
        // 同时要求持续时间，避免站立摇头被误判
        let isWalking = !isRunning && (gyrMag > 20 && accVar > 0.02);

        // 状态转换逻辑：防止从躺卧/站立状态突然跳到运动状态
        // 记录上一次的姿态状态
        if (!this._lastPosture) this._lastPosture = null;
        if (!this._lastPostureTime) this._lastPostureTime = 0;
        if (!this._highMotionStartTime) this._highMotionStartTime = 0;

        const now = Date.now();

        // 静态姿态列表（包括躺卧和站立）
        const staticPostures = ['左侧躺', '右侧躺', '仰卧', '趴卧', '站立', '低头进食'];

        // 如果上一次是静态姿态，需要持续高角速度才能转换为运动状态
        if (this._lastPosture && staticPostures.includes(this._lastPosture)) {
            if (isRunning || isWalking) {
                // 检测到高运动，记录开始时间
                if (this._highMotionStartTime === 0) {
                    this._highMotionStartTime = now;
                }

                const highMotionDuration = now - this._highMotionStartTime;

                // 需要持续高角速度至少1.5秒，且角速度足够高
                // 这样可以过滤掉站立摇头、翻身等短暂的高角速度
                if (highMotionDuration < 1500) {
                    // 时间太短，保持静态姿态
                    isRunning = false;
                    isWalking = false;
                } else if (gyrMag < 25) {
                    // 角速度不够高，可能只是摇头或翻身
                    isRunning = false;
                    isWalking = false;
                }
            } else {
                // 没有检测到高运动，重置计时器
                this._highMotionStartTime = 0;
            }
        } else {
            // 不是静态姿态，重置计时器
            this._highMotionStartTime = 0;
        }

        let bestMatch = null;
        let bestScore = Infinity;

        if (isRunning) {
            bestMatch = runningModel;
        } else if (isWalking) {
            bestMatch = walkingModel;
        } else {
            // 静态姿态：使用基于规则的分类（按优先级顺序）
            const staticModels = postureModels.filter(m => m.name !== '行走' && m.name !== '奔跑');

            // 规则优先级（从高到低）：
            // 1. 右侧躺（roll 最独特，< -157°）
            // 2. 左侧躺（roll 最独特，16-28°）
            // 3. 仰卧（pitch 最独特，> 30°）
            // 4. 低头进食（pitch < -10°）
            // 5. 趴卧 vs 站立（pitch 区分，roll 在 108-119°）

            // 规则1：右侧躺（roll < -157°）
            if (rollDeg < -157) {
                bestMatch = staticModels.find(m => m.name === '右侧躺');
                bestScore = 0.2;
            }
            // 规则2：左侧躺（roll 在 16-28°）
            else if (rollDeg > 16 && rollDeg < 29) {
                bestMatch = staticModels.find(m => m.name === '左侧躺');
                bestScore = 0.2;
            }
            // 规则3：仰卧（pitch > 30°）
            else if (pitchDeg > 30) {
                bestMatch = staticModels.find(m => m.name === '仰卧');
                bestScore = 0.2;
            }
            // 规则4：低头进食（pitch < -10° 且 roll 在 87-112°）
            else if (pitchDeg < -10 && rollDeg > 87 && rollDeg < 112) {
                bestMatch = staticModels.find(m => m.name === '低头进食');
                bestScore = 0.3;
            }
            // 规则5：趴卧 vs 站立（roll 在 108-119°）
            // 使用最佳平衡阈值 13.5° 来区分趴卧和站立
            // 趴卧92.3%, 站立75.2%, 总体95.0%, 平衡分数82.9
            else if (rollDeg > 108 && rollDeg < 119) {
                if (pitchDeg < 13.5) {
                    bestMatch = staticModels.find(m => m.name === '趴卧');
                    bestScore = 0.3;
                } else {
                    bestMatch = staticModels.find(m => m.name === '站立');
                    bestScore = 0.3;
                }
            }
            // 规则6：其他情况，使用加权距离匹配
            else {
                for (const model of staticModels) {
                    const pitchDist = Math.abs(pitchDeg - model.pitch_mean) / (model.pitch_std + 0.1);
                    const rollDist = Math.abs(rollDeg - model.roll_mean) / (model.roll_std + 0.1);

                    // pitch 权重更高
                    const weightedDist = pitchDist * 1.5 + rollDist * 1;

                    if (weightedDist < bestScore) {
                        bestScore = weightedDist;
                        bestMatch = model;
                    }
                }
            }
        }

        // 根据距离计算置信度
        let confidence;
        if (isRunning || isWalking) {
            confidence = 0.80;
        } else {
            // 距离越小，置信度越高
            // bestScore < 1: 非常接近 (95%)
            // bestScore < 2: 接近 (85%)
            // bestScore < 3: 较接近 (70%)
            // bestScore >= 3: 不确定 (50%)
            if (bestScore < 1) {
                confidence = 0.95;
            } else if (bestScore < 2) {
                confidence = 0.85;
            } else if (bestScore < 3) {
                confidence = 0.70;
            } else {
                confidence = 0.50;
            }
        }

        // 更新状态记忆
        const currentPosture = bestMatch ? bestMatch.name : 'unknown';
        if (currentPosture !== this._lastPosture) {
            this._lastPosture = currentPosture;
            this._lastPostureTime = Date.now();
        }

        return {
            posture: bestMatch ? bestMatch.name : 'unknown',
            label: bestMatch ? bestMatch.label : '❓ 未知',
            confidence: confidence,
            pitchDeg: pitchDeg.toFixed(1),
            rollDeg:  rollDeg.toFixed(1),
            accMag:   accMag.toFixed(3),
            gyrMag:   gyrMag.toFixed(1),
            isMoving: isRunning || isWalking,
            isRunning: isRunning
        };
    }

    /** 计算数组方差（内部辅助） */
    _accVariance(arr) {
        if (arr.length < 2) return 0;
        const mean = arr.reduce((s, v) => s + v, 0) / arr.length;
        return arr.reduce((s, v) => s + (v - mean) ** 2, 0) / arr.length;
    }
}

/**

 * 3D 姿态可视化器
 * 使用 Three.js 显示立方体姿态
 */
class AttitudeVisualizer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) {
            console.error('❌ 找不到容器:', containerId);
            return;
        }

        // 初始化 Three.js
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.cube = null;
        this.animationId = null;

        this.init();
    }

    /**
     * 初始化 3D 场景
     */
    init() {
        console.log('🎯 开始初始化 3D 场景...');
        console.log('THREE 对象:', typeof THREE);

        if (typeof THREE === 'undefined') {
            console.error('❌ THREE.js 未加载!');
            this.container.innerHTML = '<div style="color:#ff6b6b;text-align:center;padding:40px;">Three.js 未加载，请检查网络连接</div>';
            return;
        }

        // 创建场景
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);
        console.log('✅ 场景创建成功');

        // 创建相机 — 使用实际尺寸，若容器仍为0则用合理默认值
        const width = this.container.clientWidth || this.container.offsetWidth || 600;
        const height = this.container.clientHeight || this.container.offsetHeight || 400;
        console.log(`📐 容器尺寸: ${width}x${height}`);

        if (width === 0 || height === 0) {
            console.error('❌ 容器尺寸为 0!');
            return;
        }

        this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        this.camera.position.set(4, 4, 4);
        this.camera.lookAt(0, 0, 0);
        console.log('✅ 相机创建成功');

        // 创建渲染器
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(width, height);
        this.container.appendChild(this.renderer.domElement);
        console.log('✅ 渲染器创建成功');

        // 添加环境光
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        // 添加方向光
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 5, 5);
        this.scene.add(directionalLight);
        console.log('✅ 光照添加成功');

        // 创建立方体
        const geometry = new THREE.BoxGeometry(2, 1, 0.5);
        const materials = [
            new THREE.MeshLambertMaterial({ color: 0xff0000 }), // 右 - 红色
            new THREE.MeshLambertMaterial({ color: 0x00ff00 }), // 左 - 绿色
            new THREE.MeshLambertMaterial({ color: 0x0000ff }), // 上 - 蓝色
            new THREE.MeshLambertMaterial({ color: 0xffff00 }), // 下 - 黄色
            new THREE.MeshLambertMaterial({ color: 0xff00ff }), // 前 - 品红
            new THREE.MeshLambertMaterial({ color: 0x00ffff })  // 后 - 青色
        ];
        this.cube = new THREE.Mesh(geometry, materials);
        this.scene.add(this.cube);
        console.log('✅ 立方体创建成功');

        // 添加坐标轴辅助线
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);

        // 添加网格
        const gridHelper = new THREE.GridHelper(10, 10, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // 开始渲染
        this.animate();
        console.log('✅ 3D 场景初始化完成!');

        // 窗口大小调整
        window.addEventListener('resize', () => this.onWindowResize());
    }

    /**
     * 更新立方体姿态
     * @param {number} pitch - 俯仰角 (弧度)
     * @param {number} roll - 横滚角 (弧度)
     * @param {number} yaw - 偏航角 (弧度)
     */
    updateAttitude(pitch, roll, yaw) {
        if (!this.cube) return;

        // 设置旋转 (注意 Three.js 使用 ZYX 顺序)
        this.cube.rotation.set(pitch, yaw, roll);
    }

    /**
     * 使用四元数更新姿态
     */
    updateQuaternion(w, x, y, z) {
        if (!this.cube) return;
        this.cube.quaternion.set(x, y, z, w);
    }

    /**
     * 动画循环
     */
    animate() {
        if (!this.renderer || !this.scene || !this.camera) return;
        this.animationId = requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }

    /**
     * 窗口大小调整
     */
    onWindowResize() {
        if (!this.container || !this.camera || !this.renderer) return;

        const width = this.container.clientWidth;
        const height = this.container.clientHeight || 400;

        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    /**
     * 停止渲染
     */
    stop() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
            this.animationId = null;
        }
    }

    /**
     * 重新启动渲染
     */
    start() {
        if (!this.animationId && this.renderer) {
            this.animate();
        }
    }

    /**
     * 销毁
     */
    destroy() {
        this.stop();
        if (this.renderer && this.container) {
            this.container.removeChild(this.renderer.domElement);
        }
    }
}
