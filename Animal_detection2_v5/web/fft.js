/**
 * 简化的FFT实现
 * 用于毫米波雷达数据处理
 */

class SimpleFFT {
    /**
     * 快速傅里叶变换 (Cooley-Tukey算法)
     * @param {Array} x - 输入复数数组 [[real, imag], [real, imag], ...]
     * @returns {Array} FFT结果
     */
    static fft(x) {
        const N = x.length;
        
        // 如果长度为1，直接返回
        if (N <= 1) return x;
        
        // 确保长度为2的幂
        if (N & (N - 1)) {
            // 如果不是2的幂，填充零
            const nextPowerOf2 = Math.pow(2, Math.ceil(Math.log2(N)));
            const padded = [...x];
            while (padded.length < nextPowerOf2) {
                padded.push([0, 0]);
            }
            return this.fft(padded);
        }
        
        // 分治：分为偶数和奇数索引
        const even = [];
        const odd = [];
        
        for (let i = 0; i < N; i++) {
            if (i % 2 === 0) {
                even.push(x[i]);
            } else {
                odd.push(x[i]);
            }
        }
        
        // 递归计算
        const evenFFT = this.fft(even);
        const oddFFT = this.fft(odd);
        
        // 合并结果
        const result = new Array(N);
        
        for (let k = 0; k < N / 2; k++) {
            // 计算旋转因子 W_N^k = e^(-2πik/N)
            const angle = -2 * Math.PI * k / N;
            const wk = [Math.cos(angle), Math.sin(angle)];
            
            // 复数乘法: oddFFT[k] * wk
            const oddMulWk = [
                oddFFT[k][0] * wk[0] - oddFFT[k][1] * wk[1],
                oddFFT[k][0] * wk[1] + oddFFT[k][1] * wk[0]
            ];
            
            // X[k] = E[k] + W_N^k * O[k]
            result[k] = [
                evenFFT[k][0] + oddMulWk[0],
                evenFFT[k][1] + oddMulWk[1]
            ];
            
            // X[k + N/2] = E[k] - W_N^k * O[k]
            result[k + N / 2] = [
                evenFFT[k][0] - oddMulWk[0],
                evenFFT[k][1] - oddMulWk[1]
            ];
        }
        
        return result;
    }
    
    /**
     * 实数FFT (针对实数输入优化)
     * @param {Array} realData - 实数数组
     * @returns {Array} FFT结果的幅度
     */
    static realFFT(realData) {
        // 将实数转换为复数格式
        const complexData = realData.map(val => [val, 0]);
        
        // 执行FFT
        const fftResult = this.fft(complexData);
        
        // 计算幅度
        return fftResult.map(([real, imag]) => Math.sqrt(real * real + imag * imag));
    }
    
    /**
     * 计算功率谱密度
     * @param {Array} data - 输入数据
     * @returns {Array} 功率谱密度
     */
    static powerSpectrum(data) {
        const fftResult = this.realFFT(data);
        return fftResult.map(magnitude => magnitude * magnitude);
    }
}

// 为了兼容性，创建全局FFT对象
window.FFT = {
    fft: SimpleFFT.fft,
    realFFT: SimpleFFT.realFFT,
    powerSpectrum: SimpleFFT.powerSpectrum
};

// 导出供Node.js使用
if (typeof module !== 'undefined' && module.exports) {
    module.exports = SimpleFFT;
}
