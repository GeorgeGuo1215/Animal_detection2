/**
 * Web Bluetooth 管理器
 * 协议优先级：FFF0（V1.02新协议）→ NUS → 通用自动发现
 *
 * 新协议帧格式（BLE_Telemetry_Data_Frame V1.02）：
 *   $MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3*[\r\n]
 * 服务：FFF0  通知特征：FFF1（设备→手机）  写入特征：FFF2（手机→设备）
 */

(function () {
	// 新协议 FFF0/FFF1/FFF2 完整 128-bit UUID
	const UUID_FFF0 = '0000fff0-0000-1000-8000-00805f9b34fb';
	const UUID_FFF1 = '0000fff1-0000-1000-8000-00805f9b34fb'; // Notify: 设备→手机
	const UUID_FFF2 = '0000fff2-0000-1000-8000-00805f9b34fb'; // Write:  手机→设备

	class BluetoothManager {
		constructor() {
			this.device = null;
			this.server = null;
			this.notifyCharacteristic = null;
			this.writeCharacteristic = null;
			this.decoder = new TextDecoder('utf-8');
			this._rxBuffer = '';
			this.debug = false; // 热路径日志开关，生产环境关闭
			// BLE 通知层诊断统计
			this._notifyStats = {
				count: 0,          // 通知总数
				bytesTotal: 0,     // 字节总数
				lastTs: 0,         // 上次通知时间戳(ms)
				windowStart: 0,    // 当前统计窗口起点
				windowCount: 0,    // 当前窗口通知数
				windowFrames: 0,   // 当前窗口完整帧数
				intervals: [],     // 最近通知间隔(ms)
			};
			this._notifyDiagTimer = null;
			// 事件回调
			this.onConnect = null;
			this.onDisconnect = null;
			this.onLine = null; // (line: string) => void
			this.onError = null; // (err: Error) => void
			this.onServiceDiscovered = null; // (info: string) => void
		}

		async connect() {
			if (!navigator.bluetooth) {
				this._emitError(new Error('当前浏览器不支持 Web Bluetooth'));
				return;
			}

			try {
				this.device = await navigator.bluetooth.requestDevice({
					acceptAllDevices: true,
					optionalServices: [
						// ★ 新协议 FFF0 服务（V1.02，最高优先级）
						UUID_FFF0,
						// Nordic UART Service (NUS)
						'6e400001-b5a3-f393-e0a9-e50e24dcca9e',
						// 标准 GATT 服务
						0x180D, // Heart Rate
						0x180F, // Battery
						0x1800, // Generic Access
						0x1801, // Generic Attribute
						0x180A, // Device Information
						// 其他常见透传服务
						0xFFE0,
						'0000ffe0-0000-1000-8000-00805f9b34fb'
					]
				});

				this.device.addEventListener('gattserverdisconnected', this._handleDisconnect.bind(this));
				this.server = await this.device.gatt.connect();

				// ★ 等待 BLE 协议栈就绪（对齐小程序 300ms + 余量）
				await new Promise(r => setTimeout(r, 500));

				// ★ 服务发现（带 1 次重试）
				await this._discoverAndSetup(1);

				this._saveDevice();
				this._emitConnect();
			} catch (err) {
				this._emitError(err);
				await this.disconnect();
			}
		}

		/**
		 * 服务发现主流程（对齐小程序：先获取所有服务，再匹配 FFF0）
		 * retries: 剩余重试次数
		 */
		async _discoverAndSetup(retries = 1) {
			if (!this.server) throw new Error('GATT server 已断开');

			const services = await this.server.getPrimaryServices();
			this._emitServiceDiscovered(`🔍 发现 ${services.length} 个服务`);

			// 1. 在已发现的服务中查找 FFF0
			const fff0 = services.find(s => s.uuid.toLowerCase().includes('fff0'));
			if (fff0) {
				try {
					this.notifyCharacteristic = await fff0.getCharacteristic(UUID_FFF1);
					this.writeCharacteristic  = await fff0.getCharacteristic(UUID_FFF2);
					await this._startNotifications(this.notifyCharacteristic);
					this._emitServiceDiscovered('✓ 使用 FFF0 服务协议（V1.02，FFF1 通知 / FFF2 写入）');
					return;
				} catch (e) {
					this._emitServiceDiscovered(`⚠️ FFF0 特征绑定失败: ${e.message}`);
				}
			}

			// 2. 查找 NUS
			const nus = services.find(s => s.uuid.toLowerCase().includes('6e400001'));
			if (nus) {
				try {
					const txUuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
					const rxUuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
					this.notifyCharacteristic = await nus.getCharacteristic(txUuid);
					this.writeCharacteristic  = await nus.getCharacteristic(rxUuid);
					await this._startNotifications(this.notifyCharacteristic);
					this._emitServiceDiscovered('✓ 使用 Nordic UART Service (NUS) 协议');
					return;
				} catch (e) {
					this._emitServiceDiscovered(`⚠️ NUS 特征绑定失败: ${e.message}`);
				}
			}

			// 3. 都没找到 → 重试
			if (retries > 0 && this.server) {
				this._emitServiceDiscovered(`未找到 FFF0/NUS，${1}秒后重试服务发现...`);
				await new Promise(r => setTimeout(r, 1000));
				if (this.server) return this._discoverAndSetup(retries - 1);
			}

			// 4. 最终降级：扫描所有可通知特征
			await this._setupAllNotifiableCharacteristics();
		}

		/**
		 * 新协议：绑定 FFF0 服务的 FFF1（通知）和 FFF2（写入）特征（保留作为备用）
		 */
		async _setupFFF() {
			const service = await this.server.getPrimaryService(UUID_FFF0);
			this.notifyCharacteristic = await service.getCharacteristic(UUID_FFF1);
			this.writeCharacteristic  = await service.getCharacteristic(UUID_FFF2);
			await this._startNotifications(this.notifyCharacteristic);
		}

		async _setupNUS(serviceUuid) {
			const service = await this.server.getPrimaryService(serviceUuid);
			const txUuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // notify
			const rxUuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // write
			this.notifyCharacteristic = await service.getCharacteristic(txUuid);
			this.writeCharacteristic  = await service.getCharacteristic(rxUuid);
			await this._startNotifications(this.notifyCharacteristic);
		}

		async _setupAllNotifiableCharacteristics() {
			if (!this.server) throw new Error('GATT server 已断开');
			const services = await this.server.getPrimaryServices();
			let notifiableCount = 0;
			let writableCount = 0;
			
			this._emitServiceDiscovered(`🔍 发现 ${services.length} 个服务，开始扫描特征...`);
			
			for (const service of services) {
				try {
					const characteristics = await service.getCharacteristics();
					const serviceUuid = service.uuid;
					this._emitServiceDiscovered(`📋 服务 ${this._formatUuid(serviceUuid)}: ${characteristics.length} 个特征`);
					
					for (const ch of characteristics) {
						const props = ch.properties;
						const chUuid = this._formatUuid(ch.uuid);
						const capabilities = [];
						
						if (props.read) capabilities.push('读取');
						if (props.write) capabilities.push('写入');
						if (props.writeWithoutResponse) capabilities.push('写入(无应答)');
						if (props.notify) capabilities.push('通知');
						if (props.indicate) capabilities.push('指示');
						
						this._emitServiceDiscovered(`  └─ 特征 ${chUuid}: ${capabilities.join(', ')}`);
						
						// 收集所有可通知的特征
						if (props.notify || props.indicate) {
							if (!this.notifyCharacteristic) {
								this.notifyCharacteristic = ch; // 使用第一个作为主要通知特征
								this._emitServiceDiscovered(`  ✓ 设为主要通知特征`);
							}
							await this._startNotifications(ch);
							notifiableCount++;
						}
						
						// 收集所有可写的特征
						if (props.write || props.writeWithoutResponse) {
							if (!this.writeCharacteristic) {
								this.writeCharacteristic = ch; // 使用第一个作为主要写入特征
								this._emitServiceDiscovered(`  ✓ 设为主要写入特征`);
							}
							writableCount++;
						}
					}
				} catch (error) {
					// 某些服务可能需要配对或权限，跳过即可
					this._emitServiceDiscovered(`  ❌ 服务 ${this._formatUuid(service.uuid)} 访问失败: ${error.message}`);
				}
			}
			
			if (notifiableCount === 0) {
				throw new Error('未找到可通知(Notify/Indicate)的特征');
			}
			
			this._emitServiceDiscovered(`🎉 完成扫描：${notifiableCount} 个可通知特征，${writableCount} 个可写特征`);
		}

		async _setupFirstNotifiable() {
			// 保留原方法作为兜底
			return this._setupAllNotifiableCharacteristics();
		}

		async _startNotifications(characteristic = null) {
			const ch = characteristic || this.notifyCharacteristic;
			if (!ch) throw new Error('缺少通知特征');
			
			try {
				await ch.startNotifications();
				ch.addEventListener('characteristicvaluechanged', (event) => {
					const value = event.target.value; // DataView
					const now = performance.now();
					const byteLen = value.byteLength;
					const s = this._notifyStats;
					s.count++;
					s.bytesTotal += byteLen;
					if (s.lastTs > 0) {
						s.intervals.push(now - s.lastTs);
						if (s.intervals.length > 100) s.intervals.shift();
					}
					s.lastTs = now;
					s.windowCount++;

					const chunk = this.decoder.decode(value);
					this._handleIncomingText(chunk);
				});
				// 每 2 秒输出一次 BLE 通知层诊断
				this._notifyDiagTimer = setInterval(() => {
					const s = this._notifyStats;
					if (s.windowCount === 0) return;
					const intervals = s.intervals.slice(-50);
					const avgMs = intervals.length > 0 ? intervals.reduce((a, b) => a + b, 0) / intervals.length : 0;
					const notifyHz = avgMs > 0 ? (1000 / avgMs) : 0;
					console.log(
						`[BLE诊断] 通知: ${s.count}次, 本窗口: ${s.windowCount}次/${s.windowFrames}帧, ` +
						`平均间隔: ${avgMs.toFixed(1)}ms, 通知频率: ${notifyHz.toFixed(1)}Hz, ` +
						`平均payload: ${s.windowCount > 0 ? ((s.bytesTotal / s.count) | 0) : 0}B`
					);
					s.windowCount = 0;
					s.windowFrames = 0;
				}, 2000);
				console.log(`已启动特征 ${ch.uuid} 的通知`);
			} catch (error) {
				console.warn(`无法启动特征 ${ch.uuid} 的通知:`, error.message);
				throw error;
			}
		}

		_handleIncomingText(text) {
			if (this.debug) console.log('BLE收到数据块:', text.substring(0, 100), `(长度: ${text.length})`);
			this._rxBuffer += text;
			let lineCount = 0;

			let found = true;
			while (found) {
				found = false;
				const nlIdx   = this._rxBuffer.indexOf('\n');
				const starIdx = this._rxBuffer.indexOf('*');

				let delimIdx = -1;
				if (nlIdx >= 0 && (starIdx < 0 || nlIdx < starIdx)) {
					delimIdx = nlIdx;
				} else if (starIdx >= 0) {
					delimIdx = starIdx;
				}

				if (delimIdx >= 0) {
					found = true;
					let line = this._rxBuffer.slice(0, delimIdx);
					this._rxBuffer = this._rxBuffer.slice(delimIdx + 1);
					line = line.replace(/[\r\n]+/g, '').trim();
					if (line && typeof this.onLine === 'function') {
						lineCount++;
						if (this.debug && lineCount <= 3) {
							console.log(`BLE解析帧 #${lineCount}:`, line.substring(0, 120));
						}
						try { this.onLine(line); } catch (e) {
							console.error('onLine回调错误:', e);
						}
					}
				}
			}

			if (lineCount > 0) {
				this._notifyStats.windowFrames += lineCount;
			}
		}

		async send(text) {
			if (!this.writeCharacteristic) throw new Error('该设备不支持写入或未发现写入特征');
			const data = new TextEncoder().encode(text);
			await this.writeCharacteristic.writeValue(data);
		}

		async disconnect() {
			try {
				if (this._notifyDiagTimer) { clearInterval(this._notifyDiagTimer); this._notifyDiagTimer = null; }
				if (this.notifyCharacteristic) {
					try { await this.notifyCharacteristic.stopNotifications(); } catch (_) {}
					this.notifyCharacteristic = null;
				}
				if (this.device && this.device.gatt && this.device.gatt.connected) {
					await this.device.gatt.disconnect();
				}
			} finally {
				this.server = null;
				this.device = null;
				this._emitDisconnect();
			}
		}

		// ── 设备持久化：保存/读取/清除上次连接的设备信息 ──

		_saveDevice() {
			if (!this.device) return;
			try {
				localStorage.setItem('ble_saved_device', JSON.stringify({
					id: this.device.id,
					name: this.device.name || '未知设备',
					savedAt: Date.now()
				}));
			} catch (_) {}
		}

		getSavedDeviceInfo() {
			try {
				const raw = localStorage.getItem('ble_saved_device');
				return raw ? JSON.parse(raw) : null;
			} catch (_) { return null; }
		}

		clearSavedDevice() {
			localStorage.removeItem('ble_saved_device');
		}

		async reconnectToSaved() {
			const saved = this.getSavedDeviceInfo();
			if (!saved) throw new Error('没有保存的设备');

			if (!navigator.bluetooth || !navigator.bluetooth.getDevices) {
				throw new Error('当前浏览器不支持 getDevices()，请手动重新连接');
			}

			const devices = await navigator.bluetooth.getDevices();
			const target = devices.find(d => d.id === saved.id);
			if (!target) throw new Error('未找到已保存的设备，请手动重新连接');

			this.device = target;
			this.device.addEventListener('gattserverdisconnected', this._handleDisconnect.bind(this));
			this.server = await this.device.gatt.connect();

			try {
				await this._setupFFF();
			} catch (_) {
				try {
					await this._setupNUS('6e400001-b5a3-f393-e0a9-e50e24dcca9e');
				} catch (__) {
					await this._setupAllNotifiableCharacteristics();
				}
			}

			this._saveDevice();
			this._emitConnect();
		}

		_handleDisconnect() {
			if (this._notifyDiagTimer) { clearInterval(this._notifyDiagTimer); this._notifyDiagTimer = null; }
			this.server = null;
			this.notifyCharacteristic = null;
			this.writeCharacteristic = null;
			this._emitDisconnect();
		}

		_emitConnect() {
			if (typeof this.onConnect === 'function') {
				try { this.onConnect(this.device); } catch (e) { console.error('[BLE] onConnect callback error:', e); }
			}
		}

		_emitDisconnect() {
			if (typeof this.onDisconnect === 'function') {
				try { this.onDisconnect(); } catch (e) { console.error('[BLE] onDisconnect callback error:', e); }
			}
		}

		_emitError(err) {
			if (typeof this.onError === 'function') {
				try { this.onError(err); } catch (_) {}
			}
			console.error('[BLE]', err);
		}

		_emitServiceDiscovered(info) {
			if (typeof this.onServiceDiscovered === 'function') {
				try { this.onServiceDiscovered(info); } catch (_) {}
			}
			console.log('[BLE]', info);
		}

		_formatUuid(uuid) {
			// 简化UUID显示，对于标准16位UUID显示为十六进制，128位UUID显示前8位
			if (typeof uuid === 'number') {
				return `0x${uuid.toString(16).toUpperCase().padStart(4, '0')}`;
			}
			if (uuid.length === 36) {
				// 128位UUID: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
				const short = uuid.substring(0, 8);
				return uuid.startsWith('0000') && uuid.endsWith('-0000-1000-8000-00805f9b34fb') 
					? `0x${uuid.substring(4, 8).toUpperCase()}` 
					: `${short}...`;
			}
			return uuid;
		}
	}

	// 暴露到全局
	window.BluetoothManager = BluetoothManager;
	window.BLE = new BluetoothManager();
})();


