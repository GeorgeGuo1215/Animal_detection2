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

				// 协议尝试顺序：FFF0 → NUS → 通用自动发现
				try {
					await this._setupFFF();
					this._emitServiceDiscovered('✓ 使用 FFF0 服务协议（V1.02，FFF1 通知 / FFF2 写入）');
				} catch (_fffErr) {
					try {
						await this._setupNUS('6e400001-b5a3-f393-e0a9-e50e24dcca9e');
						this._emitServiceDiscovered('✓ 使用 Nordic UART Service (NUS) 协议');
					} catch (_nusErr) {
						await this._setupAllNotifiableCharacteristics();
					}
				}

				this._emitConnect();
			} catch (err) {
				this._emitError(err);
				await this.disconnect();
			}
		}

		/**
		 * 新协议：绑定 FFF0 服务的 FFF1（通知）和 FFF2（写入）特征
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
					const chunk = this.decoder.decode(value);
					this._handleIncomingText(chunk);
				});
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

			if (this.debug && lineCount > 0) {
				console.log(`本次处理了 ${lineCount} 帧数据`);
			}
		}

		async send(text) {
			if (!this.writeCharacteristic) throw new Error('该设备不支持写入或未发现写入特征');
			const data = new TextEncoder().encode(text);
			await this.writeCharacteristic.writeValue(data);
		}

		async disconnect() {
			try {
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

		_handleDisconnect() {
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


