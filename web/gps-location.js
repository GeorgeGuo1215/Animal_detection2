/**
 * GPS 定位追踪模块
 * 基于 BLE V1.02 协议的 Lon/Lat 字段，使用 Leaflet + OpenStreetMap 实时显示位置和轨迹
 *
 * 协议格式: $MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3*
 * Lon 格式: E114.2010058 / W114.2010058 (E=东经正, W=西经负)
 * Lat 格式: N22.3787343  / S22.3787343  (N=北纬正, S=南纬负)
 */

class GPSLocationMonitor {
    constructor() {
        this.enabled = false;
        this.initialized = false;

        // Leaflet 实例
        this.map = null;
        this.marker = null;
        this.polyline = null;
        this.accuracyCircle = null;

        // 轨迹数据
        this.trackPoints = [];   // [{lat, lng, timestamp}]
        this.maxPoints = 5000;

        // 统计
        this.totalDistance = 0;   // 米
        this.currentSpeed = 0;   // m/s
        this.lastValidPoint = null;
        this.noFixCount = 0;

        // 节流：地图最多 1Hz 刷新
        this.lastUpdateTime = 0;
        this.updateThrottleMs = 1000;
    }

    // ── 坐标解析 ──────────────────────────────────────────

    /**
     * 解析 BLE 协议的经纬度字符串
     * @param {string} lonStr  如 "E114.2010058" 或 "W73.9857"
     * @param {string} latStr  如 "N22.3787343" 或 "S33.8688"
     * @returns {{lat: number, lng: number}|null}  null 表示无定位
     */
    static parseGPSCoord(lonStr, latStr) {
        if (!lonStr || !latStr) return null;

        const lonDir = lonStr.charAt(0).toUpperCase();
        const latDir = latStr.charAt(0).toUpperCase();
        const lonVal = parseFloat(lonStr.slice(1));
        const latVal = parseFloat(latStr.slice(1));

        if (!Number.isFinite(lonVal) || !Number.isFinite(latVal)) return null;

        // 无定位信号 (0.0000000)
        if (Math.abs(lonVal) < 0.0001 && Math.abs(latVal) < 0.0001) return null;

        // 范围校验
        if (Math.abs(lonVal) > 180 || Math.abs(latVal) > 90) return null;

        const lng = (lonDir === 'W') ? -lonVal : lonVal;
        const lat = (latDir === 'S') ? -latVal : latVal;

        return { lat, lng };
    }

    // ── 生命周期 ──────────────────────────────────────────

    init() {
        if (this.initialized) return;
        if (typeof L === 'undefined') {
            console.error('[GPS] Leaflet 未加载，无法初始化地图');
            return;
        }

        const container = document.getElementById('gpsMapContainer');
        if (!container) {
            console.error('[GPS] 未找到 #gpsMapContainer');
            return;
        }

        // 默认中心：中国 (30, 110)，缩放 4
        this.map = L.map('gpsMapContainer').setView([30, 110], 4);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '&copy; OpenStreetMap contributors'
        }).addTo(this.map);

        // 自定义宠物图标
        const petIcon = L.divIcon({
            className: 'gps-pet-marker',
            html: '<div style="font-size:28px;text-shadow:0 2px 6px rgba(0,0,0,0.3);">🐾</div>',
            iconSize: [32, 32],
            iconAnchor: [16, 16]
        });

        this.marker = L.marker([30, 110], { icon: petIcon }).addTo(this.map);
        this.marker.bindPopup('等待 GPS 定位...');

        this.polyline = L.polyline([], {
            color: '#3498db',
            weight: 3,
            opacity: 0.8
        }).addTo(this.map);

        this.initialized = true;
        console.log('[GPS] 地图初始化完成');
    }

    start() {
        if (!this.initialized) this.init();
        this.enabled = true;
        this._updateStatusUI(true);
        // Leaflet 需要在容器可见后刷新尺寸
        if (this.map) {
            setTimeout(() => this.map.invalidateSize(), 200);
        }
    }

    stop() {
        this.enabled = false;
        this._updateStatusUI(false);
    }

    reset() {
        this.trackPoints = [];
        this.totalDistance = 0;
        this.currentSpeed = 0;
        this.lastValidPoint = null;
        this.noFixCount = 0;

        if (this.polyline) this.polyline.setLatLngs([]);
        if (this.marker) {
            this.marker.setLatLng([30, 110]);
            this.marker.setPopupContent('等待 GPS 定位...');
        }
        if (this.map) this.map.setView([30, 110], 4);

        this._updateStatsUI();
    }

    // ── 数据入口 ──────────────────────────────────────────

    addGPSData(lonStr, latStr, timestamp) {
        if (!this.enabled) return;

        const coord = GPSLocationMonitor.parseGPSCoord(lonStr, latStr);
        if (!coord) {
            this.noFixCount++;
            this._setEl('gpsFixStatus', '无信号');
            this._setColor('gpsFixStatus', '#e74c3c');
            return;
        }

        this.noFixCount = 0;
        this._setEl('gpsFixStatus', '已定位');
        this._setColor('gpsFixStatus', '#27ae60');

        // 节流
        if (timestamp - this.lastUpdateTime < this.updateThrottleMs) return;
        this.lastUpdateTime = timestamp;

        // GPS 跳变保护：> 500m/s (~1800 km/h) 视为异常
        if (this.lastValidPoint) {
            const dt = (timestamp - this.lastValidPoint.timestamp) / 1000;
            if (dt > 0) {
                const dist = this._haversine(
                    this.lastValidPoint.lat, this.lastValidPoint.lng,
                    coord.lat, coord.lng
                );
                const speed = dist / dt;
                if (speed > 500) return; // 丢弃异常跳变

                this.totalDistance += dist;
                this.currentSpeed = speed;
            }
        }

        const point = { lat: coord.lat, lng: coord.lng, timestamp };
        this.trackPoints.push(point);
        if (this.trackPoints.length > this.maxPoints) {
            this.trackPoints.shift();
        }
        this.lastValidPoint = point;

        // 更新地图
        this._updateMap(coord.lat, coord.lng, timestamp);
        // 更新统计
        this._updateStatsUI();
    }

    // ── 地图更新 ──────────────────────────────────────────

    _updateMap(lat, lng, timestamp) {
        if (!this.map) return;

        const latlng = [lat, lng];
        this.marker.setLatLng(latlng);

        const timeStr = new Date(timestamp).toLocaleTimeString();
        this.marker.setPopupContent(
            `<b>🐾 当前位置</b><br>` +
            `经度: ${lng.toFixed(7)}<br>` +
            `纬度: ${lat.toFixed(7)}<br>` +
            `时间: ${timeStr}`
        );

        this.polyline.addLatLng(latlng);

        // 平滑跟随：如果当前可见范围不包含新点，才平移
        if (!this.map.getBounds().contains(latlng)) {
            this.map.panTo(latlng);
        }

        // 第一个定位点：缩放到合适级别
        if (this.trackPoints.length === 1) {
            this.map.setView(latlng, 16);
        }
    }

    // ── 统计 UI ──────────────────────────────────────────

    _updateStatsUI() {
        // 距离
        let distStr;
        if (this.totalDistance >= 1000) {
            distStr = (this.totalDistance / 1000).toFixed(2) + ' km';
        } else {
            distStr = Math.round(this.totalDistance) + ' m';
        }
        this._setEl('gpsDistance', distStr);

        // 速度 (km/h)
        const kmh = this.currentSpeed * 3.6;
        this._setEl('gpsSpeed', kmh.toFixed(1) + ' km/h');

        // 轨迹点数
        this._setEl('gpsPointCount', String(this.trackPoints.length));

        // 当前坐标
        if (this.lastValidPoint) {
            this._setEl('gpsCurrentLon', this.lastValidPoint.lng.toFixed(7));
            this._setEl('gpsCurrentLat', this.lastValidPoint.lat.toFixed(7));
            this._setEl('gpsLastUpdate', new Date(this.lastValidPoint.timestamp).toLocaleTimeString());
        }
    }

    _updateStatusUI(running) {
        this._setEl('gpsStatus', running ? '追踪中...' : '已停止');
    }

    // ── Haversine 公式 ───────────────────────────────────

    _haversine(lat1, lng1, lat2, lng2) {
        const R = 6371000; // 地球半径 (米)
        const toRad = v => v * Math.PI / 180;
        const dLat = toRad(lat2 - lat1);
        const dLng = toRad(lng2 - lng1);
        const a = Math.sin(dLat / 2) ** 2 +
                  Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) *
                  Math.sin(dLng / 2) ** 2;
        return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    // ── DOM 工具 ─────────────────────────────────────────

    _setEl(id, text) {
        const el = document.getElementById(id);
        if (el) el.textContent = text;
    }

    _setColor(id, color) {
        const el = document.getElementById(id);
        if (el) el.style.color = color;
    }
}
