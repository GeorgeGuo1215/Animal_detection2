/**
 * GPS 定位追踪模块 v2.0
 * 基于 BLE V1.02 协议的 Lon/Lat 字段，使用 Leaflet + 高德地图 实时显示位置和轨迹
 *
 * 协议格式: $MAC,Time,Lon,Lat,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,V1,V2,V3*
 * Lon 格式: E114.2010058 / W114.2010058 (E=东经正, W=西经负)
 * Lat 格式: N22.3787343  / S22.3787343  (N=北纬正, S=南纬负)
 *
 * 新功能（v2.0）：
 *   - 高德普通图 + 卫星图双底图（官方 API Key，右上角切换）
 *   - 热力色彩轨迹线（速度→颜色：蓝→青→绿→黄→橙→红）
 *   - 轨迹时间点标记（每30秒或50米放置可点击圆圈）
 *   - 起点🚩 / 终点🐾 区分标记
 *   - 停留点检测（原地静止>60s自动标注）
 *   - 统计侧边栏：最高速度、平均速度、运动时长、停留点数
 *
 * 注意：高德使用 GCJ-02 坐标系，GPS原始为 WGS-84，国内有约100-500m视觉偏移
 *       不影响轨迹相对精度、速度/距离/停留点计算
 */

class GPSLocationMonitor {
    constructor() {
        this.enabled = false;
        this.initialized = false;

        // Leaflet 实例
        this.map = null;
        this.marker = null;          // 当前位置🐾（终点）
        this.polyline = null;        // 保留引用（兼容外部访问），初始化后改为空壳
        this.accuracyCircle = null;

        // 轨迹数据（每个点含 speed 字段）
        this.trackPoints = [];       // [{lat, lng, timestamp, speed}]
        this.maxPoints = 5000;

        // 统计（基础）
        this.totalDistance = 0;      // 米
        this.currentSpeed = 0;       // m/s
        this.lastValidPoint = null;
        this.noFixCount = 0;

        // 节流：地图最多 1Hz 刷新
        this.lastUpdateTime = 0;
        this.updateThrottleMs = 1000;

        // ── 新增：热力轨迹 ──
        this.heatSegments = [];      // L.polyline 分段数组，每段一种颜色

        // ── 新增：时间标记层组 ──
        this.timeMarkerLayer = null; // L.layerGroup
        this.timeMarkerInterval = 30;  // 每隔30秒放一个标记
        this.distMarkerInterval = 50;  // 或每50米放一个（取先到者）
        this.lastMarkerTime = 0;
        this.lastMarkerDist = 0;

        // ── 新增：起点标记 ──
        this.startMarker = null;     // 绿色🚩（终点继续用🐾 this.marker）

        // ── 新增：停留点检测 ──
        this.stayPoints = [];        // [{lat,lng,startTime,endTime,duration}]
        this.STAY_RADIUS_M = 15;     // 停留判定半径（米）
        this.STAY_MIN_SEC  = 60;     // 最短停留时长（秒）

        // ── 新增：扩展统计 ──
        this.maxSpeed = 0;           // m/s，记录历史最高速度
        this.trackStartTime = null;  // 第一个有效点时间戳
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

    // ── 速度→颜色映射 ────────────────────────────────────

    /**
     * 将速度（m/s）映射为 RGB 颜色字符串（线性插值）
     * 宠物场景分档：静止=深蓝 → 慢走=青 → 步行=绿 → 小跑=黄 → 快跑=橙 → 全速=红
     */
    _speedToColor(speedMs) {
        const stops = [
            { v: 0,    r: 30,  g: 100, b: 230 },  // 深蓝 — 静止
            { v: 0.5,  r: 0,   g: 200, b: 200 },  // 青色 — 慢踱步 (~1.8 km/h)
            { v: 1.5,  r: 50,  g: 200, b: 50  },  // 绿色 — 正常步行 (~5.4 km/h)
            { v: 3.0,  r: 230, g: 230, b: 0   },  // 黄色 — 小跑 (~10.8 km/h)
            { v: 6.0,  r: 255, g: 140, b: 0   },  // 橙色 — 快跑 (~21.6 km/h)
            { v: 10.0, r: 220, g: 20,  b: 20  },  // 红色 — 全速冲刺 (36+ km/h)
        ];
        for (let i = 1; i < stops.length; i++) {
            if (speedMs <= stops[i].v) {
                const t = (speedMs - stops[i-1].v) / (stops[i].v - stops[i-1].v);
                const lerp = (a, b) => Math.round(a + (b - a) * t);
                return `rgb(${lerp(stops[i-1].r, stops[i].r)},${lerp(stops[i-1].g, stops[i].g)},${lerp(stops[i-1].b, stops[i].b)})`;
            }
        }
        return 'rgb(220,20,20)';  // 超出最大值 → 纯红
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
        this.map = L.map('gpsMapContainer', { zoomControl: true }).setView([30, 110], 4);

        // ── 高德地图 API Key（Web端 JS Key）──
        // 防盗用：请在高德开放平台 lbs.amap.com → 应用管理 → 设置 → 白名单 中绑定您的域名
        const AMAP_KEY = 'ef0294b5c25fa7a3219f067d5f66d5b4';

        // ── 底图：高德普通图（官方瓦片，带 Key） ──
        const gaodeNormal = L.tileLayer(
            `https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}&key=${AMAP_KEY}`,
            {
                subdomains: '1234',
                maxZoom: 20,
                attribution: '&copy; <a href="https://www.amap.com/" target="_blank">高德地图</a>'
            }
        );

        // ── 底图：高德卫星图（官方瓦片，带 Key） ──
        const gaodeSatellite = L.tileLayer(
            `https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}&key=${AMAP_KEY}`,
            {
                subdomains: '1234',
                maxZoom: 20,
                attribution: '&copy; <a href="https://www.amap.com/" target="_blank">高德卫星图</a>'
            }
        );

        // 默认显示普通地图
        gaodeNormal.addTo(this.map);

        // 图层切换控件（右上角）
        L.control.layers(
            {
                '🗺️ 普通地图': gaodeNormal,
                '🛰️ 卫星图':   gaodeSatellite
            },
            {},
            { position: 'topright', collapsed: false }
        ).addTo(this.map);

        // ── 🐾 当前位置标记（终点） ──
        const petIcon = L.divIcon({
            className: 'gps-pet-marker',
            html: '<div style="font-size:28px;text-shadow:0 2px 6px rgba(0,0,0,0.3);">🐾</div>',
            iconSize: [32, 32],
            iconAnchor: [16, 16]
        });
        this.marker = L.marker([30, 110], { icon: petIcon, zIndexOffset: 1000 }).addTo(this.map);
        this.marker.bindPopup('等待 GPS 定位...');

        // ── 时间标记 LayerGroup ──
        this.timeMarkerLayer = L.layerGroup().addTo(this.map);

        // ── 兼容：polyline 设为空壳，防止外部 .addTo()/.setLatLngs() 报错 ──
        this.polyline = { setLatLngs: () => {}, addLatLng: () => {} };

        this.initialized = true;
        console.log('[GPS] 地图初始化完成（高德官方底图 + API Key + 热力轨迹）');
    }

    start() {
        this.enabled = true;
        this._updateStatusUI(true);

        if (!this.initialized) {
            // 确保在浏览器完成 display:block 重排后再初始化地图
            // requestAnimationFrame 在下一帧绘制前执行，此时容器已有真实宽高
            requestAnimationFrame(() => {
                requestAnimationFrame(() => {          // 双 rAF：第一帧触发重排，第二帧读取布局
                    this.init();
                    if (this.map) {
                        this.map.invalidateSize();
                        setTimeout(() => this.map.invalidateSize(), 300); // 兜底
                    }
                });
            });
        } else if (this.map) {
            // 已初始化过，只刷新尺寸（比如从隐藏重新显示）
            requestAnimationFrame(() => {
                this.map.invalidateSize();
                setTimeout(() => this.map.invalidateSize(), 300);
            });
        }
    }

    stop() {
        this.enabled = false;
        this._updateStatusUI(false);
    }

    reset() {
        this.trackPoints      = [];
        this.totalDistance    = 0;
        this.currentSpeed     = 0;
        this.lastValidPoint   = null;
        this.noFixCount       = 0;
        this.maxSpeed         = 0;
        this.trackStartTime   = null;
        this.stayPoints       = [];
        this.lastMarkerTime   = 0;
        this.lastMarkerDist   = 0;

        // 清除热力轨迹段
        this.heatSegments.forEach(seg => seg.remove());
        this.heatSegments = [];

        // 清除时间标记
        if (this.timeMarkerLayer) this.timeMarkerLayer.clearLayers();

        // 清除起点标记
        if (this.startMarker) {
            this.startMarker.remove();
            this.startMarker = null;
        }

        // 重置🐾标记
        if (this.marker) {
            this.marker.setLatLng([30, 110]);
            this.marker.setPopupContent('等待 GPS 定位...');
        }
        if (this.map) this.map.setView([30, 110], 4);

        this._updateStatsUI();
    }

    // ── 数据入口（对外接口不变） ──────────────────────────

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
        let speed = 0;
        if (this.lastValidPoint) {
            const dt = (timestamp - this.lastValidPoint.timestamp) / 1000;
            if (dt > 0) {
                const dist = this._haversine(
                    this.lastValidPoint.lat, this.lastValidPoint.lng,
                    coord.lat, coord.lng
                );
                speed = dist / dt;
                if (speed > 500) return; // 丢弃异常跳变

                this.totalDistance += dist;
                this.currentSpeed = speed;
                if (speed > this.maxSpeed) this.maxSpeed = speed;
            }
        }

        // 记录运动起始时间
        if (!this.trackStartTime) this.trackStartTime = timestamp;

        const point = { lat: coord.lat, lng: coord.lng, timestamp, speed };
        this.trackPoints.push(point);
        if (this.trackPoints.length > this.maxPoints) {
            this.trackPoints.shift();
        }
        this.lastValidPoint = point;

        // 更新地图
        this._updateMap(coord.lat, coord.lng, timestamp, speed);
        // 更新统计
        this._updateStatsUI();
    }

    // ── 地图更新 ──────────────────────────────────────────

    _updateMap(lat, lng, timestamp, speed) {
        if (!this.map) return;

        const latlng = [lat, lng];

        // 1. 更新🐾当前位置标记
        this.marker.setLatLng(latlng);
        const timeStr = new Date(timestamp).toLocaleTimeString();
        this.marker.setPopupContent(
            `<b>🐾 当前位置</b><br>` +
            `经度: ${lng.toFixed(7)}<br>` +
            `纬度: ${lat.toFixed(7)}<br>` +
            `速度: ${(speed * 3.6).toFixed(1)} km/h<br>` +
            `时间: ${timeStr}`
        );

        // 2. 第一个定位点：放起点标记 + 缩放到合适级别
        if (this.trackPoints.length === 1) {
            this._placeStartMarker(lat, lng, timestamp);
            this.map.setView(latlng, 16);
        }

        // 3. 追加热力轨迹段（需要至少2个点）
        if (this.trackPoints.length >= 2) {
            this._appendHeatSegment();
        }

        // 4. 时间点标记
        this._tryAddTimeMarker(lat, lng, timestamp, speed);

        // 5. 平滑跟随：若新点不在可见范围内才平移
        if (!this.map.getBounds().contains(latlng)) {
            this.map.panTo(latlng);
        }
    }

    // ── 起点标记 ─────────────────────────────────────────

    _placeStartMarker(lat, lng, timestamp) {
        if (this.startMarker) {
            this.startMarker.remove();
        }
        const startIcon = L.divIcon({
            className: '',
            html: `<div style="font-size:26px;filter:drop-shadow(0 2px 4px rgba(0,0,0,0.4));transform:translateX(-50%);">🚩</div>`,
            iconSize: [28, 28],
            iconAnchor: [14, 28]
        });
        this.startMarker = L.marker([lat, lng], { icon: startIcon, zIndexOffset: 500 })
            .addTo(this.map)
            .bindPopup(
                `<b>🚩 起点</b><br>` +
                `经度: ${lng.toFixed(7)}<br>` +
                `纬度: ${lat.toFixed(7)}<br>` +
                `时间: ${new Date(timestamp).toLocaleTimeString()}`
            );
    }

    // ── 热力分段轨迹 ──────────────────────────────────────

    /**
     * 追加一段热力轨迹（前一点→当前点），颜色由平均速度决定
     */
    _appendHeatSegment() {
        const pts = this.trackPoints;
        const prev = pts[pts.length - 2];
        const curr = pts[pts.length - 1];

        const avgSpeed = (prev.speed + curr.speed) / 2;
        const color = this._speedToColor(avgSpeed);

        const seg = L.polyline(
            [[prev.lat, prev.lng], [curr.lat, curr.lng]],
            { color, weight: 5, opacity: 0.85, lineJoin: 'round', lineCap: 'round' }
        ).addTo(this.map);

        this.heatSegments.push(seg);

        // 内存管理：超出 maxPoints 时移除最旧段
        if (this.heatSegments.length > this.maxPoints) {
            const old = this.heatSegments.shift();
            old.remove();
        }
    }

    // ── 轨迹时间点标记 ───────────────────────────────────

    /**
     * 按时间或距离间隔，在轨迹上放置可点击的彩色圆圈标记
     */
    _tryAddTimeMarker(lat, lng, timestamp, speed) {
        const timeDiff = (timestamp - this.lastMarkerTime) / 1000;
        const distDiff = this.totalDistance - this.lastMarkerDist;

        // 触发条件：首次 或 时间≥30秒 或 里程≥50米
        const isFirst   = (this.lastMarkerTime === 0);
        const byTime    = timeDiff >= this.timeMarkerInterval;
        const byDist    = distDiff >= this.distMarkerInterval;

        if (!isFirst && !byTime && !byDist) return;

        this.lastMarkerTime = timestamp;
        this.lastMarkerDist = this.totalDistance;

        const timeStr  = new Date(timestamp).toLocaleTimeString();
        const speedKmh = (speed * 3.6).toFixed(1);
        const distStr  = this.totalDistance >= 1000
            ? (this.totalDistance / 1000).toFixed(2) + ' km'
            : Math.round(this.totalDistance) + ' m';
        const color = this._speedToColor(speed);

        const circleIcon = L.divIcon({
            className: '',
            html: `<div style="width:10px;height:10px;background:${color};border:2px solid #fff;border-radius:50%;box-shadow:0 1px 4px rgba(0,0,0,0.4);"></div>`,
            iconSize: [10, 10],
            iconAnchor: [5, 5]
        });

        const m = L.marker([lat, lng], { icon: circleIcon, zIndexOffset: 200 })
            .bindPopup(
                `<div style="min-width:140px;font-size:13px;line-height:1.7">` +
                `<b>⏱ 轨迹记录点</b><br>` +
                `🕐 时间: ${timeStr}<br>` +
                `📍 经度: ${lng.toFixed(6)}<br>` +
                `📍 纬度: ${lat.toFixed(6)}<br>` +
                `💨 速度: ${speedKmh} km/h<br>` +
                `📏 累计: ${distStr}` +
                `</div>`,
                { maxWidth: 200 }
            );

        this.timeMarkerLayer.addLayer(m);
    }

    // ── 停留点检测 ────────────────────────────────────────

    /**
     * 滑动窗口算法：取最近60个点，若所有点在15m质心范围内且时间≥60s，判定为停留点
     */
    _detectStayPoints() {
        const pts = this.trackPoints;
        if (pts.length < 3) return;

        const windowSize = Math.min(60, pts.length);
        const window = pts.slice(-windowSize);

        // 计算质心
        const centLat = window.reduce((s, p) => s + p.lat, 0) / window.length;
        const centLng = window.reduce((s, p) => s + p.lng, 0) / window.length;

        // 所有点到质心的最大距离
        const maxDist = window.reduce((mx, p) =>
            Math.max(mx, this._haversine(p.lat, p.lng, centLat, centLng)), 0);

        // 时间跨度
        const duration = (window[window.length - 1].timestamp - window[0].timestamp) / 1000;

        if (maxDist <= this.STAY_RADIUS_M && duration >= this.STAY_MIN_SEC) {
            // 检查是否与上一停留点重叠（避免重复记录）
            const lastStay = this.stayPoints[this.stayPoints.length - 1];
            if (lastStay) {
                const distFromLast = this._haversine(lastStay.lat, lastStay.lng, centLat, centLng);
                if (distFromLast < this.STAY_RADIUS_M * 2) return;
            }

            const stayPoint = {
                lat: centLat, lng: centLng,
                startTime: window[0].timestamp,
                endTime:   window[window.length - 1].timestamp,
                duration:  Math.round(duration)
            };
            this.stayPoints.push(stayPoint);
            this._placeStayMarker(stayPoint);
        }
    }

    /**
     * 在地图上放置带编号的紫色停留点标记
     */
    _placeStayMarker(stay) {
        const min = Math.floor(stay.duration / 60);
        const sec = stay.duration % 60;
        const durationStr = min > 0 ? `${min}分${sec}秒` : `${sec}秒`;
        const idx = this.stayPoints.length;

        const stayIcon = L.divIcon({
            className: '',
            html: `<div style="background:rgba(155,89,182,0.85);color:#fff;border-radius:50%;width:22px;height:22px;display:flex;align-items:center;justify-content:center;font-size:12px;font-weight:bold;border:2px solid #fff;box-shadow:0 2px 5px rgba(0,0,0,0.3);">${idx}</div>`,
            iconSize: [22, 22],
            iconAnchor: [11, 11]
        });

        L.marker([stay.lat, stay.lng], { icon: stayIcon, zIndexOffset: 300 })
            .addTo(this.map)
            .bindPopup(
                `<b>⏸ 停留点 #${idx}</b><br>` +
                `🕐 开始: ${new Date(stay.startTime).toLocaleTimeString()}<br>` +
                `🕑 结束: ${new Date(stay.endTime).toLocaleTimeString()}<br>` +
                `⏱ 时长: ${durationStr}<br>` +
                `📍 位置: ${stay.lat.toFixed(6)}, ${stay.lng.toFixed(6)}`
            );
    }

    // ── 统计 UI ──────────────────────────────────────────

    _updateStatsUI() {
        // ── 基础4项（保持不变） ──
        let distStr;
        if (this.totalDistance >= 1000) {
            distStr = (this.totalDistance / 1000).toFixed(2) + ' km';
        } else {
            distStr = Math.round(this.totalDistance) + ' m';
        }
        this._setEl('gpsDistance', distStr);
        this._setEl('gpsSpeed', (this.currentSpeed * 3.6).toFixed(1) + ' km/h');
        this._setEl('gpsPointCount', String(this.trackPoints.length));
        if (this.lastValidPoint) {
            this._setEl('gpsCurrentLon', this.lastValidPoint.lng.toFixed(7));
            this._setEl('gpsCurrentLat', this.lastValidPoint.lat.toFixed(7));
            this._setEl('gpsLastUpdate', new Date(this.lastValidPoint.timestamp).toLocaleTimeString());
        }

        // ── 扩展4项（新增侧边栏） ──

        // 最高速度
        this._setEl('gpsMaxSpeed', (this.maxSpeed * 3.6).toFixed(1) + ' km/h');

        // 平均速度（总距离 / 总时间）
        let avgSpeedKmh = 0;
        if (this.trackStartTime && this.lastValidPoint && this.trackPoints.length > 1) {
            const elapsedSec = (this.lastValidPoint.timestamp - this.trackStartTime) / 1000;
            if (elapsedSec > 0) {
                avgSpeedKmh = (this.totalDistance / elapsedSec) * 3.6;
            }
        }
        this._setEl('gpsAvgSpeed', avgSpeedKmh.toFixed(1) + ' km/h');

        // 运动时长
        if (this.trackStartTime && this.lastValidPoint) {
            const elapsedSec = Math.floor((this.lastValidPoint.timestamp - this.trackStartTime) / 1000);
            const h = Math.floor(elapsedSec / 3600);
            const m = Math.floor((elapsedSec % 3600) / 60);
            const s = elapsedSec % 60;
            this._setEl('gpsDuration',
                h > 0 ? `${h}h ${m}m` : m > 0 ? `${m}m ${s}s` : `${s}s`);
        } else {
            this._setEl('gpsDuration', '0s');
        }

        // 停留点检测（每10个新点触发一次，降低计算频率）
        if (this.trackPoints.length > 0 && this.trackPoints.length % 10 === 0) {
            this._detectStayPoints();
        }
        this._setEl('gpsStayCount', String(this.stayPoints.length));
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
