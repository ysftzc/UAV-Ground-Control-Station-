// UAV-GCS-01 web dashboard frontend. No build step, no CDN - everything in
// static/vendor/ is fetched from the local server. See server.py's module
// docstring and uav_gcs_node/CLAUDE.md for which fields below are real vs
// synthetic/derived (mirrored here only in comments, never silently changed).

const HOME = { lat: 39.7767, lon: 30.5206 }; // must match server.py HOME_LAT_DEG/HOME_LON_DEG
const WS_URL = `ws://${location.host}/ws/telemetry`;

// ---------- geo helpers (local tangent-plane approximation, fine at this scale) ----------
function offsetLatLon(lat, lon, dNorthM, dEastM) {
  const R = 6378137;
  const dLat = dNorthM / R;
  const dLon = dEastM / (R * Math.cos((Math.PI * lat) / 180));
  return [lon + (dLon * 180) / Math.PI, lat + (dLat * 180) / Math.PI];
}
function polarOffset(lat, lon, bearingDeg, distM) {
  const rad = (bearingDeg * Math.PI) / 180;
  return offsetLatLon(lat, lon, distM * Math.cos(rad), distM * Math.sin(rad));
}
function circlePolygon(lat, lon, radiusM, n = 48) {
  const ring = [];
  for (let i = 0; i <= n; i++) ring.push(polarOffset(lat, lon, (i / n) * 360, radiusM));
  return ring;
}
function conePolygon(lat, lon, headingDeg, halfAngleDeg, rangeM, n = 20) {
  const ring = [[lon, lat]];
  for (let i = 0; i <= n; i++) {
    const b = headingDeg - halfAngleDeg + (i / n) * (2 * halfAngleDeg);
    ring.push(polarOffset(lat, lon, b, rangeM));
  }
  ring.push([lon, lat]);
  return ring;
}
function gridLinesGeoJSON(lat, lon, spacingM, halfExtentM) {
  const features = [];
  for (let e = -halfExtentM; e <= halfExtentM + 1; e += spacingM) {
    const a = offsetLatLon(lat, lon, -halfExtentM, e);
    const b = offsetLatLon(lat, lon, halfExtentM, e);
    features.push({ type: "Feature", geometry: { type: "LineString", coordinates: [a, b] } });
  }
  for (let n = -halfExtentM; n <= halfExtentM + 1; n += spacingM) {
    const a = offsetLatLon(lat, lon, n, -halfExtentM);
    const b = offsetLatLon(lat, lon, n, halfExtentM);
    features.push({ type: "Feature", geometry: { type: "LineString", coordinates: [a, b] } });
  }
  return { type: "FeatureCollection", features };
}

// ---------- demo (clearly non-real) mission layers ----------
const DEMO_WAYPOINTS = [
  polarOffset(HOME.lat, HOME.lon, 30, 400),
  polarOffset(HOME.lat, HOME.lon, 75, 900),
  polarOffset(HOME.lat, HOME.lon, 140, 700),
  polarOffset(HOME.lat, HOME.lon, 200, 1100),
];
const DEMO_NFZ_CENTER = polarOffset(HOME.lat, HOME.lon, 300, 1300);
const DEMO_NFZ_RADIUS_M = 350;

// ---------- HUD (artificial horizon) ----------
const hudCanvas = document.getElementById("hud-canvas");
const hudCtx = hudCanvas.getContext("2d");
function drawHUD(rollDeg, pitchDeg) {
  const w = hudCanvas.width, h = hudCanvas.height, cx = w / 2, cy = h / 2, r = Math.min(w, h) / 2 - 4;
  hudCtx.clearRect(0, 0, w, h);
  hudCtx.save();
  hudCtx.beginPath();
  hudCtx.arc(cx, cy, r, 0, Math.PI * 2);
  hudCtx.clip();

  hudCtx.translate(cx, cy);
  hudCtx.rotate((-rollDeg * Math.PI) / 180);
  const pitchOffset = Math.max(-r, Math.min(r, (pitchDeg / 90) * r * 1.4));
  hudCtx.translate(0, pitchOffset);

  hudCtx.fillStyle = "#3a5f8a";
  hudCtx.fillRect(-2 * r, -2 * r, 4 * r, 2 * r);
  hudCtx.fillStyle = "#4a3620";
  hudCtx.fillRect(-2 * r, 0, 4 * r, 2 * r);
  hudCtx.strokeStyle = "#00e676";
  hudCtx.lineWidth = 2;
  hudCtx.beginPath();
  hudCtx.moveTo(-2 * r, 0);
  hudCtx.lineTo(2 * r, 0);
  hudCtx.stroke();

  hudCtx.restore();

  // fixed aircraft reference (does not rotate)
  hudCtx.strokeStyle = "#00e676";
  hudCtx.lineWidth = 3;
  hudCtx.beginPath();
  hudCtx.moveTo(cx - 30, cy);
  hudCtx.lineTo(cx - 10, cy);
  hudCtx.moveTo(cx + 10, cy);
  hudCtx.lineTo(cx + 30, cy);
  hudCtx.stroke();
  hudCtx.beginPath();
  hudCtx.arc(cx, cy, 3, 0, Math.PI * 2);
  hudCtx.fillStyle = "#00e676";
  hudCtx.fill();

  hudCtx.strokeStyle = "#1c2733";
  hudCtx.lineWidth = 2;
  hudCtx.beginPath();
  hudCtx.arc(cx, cy, r, 0, Math.PI * 2);
  hudCtx.stroke();
}

// ---------- sparkline ----------
const sparkCanvas = document.getElementById("spark-canvas");
const sparkCtx = sparkCanvas.getContext("2d");
const SPARK_LEN = 180;
const sparkBuf = { ax: [], ay: [], az: [] };
function pushSpark(accel) {
  sparkBuf.ax.push(accel[0]);
  sparkBuf.ay.push(accel[1]);
  sparkBuf.az.push(accel[2]);
  for (const k of Object.keys(sparkBuf)) if (sparkBuf[k].length > SPARK_LEN) sparkBuf[k].shift();
}
function drawSpark() {
  const w = sparkCanvas.width, h = sparkCanvas.height;
  sparkCtx.clearRect(0, 0, w, h);
  sparkCtx.strokeStyle = "#1c2733";
  sparkCtx.beginPath();
  sparkCtx.moveTo(0, h / 2);
  sparkCtx.lineTo(w, h / 2);
  sparkCtx.stroke();

  const series = [
    { data: sparkBuf.ax, color: "#00e676" },
    { data: sparkBuf.ay, color: "#6a89ff" },
    { data: sparkBuf.az, color: "#ffab40" },
  ];
  const scale = 4; // m/s^2 per half-height unit
  for (const s of series) {
    if (s.data.length < 2) continue;
    sparkCtx.strokeStyle = s.color;
    sparkCtx.lineWidth = 1.5;
    sparkCtx.beginPath();
    s.data.forEach((v, i) => {
      const x = (i / (SPARK_LEN - 1)) * w;
      const y = h / 2 - (v / (9.8 * scale)) * (h / 2);
      if (i === 0) sparkCtx.moveTo(x, y);
      else sparkCtx.lineTo(x, y);
    });
    sparkCtx.stroke();
  }
}

// ---------- CAN table ----------
function renderCan(can) {
  const tbody = document.getElementById("can-tbody");
  tbody.innerHTML = "";
  for (const f of can.frames) {
    const tr = document.createElement("tr");
    const pendingCls = f.pending ? ' class="pending"' : "";
    tr.innerHTML = `<td class="id">${f.id}</td><td>${f.label}</td><td${pendingCls}>${f.bytes ?? "— beklemede —"}</td>`;
    tbody.appendChild(tr);
  }
}

// ---------- alarm log ----------
let lastAnomaly = false;
const alarmLog = document.getElementById("alarm-log");
function pushAlarm(text, cls = "") {
  const li = document.createElement("li");
  if (cls) li.className = cls;
  const t = new Date().toLocaleTimeString("tr-TR");
  li.textContent = `[${t}] ${text}`;
  alarmLog.prepend(li);
  while (alarmLog.children.length > 40) alarmLog.removeChild(alarmLog.lastChild);
}

// ---------- map ----------
let map, droneMarkerEl;
function initMap() {
  map = new maplibregl.Map({
    container: "map",
    style: {
      version: 8,
      sources: {
        "esri-sat": {
          type: "raster",
          tiles: [
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
          ],
          tileSize: 256,
          attribution: "Esri, Maxar, Earthstar Geographics",
        },
      },
      layers: [
        {
          id: "esri-sat-layer",
          type: "raster",
          source: "esri-sat",
          paint: {
            "raster-saturation": -0.5,
            "raster-brightness-max": 0.6,
            "raster-contrast": 0.15,
          },
        },
      ],
    },
    center: [HOME.lon, HOME.lat],
    zoom: 15,
    pitch: 0,
    attributionControl: { compact: true },
  });
  map.addControl(new maplibregl.NavigationControl(), "top-right");

  map.on("load", () => {
    map.addSource("grid", { type: "geojson", data: gridLinesGeoJSON(HOME.lat, HOME.lon, 500, 2500) });
    map.addLayer({
      id: "grid-lines",
      type: "line",
      source: "grid",
      paint: { "line-color": "#00e676", "line-width": 0.6, "line-opacity": 0.35 },
    });

    map.addSource("fov", { type: "geojson", data: emptyPoly() });
    map.addLayer({
      id: "fov-fill",
      type: "fill",
      source: "fov",
      paint: { "fill-color": "#00e676", "fill-opacity": 0.12 },
    });
    map.addLayer({
      id: "fov-line",
      type: "line",
      source: "fov",
      paint: { "line-color": "#00e676", "line-width": 1, "line-opacity": 0.6 },
    });

    map.addSource("waypoints", {
      type: "geojson",
      data: {
        type: "Feature",
        geometry: { type: "LineString", coordinates: [[HOME.lon, HOME.lat], ...DEMO_WAYPOINTS] },
      },
    });
    map.addLayer({
      id: "waypoint-line",
      type: "line",
      source: "waypoints",
      paint: { "line-color": "#ffab40", "line-width": 2, "line-dasharray": [2, 2] },
    });
    map.addSource("waypoint-pts", {
      type: "geojson",
      data: {
        type: "FeatureCollection",
        features: DEMO_WAYPOINTS.map((c) => ({ type: "Feature", geometry: { type: "Point", coordinates: c } })),
      },
    });
    map.addLayer({
      id: "waypoint-pts-layer",
      type: "circle",
      source: "waypoint-pts",
      paint: { "circle-radius": 5, "circle-color": "#ffab40", "circle-stroke-color": "#0a0c0f", "circle-stroke-width": 1.5 },
    });

    map.addSource("nfz", {
      type: "geojson",
      data: { type: "Feature", geometry: { type: "Polygon", coordinates: [circlePolygon(DEMO_NFZ_CENTER[1], DEMO_NFZ_CENTER[0], DEMO_NFZ_RADIUS_M)] } },
    });
    map.addLayer({
      id: "nfz-fill",
      type: "fill",
      source: "nfz",
      paint: { "fill-color": "#ff5252", "fill-opacity": 0.1 },
    });
    map.addLayer({
      id: "nfz-line",
      type: "line",
      source: "nfz",
      paint: { "line-color": "#ff5252", "line-width": 1.5, "line-dasharray": [1, 1.5] },
    });

    droneMarkerEl = document.createElement("div");
    droneMarkerEl.className = "drone-marker";
    droneMarkerEl.innerHTML = `<svg width="34" height="34" viewBox="0 0 34 34">
      <polygon points="17,2 27,29 17,22 7,29" fill="#00e676" stroke="#0a0c0f" stroke-width="1.5"/>
    </svg>`;
    window.droneMarker = new maplibregl.Marker({ element: droneMarkerEl, rotationAlignment: "map" })
      .setLngLat([HOME.lon, HOME.lat])
      .addTo(map);
  });

  const mgrsReadout = document.getElementById("mgrs-readout");
  const homeMgrs = mgrs.forward([HOME.lon, HOME.lat], 5);
  mgrsReadout.textContent = `HOME MGRS ${homeMgrs}`;
  map.on("mousemove", (e) => {
    const m = mgrs.forward([e.lngLat.lng, e.lngLat.lat], 5);
    mgrsReadout.textContent = `İMLEÇ MGRS ${m}`;
  });
  map.on("mouseout", () => {
    mgrsReadout.textContent = `HOME MGRS ${homeMgrs}`;
  });
}
function emptyPoly() {
  return { type: "Feature", geometry: { type: "Polygon", coordinates: [[]] } };
}
function updateMapForHeading(headingDeg) {
  if (!map || !window.droneMarker) return;
  window.droneMarker.setRotation(headingDeg);
  const cone = conePolygon(HOME.lat, HOME.lon, headingDeg, 20, 180);
  const src = map.getSource("fov");
  if (src) src.setData({ type: "Feature", geometry: { type: "Polygon", coordinates: [cone] } });
}

// ---------- websocket ----------
function fmt(v, digits = 2) {
  return Number(v).toFixed(digits);
}
function connect() {
  const ws = new WebSocket(WS_URL);
  const linkDot = document.getElementById("link-dot");
  const linkVal = document.getElementById("link-val");

  ws.onopen = () => {
    linkVal.textContent = "BEKLENİYOR";
  };
  ws.onclose = () => {
    linkDot.className = "link-dot lost";
    linkVal.textContent = "BAĞLANTI KOPTU";
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();

  ws.onmessage = (ev) => {
    const s = JSON.parse(ev.data);

    linkDot.className = "link-dot " + (s.link === "OK" ? "ok" : "lost");
    linkVal.textContent = s.link === "OK" ? "LINK OK" : "LINK LOST";
    document.getElementById("mode-val").textContent = s.mode;
    const up = Math.floor(s.uptime_s);
    const hh = String(Math.floor(up / 3600)).padStart(2, "0");
    const mm = String(Math.floor((up % 3600) / 60)).padStart(2, "0");
    const ss = String(up % 60).padStart(2, "0");
    document.getElementById("uptime-val").textContent = `${hh}:${mm}:${ss}`;

    document.getElementById("roll-val").textContent = fmt(s.attitude.roll) + "°";
    document.getElementById("pitch-val").textContent = fmt(s.attitude.pitch) + "°";
    document.getElementById("yaw-val").textContent = fmt(s.attitude.yaw) + "°";
    drawHUD(s.attitude.roll, s.attitude.pitch);

    document.getElementById("kf-roll-val").textContent = fmt(s.kalman.roll) + "°";
    document.getElementById("kf-pitch-val").textContent = fmt(s.kalman.pitch) + "°";
    document.getElementById("d-roll-val").textContent = fmt(s.kalman.d_roll) + "°";
    document.getElementById("d-pitch-val").textContent = fmt(s.kalman.d_pitch) + "°";
    const banner = document.getElementById("anomaly-banner");
    banner.classList.toggle("hidden", !s.kalman.anomaly);
    if (s.kalman.anomaly && !lastAnomaly) {
      pushAlarm(`ANOMALİ: Δroll=${fmt(s.kalman.d_roll)}° Δpitch=${fmt(s.kalman.d_pitch)}° (eşik ${s.kalman.threshold}°)`);
    } else if (!s.kalman.anomaly && lastAnomaly) {
      pushAlarm("Anomali temizlendi, KF/EKF2 tekrar uyumlu", "info");
    }
    lastAnomaly = s.kalman.anomaly;

    pushSpark(s.sensor.accel);
    drawSpark();

    document.getElementById("batt-volt-val").textContent = fmt(s.battery.voltage, 1) + " V";
    document.getElementById("batt-pct-val").textContent = s.battery.percent + " %";

    renderCan(s.can);

    updateMapForHeading(s.position.heading);
  };
}

initMap();
connect();
