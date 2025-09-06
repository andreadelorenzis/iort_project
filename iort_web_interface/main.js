
// ==========================
// CONFIGURATION PARAMETERS
// ==========================

const resolution = 0.05; // m/pixel
const origin = [-9.38, -5.6, 0]; // [x, y, theta]
const closeThreshold = 0.2; // meters

const MQTT_BROKER = "ws://localhost:9001";
const MQTT_SUB_TOPIC = "mqtt/out";
const MQTT_PUB_TOPIC = "mqtt/in";

const streamUrl = 'http://192.168.1.18:81/stream'; 
const nodeRedUrl = 'http://localhost:1880/';
const webSocketUrl = 'ws://192.168.1.9:8080';


// ==========================
// GLOBAL VARIABLES
// ==========================

let map;
let ws;
let mqttClient;


let rosPoints = [];  
let leafletPoints = [];
let polyline;
let markers = [];
let virtualZonesData = [];
let selectedZone;
let selectedColor = '#2ecc71';


let rulesData = [];
let sensorsData = [];
let robotsData = [];
let zonesData = [];
let operationsData = [];
let conditionsData = [];
let actionsData = [];

let editingRule = null;
let videoCollapsed = true;
let videoConnected = false;

editVirtualZoneBtn = document.getElementById('edit-zone');
deleteVirtualZoneBtn = document.getElementById('delete-zone');



// ==========================
// MQTT CONNECTION
// ==========================

function connectMQTT() {
  mqttClient = mqtt.connect(MQTT_BROKER);

  mqttClient.on("connect", function () {
    console.log("Connesso al broker MQTT!");
    document.getElementById('status-dot').classList.add('connected');
    document.getElementById('status-text').textContent = 'Connesso';

    mqttClient.subscribe(MQTT_SUB_TOPIC, function (err) {
      if (!err) {
        console.log("Sottoscritto a", MQTT_SUB_TOPIC);
      }
    });
  });

  mqttClient.on("message", function (topic, message) {
    const msg = message.toString();
    console.log("Ricevuto:", msg);

    try {
      const j = JSON.parse(msg);

      if (j.type === "coverage" && j.points) {
        console.log("Polygon coverage ricevuto:", j.points);
      } else if (j.type === "manual" && j.command) {
        console.log("Manual command ricevuto:", j.command);
      }

    } catch (err) {
      console.error("Errore parsing MQTT message:", err);
    }
  });

  mqttClient.on("close", function () {
    console.log("Disconnesso dal broker MQTT");
    document.getElementById('status-dot').classList.remove('connected');
    document.getElementById('status-text').textContent = 'Disconnesso';
    setTimeout(connectMQTT, 3000); // reconnect
  });

  mqttClient.on("error", function () {
    document.getElementById('status-dot').classList.remove('connected');
    document.getElementById('status-text').textContent = 'Errore';
  });
}


// ==========================
// API CALLS
// ==========================

async function fetchRules() {
  try {
    const response = await fetch(`${nodeRedUrl}rules`);
    if (!response.ok) {
      throw new Error("Errore nella risposta: " + response.status);
    }

    const data = await response.json();

    console.log("data", data);

    rulesData = data;

    renderRules();
  } catch (err) {
    console.error("Errore nel fetch delle regole:", err);
  }
}

async function addRule(rule) {
  try {
    const res = await fetch(`${nodeRedUrl}rules`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(rule)
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newRule = json[0];

    rulesData.push(newRule);
    renderRules();

    console.log("Regola aggiunta:", newRule);
  } catch (err) {
    console.error("Errore nell'aggiunta della regola:", err);
  }
}

async function deleteRule(id) {
  if (!confirm("Sei sicuro di voler eliminare questa regola?")) return;

  try {
    const res = await fetch(`${nodeRedUrl}rules/${id}`, {
      method: "DELETE"
    });

    if (!res.ok) {
      throw new Error(await res.text());
    }

    rulesData = rulesData.filter(r => r.id !== id);
    renderRules();

    console.log(`Regola con id ${id} eliminata correttamente`);
  } catch (err) {
    console.error("Errore durante l'eliminazione della regola:", err);
    alert("Errore durante l'eliminazione della regola");
  }
}

async function fetchSensors() {
  try {
    const res = await fetch(`${nodeRedUrl}sensors`);
    if (!res.ok) throw new Error(res.statusText);
    sensorsData = await res.json();

    const select = document.getElementById("rule-sensor");
    select.innerHTML = "";
    sensorsData.forEach(sensor => {
      const opt = document.createElement("option");
      opt.value = sensor.id;
      opt.textContent = sensor.name;
      select.appendChild(opt);
    });
    console.log("Sensori caricati:", sensorsData);
  } catch (err) {
    console.error("Errore nel fetch dei sensori:", err);
  }
}

async function addSensor(name) {
  try {
    const res = await fetch(`${nodeRedUrl}sensors`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name })
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newSensor = json[0];

    sensorsData.push(newSensor);

    const opt = document.createElement("option");
    opt.value = newSensor.id;
    opt.textContent = newSensor.name;
    document.getElementById("rule-sensor").appendChild(opt);

    console.log("Sensore aggiunto:", newSensor);
  } catch (err) {
    console.error("Errore nell'aggiunta del sensore:", err);
  }
}

async function fetchRobots() {
  try {
    const res = await fetch(`${nodeRedUrl}robots`);
    if (!res.ok) throw new Error(res.statusText);
    robotsData = await res.json();

    const select = document.getElementById("rule-robot");
    select.innerHTML = "";
    robotsData.forEach(robot => {
      const opt = document.createElement("option");
      opt.value = robot.id;
      opt.textContent = robot.name;
      select.appendChild(opt);
    });
    console.log("Robot caricati:", robotsData);
  } catch (err) {
    console.error("Errore nel fetch dei robot:", err);
  }
}

async function addRobot(name) {
  try {
    const res = await fetch(`${nodeRedUrl}robots`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name })
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newRobot = json[0];

    robotsData.push(newRobot);

    const opt = document.createElement("option");
    opt.value = newRobot.id;
    opt.textContent = newRobot.name;
    document.getElementById("rule-robot").appendChild(opt);

    console.log("Robot aggiunto:", newRobot);
  } catch (err) {
    console.error("Errore nell'aggiunta del robot:", err);
  }
}

async function fetchZones() {
  try {
    const res = await fetch(`${nodeRedUrl}zones`);
    if (!res.ok) throw new Error(res.statusText);
    zonesData = await res.json();

    console.log("zonesData: ", zonesData[1]);

    const select = document.getElementById("rule-zone");
    select.innerHTML = "";
    zonesData.forEach(zone => {

      virtualZonesData.push({
        id: zone.id,
        name: zone.name,
        rosPoints: JSON.parse(zone.ros_points),
        leafletPoints: JSON.parse(zone.leaflet_points),
        color: zone.color
      });

      const opt = document.createElement("option");
      opt.value = zone.id;
      opt.textContent = zone.name;
      select.appendChild(opt);
    });


    console.log("Zone caricate:", zonesData);
  } catch (err) {
    console.error("Errore nel fetch delle zone:", err);
  }
}

async function addZone(name, color, rosPoints, leafletPoints) {
  try {
    const res = await fetch(`${nodeRedUrl}zones`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, color, ros_points: rosPoints, leaflet_points: leafletPoints })
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newZone = json[0];

    zonesData.push(newZone);

    virtualZonesData.push({
      id: newZone.id,
      name: newZone.name,
      rosPoints: JSON.parse(newZone.ros_points),
      leafletPoints: JSON.parse(newZone.leaflet_points),
      color: newZone.color
    });

    const opt = document.createElement("option");
    opt.value = newZone.id;
    opt.textContent = newZone.name;
    document.getElementById("rule-zone").appendChild(opt);

    console.log("Zona aggiunta:", newZone);
  } catch (err) {
    console.error("Errore nell'aggiunta della zona:", err);
  }
}

async function fetchOperations() {
  try {
    const res = await fetch(`${nodeRedUrl}operations`);
    if (!res.ok) throw new Error(res.statusText);
    operationsData = await res.json();

    const select = document.getElementById("rule-operation");
    select.innerHTML = "";
    operationsData.forEach(op => {
      const opt = document.createElement("option");
      opt.value = op.id;
      opt.textContent = op.description;
      select.appendChild(opt);
    });
    console.log("Operazioni caricate:", operationsData);
  } catch (err) {
    console.error("Errore nel fetch delle operazioni:", err);
  }
}

async function fetchConditions() {
  try {
    const res = await fetch(`${nodeRedUrl}conditions`);
    if (!res.ok) throw new Error(res.statusText);
    conditionsData = await res.json();

    const select = document.getElementById("rule-condition");
    select.innerHTML = "";
    conditionsData.forEach(cond => {
      const opt = document.createElement("option");
      opt.value = cond.id;
      opt.textContent = cond.description;
      select.appendChild(opt);
    });
    console.log("Condizioni caricate:", conditionsData);
  } catch (err) {
    console.error("Errore nel fetch delle condizioni:", err);
  }
}

async function fetchActions() {
  try {
    const res = await fetch(`${nodeRedUrl}actions`);
    if (!res.ok) throw new Error(res.statusText);
    actionsData = await res.json();

    const select = document.getElementById("rule-action");
    select.innerHTML = "";
    actionsData.forEach(act => {
      const opt = document.createElement("option");
      opt.value = act.id;
      opt.textContent = act.description;
      select.appendChild(opt);
    });
    console.log("Azioni caricate:", actionsData);
  } catch (err) {
    console.error("Errore nel fetch delle azioni:", err);
  }
}


// ==========================
// VIDEO FUNCTIONS
// ==========================

function toggleVideoPanel() {
  const panel = document.getElementById('video-panel');
  const icon = document.getElementById('collapse-icon');
  const mapInfo = document.getElementById('map-info');
  
  videoCollapsed = !videoCollapsed;
  
  if (videoCollapsed) {
    panel.classList.remove('expanded');
    panel.classList.add('collapsed');
    icon.textContent = '▼';
  } else {
    panel.classList.remove('collapsed');
    panel.classList.add('expanded');
    icon.textContent = '▲';
    
    if (!videoConnected) {
      connectVideoStream();
    }
  }
}

function connectVideoStream() {
  const img = document.getElementById('video-stream');
  const placeholder = document.getElementById('video-placeholder');
  const status = document.getElementById('video-status');
  
  img.src = streamUrl;

  img.style.display = 'block';
  placeholder.style.display = 'none';

    img.onload = function() {
      console.log("Stream ricevuto correttamente!");
      status.classList.add('connected');
      videoConnected = true;
    };

    img.onerror = function() {
      console.log("Errore nel ricevere lo stream");
      status.classList.remove('connected');
      placeholder.style.display = 'block';
      img.style.display = 'none';
      videoConnected = false;

      setTimeout(() => connectVideoStream(), 3000);
    };
}

// ==========================
// NAVIGATION
// ==========================

// Page navigation
function showPage(pageId) {
  // Hide all pages
  document.querySelectorAll('.page').forEach(page => page.classList.remove('active'));
  // Show selected page
  document.getElementById(pageId + '-page').classList.add('active');
  
  // Update nav buttons
  document.querySelectorAll('.nav-btn').forEach(btn => btn.classList.remove('active'));
  event.target.classList.add('active');
  
  // Initialize map if showing map page
  if (pageId === 'map' && !map) {
    initializeMap();
  }
}


// ==========================
// MAP PAGE
// ==========================

function initializeMap() {
  const img = new Image();
  img.src = "maps/complete_house_map_save.png";

  img.onload = () => {
    const width = img.width;
    const height = img.height;

    map = L.map('map', {
      crs: L.CRS.Simple,
      minZoom: -4
    });

    const bounds = [[0,0], [height,width]];
    L.imageOverlay("maps/complete_house_map_save.png", bounds).addTo(map);
    map.fitBounds(bounds);

    map.on("click", (e) => {

      if (selectedZone) {
        clearSelectedZone();
      }

      const x_pix = e.latlng.lng;
      const y_pix = e.latlng.lat;

      // Convert pixel → ROS2 coordinates
      let x_ros = origin[0] + (x_pix * resolution);
      let y_ros = origin[1] + (y_pix * resolution);

      let latlng = e.latlng;
      // Snap to first point if close enough (from third point onward)
      if (rosPoints.length >= 2) {
        const dx = x_ros - parseFloat(rosPoints[0].x);
        const dy = y_ros - parseFloat(rosPoints[0].y);
        const dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < closeThreshold) {
          x_ros = parseFloat(rosPoints[0].x);
          y_ros = parseFloat(rosPoints[0].y);
          latlng = leafletPoints[0];
          console.log("Snapped to first point to close polygon");
        }
      }

      // Save point
      rosPoints.push({x: x_ros.toFixed(2), y: y_ros.toFixed(2)});
      leafletPoints.push(latlng);

      // Draw polyline
      if (leafletPoints.length > 1) {
        if (polyline) {
          map.removeLayer(polyline);
        }
        polyline = L.polyline(leafletPoints, {color: '#ff6b6b', weight: 3}).addTo(map);
      }

      // Draw marker
      const marker = L.circleMarker(latlng, {
        radius: 6, 
        color: '#667eea', 
        fillColor: '#667eea',
        fillOpacity: 0.8,
        weight: 2
      }).addTo(map);
      markers.push(marker);
    });
  };
}

function sendPolygon() {
  if (rosPoints.length < 3) { 
    alert("Il poligono necessita di almeno 3 punti"); 
    return; 
  }
  const msg = { type: "coverage", points: rosPoints };
  mqttClient.publish(MQTT_PUB_TOPIC, JSON.stringify(msg));
  console.log("Sent Polygon via MQTT:", msg);
}

function drawVirtualZones() {
  // Draw polylines
  for (let i = 0; i < virtualZonesData.length; i++) {

    console.log("virtual zone: ", virtualZonesData[i]);

    const zone = virtualZonesData[i];

    const polygon = L.polygon(virtualZonesData[i].leafletPoints, {
        color: virtualZonesData[i].color,
        weight: 2,
        fillColor: virtualZonesData[i].color,
        fillOpacity: 0.4,
        bubblingMouseEvents: false
    }).addTo(map);

    const center = polygon.getBounds().getCenter();
    const label = L.tooltip({
        permanent: true,
        direction: 'center',
        className: 'zone-label'
    })
    .setContent(zone.name)
    .setLatLng(center);
    polygon.bindTooltip(label).openTooltip();

    // Draw markers
    // for (let j = 0; j < virtualZonesMarkers[i].length; j++) {
    //   const marker = virtualZonesMarkers[i][j];
    //   marker.addTo(map);
    // }
  }
}

function clearMap() {
  rosPoints = [];
  leafletPoints = [];
  
  if (polyline) {
    map.removeLayer(polyline);
    polyline = null;
  }
  
  markers.forEach(marker => map.removeLayer(marker));
  markers = [];
}

function undoLastPoint() {
  if (rosPoints.length === 0) return;
  
  rosPoints.pop();
  leafletPoints.pop();
  
  // Remove last marker
  if (markers.length > 0) {
    map.removeLayer(markers.pop());
  }
  
  // Redraw polyline
  if (polyline) {
    map.removeLayer(polyline);
    polyline = null;
  }
  
  if (leafletPoints.length > 1) {
    polyline = L.polyline(leafletPoints, {color: '#ff6b6b', weight: 3}).addTo(map);
  }
}

function createVirtualZone() {
  if (rosPoints.length <= 2) {
    alert("Disegnare un poligono chiuso");
    return;
  }

  // Check if first element is equal to last element
  let firstEqualToLast = rosPoints[0].x == rosPoints[rosPoints.length - 1].x &&
                          rosPoints[0].y == rosPoints[rosPoints.length - 1].y;
  if (!firstEqualToLast) {
    alert("Primo e ultimo punto non coincidono");
    return;
  }

  openZoneModal();
}

function editVirtualZone(zoneId) {
  if (!selectedZone) return;

  openZoneModal();

  document.getElementById("zoneNameInput").value = selectedZone.name;

  const color = selectedZone.color;
  document.querySelectorAll('.color-swatch').forEach(swatch => {
    swatch.classList.toggle('selected', swatch.dataset.color === color);
  });

}

function openZoneModal() {
  document.getElementById("zoneModal").style.display = "flex";
}

function closeZoneModal() {
  document.getElementById("zoneModal").style.display = "none";
  clearSelectedZone();
}

async function confirmVirtualZone() {
  const name = document.getElementById("zoneNameInput").value.trim();
  if (!name) {
    alert("Inserisci un nome valido");
    return;
  }

  if (!selectedColor) {
    alert("Selezionare un colore");
    return;
  }

  if (selectedZone) {
      selectedZone.name = name;
      selectedZone.polygon.setStyle({
          color: selectedZone.color,
          fillColor: selectedZone.color
      });

      const center = selectedZone.polygon.getBounds().getCenter();
      selectedZone.polygon.unbindTooltip();
      const label = L.tooltip({
          permanent: true,
          direction: 'center',
          className: 'zone-label'
      })
      .setContent(name)
      .setLatLng(center);
      selectedZone.polygon.bindTooltip(label).openTooltip();

      clearSelectedZone();
      drawVirtualZones();
      return;
  }
  console.log(selectedColor);
  await addZone(name, selectedColor, rosPoints, leafletPoints);

  clearMap();
  drawVirtualZones();
  closeZoneModal();

  console.log("Creata stanza virtuale:", name);
}

function clearSelectedZone() {
  if (selectedZone) {
    selectedZone.polygon.setStyle({ fillOpacity: 0.4 });
    selectedZone = null;
  }
  editVirtualZoneBtn.style.display = 'none';
  deleteVirtualZoneBtn.style.display = 'none';

  document.getElementById("zoneNameInput").value = "";
  document.querySelectorAll('.color-swatch').forEach(s => s.classList.remove('selected'));
  selectedColor = null;
}


// ==========================
// MANUAL CONTROL PAGE
// ==========================

function sendManualCmd(command) {
  const msg = { type: "manual", command };
  mqttClient.publish(MQTT_PUB_TOPIC, JSON.stringify(msg));
  console.log("Sent command via MQTT:", command);

  const buttons = document.querySelectorAll('.direction-btn, .center-btn');
  buttons.forEach(btn => {
    if (btn.onclick.toString().includes(command)) {
      btn.style.transform = btn.classList.contains('center-btn') ? 
        'translate(-50%, -50%) scale(0.9)' : 'scale(0.9)';
      setTimeout(() => {
        btn.style.transform = btn.classList.contains('center-btn') ? 
          'translate(-50%, -50%)' : '';
      }, 150);
    }
  });
}


// ==========================
// RULES PAGE
// ==========================


function openRuleForm(rule = null) {
  editingRule = rule;

  console.log("rule", rule);

  if (rule) {
    document.getElementById("rule-name").value = rule.name;
    document.getElementById("rule-sensor").value = rule.sensor_id;
    document.getElementById("rule-robot").value = rule.robot_id;
    document.getElementById("rule-zone").value = rule.zone_id;
    document.getElementById("rule-operation").value = rule.operation_id;
    document.getElementById("rule-condition").value = rule.condition_id;
    document.getElementById("rule-value").value = rule.value;
    document.getElementById("rule-action").value = rule.action_id;
  } else {
    document.getElementById("rule-value").value = "";
  }

  document.getElementById("ruleModal").style.display = "flex";
}

function closeRuleModal() {
  document.getElementById("ruleModal").style.display = "none";
  editingRule = null;
}

// Salva regola
function saveRule() {
  const newRule = {
    id: editingRule ? editingRule.id : null,
    name: document.getElementById("rule-name").value,
    sensor_id: parseInt(document.getElementById("rule-sensor").value),
    robot_id: parseInt(document.getElementById("rule-robot").value),
    zone_id: parseInt(document.getElementById("rule-zone").value),
    operation_id: parseInt(document.getElementById("rule-operation").value),
    condition_id: parseInt(document.getElementById("rule-condition").value),
    value: parseFloat(document.getElementById("rule-value").value),
    interv: document.getElementById("rule-interval").value,
    action_id: parseInt(document.getElementById("rule-action").value),
  };

  if (editingRule) {
    const idx = rulesData.findIndex(r => r.id === editingRule.id);
    rulesData[idx] = newRule;
  } else {
    addRule(newRule);
  }

  renderRules();
  closeRuleModal();
}
  
function openSensorForm() {
  document.getElementById("sensorModal").style.display = "flex";
}

function closeSensorModal() {
  document.getElementById("sensorModal").style.display = "none";
}

function confirmSensor() {
  const name = document.getElementById("sensorNameInput").value.trim();
  if (!name) return alert("Inserisci un nome valido");
  
  addSensor(name);
  closeSensorModal();
}

function openRobotForm() {
  document.getElementById("robotModal").style.display = "flex";
}

function closeRobotModal() {
  document.getElementById("robotModal").style.display = "none";
}

function confirmRobot() {
  const name = document.getElementById("robotNameInput").value.trim();
  if (!name) return alert("Inserisci un nome valido");

  addRobot(name);

  closeRobotModal();
}


function openZoneForm() {
  document.getElementById("zoneFormModal").style.display = "flex";
}

function closeZoneFormModal() {
  document.getElementById("zoneFormModal").style.display = "none";
}

function confirmZone() {
  const name = document.getElementById("zoneNameFormInput").value.trim();
  if (!name) return alert("Inserisci un nome valido");

  const zoneSelect = document.getElementById("rule-zone");
  const opt = document.createElement("option");
  opt.value = name;
  opt.textContent = name;
  zoneSelect.appendChild(opt);

  closeZoneFormModal();
}

function renderRules() {
  const tbody = document.getElementById("rules-list");
  tbody.innerHTML = "";

  if (rulesData.length === 0) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 10;
    cell.style.textAlign = "center";
    cell.textContent = "Nessuna regola definita";
    row.appendChild(cell);
    tbody.appendChild(row);
    return;
  }

  rulesData.forEach(rule => {
    const row = document.createElement("tr");

    const operation = operationsData.find(op => op.id === rule.operation_id);
    const condition = conditionsData.find(cond => cond.id === rule.condition_id);
    const action = actionsData.find(act => act.id === rule.action_id);
    const sensor = sensorsData.find(s => s.id === rule.sensor_id);
    const robot = robotsData.find(r => r.id === rule.robot_id);
    const zone = zonesData.find(z => z.id === rule.zone_id);

    row.innerHTML = `
      <td>${rule.name}</td>
      <td>${sensor.name}</td>
      <td>${robot.name}</td>
      <td>${zone.name}</td>
      <td>${operation.name}</td>
      <td>${condition.name}</td>
      <td>${rule.interv}</td>
      <td>${rule.value}</td>
      <td>${action.name}</td>
      <td>
        <button class="edit-btn" onclick='openRuleForm(${JSON.stringify(rule).replace(/"/g, "&quot;")})'>Modifica</button>
        <button class="delete-btn" onclick="deleteRule(${rule.id})">Elimina</button>
      </td>
    `;
    tbody.appendChild(row);
  });
}

async function fetchDecodings() {
  await fetchOperations();
  await fetchConditions();
  await fetchActions();
}


// ==========================
// APP INITIALIZATION
// ==========================

window.onload = async function() {
    connectMQTT();
    initializeMap();
    await fetchSensors();
    await fetchRobots();
    await fetchZones();
    await fetchDecodings();
    await fetchRules();
    renderRules();
    drawVirtualZones();

    document.getElementById("zoneNameInput").value = "";

    document.querySelectorAll('.color-swatch').forEach(swatch => {
      swatch.addEventListener('click', function() {
        document.querySelectorAll('.color-swatch').forEach(s => s.classList.remove('selected'));
        this.classList.add('selected');
        selectedColor = this.dataset.color;
      });
    });
};