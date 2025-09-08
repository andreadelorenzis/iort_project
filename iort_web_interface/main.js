
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
const nodeRedUrl = 'http://localhost:1880';
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
let sessionData = [];


let editingRule = null;
let editingSessionId = null;
let editingSensorId = null;
let editingRobotId = null;


let videoCollapsed = true;
let videoConnected = false;


let placingHomeBase = false;
let homeBaseMarker = null;
let homeBaseIconMarker = null;

const homeBaseSize = 16; // square side size
let homeBases = {};
let placingHomeBaseRobotId = null;
let selectedRobotColor;

const homeIcon = L.icon({
  iconUrl: 'imgs/home_white.png', 
  iconSize: [22, 22], 
  iconAnchor: [11, 11] 
});

let selectedZonesInOrder = [];

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

async function sendPlan() {
  try {
    const response = await fetch(`${nodeRedUrl}/start_plan`);
    if (!response.ok) {
        throw new Error("Errore nella risposta: " + response.status);
    }
  } catch (err) {
    console.error("Errore nell'invio del piano:", err);
  }
}

async function fetchRules() {
  try {
    const response = await fetch(`${nodeRedUrl}/rules`);
    if (!response.ok) {
      throw new Error("Errore nella risposta: " + response.status);
    }

    const data = await response.json();


    rulesData = data;

    renderRules();
  } catch (err) {
    console.error("Errore nel fetch delle regole:", err);
  }
}

async function addRule(rule) {
  try {
    const res = await fetch(`${nodeRedUrl}/rules`, {
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
    const res = await fetch(`${nodeRedUrl}/rules/${id}`, {
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
    const res = await fetch(`${nodeRedUrl}/sensors`);
    if (!res.ok) throw new Error(res.statusText);
    sensorsData = await res.json();

    renderSensors();

    const select = document.getElementById("rule-sensor");
    select.innerHTML = "";
    sensorsData.forEach(sensor => {
      const opt = document.createElement("option");
      opt.value = sensor.id;
      opt.textContent = sensor.name;
      select.appendChild(opt);
    });
  } catch (err) {
    console.error("Errore nel fetch dei sensori:", err);
  }
}

async function addSensor(name) {
  try {
    const res = await fetch(`${nodeRedUrl}/sensors`, {
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

async function deleteSensor(id) {
  if (!confirm("Vuoi davvero eliminare questo sensore?")) return;

  try {
    await fetch(`${nodeRedUrl}/sensors/${id}`, { method: "DELETE" });
    sensorsData = sensorsData.filter(s => s.id !== id);
    renderSensors();
  } catch (err) {
    console.error("Errore eliminazione sensore:", err);
  }
}

async function fetchRobots() {
  try {
    const res = await fetch(`${nodeRedUrl}/robots`);
    if (!res.ok) throw new Error(res.statusText);
    robotsData = await res.json();

    renderRobots();
    renderRobotBases();

    const ruleRobotSelect = document.getElementById("rule-robot");
    const sessionRobotSelect = document.getElementById("session-robot");

    // Populate rule modal select
    ruleRobotSelect.innerHTML = "";
    robotsData.forEach(robot => {
      const opt = document.createElement("option");
      opt.value = robot.id;
      opt.textContent = robot.name;
      ruleRobotSelect.appendChild(opt);
    });

    // Populate session modal select
    sessionRobotSelect.innerHTML = "";
    robotsData.forEach(robot => {
      const opt = document.createElement("option");
      opt.value = robot.id;
      opt.textContent = robot.name;
      sessionRobotSelect.appendChild(opt);
    });

    

  } catch (err) {
    console.error("Errore nel fetch dei robot:", err);
  }
}

function clearRobotBases() {
  for (const robotId in homeBases) {
    if (homeBases[robotId]) {
      map.removeLayer(homeBases[robotId].marker);
      map.removeLayer(homeBases[robotId].icon);
    }
  }
  homeBases = {}; 
}


function renderRobotBases() {
  clearRobotBases();

  // Draw base stations
  robotsData.forEach(robot => {
    if (robot.leaflet_base_pos) {
      const { lat, lng } = JSON.parse(robot.leaflet_base_pos);
      console.log("Drawing base station for robot: " + robot.id);
      console.log(lat, lng)

      // Rettangolo
      const rect = L.rectangle([
        [lat - homeBaseSize/2, lng - homeBaseSize/2],
        [lat + homeBaseSize/2, lng + homeBaseSize/2]
      ], {
        color: robot.color,
        weight: 2,
        fillColor: robot.color,
        fillOpacity: 0.5
      }).addTo(map);

      // Icona home
      const icon = L.marker([lat, lng], { icon: homeIcon }).addTo(map);

      homeBases[robot.id] = { marker: rect, icon: icon };
    }
  });
}

async function addRobot(name) {
  try {
    const res = await fetch(`${nodeRedUrl}/robots`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name })
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newRobot = json[0];

    robotsData.push(newRobot);
    renderRobotBases();

    const opt = document.createElement("option");
    opt.value = newRobot.id;
    opt.textContent = newRobot.name;
    document.getElementById("rule-robot").appendChild(opt);

    console.log("Robot aggiunto:", newRobot);
  } catch (err) {
    console.error("Errore nell'aggiunta del robot:", err);
  }
}

async function deleteRobot(id) {
  if (!confirm("Vuoi davvero eliminare questo robot?")) return;

  try {
    await fetch(`${nodeRedUrl}/robots/${id}`, { method: "DELETE" });
    robotsData = robotsData.filter(r => r.id !== id);
    renderRobots();
    renderRobotBases();
  } catch (err) {
    console.error("Errore eliminazione robot:", err);
  }
}

async function fetchZones() {
  try {
    const res = await fetch(`${nodeRedUrl}/zones`);
    if (!res.ok) throw new Error(res.statusText);
    zonesData = await res.json();

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


  } catch (err) {
    console.error("Errore nel fetch delle zone:", err);
  }
}

async function addZone(name, color, rosPoints, leafletPoints) {
  try {
    const res = await fetch(`${nodeRedUrl}/zones`, {
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

async function deleteZone() {
  if (!confirm("Sei sicuro di voler eliminare questa zona?")) return;

  if (!selectedZone) return;

  const zoneId = selectedZone.id;

  try {
    const res = await fetch(`${nodeRedUrl}/zones/${zoneId}`, { method: "DELETE" });
    if (!res.ok) throw new Error(await res.text());

    // Rimuovi il layer della zona dalla mappa
    if (selectedZone.polygon) {
      map.removeLayer(selectedZone.polygon);
    }

    // Rimuovi zona dai dati locali
    virtualZonesData = virtualZonesData.filter(z => z.id !== zoneId);
    zonesData = zonesData.filter(z => z.id !== zoneId);

    clearSelectedZone();
    renderZones();

    console.log(`Zona con id ${zoneId} eliminata correttamente`);
  } catch (err) {
    console.error("Errore durante l'eliminazione della zona:", err);
    alert("Errore durante l'eliminazione della zona");
  }
}


async function fetchOperations() {
  try {
    const res = await fetch(`${nodeRedUrl}/operations`);
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
  } catch (err) {
    console.error("Errore nel fetch delle operazioni:", err);
  }
}

async function fetchConditions() {
  try {
    const res = await fetch(`${nodeRedUrl}/conditions`);
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
  } catch (err) {
    console.error("Errore nel fetch delle condizioni:", err);
  }
}

async function fetchActions() {
  try {
    const res = await fetch(`${nodeRedUrl}/actions`);
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
  } catch (err) {
    console.error("Errore nel fetch delle azioni:", err);
  }
}

async function fetchSessions() {
  try {
    const res = await fetch(`${nodeRedUrl}/sessions`);
    if (!res.ok) throw new Error(await res.text());
    const data = await res.json();

    console.log(data)

    const ruleSessionSelect = document.getElementById("rule-session");

    sessionData = [];
    data.forEach(dataRow => {
      if (!sessionData.find(s => s.id === dataRow.id)) {
        // Find all zones data for this session id
        const zones = data.filter(row => row.id == dataRow.id).map(row => {
          return {
            id: row.zone_id,
            name: row.zone_name,
            ordering: row.ordering
          };
        })
        
        sessionData.push({
          id: dataRow.id,
          name: dataRow.name,
          robot_id: dataRow.robot_id,
          scheduled_start: dataRow.scheduled_start,
          status: dataRow.status,
          zones
        });
      }
    });

    console.log(sessionData);
    
    // Populate rule modal select
    ruleSessionSelect.innerHTML = "";
    sessionData.forEach(session => {
      const opt = document.createElement("option");
      opt.value = session.id;
      opt.textContent = session.name;
      ruleSessionSelect.appendChild(opt);
    });

    // Popola la tabella sessioni
    renderSessions(sessionData);
  } catch (err) {
    console.error("Errore nel caricamento delle sessioni:", err);
    alert("Errore durante il recupero delle sessioni");
  }
}


async function addSession(session) {
  try {
    const res = await fetch(`${nodeRedUrl}/sessions`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(session)
    });

    if (!res.ok) throw new Error(await res.text());
    const json = await res.json();
    const newSession = json[0];

    console.log("Sessione aggiunta:", newSession);
    alert("Sessione creata con successo!");

    closeSessionModal();
  } catch (err) {
    console.error("Errore nell'aggiunta della sessione:", err);
    alert("Errore durante la creazione della sessione");
  }
}

async function deleteSession(id) {
  if (!confirm("Vuoi davvero eliminare questa sessione?")) return;

  try {
    await fetch(`${nodeRedUrl}/sessions/${id}`, { method: "DELETE" });
    sessionsData = sessionsData.filter(s => s.id !== id);
    renderSessions(sessionsData);
  } catch (err) {
    console.error("Errore eliminazione sessione:", err);
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

    map.on('mousemove', followMouse);

    map.on("click", (e) => {
      let latlng = e.latlng;
      const x_pix = e.latlng.lng;
      const y_pix = e.latlng.lat;

      // Convert pixel → ROS2 coordinates
      let x_ros = origin[0] + (x_pix * resolution);
      let y_ros = origin[1] + (y_pix * resolution);

      if (selectedZone) {
        clearSelectedZone();
      }

      if (placingHomeBase && placingHomeBaseRobotId) {
        placingHomeBase = false;
        map.off("mousemove", followMouse);

        const bounds = homeBases[placingHomeBaseRobotId].marker.getBounds();
        const center = bounds.getCenter();
        console.log(`Homebase robot ${placingHomeBaseRobotId} posizionata in:`, center);

        // Salvataggio della base del robot
        const payload = {
          id: placingHomeBaseRobotId,
          ros_base_pos: { x: parseFloat(x_ros.toFixed(2)), y: parseFloat(y_ros.toFixed(2)) },       
          leaflet_base_pos: latlng,
          color: selectedRobotColor
        };
        fetch(`${nodeRedUrl}/robot_base`, {
          method: "PUT",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        })
        .then((res) => {
          if (!res.ok) throw new Error("Errore PUT verso Node-RED");
          return res.json();
        })
        .then((data) => {
          console.log("Node-RED ha ricevuto la base:", data);
        })
        .catch((err) => {
          console.error("Errore chiamata Node-RED:", err);
        });

        placingHomeBaseRobotId = null;
        selectedRobotColor = null;
        return;
      }

      
      // Snap to first point if close enough (from third point onward)
      if (rosPoints.length >= 2) {
        const dx = x_ros - parseFloat(rosPoints[0].x);
        const dy = y_ros - parseFloat(rosPoints[0].y);
        const dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < closeThreshold) {
          x_ros = parseFloat(rosPoints[0].x);
          y_ros = parseFloat(rosPoints[0].y);
          latlng = leafletPoints[0];
        }
      }

      // Save point
      rosPoints.push({x: parseFloat(x_ros.toFixed(2)), y: parseFloat(y_ros.toFixed(2))});
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

function followMouse(e) {
  if (placingHomeBase && placingHomeBaseRobotId) {
    const latlng = e.latlng;
    console.log(latlng)

    if (!homeBases[placingHomeBaseRobotId]) {
      const rect = L.rectangle([
        [latlng.lat - homeBaseSize/2, latlng.lng - homeBaseSize/2],
        [latlng.lat + homeBaseSize/2, latlng.lng + homeBaseSize/2]
      ], {
        color: selectedRobotColor,
        weight: 2,
        fillColor: selectedRobotColor,
        fillOpacity: 0.5
      }).addTo(map);

      const icon = L.marker(latlng, { icon: homeIcon }).addTo(map);

      homeBases[placingHomeBaseRobotId] = { marker: rect, icon: icon };
    } else {
      homeBases[placingHomeBaseRobotId].marker.setBounds([
        [latlng.lat - 5, latlng.lng - 5],
        [latlng.lat + 5, latlng.lng + 5]
      ]);
      homeBases[placingHomeBaseRobotId].icon.setLatLng(latlng);
    }
  }
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

function renderZones() {
  // Draw polylines
  for (let i = 0; i < virtualZonesData.length; i++) {

    const zone = virtualZonesData[i];

    const polygon = L.polygon(virtualZonesData[i].leafletPoints, {
        color: virtualZonesData[i].color,
        weight: 2,
        fillColor: virtualZonesData[i].color,
        fillOpacity: 0.4,
        bubblingMouseEvents: false
    }).addTo(map);
    zone.polygon = polygon;

    const center = polygon.getBounds().getCenter();
    const label = L.tooltip({
        permanent: true,
        direction: 'center',
        className: 'zone-label'
    })
    .setContent(zone.name)
    .setLatLng(center);
    polygon.bindTooltip(label).openTooltip();

    polygon.on("click", function (e) {
      // Deseleziona zona precedentemente selezionata
      if (selectedZone && selectedZone.id !== zone.id) {
        clearSelectedZone();
      }

      // Seleziona questa zona
      selectedZone = virtualZonesData.find(z => z.id === zone.id);
      console.log(selectedZone);
      polygon.setStyle({ fillOpacity: 0.8 });

      editVirtualZoneBtn.style.display = 'block';
      deleteVirtualZoneBtn.style.display = 'block';

      L.DomEvent.stop(e);
    });

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

function editVirtualZone() {
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
      renderZones();
      return;
  }
  await addZone(name, selectedColor, rosPoints, leafletPoints);

  clearMap();
  renderZones();
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

function setHomePosition() {
  // Apri modale selezione robot
  const select = document.getElementById("homeBaseRobotSelect");
  select.innerHTML = "";
  robotsData.forEach(robot => {
    const opt = document.createElement("option");
    opt.value = robot.id;
    opt.textContent = robot.name;
    select.appendChild(opt);
  });

  document.getElementById("homeBaseModal").style.display = "flex";
}

function confirmHomeBaseSelection() {
  placingHomeBaseRobotId = parseInt(document.getElementById("homeBaseRobotSelect").value);
  selectedRobotColor = document.getElementById("robotColor").value;
  placingHomeBase = true;

  console.log(placingHomeBaseRobotId)

  // Se il robot aveva già una base, rimuovila dalla mappa
  if (homeBases[placingHomeBaseRobotId]) {
    map.removeLayer(homeBases[placingHomeBaseRobotId].marker);
    map.removeLayer(homeBases[placingHomeBaseRobotId].icon);
    homeBases[placingHomeBaseRobotId] = null;
  }

  map.on("mousemove", followMouse);
  document.getElementById("homeBaseModal").style.display = "none";
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

  if (rule) {
    document.getElementById("rule-name").value = rule.name;
    document.getElementById("rule-sensor").value = rule.sensor_id;
    document.getElementById("rule-session").value = rule.session_id;
    document.getElementById("rule-robot").value = rule.robot_id;
    document.getElementById("rule-zone").value = rule.zone_id;
    document.getElementById("rule-operation").value = rule.operation_id;
    document.getElementById("rule-condition").value = rule.comparator_id;
    document.getElementById("rule-value").value = rule.value;
    document.getElementById("rule-interval").value = rule.interv;
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

async function confirmRule() {
  const ruleData = {
    name: document.getElementById("rule-name").value,
    sensor_id: parseInt(document.getElementById("rule-sensor").value),
    session_id: parseInt(document.getElementById("rule-session").value),
    robot_id: parseInt(document.getElementById("rule-robot").value),
    zone_id: parseInt(document.getElementById("rule-zone").value),
    operation_id: parseInt(document.getElementById("rule-operation").value),
    comparator_id: parseInt(document.getElementById("rule-condition").value),
    value: parseFloat(document.getElementById("rule-value").value),
    interv: document.getElementById("rule-interval").value,
    action_id: parseInt(document.getElementById("rule-action").value),
    id: editingRule ? editingRule.id : null
  };

  try {
    if (editingRule) {
      // PUT per aggiornamento
      await fetch(`${nodeRedUrl}/rules`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(ruleData)
      });
    } else {
      // POST per nuova regola
      await fetch(`${nodeRedUrl}/rules`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(ruleData)
      });
    }

    await fetchRules(); // ricarica le regole
    closeRuleModal();
  } catch (err) {
    console.error("Errore salvataggio regola:", err);
  }
}

  

function openSensorForm(sensor = null) {
  editingSensorId = sensor ? sensor.id : null;

  document.getElementById("sensorNameInput").value = sensor ? sensor.name : "";

  document.getElementById("sensorModal").style.display = "block";
}

async function confirmSensor() {
  const name = document.getElementById("sensorNameInput").value;

  try {
    if (editingSensorId) {
      await fetch(`${nodeRedUrl}/sensors`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, id: editingSensorId })
      });
    } else {
      await fetch(`${nodeRedUrl}/sensors`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name })
      });
    }

    await fetchSensors();
    closeSensorModal();
  } catch (err) {
    console.error("Errore salvataggio sensore:", err);
  }
}

function closeSensorModal() {
  document.getElementById("sensorModal").style.display = "none";
  editingSensorId = null;

  document.getElementById("sensorNameInput").value = "";
}


function openRobotForm(robot = null) {
  editingRobotId = robot ? robot.id : null;

  document.getElementById("robotNameInput").value = robot ? robot.name : "";

  document.getElementById("robotModal").style.display = "block";
}

async function confirmRobot() {
  const name = document.getElementById("robotNameInput").value;

  try {
    if (editingRobotId) {
      await fetch(`${nodeRedUrl}/robots`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, id: editingRobotId })
      });
    } else {
      await fetch(`${nodeRedUrl}/robots`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name })
      });
    }

    await fetchRobots();
    closeRobotModal();
  } catch (err) {
    console.error("Errore salvataggio robot:", err);
  }
}

function closeRobotModal() {
  document.getElementById("robotModal").style.display = "none";
  editingRobotId = null;

  document.getElementById("robotNameInput").value = "";
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

function toDatetimeLocal(dateStr) {
    if (!dateStr) return "";
    const date = new Date(dateStr);
    const pad = (n) => n.toString().padStart(2, "0");

    const year = date.getFullYear();
    const month = pad(date.getMonth() + 1);
    const day = pad(date.getDate());
    const hours = pad(date.getHours());
    const minutes = pad(date.getMinutes());

    return `${year}-${month}-${day}T${hours}:${minutes}`;
}


function openSessionForm(session = null) {
  editingSessionId = session ? session.id : null;

  document.getElementById("session-name").value = session ? session.name : "";
  document.getElementById("session-robot").value = session ? session.robot_id : "";
  document.getElementById("session-scheduled-start").value = session ? toDatetimeLocal(session.scheduled_start) : "";

  const select = document.getElementById("session-zones");
  select.innerHTML = "";

  selectedZonesInOrder = [];

  const sessionRow = sessionData.find(s => s.id === session.id);
  const selectedZoneIds = sessionRow ? sessionRow.zones.map(z => z.id) : [];

  console.log("Session data prima di confirm")
  console.log(sessionData);

  zonesData.forEach(zone => {
    const option = document.createElement("option");
    option.value = zone.id;
    option.text = zone.name;

    const sessionZone = sessionRow.zones.find(z => z.id === zone.id);

    if (selectedZoneIds.includes(zone.id)) {
      option.selected = true;
      option.text = option.text += ` (${sessionZone.ordering})`
    }

    select.appendChild(option);
  });

  document.getElementById("sessionModal").style.display = "block";
}

async function confirmSession() {
  if (selectedZonesInOrder.length === 0) {
    alert("Selezionare almeno una zona");
    return;
  }

  const name = document.getElementById("session-name").value;
  const robot_id = document.getElementById("session-robot").value;
  const scheduled_start = document.getElementById("session-scheduled-start").value;

  const select = document.getElementById("session-zones");
  const selectedZones = Array.from(select.selectedOptions).map(opt => opt.value);
  console.log("Zone selezionate:", selectedZones);

  const zones = selectedZonesInOrder.map(z => {
    return {
      id: parseInt(z),
      intensity: 1,
      frequency: 1
    };
  });

  const body = { name, robot_id, scheduled_start, id: editingSessionId, zones };

  try {
    if (editingSessionId) {
      await fetch(`${nodeRedUrl}/sessions`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body)
      });
      console.log("Sessione di pulizia aggiornata.")
    } else {
      await fetch(`${nodeRedUrl}/sessions`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body)
      });
      console.log("Sessione di pulizia creata.")
    }

    await fetchSessions(); 

    console.log("Session data dopo confirm")
    console.log(sessionData);

    closeSessionModal();
  } catch (err) {
    console.error("Errore salvataggio sessione:", err);
  }
}


function closeSessionModal() {
  document.getElementById("sessionModal").style.display = "none";
  editingSessionId = null;

  // reset campi
  document.getElementById("session-name").value = "";
  document.getElementById("session-robot").value = "";
  document.getElementById("session-scheduled-start").value = "";
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

  rulesData.forEach(rule => { // asdfasdfaskldfasdj
    const row = document.createElement("tr");

    const operation = operationsData.find(op => op.id === rule.operation_id);
    const condition = conditionsData.find(cond => cond.id === rule.comparator_id);
    const action = actionsData.find(act => act.id === rule.action_id);
    const sensor = sensorsData.find(s => s.id === rule.sensor_id);
    const robot = robotsData.find(r => r.id === rule.robot_id);
    const zone = zonesData.find(z => z.id === rule.zone_id);

    row.innerHTML = `
      <td>${rule.name}</td>
      <td>${sensor.name}</td>
      <td>${rule.session_name}</td>
      <td>${robot.name}</td>
      <td>${zone.name}</td>
      <td>${operation.name}</td>
      <td>${condition.name}</td>
      <td>${rule.value}</td>
      <td>${rule.interv}</td>
      <td>${action.name}</td>
      <td>
        <button class="edit-btn" onclick='openRuleForm(${JSON.stringify(rule).replace(/"/g, "&quot;")})'>Modifica</button>
        <button class="delete-btn" onclick="deleteRule(${rule.id})">Elimina</button>
      </td>
    `;
    tbody.appendChild(row);
  });
}

function renderSessions(sessions) {
  const tbody = document.querySelector("#sessions-table tbody");
  tbody.innerHTML = "";

  if (sessions.length === 0) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 10;
    cell.style.textAlign = "center";
    cell.textContent = "Nessuna sessione definita";
    row.appendChild(cell);
    tbody.appendChild(row);
    return;
  }

  sessions.forEach(session => {
    const tr = document.createElement("tr");

    const robot = robotsData.find(r => r.id === session.robot_id);

    tr.innerHTML = `
      <td>${session.id}</td>
      <td>${session.name}</td>
      <td>${robot.name}</td>
      <td>${session.scheduled_start ? new Date(session.scheduled_start).toLocaleString() : "-"}</td>
      <td>${session.status || "pending"}</td>
      <td>
        <button class="edit-btn" onclick='openSessionForm(${JSON.stringify(session).replace(/"/g, "&quot;")})'>Modifica</button>
        <button class="delete-btn" onclick="deleteSession(${session.id})">Elimina</button>
      </td>
    `;

    tbody.appendChild(tr);
  });
}


function renderSensors() {
  const tbody = document.getElementById("sensors-list");
  tbody.innerHTML = "";

  if (sensorsData.length === 0) {
    const row = document.createElement("tr");
    row.innerHTML = `<td colspan="3" style="text-align:center;">Nessun sensore disponibile</td>`;
    tbody.appendChild(row);
    return;
  }

  sensorsData.forEach(sensor => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${sensor.id}</td>
      <td>${sensor.name}</td>
      <td>
        <button class="edit-btn" onclick='openSensorForm(${JSON.stringify(sensor).replace(/"/g, "&quot;")})'>Modifica</button>
        <button class="delete-btn" onclick="deleteSensor(${sensor.id})">Elimina</button>
      </td>
    `;
    tbody.appendChild(row);
  });
}


function renderRobots() {
  const tbody = document.getElementById("robots-list");
  tbody.innerHTML = "";

  if (robotsData.length === 0) {
    const row = document.createElement("tr");
    row.innerHTML = `<td colspan="3" style="text-align:center;">Nessun robot disponibile</td>`;
    tbody.appendChild(row);
    return;
  }

  robotsData.forEach(robot => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${robot.id}</td>
      <td>${robot.name}</td>
      <td>
        <button class="edit-btn" onclick='openRobotForm(${JSON.stringify(robot).replace(/"/g, "&quot;")})'>Modifica</button>
        <button class="delete-btn" onclick="deleteRobot(${robot.id})">Elimina</button>
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
    await fetchRobots();
    await fetchSensors();
    await fetchZones();
    await fetchDecodings();
    await fetchSessions();
    await fetchRules();

    renderZones();

    document.getElementById("zoneNameInput").value = "";

    document.querySelectorAll('.color-swatch').forEach(swatch => {
      swatch.addEventListener('click', function() {
        document.querySelectorAll('.color-swatch').forEach(s => s.classList.remove('selected'));
        this.classList.add('selected');
        selectedColor = this.dataset.color;
      });
    });

    const zonesSelect = document.getElementById("session-zones");
    // Aggiorna l’array quando l’utente seleziona/deseleziona
    zonesSelect.addEventListener("change", () => {
      const newlySelected = Array.from(zonesSelect.selectedOptions).map(opt => opt.value);

      // Aggiunge i nuovi selezionati in fondo all’array, senza duplicati
      newlySelected.forEach(id => {
        if (!selectedZonesInOrder.includes(id)) selectedZonesInOrder.push(id);
      });

      // Rimuove i deselect dagli array
      selectedZonesInOrder = selectedZonesInOrder.filter(id => newlySelected.includes(id));

      console.log("Ordine selezione:", selectedZonesInOrder);
    });

};
