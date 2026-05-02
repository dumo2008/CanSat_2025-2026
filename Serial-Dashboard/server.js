const express = require("express");
const http = require("http");
const mysql = require("mysql2/promise");

const { SerialPort } = require("serialport");
const { ReadlineParser } = require("@serialport/parser-readline");

const app = express();
const server = http.createServer(app);

app.use(express.static("public"));

/* =========================
   CONFIG
========================= */
const SERIAL_PORT = process.env.SERIAL_PORT || "COM3";
const BAUD = parseInt(process.env.BAUD || "115200", 10);

const DB_POOL = mysql.createPool({
  host: "localhost",
  user: "root",
  password: "",
  database: "cansat", // must exist
});

/* =========================
   SSE (/stream)
========================= */
const sseClients = new Set();

app.get("/stream", (req, res) => {
  res.writeHead(200, {
    "Content-Type": "text/event-stream",
    "Cache-Control": "no-cache",
    Connection: "keep-alive",
    "Access-Control-Allow-Origin": "*",
  });

  res.write("event: ping\ndata: ok\n\n");

  sseClients.add(res);
  req.on("close", () => sseClients.delete(res));
});

function sseBroadcast(obj) {
  const payload = `data: ${JSON.stringify(obj)}\n\n`;
  for (const client of sseClients) client.write(payload);
}

/* =========================
   FRAME BUFFER (insert on G)
========================= */
let frame = {};
function resetFrame() {
  frame = {
    // Motor (M)
    turnL: null,
    turnR: null,
    status: null,

    // Barometer (B)
    temperature: null,
    pressure: null,
    altitude: null, // barometer altitude

    // GPS (G)
    sats: null,
    hdop: null,
    lat: null,
    lon: null,
    alt_gps: null,
  };
}
resetFrame();

/* =========================
   HELPERS
========================= */
function safeInt(v) {
  const n = parseInt(v, 10);
  return Number.isFinite(n) ? n : null;
}
function safeFloat(v) {
  const n = parseFloat(v);
  return Number.isFinite(n) ? n : null;
}

/* =========================
   STORE + DASHBOARD
========================= */
async function storeFrame() {
  const dashObj = {
    temperature: frame.temperature,
    pressure: frame.pressure,
    altitudeBM: frame.altitude,     
    altitude: frame.altitude,      
    sats: frame.sats,
    hdop: frame.hdop,
    lat: frame.lat,
    lon: frame.lon,
    turnL: frame.turnL,
    turnR: frame.turnR,
    status: frame.status,
    alt_gps: frame.alt_gps,
  };

  sseBroadcast(dashObj);

  // Insert into your existing table
  try {
    await DB_POOL.query(
      `INSERT INTO data_js
      (timestamp, turnL, turnR, status,
       temperature, pressure, altitude,
       sats, hdop, lat, lon, alt_gps)
      VALUES (NOW(), ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`,
      [
        frame.turnL,
        frame.turnR,
        frame.status,
        frame.temperature,
        frame.pressure,
        frame.altitude,
        frame.sats,
        frame.hdop,
        frame.lat,
        frame.lon,
        frame.alt_gps,
      ]
    );
  } catch (err) {
    console.error("DB insert error:", err.message);
  }
}

/* =========================
   PARSER (NOISE-PROOF)
   Only accept lines starting with [Lora] prefix,
   strip it, then handle M; B; G; records as before
========================= */

async function handleLine(raw) {
  if (!raw) return;

  let line = String(raw).trim();

  /*
  // Strip [Lora] prefix if present, otherwise ignore
  if (line.startsWith("[Lora]")) {
    line = line.replace("[Lora]", "").trim();
  } else {
    return; // ignore all other lines like [INFO], [WARN], etc.
  }

  // ignore debug like record_counter, dict prints, etc.
  if (!/^[MBG];/.test(line)) return;
  */

  console.log(line);

  const p = line.split(";");
  const type = p[0];

  // M;turnL;turnR;status
  if (type === "M") {
    if (p.length < 4) return console.warn("Invalid M line:", line);
    frame.turnL = safeInt(p[1]);
    frame.turnR = safeInt(p[2]);
    frame.status = p[3] ?? null;
    return;
  }

  // B;temperature;pressure;altitude
  if (type === "B") {
    if (p.length < 4) return console.warn("Invalid B line:", line);
    frame.temperature = safeFloat(p[1]);
    frame.pressure = safeFloat(p[2]);
    frame.altitude = safeFloat(p[3]);
    return;
  }

  // G;sats;hdop;lat;lon;alt_gps
  if (type === "G") {
    if (p.length < 6) return console.warn("Invalid G line:", line);
    frame.sats = safeInt(p[1]);
    frame.hdop = safeFloat(p[2]);
    frame.lat = p[3] ?? null;
    frame.lon = p[4] ?? null;
    frame.alt_gps = p[5] ?? null;

    await storeFrame();
    resetFrame();
    return;
  }
}

/* =========================
   SERIAL
========================= */
function startSerial() {
  const port = new SerialPort({ path: SERIAL_PORT, baudRate: BAUD });
  const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

  console.log(`Listening on ${SERIAL_PORT} @ ${BAUD}`);

  parser.on("data", async (line) => {
    try {
      await handleLine(line);
    } catch (e) {
      console.error("Parse/store error:", e.message);
    }
  });

  port.on("error", (err) => console.error("Serial error:", err.message));
}

/* =========================
   START
========================= */
server.listen(3000, () => {   
  console.log("Server running on http://localhost:3000");
  startSerial();
});