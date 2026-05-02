"""
Cansat Receiver
CanSat Project 2025-2026 - Team SkyByte
"""
import sdcard, os
from machine import Pin, SPI, I2C
from time import sleep, sleep_ms
from ulora import LoRa, ModemConfig, SPIConfig
from sh1106 import SH1106
import network
from umqtt.simple import MQTTClient
import ujson

# ===== Log levels =====
LOG_INFO  = 0
LOG_WARN  = 1
LOG_ERROR = 2

# ===== Stop Pin =====
stop_pin = Pin(22, Pin.IN, Pin.PULL_UP)

# ===== OLED Display set up =====
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=400000)

# ===== SD kaart setup =====
spi = SPI(0, sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs = Pin(17, Pin.OUT)
cd = Pin(15, Pin.IN, Pin.PULL_UP)

# --- Filename on SD card ---
VERSION_FILE_NAME = "version.txt"
LOG_FILE_NAME = "/sd/log_v"
DATA_FILE_NAME = "/sd/lora_v"

# ===== WIFI INSTELLINGEN =====
SSID = "xxx"
PASSWORD = "xxx"

# ===== MQTT INSTELLINGEN =====
MQTT_BROKER = "192.168.0.x"
CLIENT_ID = "SkyByte"
TOPIC = "SkyByte"

# ===== Lora Parameters =====
# spi pin defs for various boards (channel, sck, mosi, miso)
# rp2_1 = (SPIBus = 1, sck=Pin(10), mosi=Pin(11), miso=Pin(8) )
RFM95_RST = 27
RFM95_SPIBUS = SPIConfig.rp2_1
RFM95_CS = 9
RFM95_INT = 28
RF95_FREQ = 868.8
RF95_POW = 20
CLIENT_ADDRESS = 0x7A # address of SkyByte
SERVER_ADDRESS = 0xBB 

# ***** GLOBAL VARS *****
logfile = None
display = None
client = None
sdOK = False
data_filename = ""
log_filename = ""
pending_records = []  # holds current incomplete group of 3
msgCount = 0

def log(level, message, oled_msg=None):
    """
    level     : LOG_INFO / LOG_WARN / LOG_ERROR
    message   : text toserial monitor
    oled_msg  : optional short text to OLED
    """

    # ---- Serial output ----
    prefix = {
        LOG_INFO:  "[INFO]",
        LOG_WARN:  "[WARN]",
        LOG_ERROR: "[ERROR]"
    }

    print(prefix.get(level, "[LOG] "), message)

    # ---- OLED output ----
    if display is None:
        return

    if oled_msg is None:
        oled_msg = message[:16]  # truncate for OLED

    display.fill(0)

    if level == LOG_ERROR:
        display.rect(0, 0, 128, 20, 1)
        display.text("ERROR", 40, 4, 1)
        display.text(oled_msg, 2, 24, 1)

    elif level == LOG_WARN:
        display.text("WARNING", 30, 4, 1)
        display.text(oled_msg, 2, 24, 1)

    else:
        display.text("INFO", 45, 4, 1)
        display.text(oled_msg, 2, 24, 1)

    # Write to SD log file
    if logfile is not None:
        try:
            level_text = {
                LOG_INFO:  "INFO",
                LOG_WARN:  "WARN",
                LOG_ERROR: "ERROR"
            }.get(level, "LOG")

            logfile.write(f"[{level_text}] {message}\n")
            logfile.flush()  # meteen schrijven naar SD
        except Exception as e:
            print("[WARN] Failed to write log to SD:", e)

    display.show()

# mount SD card and test if card is present
# if SD card is present & mounted, then return True
def connectSD():
    if cd.value() != 1:  #SD Kaart NOT inserted
        log(LOG_WARN, "No SD-card inserted!", "SD missing")
        return False
    
    try:
        log(LOG_INFO, "Mounting SD-Card")
        sd = sdcard.SDCard(spi, cs)
        vfs = os.VfsFat(sd)
        os.mount(vfs, "/sd")
        log(LOG_INFO, "SD Card successfully mounted")
        return True
    
    except Exception as e:    
        log(LOG_WARN, "SD-card module not connected", "SD nt conn")
        log(LOG_ERROR, f"SD mount error: {e}", "SD mount err")
        return False
    
# ===== Set version in log_filename =====
def get_next_version():
    sd_root = "/sd"
    version_path = f"{sd_root}/{VERSION_FILE_NAME}"
    
    # Controleer of version.txt bestaat
    if VERSION_FILE_NAME in os.listdir(sd_root):
        try:
            with open(version_path, "r") as f:
                version = int(f.read().strip())
        except:
            version = 0
    else:
        version = 0

    # Maak nieuwe logfile naam
    data_filename = f"{sd_root}/lora_v{version}.txt"
    log_filename = f"{sd_root}/log_v{version}.txt"

    # Verhoog versie en sla op
    with open(version_path, "w") as f:
        f.write(str(version + 1))

    return data_filename, log_filename
    
# ===== WIFI VERBINDING =====
def connectWiFi( SSID, PASSWORD, retries=5, timeout=10 ):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)  # reset interface
    wlan.active(True)
    
    attempt = 0
    while attempt < retries:
        log(LOG_INFO, "Connecting WiFi...", "WiFi conn")
        wlan.connect(SSID, PASSWORD)

        # Try to connect within timeout=10 sec to WiFi
        wait_time = 0
        while not wlan.isconnected() and wait_time < timeout:
            sleep(1)
            wait_time += 1

        if wlan.isconnected():
            log(LOG_INFO, f"WiFi connected IP {wlan.ifconfig()[0]}", "WiFi OK")
            return wlan

        attempt += 1
        log(LOG_WARN, f"WiFi failed ({attempt}/{retries})", "WiFi retry")
        sleep(2)
    
    log(LOG_ERROR, "WiFi connection failed", "WiFi FAIL")
    return None

# ===== MQTT VERBINDING =====
def connectMqtt(client_id, broker, topic, retries=5):
    client = MQTTClient(client_id, broker)

    attempt = 0
    while attempt < retries:
        try:
            log(LOG_INFO, "Connecting MQTT...", "MQTT conn")
            client.connect()
            log(LOG_INFO, f"MQTT connected topic {topic}", "MQTT OK")
            return client
        except Exception as e:
            attempt += 1
            log(LOG_WARN, f"MQTT failed ({attempt}/{retries}) {e}", "MQTT retry")
            sleep(2)

    log(LOG_ERROR, "MQTT connection failed", "MQTT FAIL")
    return None

# ===== LoRa connect ====
# initialise LoRA module
# 	bw125: bandwith = 125MHz
# 	Cr45 : Coding Rate = 4/5
# 	Sf2048: Spreading Factor = 2048 chips/symbol = SF12
#   these settings need to be equal between send and rcv
def loraConnect(retries=3):
    attempt = 0

    while attempt < retries:
        try:
            log(LOG_INFO, "Initializing LoRa...", "LoRa init")

            lora = LoRa(
                RFM95_SPIBUS,
                RFM95_INT,
                SERVER_ADDRESS,
                RFM95_CS,
                reset_pin=RFM95_RST,
                freq=RF95_FREQ,
                tx_power=RF95_POW,
                acks=False,
                modem_config=ModemConfig.Bw125Cr45Sf2048
            )

            log(LOG_INFO, "LoRa initialized", "LoRa OK")
            return lora

        except Exception as e:
            attempt += 1
            log(LOG_WARN, f"LoRa failed ({attempt}/{retries}) {e}", "LoRa retry")
            sleep(2)

    log(LOG_ERROR, "LoRa init failed", "LoRa FAIL")
    return None

# --- Publish message to MQTT broker ---
def mqttPublish(message):
    global client

    if client is None:
        return False

    try:
        client.publish(TOPIC, message)
        return True

    except Exception as e:
        log(LOG_WARN, "MQTT publish failed", str(e))

        try:
            client.connect()
            client.publish(TOPIC, message)
            return True
        except Exception as e:
            log(LOG_WARN, "MQTT reconnect failed", str(e))
            return False

    
# --- InitDisplay function ---
def initDisplay():
    try:
        display = SH1106(128, 64, i2c) 
        print("[INFO] OLED initialized")
        return display
    except Exception as e:
        print(f"[ERROR]: OLED display niet bereikbaar. Reason {e}")
        return None

# --- Start of SkyByte reveiver on display function ---
def display_start( display ):
    if display is None:
        return
    
    display.fill(0)
    display.rect(0, 0, 128, 64, 1)
    display.text( "SkyByte",   30, 10, 1)
    display.text( "Receiver",  25, 30, 1 )
    display.text( "Started !!!", 20, 40, 1 )
    display.show()
    sleep( 3 )
    
def display_startup_status(display, status_dict):
    if display is None:
        return

    display.fill(0)
    display.rect(0, 0, 128, 64, 1)
    display.text("SYSTEM STARTUP", 10, 4, 1)

    y = 15
    key_x = 5
    val_x = 90  # vaste kolom voor status
    for key, value in status_dict.items():
        display.text(f"{key}:", key_x, y, 1)
        display.text("OK" if value else "FAIL", val_x, y, 1)
        y += 10

    display.show()
 
# --- Error on display function ---
def display_error( display, message ):
    if display is None:
        return
    
    display.fill(0)
    display.rect(0, 0, 128, 20, 1)
    display.text(message, 2, 4, 1)
    display.show()

# --- show progress & transmission quality on display function ---
def display_progress(display, msgCount, rssi, snr ):
    if display is None:
        return
    
    msg1 = f"Data write {msgCount:3d}"
    msg2 = f"RSSI: {rssi:6.2f}"
    msg3 = f" SNR: {snr:6.2f}"
    display.fill(0)
    display.rect(0, 0, 128, 20, 1)
    display.text('Data rcv OK', 5, 4, 1)
    display.rect(0, 22, 128, 40, 1)
    display.text( msg1, 5, 26, 1)
    display.text( msg2, 5, 36, 1)
    display.text( msg3, 5, 46, 1)
    display.show()
    
# --- Startup function to validate proper startup of:
#   (1) OLED Display
#   (2) SD_Card module + SD-Card inserted
#   (3) WiFi connected
#   (4) MQTT connected
# The program will continue if any of these parts above fails
def startup():
    global logfile, display, client, sdOK, data_filename, log_filename
    
    print("\n=== SYSTEM STARTUP ===")
    status = {
        "OLED": False,
        "SD": False,
        "WiFi": False,
        "MQTT": False,
        "LoRa": False
    }
    
     # ---- OLED ----
    display = initDisplay()
    if display:
        status["OLED"] = True
        display_start(display)

    # ---- SD ----
    sdOK = connectSD()
    status["SD"] = sdOK
    
    data_filename = ""
    log_filename = ""
    if sdOK:
        # get version names of log and data files
        data_filename, log_filename = get_next_version()
        
        try:
            logfile = open(log_filename, "a")

            # Nu pas loggen!
            log(LOG_INFO, f"Logging to file: {log_filename}", "Log file")
            log(LOG_INFO, f"Writing data to: {data_filename}", "New datafile")

        except Exception as e:
            print(f"Could not open log file: {e}")

    # ---- WiFi ----
    wlan = connectWiFi(SSID, PASSWORD)
    if wlan:
        status["WiFi"] = True

    # ---- MQTT ----
    client = None
    if wlan:
        try:
            client = connectMqtt(CLIENT_ID, MQTT_BROKER, TOPIC)
            status["MQTT"] = True
        except:
            log(LOG_ERROR, "MQTT not connected", "ERR: MQTT")

    # ---- LoRa ----
    lora = loraConnect()
    if lora:
        status["LoRa"] = True

    # Toon overzicht op OLED
    display_startup_status(display, status)
    sleep(5)

    log(LOG_INFO, "Startup complete", "SYSTEM OK")

    return display, sdOK, wlan, client, lora, data_filename, log_filename

# --- Callback functie for received data ---
def on_recv(payload):
    global pending_records, sdOK, data_filename, display, client, msgCount

    message_sender = payload.header_from
    expected_len   = payload.header_flags
    message_bytes  = payload.message
    message_len    = len(message_bytes)
    message_rssi   = payload.rssi
    message_snr    = payload.snr

    if message_sender != CLIENT_ADDRESS:
        log(LOG_ERROR, f"Incoming message not from SkyByte but from {message_sender} !", "ERR: Snd-ID")
        return

    if message_len != expected_len:
        log(LOG_ERROR, f"Wrong length: received {message_len} expected {expected_len}", "ERR: len")
        return

    try:
        message_str = message_bytes.decode("utf-8")
    except Exception as e:
        log(LOG_ERROR, f"Decoding payload failed: {e}\tData: {message_bytes}", "ERR: decode")
        return

    fields = message_str.split(";")
    if len(fields) < 2:
        log(LOG_ERROR, f"Wrong format: {message_str}", "ERR: CSV")
        return

    # If M arrives, start a fresh group
    if message_str.startswith("M;"):
        pending_records = [message_str]
    elif len(pending_records) > 0:
        pending_records.append(message_str)
    else:
        log(LOG_WARN, "Received B or G without M, discarding\n", "ERR: order")
        return

    if len(pending_records) == 3:
        records = pending_records
        pending_records = []  # reset immediately to free memory

        types_ok = (
            records[0].startswith("M;") and
            records[1].startswith("B;") and
            records[2].startswith("G;")
        )

        if types_ok:
            msgCount += 1  # only count valid complete sets
            if sdOK:
                with open(data_filename, "a") as f:
                    for rec in records:
                        f.write(rec + "\n")
                log(LOG_INFO, f"msgCount {msgCount} correct and written to SD-card")
            else:
                log(LOG_WARN, f"msgCount {msgCount} correct but SD not available", "SD missing")

            mqtt_ok = 0
            for rec in records:
                json_str = rec_to_json(rec)
                if json_str and mqttPublish(json_str):
                    mqtt_ok += 1

            status = "MQTT OK" if mqtt_ok == len(records) else f"MQTT {mqtt_ok}/{len(records)}"
            log(LOG_INFO, f"msgCount {msgCount} {status}")

            for rec in records:
                print(f"[Lora] {rec}")

            display_progress(display, msgCount, message_rssi, message_snr)
        else:
            log(LOG_ERROR, f"msgCount {msgCount} incorrect format: {records}", "ERR: data")
            
        print("")


# --- Convert records to JSON format
def rec_to_json(rec):
    parts = rec.strip().split(";")
    if len(parts) == 0:
        return None
    record_type = parts[0]

    if record_type == "M":
        if len(parts) < 4:
            return None
        data = {"type": "M", "turnL": int(parts[1]), "turnR": int(parts[2]), "status": parts[3]}

    elif record_type == "B":
        if len(parts) < 4:
            return None
        data = {"type": "B", "temperature": float(parts[1]), "pressure": float(parts[2]), "altitude": float(parts[3])}

    elif record_type == "G":
        if len(parts) < 6:
            return None
        data = {"type": "G", "sats": int(parts[1]), "hdop": float(parts[2]), "lat": parts[3], "long": parts[4], "alt_gps": parts[5]}

    else:
        return None

    return ujson.dumps(data)
        
        
# --- Function to test if stop button is pressed
def check_stop():
    if stop_pin.value() == 0:
        return True
    else:
        return False
    
# --- Function to close everything properly ---
def close_all(display, client, logfile, lora):
    # Graceful shutdown
    log(LOG_INFO, "Shutdown requested by user", "Stopping")
        
    # Stop LoRa receiving
    if lora is not None:
        lora.set_mode_idle()   # stop RX mode
        lora.on_recv = None    # disable callback
        # sleep to handle possible interrupts
        sleep(0.2)

    # Close MQTT connection
    if client is not None:
        try:
            client.disconnect()
            log(LOG_INFO, "MQTT client disconnected", "MQTT closed")
        except:
            print("[INFO] Failed to disconnect MQTT client")

    # Close logfile
    if logfile is not None:
        try:
            logfile.close()
            print("[INFO] Log file closed")
        except:
            print("Failed to close logfile")

    # Reset display
    if display is not None:
        display.fill(0)
        display.text("Stopped", 30, 30, 1)
        display.show()

    print("[INFO] System stopped")

# ---------------------------
# main
# ---------------------------

buffered_records = {}  # key = msgCount, value = list of records

display, sdOK, wlan, client, lora, data_filename, log_filename = startup()

if lora:
    lora.on_recv = on_recv
    lora.set_mode_rx()

    log(LOG_INFO, "Entering main loop. Press Ctrl+C to stop.\n", "Running")
    
    try:
        while True:
            sleep(0.1)
            if check_stop(): # if button is pressed, stop gracefull
                close_all(display, client, logfile, lora)
                break
    except KeyboardInterrupt: # if Ctrl-C pressed, stop gracefull
        close_all(display, client, logfile, lora)

else:
    log(LOG_ERROR, "LoRa not available - system halted", "HALTED")
    while True:
        sleep(1)
