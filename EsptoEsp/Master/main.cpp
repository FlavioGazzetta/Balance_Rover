/********************************************************************
 * ESP-NOW MASTER – Wi-Fi UI bridge for Balance-Bot
 ********************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <string.h>

/* ─────────── SLAVE MAC: change to your slave’s MAC ─────────── */
uint8_t slaveMac[6] = { 0xE8, 0x68, 0xE7, 0x31, 0x0E, 0x50 };

/* ─────────── Packet shared with slave ─────────── */
typedef struct __attribute__((packed))
{
  bool  isSet;
  char  act[16];
  float val;
} Packet;

/* ─────────── Globals ─────────── */
Packet  txPkt;
String  latestStatusJSON;
WebServer server(80);

/* ─────────── HTML / JS UI (unchanged markup) ─────────── */
const char HOMEPAGE[] PROGMEM = R"====(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Balance-Bot Controller</title>
<style>
  /* minimal CSS (same as before) */
  body{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",
       Roboto,Helvetica,Arial,sans-serif;background:#f4f5f7;display:flex;
       justify-content:center;padding:20px}
  .card{background:#fff;border-radius:12px;box-shadow:0 4px 12px rgba(0,0,0,.1);
        max-width:500px;width:100%;padding:20px}
  h1{margin:0;font-size:1.4rem;text-align:center;color:#333}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(3,1fr);
       gap:12px;justify-items:center;align-items:center;margin:20px 0}
  .pad button{width:60px;height:60px;border:none;font-size:1.2rem;font-weight:600;
              border-radius:50%;background:#e0e2e5;color:#333;cursor:pointer;
              transition:background .1s,transform .1s;user-select:none}
  .pad button:active,.pad button.pressed{background:#3f51b5;color:#fff;transform:scale(.96)}
  form{display:flex;flex-direction:column;gap:10px;text-align:center;margin-bottom:20px}
  input[type=number]{padding:8px;border:1px solid #ccc;border-radius:8px;width:100px;
       margin:0 auto;font-size:1rem;text-align:center}
  input[type=submit]{padding:10px 0;border:none;border-radius:8px;background:#3f51b5;
       color:#fff;font-size:1rem;font-weight:600;cursor:pointer;transition:background .1s}
  input[type=submit]:hover{background:#303f9f}
  table{width:100%;border-collapse:collapse;margin-top:10px}
  th,td{padding:8px 6px;text-align:left;border-bottom:1px solid #ddd}
  th{background:#f1f3f7;color:#444;font-weight:600}
  td.value{font-weight:500;color:#222}
</style>

<script>
function send(cmd){fetch("/cmd?act="+cmd)}
function bind(id,start,stop){
  const el=document.getElementById(id);let rpt=null;
  const down=e=>{e.preventDefault();el.classList.add("pressed");
                 send(start);rpt=setInterval(()=>send(start),200)};
  const up  =e=>{e.preventDefault();el.classList.remove("pressed");
                 clearInterval(rpt);rpt=null;send(stop)};
  el.addEventListener("pointerdown",down,{passive:false});
  ["pointerup","pointercancel","pointerleave"].forEach(ev=>
        el.addEventListener(ev,up,{passive:false}));
}

async function poll(){
  try{
    const r=await fetch("/status"); if(!r.ok)return;
    const d=await r.json();
    /* numeric fields */
    const num=["tiltSP","theta","posEst","w1pos","w2pos","sp1","sp2",
               "desPos","diff","heading","avgSpeed","desiredHeading"];
    num.forEach(k=>{if(k in d)document.getElementById(kMap[k]).innerText=d[k].toFixed(3)});
    /* ints / bools */
    ["L","R","fallen"].forEach(k=>{
      if(k in d)document.getElementById(k).innerText=d[k];
    });
  }catch(_){}
}

const kMap={tiltSP:"tiltSP",theta:"theta",posEst:"posEst",w1pos:"w1pos",
            w2pos:"w2pos",sp1:"sp1",sp2:"sp2",desPos:"desPos",diff:"diff",
            heading:"heading",desiredHeading:"desHeading",avgSpeed:"avgSpeed"};

window.addEventListener("load",()=>{
  bind("up","up_start","up_stop");
  bind("down","down_start","down_stop");
  bind("left","left_start","left_stop");
  bind("right","right_start","right_stop");
  setInterval(poll,500);

  document.getElementById("posForm").addEventListener("submit",async e=>{
    e.preventDefault();
    const v=document.getElementById("posInput").value;
    const ack=document.getElementById("posAck");
    try{
      const r=await fetch("/set?val="+encodeURIComponent(v));
      ack.innerText=r.ok?"✔︎":"✘";
    }catch(_){ack.innerText="✘"}
    setTimeout(()=>ack.innerText="",1200);
  });
});
</script>
</head>
<body>
<div class="card">
  <h1>Balance-Bot Remote Controller</h1>

  <div class="pad">
    <div></div><button id="up">↑</button><div></div>
    <button id="left">←</button><div></div><button id="right">→</button>
    <div></div><button id="down">↓</button><div></div>
  </div>

  <form id="posForm">
    <label>Desired position:
      <input id="posInput" type="number" step="0.01" name="val" value="0.00">
    </label>
    <input type="submit" value="Submit">
    <span id="posAck" style="margin-left:8px;color:green;font-size:0.9em;"></span>
  </form>

  <table>
    <thead><tr><th>Parameter</th><th>Value</th></tr></thead><tbody>
      <tr><td>tiltSP</td> <td class="value" id="tiltSP">0.000</td></tr>
      <tr><td>θ (theta)</td><td class="value" id="theta">0.000</td></tr>
      <tr><td>posEst</td> <td class="value" id="posEst">0.000</td></tr>
      <tr><td>w1pos</td>  <td class="value" id="w1pos">0.000</td></tr>
      <tr><td>w2pos</td>  <td class="value" id="w2pos">0.000</td></tr>
      <tr><td>sp1</td>    <td class="value" id="sp1">0.000</td></tr>
      <tr><td>sp2</td>    <td class="value" id="sp2">0.000</td></tr>
      <tr><td>desPos</td> <td class="value" id="desPos">0.000</td></tr>
      <tr><td>LeftMode</td><td class="value" id="L">0</td></tr>
      <tr><td>RightMode</td><td class="value" id="R">0</td></tr>
      <tr><td>diff</td>   <td class="value" id="diff">0.000</td></tr>
      <tr><td>fallen</td> <td class="value" id="fallen">0</td></tr>
      <tr><td>heading (rad)</td><td class="value" id="heading">0.000</td></tr>
      <tr><td>desiredHeading (rad)</td><td class="value" id="desHeading">0.000</td></tr>
      <tr><td>avgSpeed</td><td class="value" id="avgSpeed">0.000</td></tr>
    </tbody>
  </table>
</div>
</body></html>
)====";

/* ─────────── ESP-NOW CALLBACKS ─────────── */
void onMasterSend(const uint8_t*, esp_now_send_status_t s){
  Serial.printf("ESP-NOW send: %s\n", s==ESP_NOW_SEND_SUCCESS ? "OK":"FAIL");
}
void onMasterRecv(const uint8_t*, const uint8_t* d, int len){
  latestStatusJSON = String((const char*)d, len);
}

/* ─────────── HTTP HANDLERS ─────────── */
void handleRoot(){ server.send_P(200,"text/html",HOMEPAGE); }
void handleStatus(){
  if(latestStatusJSON.length())
       server.send(200,"application/json",latestStatusJSON);
  else server.send(204,"application/json","");
}
void handleSet(){
  if(!server.hasArg("val")){server.send(400);return;}
  txPkt.isSet=true; txPkt.val=server.arg("val").toFloat();
  txPkt.act[0]='\0';
  esp_now_send(slaveMac,(uint8_t*)&txPkt,sizeof(txPkt));
  server.send(200,"text/plain","OK");
}
void handleCmd(){
  if(!server.hasArg("act")){server.send(400);return;}
  txPkt.isSet=false;
  strncpy(txPkt.act,server.arg("act").c_str(),sizeof(txPkt.act)-1);
  txPkt.act[sizeof(txPkt.act)-1]='\0';
  txPkt.val=0;
  esp_now_send(slaveMac,(uint8_t*)&txPkt,sizeof(txPkt));
  server.send(200,"text/plain","OK");
}

/* ─────────── SETUP ─────────── */
void setup(){
  Serial.begin(115200);

  /* 1. Access-Point + WebServer */
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ESP32_BalanceBot","12345678");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  Serial.print("STA  MAC: ");              // station-interface MAC
  Serial.println(WiFi.macAddress());

  Serial.print("SoftAP MAC: ");            // AP-interface MAC (if you ever use softAP)
  Serial.println(WiFi.softAPmacAddress());

  server.on("/",handleRoot);
  server.on("/status",handleStatus);
  server.on("/set",handleSet);
  server.on("/cmd",handleCmd);
  server.begin();

  /* 2. ESP-NOW */
  if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW init failed");while(1)delay(1);}
  esp_now_register_send_cb(onMasterSend);
  esp_now_register_recv_cb(onMasterRecv);

  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr,slaveMac,6);
  peer.channel=0; peer.encrypt=false; esp_now_add_peer(&peer);
}

/* ─────────── LOOP ─────────── */
void loop(){
  server.handleClient();
  delay(10);               // yield
}
