async function refreshState() {
  try {
    const res = await fetch("/state.json", { cache: "no-store" });
    const data = await res.json();
    document.getElementById("state-line").textContent = data.state_text || "no state";
    document.getElementById("fsm").textContent = data.fsm || "UNKNOWN";
    document.getElementById("fresh").textContent = data.freshness || "no data";
    const dbg = data.debug || [];
    setMetric("vx", dbg[2]);
    setMetric("vy", dbg[3]);
    setMetric("omega", dbg[4]);
    setMetric("pos", dbg[5]);
    setMetric("yaw", radToDeg(dbg[6]), " deg");
    setMetric("score", dbg[10]);
  } catch (err) {
    document.getElementById("state-line").textContent = "web node is waiting for ROS topics";
  }
}

function attachStreams() {
  for (const id of ["wide", "body"]) {
    const img = document.getElementById(id);
    const stream = img.dataset.stream;
    if (!stream || img.src) {
      continue;
    }
    img.src = `${stream}?t=${Date.now()}`;
  }
}

function setMetric(id, value, suffix = "") {
  const el = document.getElementById(id);
  const n = Number(value || 0);
  el.textContent = `${n.toFixed(suffix ? 1 : 3)}${suffix}`;
}

function radToDeg(v) {
  return Number(v || 0) * 180 / Math.PI;
}

setInterval(refreshState, 250);
refreshState();
window.addEventListener("load", () => {
  window.setTimeout(attachStreams, 50);
});
