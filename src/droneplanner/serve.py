"""Interactive route explorer — a local web app for playing with the planner.

Runs the *real* M1 planner behind a tiny stdlib HTTP server (no web framework
dependency). The browser page lets you drag observers and the start/end, move the
objective-weight sliders, and switch display layers; every change re-plans the
concealment-aware route on the built-in demo terrain and re-renders. This is the
"see the algorithm in action" surface (Software-Spec §6.6, made interactive).

    droneplanner serve            # then open http://127.0.0.1:8765
"""

from __future__ import annotations

import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from droneplanner.demo import demo_flyability, demo_mission, demo_terrain
from droneplanner.logging_config import get_logger
from droneplanner.model.geometry import Coordinate
from droneplanner.model.mission import Mission, ThreatModel, Weights
from droneplanner.planner.corridor import plan_over_terrain
from droneplanner.planner.theta_star import PlanningError
from droneplanner.report.build import build_report
from droneplanner.viz import LAYERS, render_interactive

_log = get_logger(__name__)
_RENDER_LOCK = threading.Lock()
_IMAGE_PX = 620


class _Scene:
    """Cached demo terrain + base mission shared across requests."""

    def __init__(self, size: int) -> None:
        self.dtm, self.dsm, self.grid = demo_terrain(size=size)
        self.base = demo_mission(self.grid)
        # Fixed forest flyability (terrain-only, independent of observers/weights).
        self.flyability = demo_flyability(self.grid, self.base.drone)

    def extent(self) -> list[float]:
        a, _, c, _, e, f = self.grid.transform
        return [c, c + a * self.grid.cols, f + e * self.grid.rows, f]


def _mission_from_params(scene: _Scene, params: dict[str, Any]) -> Mission:
    start = Coordinate(x=float(params["start"][0]), y=float(params["start"][1]))
    end = Coordinate(x=float(params["end"][0]), y=float(params["end"][1]))
    observers = tuple(Coordinate(x=float(o[0]), y=float(o[1])) for o in params.get("observers", []))
    w = params.get("weights", {})
    weights = Weights(
        exposure=float(w.get("exposure", 1.0)),
        cover=float(w.get("cover", 1.0)),
        survivability=float(w.get("survivability", 1.0)),
        snag=float(w.get("snag", 0.5)),
    )
    updated = scene.base.model_copy(
        update={
            "start": start,
            "end": end,
            "threat": ThreatModel(observers=observers),
            "weights": weights,
        }
    )
    # model_copy skips validation; re-validate so invalid input (e.g. coincident
    # start/end) is caught and surfaced as an error rather than silently planned.
    return Mission.model_validate(updated.model_dump())


def _plan_response(scene: _Scene, params: dict[str, Any]) -> dict[str, Any]:
    layer = params.get("layer", "exposure")
    if layer not in LAYERS:
        layer = "exposure"
    try:
        mission = _mission_from_params(scene, params)
        with _RENDER_LOCK:
            plan, surface = plan_over_terrain(
                mission, scene.dtm, scene.dsm, scene.grid, flyability=scene.flyability
            )
            png = render_interactive(mission, surface, plan, layer=layer, size_px=_IMAGE_PX)
        report = build_report(mission, plan, concealment_analyzed=True)
        scores = plan.objective_scores
        return {
            "image": "data:image/png;base64," + base64.b64encode(png).decode("ascii"),
            "extent": scene.extent(),
            "size_px": _IMAGE_PX,
            "stats": {
                "length_m": round(plan.total_length_m, 1),
                "fiber_used_m": round(plan.fiber_used_m, 1),
                "fiber_available_m": mission.spool.length_m,
                "mean_exposure": round(scores.get("mean_exposure", 0.0), 3),
                "max_exposure": round(scores.get("max_exposure", 0.0), 3),
                "exposed_fraction": round(scores.get("exposed_fraction", 0.0), 3),
                "recommendation": report.recommendation.value,
            },
            "error": None,
        }
    except (PlanningError, ValueError) as exc:
        return {"error": str(exc)}


def _defaults(scene: _Scene) -> dict[str, Any]:
    m = scene.base
    return {
        "start": [m.start.x, m.start.y],
        "end": [m.end.x, m.end.y],
        "observers": [[o.x, o.y] for o in m.threat.observers],
        "weights": {
            "exposure": m.weights.exposure,
            "cover": m.weights.cover,
            "survivability": m.weights.survivability,
            "snag": m.weights.snag,
        },
        "extent": scene.extent(),
        "size_px": _IMAGE_PX,
        "layers": list(LAYERS),
    }


def make_handler(scene: _Scene) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args: Any) -> None:  # keep the console quiet
            pass

        def _send_json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                body = INDEX_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/defaults":
                self._send_json(_defaults(scene))
            else:
                self.send_error(404)

        def do_POST(self) -> None:
            if self.path != "/plan":
                self.send_error(404)
                return
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                params = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                self._send_json({"error": "invalid JSON"}, status=400)
                return
            self._send_json(_plan_response(scene, params))

    return Handler


def serve(host: str = "127.0.0.1", port: int = 8765, size: int = 130) -> None:
    """Start the interactive explorer (blocking)."""
    scene = _Scene(size=size)
    # Warm the numpy/matplotlib import + JIT paths so the first click is snappy.
    _plan_response(scene, _defaults(scene))
    httpd = ThreadingHTTPServer((host, port), make_handler(scene))
    _log.info("droneplanner explorer on http://%s:%d  (Ctrl-C to stop)", host, port)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:  # pragma: no cover
        _log.info("shutting down")
    finally:
        httpd.server_close()


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>droneplanner — route explorer</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font-family: -apple-system, system-ui, sans-serif; background:#0d1117; color:#e6edf3; }
  header { padding: 12px 18px; border-bottom: 1px solid #21262d; }
  header h1 { font-size: 16px; margin: 0; }
  header p { margin: 4px 0 0; font-size: 12px; color:#8b949e; }
  .wrap { display:flex; gap:18px; padding:18px; flex-wrap:wrap; }
  .stage { position:relative; }
  #viz { width:620px; height:620px; border:1px solid #30363d; border-radius:6px; cursor:crosshair; background:#161b22; }
  .panel { width:320px; }
  .card { background:#161b22; border:1px solid #21262d; border-radius:8px; padding:14px; margin-bottom:14px; }
  .card h2 { font-size:12px; text-transform:uppercase; letter-spacing:.04em; color:#8b949e; margin:0 0 10px; }
  button { background:#21262d; color:#e6edf3; border:1px solid #30363d; border-radius:6px; padding:6px 10px; font-size:13px; cursor:pointer; }
  button:hover { background:#30363d; }
  button.active { background:#1f6feb; border-color:#1f6feb; }
  .row { display:flex; gap:6px; flex-wrap:wrap; align-items:center; margin-bottom:8px; }
  label { font-size:12px; color:#c9d1d9; }
  input[type=range] { width:150px; vertical-align:middle; }
  select { background:#21262d; color:#e6edf3; border:1px solid #30363d; border-radius:6px; padding:5px; }
  .slider-row { display:flex; justify-content:space-between; align-items:center; margin-bottom:6px; }
  .slider-row .val { font-variant-numeric:tabular-nums; color:#58a6ff; font-size:12px; width:32px; text-align:right; }
  table.stats { width:100%; border-collapse:collapse; font-size:13px; }
  table.stats td { padding:3px 0; }
  table.stats td:last-child { text-align:right; font-variant-numeric:tabular-nums; }
  .rec { font-weight:700; padding:2px 8px; border-radius:4px; }
  .rec.GO { background:#238636; } .rec.REVIEW { background:#9e6a03; } .rec\.NO-GO { background:#da3633; }
  .hint { font-size:11px; color:#8b949e; margin-top:8px; line-height:1.5; }
  #err { color:#ff7b72; font-size:12px; min-height:16px; }
  #busy { position:absolute; top:8px; left:8px; background:#1f6feb; color:#fff; font-size:11px; padding:2px 8px; border-radius:10px; opacity:0; transition:opacity .15s; }
  #busy.on { opacity:.9; }
</style>
</head>
<body>
<header>
  <h1>droneplanner — interactive route explorer <span style="color:#8b949e;font-weight:400;">(M1, demo terrain)</span></h1>
  <p>Cyan = concealment-aware route · dashed = naive straight line · red ▲ = observers · green ● start · yellow ■ end. Click the map to move the active point.</p>
</header>
<div class="wrap">
  <div class="stage">
    <img id="viz" alt="route"/>
    <div id="busy">computing…</div>
  </div>
  <div class="panel">
    <div class="card">
      <h2>Active point (click map to move)</h2>
      <div class="row" id="points"></div>
      <div class="row">
        <button id="addObs">+ observer</button>
        <button id="delObs">&minus; observer</button>
      </div>
    </div>
    <div class="card">
      <h2>Display layer</h2>
      <select id="layer"></select>
    </div>
    <div class="card">
      <h2>Objective weights</h2>
      <div id="sliders"></div>
    </div>
    <div class="card">
      <h2>Result</h2>
      <table class="stats"><tbody id="stats"></tbody></table>
      <div id="err"></div>
    </div>
    <div class="hint">The planner minimizes an any-angle least-cost path (cost-aware Theta*) over a composite of viewshed exposure, terrain (TPI/ridge), survivability and snag. Raising <b>exposure</b> pushes the route into cover; raising <b>cover</b> pulls it into valleys/defilade.</div>
  </div>
</div>
<script>
const WEIGHTS = ["exposure","cover","survivability","snag"];
let S = null;         // {start,end,observers,weights,extent,size_px,layers}
let active = "start"; // active control point id
let busyTimer = null;

const $ = id => document.getElementById(id);
function busy(on){ $("busy").classList.toggle("on", on); }

async function init(){
  S = await (await fetch("/defaults")).json();
  const sel = $("layer");
  S.layers.forEach(l => { const o=document.createElement("option"); o.value=l; o.textContent=l; sel.appendChild(o); });
  sel.onchange = replan;
  const sl = $("sliders");
  WEIGHTS.forEach(w => {
    const row=document.createElement("div"); row.className="slider-row";
    row.innerHTML = `<label>${w}</label><input type="range" min="0" max="3" step="0.1" id="w_${w}" value="${S.weights[w]}"><span class="val" id="v_${w}">${(+S.weights[w]).toFixed(1)}</span>`;
    sl.appendChild(row);
    setTimeout(()=>{ const i=$("w_"+w); i.addEventListener("input",()=>{ $("v_"+w).textContent=(+i.value).toFixed(1); debounced(); }); },0);
  });
  $("addObs").onclick = ()=>{ const c=center(); S.observers.push(c); active="obs"+(S.observers.length-1); renderPoints(); replan(); };
  $("delObs").onclick = ()=>{ if(S.observers.length){ S.observers.pop(); if(active.startsWith("obs")) active="start"; renderPoints(); replan(); } };
  $("viz").addEventListener("click", onMapClick);
  renderPoints();
  replan();
}

function center(){ const e=S.extent; return [(e[0]+e[1])/2,(e[2]+e[3])/2]; }
function renderPoints(){
  const p=$("points"); p.innerHTML="";
  const ids=["start","end"].concat(S.observers.map((_,i)=>"obs"+i));
  ids.forEach(id=>{ const b=document.createElement("button"); b.textContent=label(id); b.className=(id===active?"active":""); b.onclick=()=>{active=id; renderPoints();}; p.appendChild(b); });
}
function label(id){ if(id==="start")return "start ●"; if(id==="end")return "end ■"; return "obs "+(+id.slice(3)+1)+" ▲"; }

function onMapClick(ev){
  const r=ev.target.getBoundingClientRect();
  const px=(ev.clientX-r.left)/r.width, py=(ev.clientY-r.top)/r.height;
  const e=S.extent; // [left,right,bottom,top]
  const x=e[0]+px*(e[1]-e[0]);
  const y=e[3]-py*(e[3]-e[2]);
  if(active==="start") S.start=[x,y];
  else if(active==="end") S.end=[x,y];
  else { const i=+active.slice(3); if(S.observers[i]) S.observers[i]=[x,y]; }
  replan();
}

function readWeights(){ const w={}; WEIGHTS.forEach(k=>w[k]=+$("w_"+k).value); return w; }
let t=null; function debounced(){ clearTimeout(t); t=setTimeout(replan,140); }

async function replan(){
  busy(true);
  const body={start:S.start,end:S.end,observers:S.observers,weights:readWeights(),layer:$("layer").value};
  let res;
  try { res=await (await fetch("/plan",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)})).json(); }
  catch(e){ $("err").textContent="request failed"; busy(false); return; }
  if(res.error){ $("err").textContent=res.error; busy(false); return; }
  $("err").textContent="";
  $("viz").src=res.image;
  const s=res.stats;
  $("stats").innerHTML =
    `<tr><td>Recommendation</td><td><span class="rec ${s.recommendation}">${s.recommendation}</span></td></tr>`+
    `<tr><td>Route length</td><td>${s.length_m.toLocaleString()} m</td></tr>`+
    `<tr><td>Fiber used / avail</td><td>${s.fiber_used_m.toLocaleString()} / ${s.fiber_available_m.toLocaleString()} m</td></tr>`+
    `<tr><td>Mean exposure</td><td>${s.mean_exposure}</td></tr>`+
    `<tr><td>Max exposure</td><td>${s.max_exposure}</td></tr>`+
    `<tr><td>Exposed fraction</td><td>${(s.exposed_fraction*100).toFixed(0)}%</td></tr>`;
  busy(false);
}
init();
</script>
</body>
</html>
"""
