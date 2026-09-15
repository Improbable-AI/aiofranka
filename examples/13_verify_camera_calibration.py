#!/usr/bin/env python3
"""Overlay the calibrated MuJoCo robot and AprilCube on recorded camera images.

Run offline, including over SSH (no robot or camera connection)::

    python examples/13_verify_camera_calibration.py \
        --session camera_calibration/RUN

Images, four-panel comparisons, a matrix summary, and a report are saved under
SESSION/verification. calibration_summary.jpg shows the view with highest RMS
beside the fitted camera transform and calibration scores.
Add --serve for an optional browser with opacity, edges, zoom, and tag residuals;
--serve-only reopens an existing export. Stop serving with Ctrl+C. The web address
defaults to 0.0.0.0:8081, separate from the collection UI.

Rendering uses measured joints, robot FK, and the saved T_base_camera/T_ee_cube.
There is no image alignment, per-frame pose adjustment, or calibration refit.
Real images are undistorted into the same pinhole intrinsics as the renderer.
These overlays check the final fit, which used all recorded views. The original
held-out score is shown separately; image agreement does not establish absolute
physical accuracy. Only geometry present in the supplied meshes is rendered.

Headless rendering defaults to MUJOCO_GL=egl. Set MUJOCO_GL=osmesa before launch
for software rendering on machines with OSMesa installed.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from functools import partial
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import os
from pathlib import Path
import socket
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


HTML = """<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibration overlay review</title><style>
*{box-sizing:border-box}body{margin:0;background:#11171e;color:#ecf3fa;font:15px system-ui,sans-serif}
main{max-width:1600px;margin:auto;padding:24px}h1{margin:0;font-size:27px}p{color:#abbacc;line-height:1.5}
.toolbar{display:flex;gap:16px;align-items:center;flex-wrap:wrap;margin:18px 0}
button,select{font:inherit;color:inherit;background:#263547;border:1px solid #3a5068;border-radius:7px;padding:9px 13px;cursor:pointer}
label{display:flex;align-items:center;gap:8px}input{accent-color:#65b3ff}#viewport{overflow:auto;max-height:76vh;background:#090d12;border:1px solid #304055;border-radius:10px}
canvas{display:block}#details{padding:12px 0;color:#b4c6da}#summary{color:#94c4ff}.note{font-size:13px}
a{color:#88c6ff}.legend{display:flex;gap:20px;font-size:13px}.cyan{color:#43eaff}.pink{color:#ff61d8}
.review{display:grid;grid-template-columns:minmax(0,1fr) 375px;gap:20px;align-items:start}
.matrices{background:#1b2734;border:1px solid #304055;border-radius:10px;padding:18px}
.matrices h2{font-size:18px;margin:0 0 8px}.matrices pre{font:12px/1.8 monospace;overflow:auto;color:#d9ebff}
.matrices p{font-size:13px;margin:8px 0 20px}.matrices p:last-child{margin-bottom:0}
@media(max-width:1000px){.review{grid-template-columns:1fr}.matrices{max-width:500px}}
@media(max-width:700px){main{padding:14px}.toolbar{gap:10px}h1{font-size:23px}}
</style><main><h1>Calibration overlay review</h1><p id="summary">Loading recorded session…</p>
<div class="toolbar"><button id="prev">← Previous</button><select id="view"></select><button id="next">Next →</button>
<select id="mode"><option value="blend">Blend</option><option value="edges">Simulated edges</option><option value="split">Side by side</option><option value="real">Recorded image</option><option value="sim">Simulation</option></select>
<label>Simulation <input id="alpha" type="range" min="0" max="100" value="45"><span id="alphaValue">45%</span></label>
<label>Zoom <input id="zoom" type="range" min="100" max="300" value="100"></label>
<label><input id="corners" type="checkbox">Tag residuals</label></div>
<div class="review"><div><div id="viewport"><canvas id="canvas"></canvas></div><div id="details"></div></div>
<aside class="matrices"><h2>T_base_camera</h2><p>Camera coordinates → robot base</p><pre id="cameraMatrix"></pre>
<h2>T_ee_cube</h2><p>Cube coordinates → end-effector frame</p><pre id="mountMatrix"></pre>
<p>Translations are in meters. Camera axes point right, down, and forward.</p>
<p><a href="calibration_summary.jpg" download>Download image + matrix summary</a></p></aside></div>
<div class="legend"><span class="cyan">Cyan circles: observed corners</span><span class="pink">Magenta crosses: projected corners / simulated edges</span></div>
<p class="note">The final fit uses all recorded views. The robot and target use recorded joints and the saved fixed camera/mount transforms.
Only the meshes are overlaid; lighting, cables, and unmodeled hardware may differ. Tag residuals are shown at their true pixel scale.</p>
<p><a id="overlay" download>Download overlay</a> · <a href="calibration_summary.jpg">Image + matrix summary</a> · <a href="contact_sheet.jpg">All views</a> · <a href="report.json" download>Download report</a></p>
</main><script>
const el=id=>document.getElementById(id),canvas=el('canvas'),ctx=canvas.getContext('2d');let report,index=0,images={},generation=0;
const loadImage=src=>new Promise((resolve,reject)=>{const image=new Image();image.onload=()=>resolve(image);image.onerror=reject;image.src=src;});
function draw(){if(!images.real)return;const v=report.views[index],w=report.width,h=report.height,mode=el('mode').value;
canvas.width=mode==='split'?w*2:w;canvas.height=h;const width=el('viewport').clientWidth*Number(el('zoom').value)/100;
canvas.style.width=width+'px';canvas.style.height=(width*h/canvas.width)+'px';
ctx.globalAlpha=1;ctx.drawImage(mode==='sim'?images.sim:mode==='edges'?images.edges:images.real,0,0);
if(mode==='blend'){ctx.globalAlpha=Number(el('alpha').value)/100;ctx.drawImage(images.rgba,0,0);ctx.globalAlpha=1;}
if(mode==='split')ctx.drawImage(images.sim,w,0);
if(el('corners').checked){for(let i=0;i<v.observed_px.length;i++){const a=v.observed_px[i],b=v.projected_px[i];
ctx.lineWidth=1;ctx.strokeStyle='#ff61d8';ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
ctx.beginPath();ctx.moveTo(b[0]-3,b[1]);ctx.lineTo(b[0]+3,b[1]);ctx.moveTo(b[0],b[1]-3);ctx.lineTo(b[0],b[1]+3);ctx.stroke();
ctx.strokeStyle='#43eaff';ctx.beginPath();ctx.arc(a[0],a[1],2,0,2*Math.PI);ctx.stroke();}}
el('alphaValue').textContent=el('alpha').value+'%';}
async function select(i){index=(i+report.views.length)%report.views.length;el('view').value=index;const token=++generation,v=report.views[index];
el('details').textContent='Loading view '+v.id+'…';try{const loaded=await Promise.all(['real','sim','rgba','edges'].map(k=>loadImage(v.files[k])));
if(token!==generation)return;images=Object.fromEntries(['real','sim','rgba','edges'].map((k,j)=>[k,loaded[j]]));draw();
el('details').textContent='View '+v.id+' · final-fit RMS '+v.rms_px.toFixed(3)+' px · '+v.tag_count+' tags · '+v.source_image;
el('overlay').href=v.files.overlay;}catch(e){el('details').textContent='Could not load exported images.';}}
el('prev').onclick=()=>select(index-1);el('next').onclick=()=>select(index+1);el('view').onchange=()=>select(Number(el('view').value));
for(const id of ['mode','alpha','zoom','corners'])el(id).oninput=draw;window.onresize=draw;
document.addEventListener('keydown',e=>{if(/INPUT|SELECT/.test(e.target.tagName)||!report)return;if(e.key==='ArrowLeft')select(index-1);if(e.key==='ArrowRight')select(index+1);});
fetch('report.json').then(r=>r.json()).then(r=>{report=r;el('summary').textContent=r.session_name+' · '+r.views.length+' views · final-fit RMS '+r.rms_px.toFixed(3)+' px'+
(r.original_held_out_rms_px===null?'':' · original held-out RMS '+r.original_held_out_rms_px.toFixed(3)+' px');
const matrixText=m=>m.map(row=>'[ '+row.map(v=>(v>=0?' ':'')+v.toFixed(6)).join('  ')+' ]').join(String.fromCharCode(10));
el('cameraMatrix').textContent=matrixText(r.T_base_camera);el('mountMatrix').textContent=matrixText(r.T_ee_cube);
for(let i=0;i<r.views.length;i++){const v=r.views[i],o=document.createElement('option');o.value=i;o.textContent='View '+v.id+' — '+v.rms_px.toFixed(2)+' px';el('view').appendChild(o);}
select(r.views.reduce((best,v,i)=>v.rms_px>r.views[best].rms_px?i:best,0));}).catch(e=>{el('summary').textContent='Could not load report.json. Serve this directory over HTTP.';});
</script></html>"""


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(f"Could not write {path}")


def save_comparison(path, real, simulated, overlay, edges):
    tiles = []
    for title, image in (("RECORDED IMAGE", real), ("MUJOCO RENDER", simulated),
                         ("BLENDED OVERLAY", overlay), ("SIMULATED EDGES (MAGENTA)", edges)):
        tile = cv2.resize(image, (640, round(640 * image.shape[0] / image.shape[1])))
        tile = cv2.copyMakeBorder(tile, 28, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 25, 32))
        cv2.putText(tile, title, (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 245, 245), 1)
        tiles.append(tile)
    comparison = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    save_image(path, comparison)
    return comparison


def save_summary(path, comparison, report, view):
    """Save a standalone comparison and transform panel, with no browser needed."""
    height = max(comparison.shape[0], 730)
    panel_width, x = 440, comparison.shape[1] + 24
    summary = np.full((height, comparison.shape[1] + panel_width, 3), (20, 25, 32), dtype=np.uint8)
    summary[:comparison.shape[0], :comparison.shape[1]] = comparison

    def text(message, y, scale=0.48, color=(205, 216, 231)):
        cv2.putText(summary, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

    def matrix(value, y):
        for row in np.asarray(value):
            text("[ " + "  ".join(f"{number: .6f}" for number in row) + " ]", y, 0.44)
            y += 27

    text("CAMERA CALIBRATION", 35, 0.72, (245, 245, 245))
    text(report["session_name"][:42], 65)
    text(f"View {view['id']} - highest view RMS", 105)
    text(f"View RMS: {view['rms_px']:.3f} px", 135)
    text(f"All-view fit: {report['rms_px']:.3f} px RMS", 165)
    held_out = report["original_held_out_rms_px"]
    text("Held-out RMS: " + (f"{held_out:.3f} px" if held_out is not None else "unavailable"), 195)
    text("T_base_camera", 245, 0.65, (255, 210, 150))
    text("Camera coordinates -> robot base", 272)
    matrix(report["T_base_camera"], 310)
    text("T_ee_cube", 440, 0.65, (255, 210, 150))
    text("Cube coordinates -> end-effector", 467)
    matrix(report["T_ee_cube"], 505)
    text("Translations: meters", 627)
    text("Camera axes: right / down / forward", 654)
    text("Overlays use the final fit on all views.", 691, 0.43)
    text("Held-out score uses a separate training fit.", 715, 0.43)
    save_image(path, summary)


def render_review(args, progress=None):
    """Export saved-pose overlays; optionally report each completed view as text."""
    # Set the backend before importing MuJoCo, including on SSH hosts without DISPLAY.
    os.environ.setdefault("MUJOCO_GL", "egl")
    # File-path loading also works when this example is imported by the capture UI.
    renderer_path = Path(__file__).with_name("camera_calibration_render.py")
    spec = importlib.util.spec_from_file_location("camera_calibration_render", renderer_path)
    rendering = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rendering)
    CalibrationRenderer = rendering.CalibrationRenderer

    session = args.session.expanduser().resolve()
    calibration_path = (args.calibration or session / "calibration.json").expanduser().resolve()
    views_path = session / "views.json"
    dataset = json.loads(views_path.read_text())
    calibration = json.loads(calibration_path.read_text())
    if dataset.get("schema_version") != 1 or dataset.get("setup") != "fixed_camera_robot_held_cube":
        raise ValueError("Expected a session from 12_camera_calibration.py.")
    if calibration.get("dataset_sha256") != digest(views_path):
        raise ValueError("Calibration does not match this views.json dataset.")
    robot_xml, cube_xml = args.robot_xml.expanduser().resolve(), args.cube_xml.expanduser().resolve()
    if any(d.get("robot_model_sha256") != digest(robot_xml) for d in (dataset, calibration)):
        raise ValueError("Robot XML differs from the model used for calibration.")
    cube_config_path = cube_xml.parent.parent / "config.json"
    if not cube_config_path.is_file() or json.loads(cube_config_path.read_text()) != dataset["cube_config"]:
        raise ValueError("Cube mesh's neighboring config.json must match the captured cube configuration.")
    camera = dataset["camera"]
    width, height = camera["width"], camera["height"]
    matrix = np.asarray(calibration["camera_matrix"], dtype=float)
    distortion = np.asarray(calibration["dist_coeffs"], dtype=float)
    # Both real pixels and measured corners enter the same undistorted pinhole view.
    map_x, map_y = cv2.initUndistortRectifyMap(matrix, distortion, None, matrix,
                                            (width, height), cv2.CV_32FC1)
    valid_pixels = (map_x >= 0) & (map_x <= width - 1) & (map_y >= 0) & (map_y <= height - 1)
    output = (args.output or session / "verification").expanduser().resolve()
    if output in (session, session / "images"):
        raise ValueError("Choose a separate verification output directory.")
    destinations = {output / name for name in
                    ("report.json", "index.html", "contact_sheet.jpg", "calibration_summary.jpg")}
    for index in range(len(dataset["views"])):
        destinations.update(output / f"{index:04d}" / name for name in
                            ("real.png", "simulation.png", "render_rgba.png", "overlay.png",
                             "edge_overlay.png", "comparison.jpg"))
    sources = {calibration_path, views_path, robot_xml, cube_xml, cube_config_path}
    sources.update((session / view["image"]).resolve() for view in dataset["views"])
    sources.update(path.resolve() for path in cube_xml.parent.iterdir() if path.is_file())
    collisions = sources & {path.resolve() for path in destinations}
    if collisions:
        raise ValueError(f"Verification output would overwrite an input: {min(map(str, collisions))}")
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "session_name": session.name, "session": str(session), "calibration": str(calibration_path),
        "calibration_sha256": digest(calibration_path), "dataset_sha256": digest(views_path),
        "robot_xml": str(robot_xml), "robot_model_sha256": digest(robot_xml),
        "cube_xml": str(cube_xml), "cube_config_sha256": digest(cube_config_path),
        "width": width, "height": height, "camera_matrix": matrix.tolist(),
        "overlay_alpha": args.alpha, "rms_space": "undistorted pinhole pixels",
        "source_dist_coeffs": distortion.tolist(), "render_dist_coeffs": [0.0] * 5,
        "T_base_camera": calibration["T_base_camera"], "T_ee_cube": calibration["T_ee_cube"],
        "alignment": "Recorded qpos + MuJoCo FK + saved final camera/mount fit; no per-image alignment or refit",
        "validation": "Final-fit overlays use all recorded views; held-out score came from a separate training-only fit",
        "original_held_out_rms_px": calibration.get("metrics", {}).get("held_out", {}).get("rms_px"),
        "views": [],
    }
    tiles, squared_errors = [], []
    summary_comparison, summary_index = None, 0
    # The renderer takes image dimensions from the saved active camera profile.
    render_calibration = dict(calibration, camera=dict(camera, camera_matrix=matrix.tolist()))
    with CalibrationRenderer(robot_xml, cube_xml, render_calibration) as renderer:
        for index, view in enumerate(dataset["views"]):
            source = (session / view["image"]).resolve()
            if not source.is_relative_to(session):
                raise ValueError("Recorded image path must stay inside the session.")
            original = cv2.imread(str(source))
            if original is None or original.shape != (height, width, 3):
                raise ValueError(f"Image missing or wrong resolution: {source}")
            real = cv2.remap(original, map_x, map_y, cv2.INTER_LINEAR)
            simulated, mask, pose = renderer.render(view["qpos"])
            if not np.allclose(pose, view["T_base_ee"], rtol=0, atol=1e-7):
                raise ValueError(f"View {index}: FK does not reproduce the recorded end-effector pose.")
            mask &= valid_pixels
            rgba = cv2.cvtColor(simulated, cv2.COLOR_BGR2BGRA)
            rgba[:, :, 3] = mask.astype(np.uint8) * 255
            overlay = real.copy()
            overlay[mask] = cv2.addWeighted(real, 1 - args.alpha, simulated, args.alpha, 0)[mask]
            edge_pixels = cv2.Canny(simulated, 60, 140) > 0
            interior = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
            outline = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
            edges = real.copy()
            edges[((edge_pixels & interior) | outline) & valid_pixels] = (220, 40, 255)
            camera_cube = np.linalg.inv(np.asarray(calibration["T_base_camera"])) @ pose @ calibration["T_ee_cube"]
            points = np.asarray(view["object_points_m"], dtype=float)
            rotation = cv2.Rodrigues(camera_cube[:3, :3])[0]
            projected = cv2.projectPoints(points, rotation, camera_cube[:3, 3], matrix, np.zeros(5))[0].reshape(-1, 2)
            observed = cv2.undistortPoints(np.asarray(view["image_points_px"], dtype=float).reshape(-1, 1, 2),
                                         matrix, distortion, P=matrix).reshape(-1, 2)
            errors = np.sum((projected - observed) ** 2, axis=1)
            squared_errors.extend(errors.tolist())
            rms = float(np.sqrt(errors.mean()))
            name = f"{index:04d}"
            folder = output / name
            folder.mkdir(exist_ok=True)
            files = {"real": "real.png", "sim": "simulation.png", "rgba": "render_rgba.png",
                     "overlay": "overlay.png", "edges": "edge_overlay.png"}
            for key, pixels in (("real", real), ("sim", simulated), ("rgba", rgba),
                                ("overlay", overlay), ("edges", edges)):
                save_image(folder / files[key], pixels)
            comparison = save_comparison(folder / "comparison.jpg", real, simulated, overlay, edges)
            files["comparison"] = "comparison.jpg"
            report["views"].append({
                "id": view.get("id", index), "index": index, "source_image": view["image"],
                "source_image_sha256": digest(source), "rms_px": rms, "tag_count": len(view["tag_ids"]),
                "qpos": view["qpos"], "T_base_ee": pose.tolist(), "T_camera_cube_projected": camera_cube.tolist(),
                "observed_px": observed.tolist(), "projected_px": projected.tolist(),
                "files": {key: f"{name}/{filename}" for key, filename in files.items()},
            })
            if summary_comparison is None or rms > report["views"][summary_index]["rms_px"]:
                summary_comparison, summary_index = comparison, index
            tile = cv2.resize(overlay, (384, round(384 * height / width)))
            cv2.rectangle(tile, (0, 0), (384, 26), (20, 25, 32), -1)
            cv2.putText(tile, f"View {index:02d} | {rms:.2f} px RMS", (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (245, 245, 245), 1)
            tiles.append(tile)
            message = f"Rendered {index + 1}/{len(dataset['views'])}: view {view.get('id', index)}, {rms:.3f} px RMS"
            print(message, flush=True)
            if progress is not None:
                progress(message)
    if not tiles:
        raise ValueError("Session contains no views.")
    report["rms_px"] = float(np.sqrt(np.mean(squared_errors)))
    report["summary"] = {
        "file": "calibration_summary.jpg", "view_index": summary_index,
        "view_id": report["views"][summary_index]["id"],
        "selection": "largest final-fit reprojection RMS",
    }
    save_summary(output / report["summary"]["file"], summary_comparison,
                 report, report["views"][summary_index])
    report["cube_assets_sha256"] = {str(path): digest(path) for path in cube_xml.parent.iterdir() if path.is_file()}
    columns = min(4, len(tiles))
    while len(tiles) % columns:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + columns]) for i in range(0, len(tiles), columns)])
    save_image(output / "contact_sheet.jpg", sheet)
    (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (output / "index.html").write_text(HTML)
    print(f"Saved overlays: {output}", flush=True)
    return output


def review_url(host, port):
    if host == "0.0.0.0":
        connection = os.environ.get("SSH_CONNECTION", "").split()
        host = connection[2] if len(connection) == 4 else ""
        try:
            socket.inet_pton(socket.AF_INET, host)
        except OSError:
            host = ""
        if not host or host.startswith("127.") or host == "0.0.0.0":
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    probe.connect(("192.0.2.1", 9))
                    host = probe.getsockname()[0]
            except OSError:
                host = socket.gethostname()
    return f"http://{host}:{port}/"


def serve_review(output, host, port):
    class Handler(SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def end_headers(self):
            self.send_header("Cache-Control", "no-cache")
            super().end_headers()

    with ThreadingHTTPServer((host, port), partial(Handler, directory=str(output))) as server:
        print(f"\nCalibration overlay review:\n{review_url(host, server.server_address[1])}\n"
              "Press Ctrl+C to stop serving. Saved overlays remain available.", flush=True)
        server.serve_forever(poll_interval=0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, help="Result JSON (default: SESSION/calibration.json)")
    parser.add_argument("--robot-xml", type=Path, default=ROOT / "aiofranka/model/fr3.xml")
    parser.add_argument("--cube-xml", type=Path, default=ROOT / "assets/aprilcube/mujoco/cube.xml")
    parser.add_argument("--output", type=Path, help="Export directory (default: SESSION/verification)")
    parser.add_argument("--alpha", type=float, default=0.45, help="Exported overlay opacity (default: 0.45)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--serve", action="store_true", help="Serve the review after exporting images")
    mode.add_argument("--no-serve", action="store_true", help="Export images without starting HTTP (default)")
    mode.add_argument("--serve-only", action="store_true", help="Serve existing exports without rendering")
    args = parser.parse_args()
    if not np.isfinite(args.alpha) or not 0 <= args.alpha <= 1 or not 1 <= args.port <= 65535:
        parser.error("--alpha must be in [0, 1], and --port in [1, 65535]")
    cv2.setNumThreads(1)
    try:
        if args.serve_only:
            output = (args.output or args.session / "verification").expanduser().resolve()
            if not (output / "report.json").is_file() or not (output / "index.html").is_file():
                raise ValueError("No exported review found; run without --serve-only first.")
        else:
            output = render_review(args)
        if args.serve or args.serve_only:
            serve_review(output, args.host, args.port)
    except KeyboardInterrupt:
        print("\nReview server stopped; exported images are retained.")
    except (OSError, ValueError, RuntimeError, ImportError, KeyError) as exc:
        print(f"Overlay verification error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
