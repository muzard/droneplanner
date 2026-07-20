"""Tests for the interactive explorer backend (Software-Spec §6.6)."""

from __future__ import annotations

import json
import threading
import urllib.request
from http.server import ThreadingHTTPServer

from droneplanner.serve import _defaults, _plan_response, _Scene, make_handler


def test_defaults_shape() -> None:
    scene = _Scene(size=60)
    d = _defaults(scene)
    assert len(d["start"]) == 2
    assert len(d["observers"]) == 2
    assert len(d["extent"]) == 4
    assert "exposure" in d["layers"]


def test_plan_response_ok() -> None:
    scene = _Scene(size=60)
    res = _plan_response(scene, _defaults(scene))
    assert res["error"] is None
    assert res["image"].startswith("data:image/png;base64,")
    assert res["stats"]["length_m"] > 0
    assert res["stats"]["recommendation"] in ("GO", "REVIEW", "NO-GO")


def test_plan_response_reports_coincident_endpoints() -> None:
    scene = _Scene(size=60)
    params = _defaults(scene)
    params["end"] = params["start"]  # invalid: start == end
    res = _plan_response(scene, params)
    assert res.get("error")


def test_plan_response_unknown_layer_falls_back() -> None:
    scene = _Scene(size=60)
    params = _defaults(scene)
    params["layer"] = "nonsense"
    res = _plan_response(scene, params)
    assert res["error"] is None


def test_live_server_roundtrip() -> None:
    scene = _Scene(size=60)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(scene))
    port = httpd.server_address[1]
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        base = f"http://127.0.0.1:{port}"
        html = urllib.request.urlopen(base + "/", timeout=10).read().decode()
        assert "route explorer" in html

        defaults = json.loads(urllib.request.urlopen(base + "/defaults", timeout=10).read())
        body = json.dumps(defaults).encode()
        req = urllib.request.Request(
            base + "/plan", data=body, headers={"Content-Type": "application/json"}
        )
        res = json.loads(urllib.request.urlopen(req, timeout=20).read())
        assert res["error"] is None
        assert res["image"].startswith("data:image/png;base64,")
    finally:
        httpd.shutdown()
        httpd.server_close()
