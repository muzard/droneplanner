

async def test_dropout_and_yaw_survive_a_record_round_trip(tmp_path: Path) -> None:
    """A recorded run must still know it dropped out when `ev-report` reads it.

    The bridge and the analyser are separated by a JSONL file, so a field that
    is written but never parsed back is silently lost — which is how a dropout
    arm would come back looking exactly like a clean one.
    """
    path = tmp_path / "run.jsonl"
    injector = ErrorInjector(
        DriftConfig(rate=0.02, seed=1), yaw=YawDriftConfig(wander_deg_per_100m=6.0, seed=2)
    )
    with TrackRecorder(path) as rec:
        bridge = EvBridge(
            injector,
            EvBridgeConfig(rate_hz=0.0),
            recorder=rec,
            dropout=DropoutModel(DropoutConfig(mean_interval_m=60.0, duration_s=1.0, seed=1)),
        )
        stats = await bridge.run(ReplayTruthSource(_straight_truth()), RecordingPublisher())

    back = load_track(path)
    assert len(back) == stats.samples
    assert sum(s.ev_down for s in back) == stats.dropout_suppressed > 0
    assert max(abs(s.yaw_error_rad) for s in back) == pytest.approx(
        stats.max_yaw_error_rad, abs=1e-5
    )


def test_load_track_defaults_the_new_fields_for_older_records() -> None:
    """The three flown acceptance records predate both fields."""
    sample = load_track_from_lines('{"t":1,"truth":{"e":0,"n":0,"u":30},'
                                   '"err":{"e":0,"n":0},"pub":{"n":0,"e":0,"d":-30,"yaw":0}}')
    assert sample.ev_down is False
    assert sample.yaw_error_rad == 0.0
