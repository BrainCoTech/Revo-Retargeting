"""Lossless raw sensor conversion; no inferred joint mapping or calibration."""


def convert_frame(frame, message, stamp, side):
    encoder = frame.sensor.encoder
    if len(encoder.angles_deg) != 21:
        raise ValueError("Expected 21 encoder channels")
    message.header.stamp = stamp
    message.header.frame_id = f"revohuman_{side}_device"
    message.side = side
    for field in ("host_time_ns", "protocol_version", "flags", "session_id",
                  "stream_sequence", "source_generation", "skipped_snapshots",
                  "transmit_tick"):
        setattr(message, field, getattr(frame, field))
    message.encoder_angles_deg = list(encoder.angles_deg)
    for field in ("sequence", "device_tick", "ready_mask", "valid_mask",
                  "reconnecting_mask", "offline_mask"):
        setattr(message, "encoder_" + field, getattr(encoder, field))
    sync = encoder.sync
    message.sync_present = sync is not None
    if sync is not None:
        message.sync_valid = sync.synchronized
        for field in ("generation", "cycle_sequence", "frame_number", "flags"):
            setattr(message, "sync_" + field, int(getattr(sync, field)))
    tactile = frame.sensor.tactile
    message.tactile_present = tactile is not None
    if tactile is not None:
        message.tactile_values = list(tactile.values)
        for field in ("sequence", "device_tick", "point_count", "vendor",
                      "wire_version", "link_state", "fault_reason", "health_flags",
                      "reconnect_attempts", "age_ms"):
            setattr(message, "tactile_" + field, int(getattr(tactile, field)))
        message.tactile_valid = tactile.valid
        message.tactile_unit = tactile.unit
    return message
