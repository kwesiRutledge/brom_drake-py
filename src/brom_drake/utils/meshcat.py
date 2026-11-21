def validate_meshcat_port_number(port):
    """Reject negative Meshcat ports before passing them to Drake."""
    if port is not None and port < 0:
        raise ValueError(
            f"meshcat_port_number must be nonnegative or None; received {port}. "
            "Use None to disable Meshcat visualization."
        )
    return port
