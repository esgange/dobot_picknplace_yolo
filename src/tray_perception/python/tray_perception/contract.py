"""Wire endpoint for the depth-capable tray contract; no legacy fallback."""

# Bump the endpoint when changing the service layout. Humble peers with an older
# layout can otherwise discover the same service type name after a live rebuild.
SERVICE_NAME = "/tray_detect/get_tray_pose_v3"
