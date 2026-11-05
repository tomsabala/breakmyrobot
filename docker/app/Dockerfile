# The public face of this repository, as an image apps.tom-sabala.dev can run.
#
# WHAT THIS IS NOT. It is not the lab. The lab needs ROS 2, Gazebo Harmonic and
# ArduPilot SITL in lockstep with the physics clock (`ral:dev`, 6.11 GB), and
# `scripts/dashboard.sh` takes a flock per ROS domain, so it admits exactly one
# operator at a time. None of that belongs behind a public URL. What this image
# serves is `site/dist`: the rendered documents, the measured evidence, and the
# operator console in REPLAY mode, playing frames exported from a real run's
# rosbag. There is no robot, no simulator and no gateway behind it, and the
# console carries a banner that says so and cannot be dismissed.
#
# BUILD INPUT. `site/dist` must exist before `docker build`. The site's own
# build pre-renders its mermaid diagrams in a headless Chrome, which belongs on
# a CI runner rather than inside this image; `.github/workflows/apps-image.yml`
# runs `scripts/build_site.sh` first and then builds this file. By hand:
#
#   bash scripts/build_site.sh
#   docker build -f docker/app/Dockerfile -t ghcr.io/tomsabala/breakmyrobot:dev .
#   docker run --rm -p 8080:8080 ghcr.io/tomsabala/breakmyrobot:dev
#   curl -sI localhost:8080/a/breakmyrobot/ | head -1
#
# A missing `site/dist` fails the COPY below rather than producing an image
# that serves 404s.

# The caddy binary, stripped of its file capability. NOT plain
# `FROM caddy:2.10-alpine`: that image ships `/usr/bin/caddy` with
# `cap_net_bind_service=ep` so a root-less process can bind port 80, and a
# process whose bounding set is empty cannot exec a file that carries
# permitted capabilities. The broker runs every instance with
# `CapDrop: ALL`, so this is not a corner case - it is how this image will
# always be started.
#
# MEASURED, three ways: `--cap-drop ALL` alone, `--cap-drop ALL
# --security-opt no-new-privileges:true`, and `COPY --from` into a clean
# alpine (which PRESERVES the xattr - `getcap` reported
# `cap_net_bind_service=ep` on the copied file). All three ended the same
# way: `exec /usr/bin/caddy: operation not permitted`, container unhealthy,
# exit 255, every request refused.
#
# `cp` does not copy the capability xattr, so one copy inside the stage is
# the whole fix. Nothing here needs the capability: this server listens on
# 8080.
FROM caddy:2.10-alpine AS caddybin
RUN cp /usr/bin/caddy /caddy

FROM alpine:3.21
COPY --from=caddybin /caddy /usr/bin/caddy

# Caddy's own image runs as root and writes to /data and /config; both are
# pointed at /tmp and the user is dropped, so this container needs no
# capability and no writable layer of its own. `admin off` and
# `persist_config off` in the Caddyfile are the other half of that.
ENV XDG_CONFIG_HOME=/tmp/caddy-config \
    XDG_DATA_HOME=/tmp/caddy-data

COPY docker/app/Caddyfile /etc/caddy/Caddyfile
COPY site/dist/ /srv/site/

USER 1000:1000

EXPOSE 8080

# The broker waits on the image's own HEALTHCHECK before it probes `readyPath`
# (gateway/README.md: the `health` and `probe` phases), so an image without one
# is an instance the broker cannot tell apart from a hung one.
HEALTHCHECK --interval=10s --timeout=3s --start-period=2s --retries=3 \
  CMD wget -q -O /dev/null http://127.0.0.1:8080/healthz || exit 1

LABEL org.opencontainers.image.title="Resilient Autonomy Lab" \
      org.opencontainers.image.description="Replay console and measured evidence for a two-robot autonomy lab (ROS 2 + Nav2, ArduPilot SITL)" \
      org.opencontainers.image.source="https://github.com/tomsabala/breakmyrobot" \
      org.opencontainers.image.licenses="Apache-2.0"

CMD ["caddy", "run", "--config", "/etc/caddy/Caddyfile", "--adapter", "caddyfile"]
