FROM golang:1.26-bookworm AS build
WORKDIR /src/apps/control-api
COPY apps/control-api/go.mod ./go.mod
COPY apps/control-api/cmd ./cmd
COPY apps/control-api/internal ./internal
RUN go build -o /out/control-api ./cmd/server

FROM debian:bookworm-slim

# OCI labels — used by GHCR to auto-link the package to the source repo so
# `docker pull ghcr.io/.../cliffscanner-control-api` works without an explicit
# package-visibility tweak. VERSION is interpolated from the build-arg passed
# by the GitHub Actions workflow (defaults to "dev" for local builds).
ARG VERSION=dev
LABEL org.opencontainers.image.source="https://github.com/Gabriel-Karpinsky/Prism-FullStack"
LABEL org.opencontainers.image.description="Cliff Scanner control-api (Go) — proxies to the Pi-native edge daemon"
LABEL org.opencontainers.image.licenses="MIT"
LABEL org.opencontainers.image.version="${VERSION}"

RUN useradd --create-home --shell /usr/sbin/nologin scanner
WORKDIR /app
COPY --from=build /out/control-api /usr/local/bin/control-api
COPY apps/web-ui /app/web-ui
ENV HTTP_BIND=127.0.0.1:8080
ENV UI_ROOT=/app/web-ui
ENV SCANNER_BACKEND=edge
ENV EDGE_DAEMON_BASE_URL=http://127.0.0.1:9090
USER scanner
EXPOSE 8080
ENTRYPOINT ["/usr/local/bin/control-api"]
