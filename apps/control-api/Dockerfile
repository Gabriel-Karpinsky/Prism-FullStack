FROM golang:1.26-bookworm AS build
WORKDIR /src/apps/control-api
COPY apps/control-api/go.mod ./go.mod
COPY apps/control-api/cmd ./cmd
COPY apps/control-api/internal ./internal
RUN go build -o /out/control-api ./cmd/server

FROM debian:bookworm-slim
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
