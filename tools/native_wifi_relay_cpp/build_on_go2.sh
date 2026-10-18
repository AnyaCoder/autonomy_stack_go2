#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RELAY_DIR="${GO2_RELAY_DIR:-$HOME/go2/relay}"
CPP_DIR="$RELAY_DIR/cppsrc"
DDS_HOME="${CYCLONEDDS_HOME:-$HOME/go2/runtime/dds-0.10.2/install}"
IDLC="$DDS_HOME/bin/idlc"

mkdir -p "$CPP_DIR/generated"
install -m 644 "$ROOT/native_wifi_relay.cpp" "$ROOT/relay_types.idl" \
  "$ROOT/unitree_relay.idl" "$CPP_DIR/"
install -m 644 "$ROOT/generated/relay_types.c" "$ROOT/generated/relay_types.h" \
  "$CPP_DIR/generated/"
"$IDLC" -o "$CPP_DIR/generated" "$CPP_DIR/unitree_relay.idl"

cd "$CPP_DIR"
gcc -O3 -DNDEBUG -std=c11 -I"$DDS_HOME/include" -Igenerated \
  -c generated/relay_types.c generated/unitree_relay.c
g++ -O3 -DNDEBUG -std=c++17 -pthread -I"$DDS_HOME/include" -Igenerated \
  -c native_wifi_relay.cpp
g++ -pthread native_wifi_relay.o relay_types.o unitree_relay.o \
  -L"$DDS_HOME/lib" -Wl,-rpath,"$DDS_HOME/lib" -lddsc \
  -o "$RELAY_DIR/native_wifi_relay_cpp.new"
printf 'Built %s\n' "$RELAY_DIR/native_wifi_relay_cpp.new"
