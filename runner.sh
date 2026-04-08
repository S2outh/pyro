#/bin/sh

TOKEN_ARGS=""
if [ -f flatsat.token ]; then
  TOKEN_ARGS="--host ws://pyro.flatsat.space --token $(cat flatsat.token)"
fi

exec probe-rs run --chip STM32G0B1KE $TOKEN_ARGS "$@"
