sim-console () {
    SIM_CONSOLE_DIR=`rospack find fast_simulator`/src/fast_simulator
    ipython -i --no-banner --no-confirm-exit --autocall 2 "${SIM_CONSOLE_DIR}/console.py" -- $*
}
