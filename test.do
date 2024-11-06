# set the working dir, where all compiled verilog goes
vlib work

# compile all verilog modules in mux.v to working dir
# could also have multiple verilog files
vlog controller.v

#load simulation using mux as the top level simulation module
vsim controller


#log all signals and add some signals to waveform window
log {/*}
# add wave {/*} would add all items in top level simulation module
add wave {/*}

#Clock
force {clock} 0 0ns, 1 {5ns} -r 10ns
force {Xcord} 0 0ns, 9'b001000100 50ns
force {Ycord} 0 0ns, 8'b00000100 50ns
force {resetn} 0 0ns, 1 60ns

run 100000000ns
