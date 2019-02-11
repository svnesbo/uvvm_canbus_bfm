echo "\n\nCompile CAN BFM SystemVerilog source..."
do compile_can_bfm_sv_src.do
echo "\n\nCompile CAN BFM SystemVerilog test bench..."
do compile_can_bfm_sv_tb.do

echo "\n\nRun simulation..."

# Start the simulation
vsim -gui -t ps -novopt work.can_bfm_sv_tb

# Add waveforms
onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -r /*

# Run simulation
run -all
