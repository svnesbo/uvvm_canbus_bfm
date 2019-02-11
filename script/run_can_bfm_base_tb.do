echo "\n\nCompile CAN BFM base source..."
do compile_can_bfm_base_src.do
echo "\n\nCompile CAN BFM base test bench..."
do compile_can_bfm_base_tb.do

echo "\n\nRun simulation..."

# Start the simulation
vsim -gui -t ps work.can_bfm_base_tb

# Add waveforms
onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -r /*

# Run simulation
run -all
