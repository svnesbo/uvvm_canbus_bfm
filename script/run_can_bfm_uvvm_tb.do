echo "\n\nCompile bitvis uvvm..."
do compile_bitvis_uvvm.do
echo "\n\nCompile osvvm..."
do compile_osvvm.do
echo "\n\nCompile Wishbone BFM..."
do compile_wishbone_bfm.do
echo "\n\nCompile OpenCores CAN controller sources..."
do compile_can_ctrl_src.do
echo "\n\nCompile CAN BFM base source..."
do compile_can_bfm_base_src.do
echo "\n\nCompile CAN bus UVVM BFM sources..."
do compile_can_bfm_uvvm_src.do
echo "\n\nCompile CAN bus UVVM BFM test bench..."
do compile_can_bfm_uvvm_tb.do

echo "\n\nRun simulation..."

# Start the simulation
vsim -gui -t ps -novopt work.can_bfm_uvvm_tb

# Add waveforms
onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -r /*

# Run simulation
run -all
