# Compile source for OpenCores CAN controller

# Set up util_part_path and lib_name
#------------------------------------------------------
quietly set lib_name "work"
quietly set part_name "uvvm_canbus_bfm"
# path from mpf-file in sim
quietly set util_part_path "../../$part_name"

# (Re-)Generate library and Compile source files
#--------------------------------------------------
echo "\n\nRe-gen lib and compile $lib_name source"
#echo "$util_part_path/sim/$lib_name"
if {[file exists $util_part_path/sim/$lib_name]} {
  file delete -force $util_part_path/sim/$lib_name
}

vlib $util_part_path/sim/$lib_name
vmap $lib_name $util_part_path/sim/$lib_name

quietly set compdirectives_vhdl "-2008 -lint -work $lib_name"

quietly set compdirectives_vlog "-mixedsvvh s -93 -suppress 1346,1236 -quiet -work $lib_name +incdir+$util_part_path/src/can_ctrl/"

echo "\n\n\n=== Compiling $lib_name source\n"

eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/timescale.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_defines.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_ibo.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_crc.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_register.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_register_asyn.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_register_asyn_syn.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_register_syn.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_acf.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_bsp.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_btl.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_fifo.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_registers.v
eval vlog  $compdirectives_vlog   $util_part_path/src/can_ctrl/can_top.v
eval vcom  $compdirectives_vhdl   $util_part_path/src/can_ctrl/can_register_pkg.vhd
