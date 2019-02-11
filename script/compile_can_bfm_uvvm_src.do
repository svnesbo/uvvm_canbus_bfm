# Set up util_part_path and lib_name
#------------------------------------------------------
quietly set lib_name "work"
quietly set part_name "uvvm_canbus_bfm"
# path from mpf-file in sim
quietly set util_part_path "../../$part_name"

vlib $util_part_path/sim/$lib_name
vmap $lib_name $util_part_path/sim/$lib_name

quietly set compdirectives_vhdl "-2008 -lint -work $lib_name"

echo "\n\n\n=== Compiling $lib_name source\n"

eval vcom  $compdirectives_vhdl   $util_part_path/src/bfm/can_bfm_uvvm/can_bfm_uvvm_pkg.vhd
