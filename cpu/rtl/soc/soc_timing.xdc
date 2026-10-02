# Timing exceptions for the Synapse-32 + TinyGPU v2 SoC (implementation only).
#
# TinyGPU's element sequencer and scalar FP unit run each element / FP op in phases
# (gpu/rtl/vpu_top.v, "Element timing"): operands are registered into mcx_* at fetch, held
# unchanged, and the results are registered into mcy_* (vector elements) / mcz_* (scalar FP)
# three / four clock cycles later. Every register that feeds that compute logic (mcx_*, the
# instruction registers c_*, the element index e, acc, found, evl, the sequencer state, v0 and
# the VRF) is stable from fetch until capture, so those paths get 3 / 4 cycles instead of 1.
# (ph changes every cycle but only enables the capture; it never feeds the captured data.)
#
# Only flops and LUTRAM cells are valid startpoints: the REF_NAME test keeps the LUTs that
# synthesis names after these registers (acc_reg[12]_i_4 ...) out of the list.

set vpu_src [get_cells -quiet -hier -filter {(REF_NAME =~ FD* || REF_NAME =~ RAMD* || REF_NAME =~ RAMS*) && (NAME =~ */vpu/mcx_*_reg* || NAME =~ */vpu/c_*_reg* || NAME =~ */vpu/e_reg* || NAME =~ */vpu/acc_reg* || NAME =~ */vpu/found_reg* || NAME =~ */vpu/evl_reg* || NAME =~ */vpu/state_reg* || NAME =~ */vpu/vrf/v0_reg* || NAME =~ */vpu/vrf/bank*)}]
# Synthesis folds the operand registers (mcx_*) into the input registers of the DSP blocks
# of the element-engine FP32 FMA (fpu) and the FP64 FMA (fpu64), so those paths start at the
# DSP cell itself. The folded registers hold the same stable operands, so they are sources too.
# (Separate commands: XDC does not support list operations such as concat.)
set vpu_dsp [get_cells -quiet -hier -filter {REF_NAME =~ DSP48* && (NAME =~ */vpu/fpu64/* || NAME =~ */vpu/fpu/*)}]
set vpu_elem_cap [get_cells -quiet -hier -filter {REF_NAME =~ FD* && NAME =~ */vpu/mcy_*_reg*}]
set vpu_sfp_cap  [get_cells -quiet -hier -filter {REF_NAME =~ FD* && NAME =~ */vpu/mcz_*_reg*}]

set_multicycle_path -setup 3 -from $vpu_src -to $vpu_elem_cap
set_multicycle_path -hold  2 -from $vpu_src -to $vpu_elem_cap
set_multicycle_path -setup 4 -from $vpu_src -to $vpu_sfp_cap
set_multicycle_path -hold  3 -from $vpu_src -to $vpu_sfp_cap
set_multicycle_path -setup 3 -from $vpu_dsp -to $vpu_elem_cap
set_multicycle_path -hold  2 -from $vpu_dsp -to $vpu_elem_cap
set_multicycle_path -setup 4 -from $vpu_dsp -to $vpu_sfp_cap
set_multicycle_path -hold  3 -from $vpu_dsp -to $vpu_sfp_cap
