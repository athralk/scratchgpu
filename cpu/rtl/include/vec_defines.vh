// Vector unit constants shared by the core (vec_decode) and TinyGPU v2 (gpu/rtl).
`ifndef VEC_DEFINES_VH
`define VEC_DEFINES_VH
`define VLEN        512
`define VLENB       64
`endif

// funct3 categories (OP-V)
localparam [2:0] F3_OPIVV = 3'b000;
localparam [2:0] F3_OPFVV = 3'b001;
localparam [2:0] F3_OPMVV = 3'b010;
localparam [2:0] F3_OPIVI = 3'b011;
localparam [2:0] F3_OPIVX = 3'b100;
localparam [2:0] F3_OPFVF = 3'b101;
localparam [2:0] F3_OPMVX = 3'b110;
localparam [2:0] F3_OPCFG = 3'b111;

// Instruction kinds (how elements map between source and destination groups)
localparam [4:0] VK_NONE        = 5'd0;
localparam [4:0] VK_ELEM        = 5'd1;   // SEW -> SEW elementwise (incl. merge/move/adc)
localparam [4:0] VK_WIDEN       = 5'd2;   // dest 2*SEW
localparam [4:0] VK_NARROW      = 5'd3;   // vs2 2*SEW -> SEW
localparam [4:0] VK_MASKDEST    = 5'd4;   // compares, vmadc/vmsbc -> mask
localparam [4:0] VK_MASKLOGIC   = 5'd5;   // mask x mask -> mask
localparam [4:0] VK_RED         = 5'd6;   // reduction -> vd[0]
localparam [4:0] VK_WRED        = 5'd7;   // widening reduction
localparam [4:0] VK_SLIDEUP     = 5'd8;   // vslideup, vslide1up, vfslide1up
localparam [4:0] VK_SLIDEDOWN   = 5'd9;   // vslidedown, vslide1down, vfslide1down
localparam [4:0] VK_GATHER      = 5'd10;
localparam [4:0] VK_GATHER16    = 5'd11;
localparam [4:0] VK_COMPRESS    = 5'd12;
localparam [4:0] VK_VMVNR       = 5'd13;  // whole-register move
localparam [4:0] VK_XUNARY      = 5'd14;  // vmv.x.s, vcpop.m, vfirst.m, vfmv.f.s
localparam [4:0] VK_SUNARY      = 5'd15;  // vmv.s.x, vfmv.s.f
localparam [4:0] VK_EXT         = 5'd16;  // vzext / vsext
localparam [4:0] VK_MSETBIT     = 5'd17;  // vmsbf / vmsif / vmsof
localparam [4:0] VK_IOTA        = 5'd18;
localparam [4:0] VK_VID         = 5'd19;
localparam [4:0] VK_MEM_UNIT    = 5'd20;
localparam [4:0] VK_MEM_STRIDED = 5'd21;
localparam [4:0] VK_MEM_INDEXED = 5'd22;
localparam [4:0] VK_MEM_WHOLE   = 5'd23;
localparam [4:0] VK_MEM_MASK    = 5'd24;
// Scalar FP (RV32F/D), executed by the same coprocessor
localparam [4:0] VK_SFP         = 5'd25;  // FP result (f[rd])
localparam [4:0] VK_SFPX        = 5'd26;  // integer result (x[rd]): compares, class, cvt.w, fmv.x.w
localparam [4:0] VK_SFLD        = 5'd27;  // flw / fld
localparam [4:0] VK_SFST        = 5'd28;  // fsw / fsd
