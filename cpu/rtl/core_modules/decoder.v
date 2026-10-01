`default_nettype none
`include "instr_defines.vh"
module decoder (
    input  wire [31:0] instr,
    output wire [ 4:0] rs2,
    output wire [ 4:0] rs1,
    output wire [31:0] imm,
    output wire [ 4:0] rd,

    output wire rs1_valid,
    output wire rs2_valid,
    output wire rd_valid,

    output wire [6:0] opcode,
    output reg  [6:0] instr_id  // changed from 32-bit one-hot to compact ID
);

    wire is_r_instr, is_u_instr, is_s_instr, is_b_instr, is_j_instr, is_i_instr, is_csr_instr, is_amo_instr;
    wire [2:0] func3;
    wire [6:0] func7;
    wire [4:0] funct5;

    assign opcode = instr[6:0];

    assign is_i_instr = (opcode == 7'b0000011) || (opcode == 7'b0010011) || (opcode == 7'b1100111) ? 1'b1 : 1'b0;
    assign is_u_instr = (opcode == 7'b0010111) || (opcode == 7'b0110111) ? 1'b1 : 1'b0;
    assign is_b_instr = (opcode == 7'b1100011) ? 1'b1 : 1'b0;
    assign is_j_instr = (opcode == 7'b1101111) ? 1'b1 : 1'b0;
    assign is_s_instr = (opcode == 7'b0100011) ? 1'b1 : 1'b0;
    assign is_r_instr = (opcode == 7'b0110011) || (opcode == 7'b0100111) || (opcode == 7'b1010011) ? 1'b1 : 1'b0;
    assign is_csr_instr = (opcode == 7'b1110011) ? 1'b1 : 1'b0;  // CSR instructions
    assign is_amo_instr = (opcode == 7'b0101111) ? 1'b1 : 1'b0;

    // Vector: OP-V (incl. vsetvl), and vector loads/stores (LOAD-FP/STORE-FP with width 8/16/32)
    wire is_opv = (opcode == 7'b1010111);
    wire is_vcfg = is_opv && (instr[14:12] == 3'b111);
    wire is_vmem = ((opcode == 7'b0000111) || (opcode == 7'b0100111)) &&
                   ((instr[14:12] == 3'b000) || (instr[14:12] == 3'b101) || (instr[14:12] == 3'b110));
    // x registers read by vector instructions
    wire v_rs1 = (is_vcfg && instr[31:30] != 2'b11) ||                     // vsetvli / vsetvl AVL
                 (is_opv && (instr[14:12] == 3'b100 || instr[14:12] == 3'b110)) ||  // OPIVX / OPMVX
                 is_vmem;                                                   // base address
    wire v_rs2 = (is_vcfg && instr[31:25] == 7'b1000000) ||                 // vsetvl vtype
                 (is_vmem && instr[27:26] == 2'b10);                       // stride
    wire v_scalar_rd = is_opv && (instr[14:12] == 3'b010) && (instr[31:26] == 6'b010000) &&
                       ((instr[19:15] == 5'd0) || (instr[19:15] == 5'd16) || (instr[19:15] == 5'd17));
    wire v_rd = is_vcfg || v_scalar_rd;
    // Scalar FP (RV32F/D): run by the same coprocessor as the vector unit
    wire is_fmem = ((opcode == 7'b0000111) || (opcode == 7'b0100111)) &&
                   ((instr[14:12] == 3'b010) || (instr[14:12] == 3'b011));
    wire is_fr4 = (opcode == 7'b1000011) || (opcode == 7'b1000111) ||
                  (opcode == 7'b1001011) || (opcode == 7'b1001111);
    wire is_fop = (opcode == 7'b1010011);
    wire [4:0] ffn = instr[31:27];
    wire f_rs1x = is_fmem || (is_fop && (ffn == 5'b11010 || ffn == 5'b11110));     // x source
    wire f_rdx = is_fop && (ffn == 5'b10100 || ffn == 5'b11100 || ffn == 5'b11000); // x result
    wire is_fp = is_fmem || is_fr4 || is_fop;
    wire is_vec = is_opv || is_vmem || is_fp;

    assign rs2 = is_fp ? 5'b0 :
                 is_vec ? (v_rs2 ? instr[24:20] : 5'b0) :
                 (is_r_instr || is_s_instr || is_b_instr || is_amo_instr) ? instr[24:20] : 5'b0;
    assign rs1 = is_fp ? (f_rs1x ? instr[19:15] : 5'b0) :
                 is_vec ? (v_rs1 ? instr[19:15] : 5'b0) :
                 (is_r_instr || is_s_instr || is_b_instr || is_i_instr || is_csr_instr || is_amo_instr) ? instr[19:15] : 5'b0;
    assign rd = is_fp ? (f_rdx ? instr[11:7] : 5'b0) :
                is_vec ? (v_rd ? instr[11:7] : 5'b0) :
                (is_r_instr || is_u_instr || is_j_instr || is_i_instr || is_csr_instr || is_amo_instr) ? instr[11:7] : 5'b0;

    assign func3 = instr[14:12];
    assign func7 = is_r_instr ? instr[31:25] : 7'b0;
    assign funct5 = instr[31:27];

    // Validity signals - CSR instructions use rs1 and rd, but rs1 validity depends on instruction type
    assign rs1_valid = is_fp ? f_rs1x :
                      is_vec ? v_rs1 :
                      is_r_instr || is_i_instr || is_s_instr || is_b_instr ||
                      (is_csr_instr && (func3[2] == 1'b0)) || is_amo_instr;  // Atomics use rs1
    assign rs2_valid = is_fp ? 1'b0 :
                      is_vec ? v_rs2 :
                      is_r_instr || is_s_instr || is_b_instr ||
                      (is_amo_instr && (funct5 != 5'b00010));  // LR.W has no rs2 operand
    assign rd_valid = is_fp ? f_rdx :
                      is_vec ? v_rd :
                      is_r_instr || is_u_instr || is_j_instr || is_i_instr || is_csr_instr || is_amo_instr;
    
    assign imm =
        (is_fmem && opcode == 7'b0000111) ? { {21{instr[31]}}, instr[30:20] } :            // flw/fld
        (is_fmem && opcode == 7'b0100111) ? { {21{instr[31]}}, instr[30:25], instr[11:7] } : // fsw/fsd
        is_i_instr ? { {21{instr[31]}}, instr[30:20] } :
        is_s_instr ? { {21{instr[31]}}, instr[30:25], instr[11:7] } :
        is_b_instr ? { {20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0 } :
        is_u_instr ? { instr[31:12], 12'b0 } :
        is_j_instr ? { {12{instr[31]}}, instr[19:12], instr[20], instr[30:25], instr[24:21], 1'b0 } :
        is_csr_instr ? { 20'b0, instr[31:20] } :  // CSR address in upper 12 bits, zero-extended
        32'b0;

    // Instruction ID encoding
    always @(*) begin
        case (opcode)
            7'b0110011: begin  // R-type
                case ({
                    func7, func3
                })
                    {7'h00, 3'h0} : instr_id = INSTR_ADD;
                    {7'h20, 3'h0} : instr_id = INSTR_SUB;
                    {7'h01, 3'h0} : instr_id = INSTR_MUL;
                    {7'h01, 3'h1} : instr_id = INSTR_MULH;
                    {7'h01, 3'h2} : instr_id = INSTR_MULHSU;
                    {7'h01, 3'h3} : instr_id = INSTR_MULHU;
                    {7'h00, 3'h4} : instr_id = INSTR_XOR;
                    {7'h00, 3'h6} : instr_id = INSTR_OR;
                    {7'h00, 3'h7} : instr_id = INSTR_AND;
                    {7'h00, 3'h1} : instr_id = INSTR_SLL;
                    {7'h00, 3'h5} : instr_id = INSTR_SRL;
                    {7'h20, 3'h5} : instr_id = INSTR_SRA;
                    {7'h00, 3'h2} : instr_id = INSTR_SLT;
                    {7'h00, 3'h3} : instr_id = INSTR_SLTU;
                    {7'h01, 3'h4} : instr_id = INSTR_DIV;
                    {7'h01, 3'h5} : instr_id = INSTR_DIVU;
                    {7'h01, 3'h6} : instr_id = INSTR_REM;
                    {7'h01, 3'h7} : instr_id = INSTR_REMU;
                    default:        instr_id = INSTR_INVALID;
                endcase
            end
            7'b0010011: begin  // I-type arithmetic
                case (func3)
                    3'h0: instr_id = INSTR_ADDI;
                    3'h4: instr_id = INSTR_XORI;
                    3'h6: instr_id = INSTR_ORI;
                    3'h7: instr_id = INSTR_ANDI;
                    3'h1: instr_id = (imm[11:5] == 7'h00) ? INSTR_SLLI : INSTR_INVALID;
                    3'h5:
                    instr_id = (imm[11:5] == 7'h00) ? INSTR_SRLI : 
                              (imm[11:5] == 7'h20) ? INSTR_SRAI : INSTR_INVALID;
                    3'h2: instr_id = INSTR_SLTI;
                    3'h3: instr_id = INSTR_SLTIU;
                    default: instr_id = INSTR_INVALID;
                endcase
            end
            7'b0000011: begin  // loads
                case (func3)
                    3'h0: instr_id = INSTR_LB;
                    3'h1: instr_id = INSTR_LH;
                    3'h2: instr_id = INSTR_LW;
                    3'h4: instr_id = INSTR_LBU;
                    3'h5: instr_id = INSTR_LHU;
                    default: instr_id = INSTR_INVALID;
                endcase
            end
            7'b0100011: begin  // stores
                case (func3)
                    3'h0: instr_id = INSTR_SB;
                    3'h1: instr_id = INSTR_SH;
                    3'h2: instr_id = INSTR_SW;
                    default: instr_id = INSTR_INVALID;
                endcase
            end
            7'b1100011: begin  // branches
                case (func3)
                    3'h0: instr_id = INSTR_BEQ;
                    3'h1: instr_id = INSTR_BNE;
                    3'h4: instr_id = INSTR_BLT;
                    3'h5: instr_id = INSTR_BGE;
                    3'h6: instr_id = INSTR_BLTU;
                    3'h7: instr_id = INSTR_BGEU;
                    default: instr_id = INSTR_INVALID;
                endcase
            end
            7'b1101111: instr_id = INSTR_JAL;
            7'b1100111: instr_id = INSTR_JALR;
            7'b0110111: instr_id = INSTR_LUI;
            7'b0010111: instr_id = INSTR_AUIPC;
            7'b0101111: begin
                if (func3 != 3'b010) begin
                    instr_id = INSTR_INVALID;
                end else begin
                    case (funct5)
                        5'b00010: instr_id = (instr[24:20] == 5'b0) ? INSTR_LR_W : INSTR_INVALID;
                        5'b00011: instr_id = INSTR_SC_W;
                        5'b00001: instr_id = INSTR_AMOSWAP_W;
                        5'b00000: instr_id = INSTR_AMOADD_W;
                        5'b01100: instr_id = INSTR_AMOAND_W;
                        5'b01000: instr_id = INSTR_AMOOR_W;
                        5'b00100: instr_id = INSTR_AMOXOR_W;
                        5'b10100: instr_id = INSTR_AMOMAX_W;
                        5'b10000: instr_id = INSTR_AMOMIN_W;
                        5'b11100: instr_id = INSTR_AMOMAXU_W;
                        5'b11000: instr_id = INSTR_AMOMINU_W;
                        default:            instr_id = INSTR_INVALID;
                    endcase
                end
            end
            7'b0001111: begin
                case (func3)
                    3'h0: instr_id = (instr == 32'h0100000f) ? INSTR_PAUSE : INSTR_FENCE;
                    3'h1: instr_id = INSTR_FENCE_I;
                    default: instr_id = INSTR_INVALID;
                endcase
            end
            7'b1110011: begin  // System instructions
                if (instr == 32'h30200073) begin      // MRET
                    instr_id = INSTR_MRET;
                end else if (instr == 32'h10200073) begin  // SRET
                    instr_id = INSTR_SRET;
                end else if (instr == 32'h10500073) begin  // WFI
                    instr_id = INSTR_WFI;
                end else if (instr == 32'h00000073) begin  // ECALL
                    instr_id = INSTR_ECALL;
                end else if (instr == 32'h00100073) begin  // EBREAK
                    instr_id = INSTR_EBREAK;
                end else if ((instr[31:25] == 7'b0001001) && (func3 == 3'b000) && (instr[11:7] == 5'b00000)) begin
                    instr_id = INSTR_SFENCE_VMA;
                end else begin
                    case (func3)
                        3'h1: instr_id = INSTR_CSRRW;
                        3'h2: instr_id = INSTR_CSRRS;
                        3'h3: instr_id = INSTR_CSRRC;
                        3'h5: instr_id = INSTR_CSRRWI;
                        3'h6: instr_id = INSTR_CSRRSI;
                        3'h7: instr_id = INSTR_CSRRCI;
                        default: instr_id = INSTR_INVALID;
                    endcase
                end
            end
            7'b1010111: instr_id = is_vcfg ? INSTR_VSETVL : v_scalar_rd ? INSTR_VSCALAR : INSTR_VARITH;
            7'b0000111: instr_id = (is_vmem || is_fmem) ? INSTR_VLOAD : INSTR_INVALID;
            7'b0100111: instr_id = (is_vmem || is_fmem) ? INSTR_VSTORE : INSTR_INVALID;
            7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: instr_id = INSTR_VARITH;
            7'b1010011: instr_id = f_rdx ? INSTR_VSCALAR : INSTR_VARITH;
            default:    instr_id = INSTR_INVALID;
        endcase
    end

endmodule
