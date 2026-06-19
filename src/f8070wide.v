// SC/MP III binary compatible soft core
// 32-bit instruction bus & 16-bit data bus
// Copyright 2026 © Yasuo Kuwahara

// MIT License

// not implemented: BND, CALL, SSM

module f8070wide(clk, reset, pc_out, insn_in, adr_out,
	data_in, data_out, wr_l, wr_u, sa, sb, f, intack);
input clk, reset, sa, sb;
input [31:0] insn_in;
input [15:0] data_in;
output [15:0] pc_out, adr_out, data_out;
output [2:0] f;
output wr_l, wr_u, intack;

localparam I = 0;
localparam V = 6;
localparam C = 7;

reg [7:0] a, e, s;
reg [15:0] pc, sp, p2, p3, t;

function [7:0] sel4x8;
	input [1:0] sel;
	input [31:0] a;
	begin
		case (sel)
			2'b00: sel4x8 = a[7:0];
			2'b01: sel4x8 = a[15:8];
			2'b10: sel4x8 = a[23:16];
			2'b11: sel4x8 = a[31:24];
		endcase
	end
endfunction

function [15:0] sel4x16;
	input [1:0] sel;
	input [63:0] a;
	begin
		case (sel)
			2'b00: sel4x16 = a[15:0];
			2'b01: sel4x16 = a[31:16];
			2'b10: sel4x16 = a[47:32];
			2'b11: sel4x16 = a[63:48];
		endcase
	end
endfunction

//
// DECODE
//

wire [7:0] o = force_nop ? 8'h00 :
	sel4x8(pc[1:0], { insn_in[7:0], insn_in[31:8] });
wire [23:0] insn = {
	sel4x8(pc[1:0], { insn_in[23:0], insn_in[31:24] }),
	sel4x8(pc[1:0], { insn_in[15:0], insn_in[31:16] }),
	o
};

localparam DIV = 0;
localparam LDAS = 1;
localparam LDEA = 2;
localparam LDSA = 3;
localparam MPY = 4;
localparam RRL = 5;
localparam I1MAX = 5;
//
localparam JMP = 6;
localparam JSRPLI = 7;
localparam LDP = 8;
localparam LDPC = 9;
localparam RET = 10;
localparam POP = 11;
localparam POP1 = 12;
localparam POPEA = 13;
localparam POPP = 14;
localparam PUSH = 15;
localparam PUSH1 = 16;
localparam PUSH2 = 17;
localparam STA = 18;
localparam STEA = 19;
localparam IMAX = 19;

wire [IMAX:0] i;
assign i[LDPC] = o[7:4] == 4'b0100 & o[2:0] == 3'b100;
assign i[STEA] = o[7:3] == 5'b10001;
assign i[STA] = o[7:3] == 5'b11001;
assign i[JSRPLI] = o[7:2] == 6'b001000;
assign i[LDP] = o[7:2] == 6'b001001 | o[7:4] == 4'b0100 & o[2];
assign i[POPP] = o[7:2] == 6'b010111;
assign i[PUSH] = i[PUSH1] | i[PUSH2];
assign i[POP] = i[POP1] | i[POPEA] | i[POPP];
assign i[LDEA] = o == 8'h01 | o == 8'h48;
assign i[LDAS] = o == 8'h06;
assign i[LDSA] = o == 8'h07;
assign i[PUSH2] = o == 8'h08 | o[7:2] == 6'b010101;
assign i[PUSH1] = o == 8'h0a;
assign i[DIV] = o == 8'h0d;
assign i[JMP] = o == 8'h24;
assign i[MPY] = o == 8'h2c;
assign i[POP1] = o == 8'h38;
assign i[POPEA] = o == 8'h3a;
assign i[RRL] = o == 8'h3f;
assign i[RET] = o == 8'h5c;

// instruction byte count

wire [15:0] byte_lut2 = 16'hd000, byte_lut3 = 16'hf50f;
wire byte1 = ~o[7] & ~o[5] | o[7:5] == 3'b011 & ~o[2] |
	o[7:4] == 4'b0010 & byte_lut2[o[3:0]] |
	o[7:4] == 4'b0011 & byte_lut3[o[3:0]];
wire byte3 = o[7:3] == 5'b00100 | o[7:6] == 2'b10 & o[2:0] == 3'b100;
wire [1:0] bytes = { ~byte1 | byte3, byte1 | byte3 };

// state

wire dbl_state = o[7:4] == 4'b1001 | i[LDPC];
wire clken = ~(div_start | div_busy);
reg state;
always @(posedge clk)
	if (reset) state <= 0;
	else if (clken)
		if (state) state <= 0;
		else if (dbl_state) state <= 1;

// interrupt

reg [2:0] intadr;
wire accept = active & ~(dbl_state & ~state) & ~i[RET];
wire valid_intr = (sa | sb) & s[I];
always @(posedge clk)
	if (reset | intack) intadr <= 0;
	else if (clken & accept & valid_intr) intadr <= sa ? 3 : 6;
assign intack = intadr[1];

// EA

wire [15:0] base = sel4x16(/*intack ? 0 : */o[1:0],
	{ fwd_p3, fwd_p2, fwd_sp, nextpc_normal });
wire [15:0] baseofs = (o[2:1] == 2'b10 ? 16'h0000 : base) +
	{ {8{ insn[15] }}, insn[15:8] };
wire [15:0] ea = o[2:1] == 2'b10 ? { 8'hff, insn[15:8] } :
	&o[2:1] & ~insn[15] ? base : baseofs;

// PC

reg active, exec_ret1;
always @(posedge clk)
	if (reset) begin
		active <= 0;
		exec_ret1 <= 0;
	end
	else if (clken) begin
		active <= 1;
		exec_ret1 <= i[RET];
	end
wire force_nop = ~active | exec_ret1 | intack;
wire zero = ~|fwd_a;
wire [3:0] cond = { ~zero, 1'b1, zero, ~fwd_a[7] };
wire cond_ok = cond[o[4:3]];
wire [15:0] nextpc_normal = pc + bytes;
wire [15:0] nextpc_rel = nextpc_normal + { {8{ insn[15] }}, insn[15:8] };
wire [15:0] nextpc = o[7:5] == 3'b011 & o[2] & cond_ok ? nextpc_rel :
	intack ? { 13'b0, intadr } :
	exec_ret1 ? data_in :
	i[JMP] | i[JSRPLI] & ~o[1] ? insn[23:8] :
	state & i[LDPC] ? { e, a } :
	nextpc_normal;
assign pc_out = active & clken & ~(dbl_state & ~state)/* & ~intack*/ ? nextpc : pc;
always @(posedge clk)
	if (reset) pc <= 0;
	else if (clken) pc <= pc_out;

// address selector

wire [15:0] sp_adr = fwd_sp +
	{ {15{ i[PUSH] | i[JSRPLI] | intack }}, i[PUSH1] };
assign adr_out = i[PUSH] | i[POP] | i[JSRPLI] | i[RET] | intack ? sp_adr : ea;

// write data (write only)

wire wr_s0_u = ~state & (i[STEA] | i[PUSH2] | i[JSRPLI] | intack);
wire wr_s0_l = ~state & (i[STA] | i[PUSH1]);
wire [15:0] wd_s0 = intack ? pc : o[3] ? { fwd_e, fwd_a } : base;

//
// EXEC
//

reg [7:0] o1;
reg [15:0] imm1;
reg [I1MAX:0] i1;
reg dbl_state1, sel_alu_b, a_and, a_or, a_or_lsb, sub_sw, sft_left;
reg logic_thru, sel_preg, sel_add, sel_mul, sel_sft, sel_logic, sel_e;
always @(posedge clk) if (clken) begin
	imm1 <= insn[23:8];
	i1 <= i[I1MAX:0];
	o1 <= o;
	dbl_state1 <= dbl_state;
	sel_alu_b <= o[7:4] == 4'b0010 | o[2:0] == 3'b100;
	sub_sw <= |o[7:6] & &o[5:3];
	a_and <= |o[7:6] & &o[5:4];
	a_or <= o[7:3] == 5'b10011;
	a_or_lsb <= o[7:4] == 4'b1001;
	sel_add <= o[7] | (o[3] | ~o[1]) & ~o[2] & ~o[0];
	sel_mul <= o[7:4] == 4'b0010;
	sel_sft <= ~|o[7:6] & &o[3:2] & (~|o[5:4] & o[1:0] != 2'b01 | &o[5:4]);
	sft_left <= ~o[5] & o[1];
	sel_logic <= o[6] & ^o[5:4] | ~o[7] & ~|o[4:2];
	logic_thru <= ~|o[5:4];
	sel_e <= ~o[7] & ~|o[2:1] & (o[6] | o[0]);
	sel_preg <= |o[7:5];
end
reg div_start1;
always @(posedge clk)
	div_start1 <= i[DIV]; // not i1[DIV] because of no clken
wire [15:0] alu_a = { e, a };
wire [15:0] alu_b = sel_alu_b ? imm1 : data_in[15:0];
wire [7:0] alu_bl = sel_e ? e : alu_b[7:0];
wire [15:0] add_a = { alu_a[15:1] & {15{ a_and }} | {15{ a_or }},
	alu_a[0] & a_and | a_or_lsb };
wire [15:0] add_b = { alu_b[15:8], alu_bl[7:0] };
wire [16:0] add_y = sub_sw ? add_a - add_b : add_a + add_b;
wire sft_r8 = ~o1[4] ? alu_a[8] : o1[0] ? s[C] : o1[1] ? alu_a[0] : 0;
wire [15:0] sft_y = sft_left ?
	{ alu_a[14:0], 1'b0 } : { 1'b0, alu_a[15:9], sft_r8, alu_a[7:1] };
wire [7:0] logic_y = logic_thru ? alu_bl :
	~o1[4] ? a ^ alu_bl : o1[3] ? a | alu_bl : a & alu_bl;
wire [31:0] mul_y = $signed(alu_a) * $signed(t);
wire [7:0] fwd_a, fwd_e;
wire [15:0] fwd_t, div_q, div_r;
wire div_start = i[DIV] & ~div_start1;
divu divu(.clk(clk), .start(div_start), .a({ fwd_e, fwd_a }), .b(fwd_t),
	.q(div_q), .r(div_r), .busy(div_busy));
wire [15:0] preg = sel4x16(o1[1:0], { p3, p2, sp, nextpc_normal });
wire [15:0] etc = sel_preg ? preg : o1[2] ? div_q : t;
wire [15:0] alu_y = sel_add ? add_y[15:0] :
	sel_mul ? mul_y[31:16] : sel_sft ? sft_y : etc;
wire [7:0] alu_yl = sel_logic ? logic_y : i1[LDAS] ? s : alu_y[7:0];
wire [7:0] alu_yu = i1[LDEA] ? a : alu_y[15:8];

// write data (after read)

reg t_wr_s1_l;
always @(posedge clk) if (clken)
	t_wr_s1_l <= dbl_state & ~i[LDPC];
wire wr_s1_l = state & t_wr_s1_l;
assign wr_u = wr_s0_u;
assign wr_l = wr_s0_l | wr_s1_l | wr_s0_u;
assign data_out = wr_s1_l ? { 8'h00, add_y[7:0] } : wd_s0;

//
// UPDATE
//

wire ren = ~dbl_state1 | state;

// A register

wire [15:0] a_lut0 = 16'hf842, a_lut3 = 16'hf50f, a_lut4 = 16'hf001;
reg load_a1;
always @(posedge clk) if (clken)
	load_a1 <= o[7] ? o[6:4] != 3'b010 : (
		o[6:4] == 3'b000 & a_lut0[o[3:0]] |
		i[MPY] |
		o[6:4] == 3'b011 & a_lut3[o[3:0]] |
		o[6:4] == 3'b100 & a_lut4[o[3:0]] |
		(o[6:4] == 3'b101 | &o[6:5]) & ~|o[2:0]);

assign fwd_a = load_a1 & ren ? alu_yl : a;
always @(posedge clk) if (clken)
	if (ren & load_a1) a <= fwd_a;

// E register

wire [15:0] e_lut0 = 16'hb802, e_lut3 = 16'h040f, e_lut4 = 16'hf100;
reg load_e1;
always @(posedge clk) if (clken)
	load_e1 <= o[7] ? ~|o[6:4] | o[6:4] == 3'b011 : (
		o[6:4] == 3'b000 & e_lut0[o[3:0]] |
		i[MPY] |
		o[6:4] == 3'b011 & e_lut3[o[3:0]] |
		o[6:4] == 3'b100 & e_lut4[o[3:0]]);

assign fwd_e = load_e1 ? alu_yu : e;
always @(posedge clk) if (clken)
	if (ren & load_e1) e <= fwd_e;

// T register

reg load_t1;
always @(posedge clk) if (clken)
	load_t1 <= o[7:3] == 5'b10100 | ~o[7] & o[4:0] == 5'b01001 | i[MPY] | i[DIV];
assign fwd_t = load_t1 ?
	i1[MPY] ? mul_y[15:0] : i1[DIV] ? div_r : o1[7] ? alu_b : alu_a : t;
always @(posedge clk) if (clken)
	if (ren & load_t1) t <= fwd_t;

// SP register

wire sp_plus1 = i[POP1];
wire sp_plus2 = i[POPEA] | i[POPP] | i[RET];
wire sp_minus1 = i[PUSH1];
wire sp_minus2 = i[PUSH2] | i[JSRPLI] | intack;
wire addsub_sp = sp_plus1 | sp_plus2 | sp_minus1 | sp_minus2;
reg load_sp1, addsub_sp1;
reg [2:0] sp_add1;
always @(posedge clk) if (clken) begin
	load_sp1 <= i[LDP] & o[1:0] == 2'b01 | addsub_sp;
	addsub_sp1 <= addsub_sp;
	sp_add1 <= { sp_minus1 | sp_minus2, ~sp_plus1, sp_plus1 | sp_minus1 };
end
wire [15:0] fwd_sp = load_sp1 ?
	addsub_sp1 ? sp + { {13{ sp_add1[2] }}, sp_add1 } :
	o1[6] ? alu_a : alu_b :
	sp;
always @(posedge clk)
	if (reset) sp <= 0;
	else if (clken & ren & load_sp1) sp <= fwd_sp;

// P2/P3 register

wire load_p23 = (i[LDP] | i[POPP] | i[JSRPLI]) & o[1];
reg p23sel1, load_p23_1, load_p2_1, load_p3_1;
always @(posedge clk) if (clken) begin
	p23sel1 <= o[6] & ~o[4];
	load_p23_1 <= load_p23;
	load_p2_1 <= load_p23 & ~o[0];
	load_p3_1 <= load_p23 & o[0];
end
wire [15:0] p23_d = p23sel1 ? alu_a : alu_b;
wire [15:0] fwd_p2 = load_p2_1 ? p23_d : p2;
wire [15:0] fwd_p3 = load_p3_1 ? p23_d : p3;
always @(posedge clk) if (clken)
	if (o[7] & &o[2:1])
		if (o[0]) p3 <= baseofs;
		else p2 <= baseofs;
	else if (ren & load_p23_1)
		if (o1[0]) p3 <= fwd_p3;
		else p2 <= fwd_p2;

// S register

wire cvalu = o[7:4] == 4'b0111 & ~|o[2:0] | o[7] & &o[5:4];
wire s_logic = o[7:2] == 6'b001110 & o[0];
reg cvalu1, s_logic1;
always @(posedge clk) if (clken) begin
	cvalu1 <= cvalu;
	s_logic1 <= s_logic;
end

wire c7add = add_a[7] & add_b[7] | add_b[7] & ~add_y[7] | ~add_y[7] & add_a[7];
wire c7sub = ~add_a[7] & add_b[7] | add_b[7] & add_y[7] | add_y[7] & ~add_a[7];
wire v7add = add_a[7] & add_b[7] & ~add_y[7] | ~add_a[7] & ~add_b[7] & add_y[7];
wire v15add = add_a[15] & add_b[15] & ~add_y[15] | ~add_a[15] & ~add_b[15] & add_y[15];
wire v7sub = add_a[7] & ~add_b[7] & ~add_y[7] | ~add_a[7] & add_b[7] & add_y[7];
wire v15sub = add_a[15] & ~add_b[15] & ~add_y[15] | ~add_a[15] & add_b[15] & add_y[15];

wire t_c = cvalu1 & (o1[3] ?
	o1[6] ? ~c7sub : ~add_y[16] : o1[6] ? c7add : add_y[16]) |
	i1[RRL] & a[0];
wire t_v = o1[3] ? o1[6] ? v7sub : v15sub : o1[6] ? v7add : v15add;

reg update_c, update_v;
always @(posedge clk) if (clken) begin
	update_c <= cvalu | i[RRL];
	update_v <= cvalu;
end

wire [7:0] fwd_s = i1[LDSA] ? a :
	s_logic1 ? o1[1] ? s | imm1[7:0] : s & imm1[7:0] : {
	ren & update_c ? t_c : s[7],
	ren & update_v ? t_v : s[6],
	sb, sa, f,
	intack ? 1'b0 : s[0]
};
assign f = s[3:1];
always @(posedge clk)
	if (reset) s <= 0;
	else if (clken) s <= fwd_s;


wire [7:0] ic = fwd_s[I] === 1 ? "I" : fwd_s[I] === 0 ? "-" : "?";
wire [7:0] vc = fwd_s[V] === 1 ? "V" : fwd_s[V] === 0 ? "-" : "?";
wire [7:0] cc = fwd_s[C] === 1 ? "C" : fwd_s[C] === 0 ? "-" : "?";
wire [15:0] _ea = { fwd_e, fwd_a };
wire [15:0] pc1 = pc + 1'b1;
initial $monitor("%x %x %x %x %x %x %x %x %s%s%s %x%xM %x %x %x",
	pc1, force_nop, o, _ea, fwd_t, fwd_sp, fwd_p2, fwd_p3, cc, vc, ic,
	wr_u, wr_l, adr_out, data_out, data_in);
endmodule
