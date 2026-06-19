module divu(clk, start, a, b, q, r, busy);
input clk, start;
input [15:0] a, b;
output reg [15:0] q, r;
output reg busy;

reg [4:0] count = 0;
reg [30:0] d;
wire [31:0] t = r - d;
wire f = ~|t[31:16];
always @(posedge clk) begin
	if (start) begin
		count <= 16;
		r <= a;
		q <= 0;
		d <= { b, 15'b0 };
	end
	else if (|count) begin
		count <= count - 1'b1;
		if (f) r <= t[15:0];
		d <= { 1'b0, d[30:1] };
		q <= { q[14:0], f };
	end
	busy <= start | |count[4:1];
end

initial $monitor("%x %x %x %x %x %x", count, d, t, r, q, busy);
endmodule
