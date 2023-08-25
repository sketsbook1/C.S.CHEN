module float_addsub (FA,FB,FS,op);
input [31:0] FA,FB;
input op;        //0:add 1:sub
output [31:0] FS;

parameter float_zero=32'b0;
parameter float_negzero=32'h8000_0000;

wire [31:0] FB_M;
wire FA_S,FB_S,FS_S;
wire [7:0] FA_E,FB_E,FS_E;
wire [22:0] FA_F,FB_F,FS_F;
wire [25:0] FB_F_sh;
wire [25:0] FA_F_ext, FB_F_ext;
wire [25:0] FA_F_com, FB_F_com;
wire [25:0] FS_F_cal, FS_F_com;
wire [4:0] FS_shift_num;
wire [7:0] Ex_diff;
wire Valid,zero;
wire bothzero;

assign FB_M = {op^FB[31], FB[30:0]};

assign {FA_S,FA_E,FA_F} = {FA[30:23]>FB[30:23]}? FA :FB_M;
assign {FB_S,FB_E,FB_F} = {FA[30:23]>FB[30:23]}? FB_M :FA;

assign FA_F_ext = {3'b001,FA_F};
assign FB_F_ext = {3'b001,FB_F};

assign Ex_diff = FA_E - FB_E;
assign FB_F_sh = FB_F_ext >> Ex_diff;

assign FA_F_com = (FA_S)? ~FA_F_ext + 26'd1 : FA_F_ext;
assign FB_F_com = (FB_S)? ~FB_F_sh + 26'd1 : FB_F_sh;

assign FS_F_cal = FA_F_com + FB_F_com;
assign FS_F_com = (FS_F_cal[25])? ~FS_F_cal + 26'd1 : FS_F_cal;

PENC32 p0(.Din({8'd0,FS_F_com[23:0]}), .Dout(FS_shift_num), .Valid(Valid));

assign FS_S = FS_F_cal[25];
assign FS_E = (FS_F_com[24])? FA_E + 8'd1 : FA_E - (5'd23 - FS_shift_num);
assign FS_F = (FS_F_com[24])? FS_F_com[23:1]: FS_F_com[22:0] << (5'd23 - FS_shift_num); 

assign bothzero = ((FA == float_zero)||(FA == float_negzero))&&((FB == float_zero)||(FB == float_negzero));

assign zero = (~(Valid | FS_F_com[24]| FS_F_com[25]))||bothzero;
assign FS = (zero)? 32'd0 : {FS_S,FS_E,FS_F};
endmodule

module PENC32 (Din,Dout,Valid);
    input [31:0] Din;
    output[4:0] Dout;
    output Valid;

    wire [2:0] D0,D1,D2,D3;
    wire v0,v1,v2,v3;

    PENC8 P0(.Din(Din[7:0]), .Dout(D0), .Valid(v0));
    PENC8 P1(.Din(Din[15:8]), .Dout(D1), .Valid(v1));
    PENC8 P2(.Din(Din[23:16]), .Dout(D2), .Valid(v2));
    PENC8 P3(.Din(Din[31:24]), .Dout(D3), .Valid(v3));

    assign Valid = v0 | v1 | v2 | v3;
    assign Dout[4] = v3 | v2;
    assign Dout[3] = v3 | (!v2 & v1);
    assign Dout[2:0] = Dout[4] ? ((Dout[3])?D3:D2) : ((Dout[3])?D1:D0);    
endmodule

module PENC8 (Din,Dout,Valid);
    input [7:0] Din;
    output [2:0] Dout;
    output Valid;

    wire[1:0] A, B;
    wire v0 ,v1;

    PENC4 P0(.Din(Din[3:0]), .Dout(B), .Valid(v0));
    PENC4 P1(.Din(Din[7:4]), .Dout(A), .Valid(v1));

    assign Valid = v0 | v1;
    assign Dout[2] = v1;
    assign Dout[1:0] = (Dout[2])? A : B;
endmodule

module PENC4 (Din, Dout, Valid);
    input [3:0] Din;
    output [1:0] Dout;
    output Valid;

    assign Dout[1] = Din[3] | Din[2];
    assign Dout[0] = Din[3] | (Din[1]&(!Din[2]));
    assign Valid = |Din;
    
endmodule