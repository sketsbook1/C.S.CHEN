module relu(Fin,Fout);

parameter data_width =32;

input [data_width-1:0] Fin;
output [data_width-1:0] Fout;

wire [data_width-1:0] allzero = {data_width{1'b0}};
assign Fout = (Fin[data_width-1] ==0 )? Fin:allzero;

endmodule 