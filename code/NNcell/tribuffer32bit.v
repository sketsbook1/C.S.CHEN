module tristate_buffer32bit(input_x, enable, output_x);
    input[31:0]input_x;
    input enable;
    output [31:0]output_x;

    assign output_x = !enable? input_x : 32'bz;

endmodule
