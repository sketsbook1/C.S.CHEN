module posedge_detect (
    input clk,
    input rst_n,
    input data_in,
    output rising_edge
);

reg data_in0;
reg data_in1;

assign rising_edge = ~data_in0 & data_in1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        data_in0 <= 0;
        data_in1 <= 0;
    end
    else begin
        data_in0 <= data_in1;
        data_in1 <= data_in;
    end
end

endmodule