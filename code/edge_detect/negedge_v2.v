module negedge_detect_v2 (
    input clk,
    input data_in,
    input rst_n,
    output reg falling_edge
);

reg edge_rst_n;
reg edge_detect;

always@(negedge data_in or negedge edge_rst_n) begin
    if(!edge_rst_n) begin
        edge_detect <= 0;
    end
    else begin
        edge_detect <= 1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        edge_rst_n <= 0;
    end
    else begin
        if(edge_detect) begin
            edge_rst_n <= 0;
        end
        else begin
            edge_rst_n <= 1;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        falling_edge <= 0;
    end
    else begin
        falling_edge <= edge_detect;
    end
end

endmodule