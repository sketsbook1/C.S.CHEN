`timescale 1ns/1ps
module package_clk_div (
    input clk,      //system clk set at 10MHz
    input rst_n,
    input scl_en,

    output reg clk_en,
    output reg clk_en_half
);

parameter scl_speed = 50;  // i2c work speed at 200khz #50
parameter scl_half = 25;    

reg [16:0] counter;         // use to div the frequence
reg flag;
reg fast_start;             //make sure start condititon can get clk_en_half immediately

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 0;
        clk_en <= 0;
        clk_en_half <= 0;
    end
    else if (fast_start) begin
        counter <= scl_half;
    end
    else begin
        case (counter)
            scl_half: begin
                clk_en_half <= 1;
                counter <= counter + 1'b1;
            end

            scl_speed: begin
                clk_en <= 1;
                counter <= 0;
            end

            default: begin
                counter <= counter + 1'b1;
                clk_en <= 0;
                clk_en_half <= 0;
            end
        endcase
    end
end

always @(posedge clk) begin
    if(!scl_en) begin
        fast_start <= 0;
        flag <= 0;
    end
    else if (scl_en && !flag) begin
        fast_start <= 1;
        flag <=1;
    end
    else if (scl_en && flag) begin
        fast_start <= 0;
    end
end
    
endmodule