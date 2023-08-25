`timescale 1ns/1ps
module clk_div (
    input clk,      //system clk set at 10MHz
    input rst_n,
    input scl_en, 
    input scl_speed_sel,            //0:200khz      1:400khz

    output reg clk_en,
    output reg clk_en_half
);

reg [16:0] counter;         // use to div the frequence
reg flag;
reg fast_start;             //make sure start condititon can get clk_en_half immediately

reg [7:0]scl_speed;
reg [7:0]scl_half;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        counter <= 0;
        clk_en <= 0;
        clk_en_half <= 0;
    end
    else if (!scl_en) begin
        counter <= 0;
        clk_en <= 0;
        clk_en_half <= 0;
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

always @(posedge clk or negedge rst_n) begin
   if (!rst_n) begin
        scl_speed <= 50;     // i2c work speed at 200khz #50, 400Khz #25
        scl_half <= 25;
   end
   else begin
        if(scl_speed_sel) begin
            scl_speed <= 25;     // i2c work speed at 200khz #50, 400Khz #25
            scl_half <= 12;
        end
        else begin
            scl_speed <= 50;     // i2c work speed at 200khz #50
            scl_half <= 25;
        end
   end 
end
    
endmodule