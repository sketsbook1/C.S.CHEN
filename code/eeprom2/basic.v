`timescale 10ns / 1 ns
module eeprom_basic (
    input clk,rst_n,
    input scl_in,sda_in,
    input rw_continue,eepromrw_flag,    //rw_flag 1: read   rw_continue: active high
    input [7:0] wrdata,
    input [15:0] eeprom_addr,                //word address
    input scl_speed_sel,
    input start,

    output reg scl_oen,sda_oen,         //active low
    output reg data_seq_out,
    output reg ack_check,no_responce,
    output reg work_ready,
    output reg [7:0] rddata,
    output reg wp_pin                   //0:can write the eeprom
);

//state machine-----------------------------
parameter state_idle = 5'd0;
parameter state_start = 5'd1;
parameter state_stop = 5'd2;
parameter state_read = 5'd3;
parameter state_write = 5'd4;
parameter state_send_dev_addr = 5'd5;
parameter state_send_data = 5'd6;
parameter state_wait_ack = 5'd7;
parameter state_send_wordaddr = 5'd8;
parameter state_read_start = 5'd9;
parameter state_ack = 5'd10;
parameter state_nack = 5'd11;
parameter state_wait_five_mini_second = 5'd12;
parameter state_after_stop = 5'd13;          //after stop condition need some time to prepare next work

//-------------------
parameter write =0;
parameter read =1;
parameter device_addr = 7'b1010111;
parameter five_mini_second = 50500;

wire clk_en;
wire clk_en_half;
wire scl_in_falling;
wire clock_gate;

reg scl_en;                         //1:start the scl
reg rw_flag;                        //1:read 0:write
reg five_mini_second_flag;          //1:have been five mini second 0:need wait
reg next_state_en;                  //1:goto next state
reg read_initial;                   //1:initialized nedd reapeat a start & device addr again to radnom read
reg [4:0] current_state;
reg [4:0] next_state;               //use in ack to mark which state be next
reg [3:0] little_counter; 
reg [3:0] next_counter_set;         //set little counter for next state
reg [15:0] five_mini_counter;
reg [7:0] data_reg;                 //normal register
reg [7:0] device_addr_reg;          //device addr reg
reg [15:0] word_addr_reg;           //word address reg

assign clock_gate = clk & start;

negedge_detect scl_neg (.clk(clk), .rst_n(rst_n), .data_in(scl_in), .falling_edge(scl_in_falling));

clk_div u1 (.clk(clk), .rst_n(rst_n), .clk_en(clk_en), .scl_en(scl_en), .clk_en_half(clk_en_half), .scl_speed_sel(scl_speed_sel));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_state <= state_idle;
        sda_oen <= 1;
        scl_en <=0;
        work_ready <= 1;
        ack_check <= 0;
        no_responce <= 0;
        rddata <= 0;
        wp_pin <= 0;
        little_counter <= 4'b0;
        read_initial <= 0;
        device_addr_reg[7:1] <= device_addr;
        data_seq_out <= 0;
    end
    else begin
        case(current_state)
        
        state_idle: begin
            work_ready <=1;
            wp_pin <= 1;
            scl_en <= 0;
            read_initial <= 0;
            if(rw_continue) begin
                current_state <= state_start;
                if (eepromrw_flag) begin
                    rw_flag <= read;
                    word_addr_reg <= eeprom_addr;
                end
                else begin
                    rw_flag <= write;
                    word_addr_reg <= eeprom_addr;
                end
            end
        end

        state_start: begin
            work_ready <=0;
            scl_en <= 1;
            little_counter <= 8;
            if(clk_en_half) begin
                sda_oen <= 0;
                current_state <= state_send_dev_addr;
                device_addr_reg[7:1] <= device_addr;
                device_addr_reg[0] <= write;
            end
        end

        state_send_dev_addr: begin
            next_counter_set <= 4'b1111;
            if(clk_en_half && !scl_in) begin
                sda_oen <= device_addr_reg[7];
                if (little_counter == 0) begin
                    current_state <= state_wait_ack;
                end
            end
            else if (clk_en_half && scl_in) begin
                device_addr_reg <= device_addr_reg << 1;
                little_counter <= little_counter - 1'b1;
                next_state <= (read_initial)? state_read:state_send_wordaddr;
            end
        end

        state_wait_ack: begin
            sda_oen <= 1;
            next_state_en <=0;
            data_seq_out <=0;
            data_reg <= wrdata;
            if (clk_en_half && scl_in) begin
                if(sda_in == 0) begin
                    ack_check <= 1;
                    no_responce <= 0;
                    current_state <= next_state;
                    little_counter <= next_counter_set;
                    if(next_state == state_send_wordaddr) begin
                        sda_oen <= 0;
                    end
                    else if(next_state == state_write) begin
                        sda_oen <= 0;
                    end
                end
                else begin
                    ack_check <= 0;
                    no_responce <= 1;
                end
            end
        end

        state_send_wordaddr: begin
            ack_check <= 0;
            no_responce <= 0;
            if(scl_in_falling) begin
                sda_oen <= word_addr_reg[15];
                case (little_counter)
                    8: begin
                        next_state_en <=1;
                        next_state <= state_send_wordaddr;
                        next_counter_set <= 4'd7;
                    end
                    0: begin
                        next_state_en <=1;
                        next_state <= (rw_flag)?state_read_start:state_write;
                        wp_pin <= (rw_flag)? 1'b1:1'b0;
                        next_counter_set <= (rw_flag)?4'd7:4'd0;
                    end
                endcase
            end
            else if (clk_en_half && scl_in) begin
                word_addr_reg <= word_addr_reg << 1;
                little_counter <= little_counter - 1'b1;
            end
            else if(next_state_en && clk_en && scl_in) begin
                current_state <= state_wait_ack;
            end
        end

        state_read_start: begin
            if(clk_en_half && scl_in) begin
                sda_oen <= 0;
                ack_check <= 0;
                no_responce <= 0;
                next_state <= state_send_dev_addr;
                little_counter <= 8;
                next_state_en <=1;
                read_initial <=1;
                device_addr_reg[7:1] <= device_addr;
                device_addr_reg[0] <= read;
            end
            else if (next_state_en && clk_en && scl_in) begin
                current_state <= next_state;
            end
        end

        state_read: begin
            sda_oen <= 1;
            next_state_en <=0;
            ack_check <= 0;
            no_responce <= 0;
            if(clk_en_half && scl_in) begin
                data_reg[0] <= sda_in;
            end
            else if(clk_en_half && !scl_in) begin
                data_reg <= data_reg << 1;
                little_counter <= little_counter + 1'b1;
            end
            else if (clk_en && scl_in && little_counter == 7) begin     //go ack
                current_state <= state_ack;
                rddata <= data_reg;
                data_seq_out <= 1;
                sda_oen <= 0;
                if(rw_continue) begin
                    next_state <= state_read;
                    current_state <= state_ack;
                end
                else begin
                    current_state <= state_nack;
                end
            end
        end

        state_ack: begin
            data_seq_out <= 0;
            sda_oen <= 0;
            if (!rw_continue) begin
                current_state <= state_nack;
            end
            begin
                if(clk_en_half && !scl_in) begin
                    little_counter <= 0;
                    if (next_state_en) begin
                        current_state <= next_state;
                    end
                end
                else if (clk_en_half && scl_in) begin
                    next_state_en <=1;
                end
            end   
        end

        state_nack: begin
            data_seq_out <= 0;
            if(clk_en_half && !scl_in) begin
                sda_oen <= 1;
                next_state_en <=1;
            end
            else if (next_state_en && clk_en_half && scl_in) begin
                current_state <= state_stop;
            end
        end

        state_stop: begin
            if(scl_in_falling) begin
                //scl_en <= 0;
                sda_oen <= 0;
            end
            else if (clk_en_half && scl_in) begin
                sda_oen <=1;
                if (!five_mini_second_flag) begin
                    current_state <= state_wait_five_mini_second;
                end
                else begin
                    //scl_en <= 0;
                    current_state <= state_idle;
                end
            end
        end

        state_write:begin
            ack_check <= 0;
            no_responce <= 0;
            if(clk_en_half && !scl_in) begin
                sda_oen <= data_reg[7];
                if (little_counter == 8) begin
                    current_state <= state_wait_ack;
                    data_seq_out <=1;
                end
            end
            else if (clk_en_half && scl_in) begin
                data_reg <= data_reg << 1;
                little_counter <= little_counter +1'b1;
                next_state <= (rw_continue)? state_write:state_stop;
            end
        end

        state_wait_five_mini_second: begin
            scl_en <=0;
            if (five_mini_second_flag) begin
                current_state <= state_idle;
            end
        end
        
        endcase
    end
end

//--------------------------------

always @(posedge clk or negedge rst_n) begin               //control the scl clk
    if (!rst_n) begin
        scl_oen <= 0;
    end
    else if(scl_en) begin
        if(clk_en) begin
            if(scl_in) begin
                scl_oen <= 0;
            end
            else begin
                scl_oen <= 1;
            end    
        end
    end
    else begin
        scl_oen <= 1;
    end
end
//----------------------------------------
always @(posedge clk or negedge rst_n) begin               //five mini second check
    if (!rst_n) begin
        five_mini_counter <= 0;
        five_mini_second_flag <=1;
    end
    else if(current_state == state_stop && rw_flag ==0) begin
        five_mini_second_flag <=0;
    end
    else if (five_mini_counter == five_mini_second) begin
        five_mini_second_flag <=1;
        five_mini_counter <= 0;
    end
    else if (!five_mini_second_flag) begin
        five_mini_counter <= five_mini_counter +1'b1;
    end
end



endmodule