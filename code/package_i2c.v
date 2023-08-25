module package_i2c (
    input clk,                      //10Mhz
    input rst_n,
    input sda_a_in,scl_a_in,        //i2c to master
    output reg sda_a_oen,scl_a_oen,

    input sda_b_in,scl_b_in,        //i2c to eeprom
    input scl_speed_sel,
    output reg sda_b_oen,scl_b_oen,
    output wp_pin,                   //eeprom wp_pin active low (can write)
    output reg NN_enable,                      //enable the NN active high
    output NN_working_flag_output,
    output NN_weight_initialize_output,
    output error_flag_output,
    output bp_running_output,
    output bp_done_output
);
//state machine-----------------------
parameter state_idle = 4'h0;
parameter state_dev_addr = 4'h1;            //get the dev addr
parameter state_get_command = 4'h2;
parameter state_determine_command = 4'h3;    //also ack
parameter state_determine_mode = 4'h4;      //also ack
parameter state_send_message = 4'h5;
parameter state_ack_check = 4'hc;          //check ack

parameter state_eeprom_dev = 4'h6;
parameter state_eeprom_ack = 4'ha;
parameter state_eeprom_rw_determine = 4'h7;
parameter state_eeprom_write = 4'h8;        //wirte eeprom
parameter state_eeprom_read_dev = 4'hb;
parameter state_eeprom_read = 4'h9;         //read eeprom

//command-----------------------------
parameter Do_NN = 8'h00;            //run predict
parameter Do_BP = 8'h01;            //run backpropergation
parameter storage_weight = 8'h02;   //storage weight
parameter start_NN_inihtial = 8'h03; //initial the NN
parameter disable_NN = 8'h04;

//i2c slave information --------------
parameter header_write = 8'b01011000;
parameter header_read = 8'b01011001;
parameter eeprom_header = 8'b10101110;
parameter eeprom_header_read = 8'b10101111;

parameter write = 0;
parameter read = 1;

//i2c recieve ------------------------
parameter i2c_tx_working = 8'h11;
parameter i2c_tx_confirm = 8'h22;

parameter weight_not_initialize = 8'h33;
parameter weight_initializing = 8'h77;

parameter system_error = 8'h44;
parameter system_bp_running = 8'h55;
parameter system_idle = 8'h66;

//------------------------------------
wire clk_en,clk_en_half;

//------------------------------------
//reg NN_enable;                      //enable the NN active high
reg NN_work_sel;                    //1:do work 0:idle
reg NN_weight_output;
reg NN_back_propergation;
reg NN_rst_n;            
wire error_flag;
wire bp_done;
wire bp_running;
wire NN_sda_oen;
wire NN_scl_oen;
wire NN_working_flag;               //0:idle 1:working
wire NN_weight_initialize;           //1:have been initalization 0:haven't been initalization
wire [7:0] NN_error_flag_code;
wire [31:0] NN_output_delta;
wire [31:0] NN_output_predict;
wire [31:0] NN_output_delta_little_endian;
wire [31:0] NN_output_predict_little_endian;

//edge detect -----------------------
wire scl_a_falling;
wire scl_a_rising;
wire sda_a_falling;
wire sda_a_rising;
wire NN_working_flag_falling;

//system -----------------------------
reg [39:0] message_reg;              //for i2c data send 
reg [7:0] data_rx_reg;               //for i2c data recieve
reg [7:0] bit_cnt;                   //count bit
reg [3:0] little_counter;            //use in sending data
reg [3:0] little_counter2;          //use in sending data
reg ack_check;                       //use in ack_check
reg header_check;                   //1: header check 
reg scl_b_oen_inter;
reg sda_a_oen_inter;
reg a2b_scl_connect;   
reg a2b_sda_connect;
reg b2a_sda_connect;             
wire i2c_start;                      //detect the start condition
wire i2c_stop;                       //show the stop condition

reg reset_doubleside_control;        //0:dont reset 1:reset

reg reg_sda_a,reg_scl_a;
reg reg_sda_b,reg_scl_b;

//state machine----------------------- 
reg [3:0]current_state;

wire clk_10m;
pll_10m U2 (.refclk(clk), .outclk_0(clk_10m), .rst(~rst_n));

layer_controller core (.clk_10m(clk_10m), .rst_n(NN_rst_n), .error_flag(error_flag), 
                     .sda_in(sda_b_in), .scl_in(scl_b_in), .sda_oen(NN_sda_oen), .scl_oen(NN_scl_oen),
                     .wp_pin(wp_pin), .work(NN_work_sel), .weight_output(NN_weight_output),
                     .back_propergation(NN_back_propergation), .weightcache_initialize(NN_weight_initialize),
                     .bp_done(bp_done), .working(NN_working_flag), .output_delta(NN_output_delta),
                     .error_flag_code(NN_error_flag_code), .bp_running_flag(bp_running), .predict_data(NN_output_predict),
                     .scl_speed_sel(scl_speed_sel));

negedge_detect_v2  sda_neg (.clk(clk_10m), .rst_n(rst_n), .data_in(sda_a_in), .falling_edge(sda_a_falling));
posedge_detect_v2  sda_pos (.clk(clk_10m), .rst_n(rst_n), .data_in(sda_a_in), .rising_edge(sda_a_rising));
negedge_detect_v2  scl_neg (.clk(clk_10m), .rst_n(rst_n), .data_in(scl_a_in), .falling_edge(scl_a_falling));
posedge_detect_v2  scl_pos (.clk(clk_10m), .rst_n(rst_n), .data_in(scl_a_in), .rising_edge(scl_a_rising));

negedge_detect_v2  working_flag_pos (.clk(clk_10m), .rst_n(rst_n), .data_in(NN_working_flag), .falling_edge(NN_working_flag_falling));

assign i2c_start = sda_a_falling && scl_a_in;
assign i2c_stop = sda_a_rising && scl_a_in;

assign NN_output_delta_little_endian[31:24] = NN_output_delta [7:0];
assign NN_output_delta_little_endian[23:16] = NN_output_delta [15:8];
assign NN_output_delta_little_endian[15:8] = NN_output_delta [23:16];
assign NN_output_delta_little_endian[7:0] = NN_output_delta [31:24];

assign NN_output_predict_little_endian [31:24] = NN_output_predict [7:0];
assign NN_output_predict_little_endian [23:16] = NN_output_predict [15:8];
assign NN_output_predict_little_endian [15:8] = NN_output_predict [23:16];
assign NN_output_predict_little_endian [7:0] = NN_output_predict [31:24];

assign NN_working_flag_output = NN_working_flag;
assign NN_weight_initialize_output =  NN_weight_initialize;
assign error_flag_output = error_flag;
assign bp_running_output = bp_running;
assign bp_done_output = bp_done;

always @(posedge clk_10m or negedge rst_n) begin
    if (!rst_n) begin
        reg_sda_a <= 1'b1;
        reg_scl_a <= 1'b1;
        reg_sda_b <= 1'b1;
        reg_scl_b <= 1'b1;
    end
    else begin
        reg_sda_a <= sda_a_in;
        reg_scl_a <= scl_a_in;
        reg_sda_b <= sda_b_in;
        reg_scl_b <= scl_b_in;
    end
end

always @(*) begin                       //inter switch
    scl_b_oen <= (a2b_scl_connect)? reg_scl_a:NN_scl_oen;
    sda_b_oen <= (a2b_sda_connect)? reg_sda_a:NN_sda_oen;
    sda_a_oen <= (b2a_sda_connect)? reg_sda_b:sda_a_oen_inter;
end

always @(posedge clk_10m or negedge rst_n) begin
    if (!rst_n) begin
        sda_a_oen_inter <= 1;
        scl_a_oen <= 1;
        scl_b_oen_inter <= 1;
        a2b_scl_connect <= 1;
        a2b_sda_connect <= 1;
        b2a_sda_connect <= 0;
        little_counter <= 0;
        little_counter2 <= 0;
        message_reg <= 40'b0;
        //---------------------------
        NN_enable <= 0;
        NN_rst_n <= 0; 
    end
    else begin
        if (i2c_stop && !NN_working_flag) begin
            current_state <= state_idle;
            a2b_sda_connect <= 1;
            b2a_sda_connect <= 0;
            a2b_scl_connect <= 1;
            sda_a_oen_inter <= 1;
        end
        else if (i2c_start && current_state == state_idle) begin
            current_state <= state_dev_addr;
        end
        else begin
            case (current_state)

            state_idle : begin
                header_check <= 0;
                bit_cnt <= 0;
                sda_a_oen_inter <= 1;
                NN_work_sel <= 0;
                if(!NN_working_flag) begin          //check the NN core is working
                    NN_back_propergation <= 0;
                    NN_weight_output <= 0;  
                    if (error_flag) begin       //if NN error
                        NN_back_propergation <= 0;
                        NN_weight_output <= 0; 
                        NN_work_sel <= 0;
                        a2b_sda_connect <= 1;
                        b2a_sda_connect <= 0;
                        a2b_scl_connect <= 1;
                        message_reg[39:32] <= system_error;
                        message_reg[31:24] <= NN_error_flag_code;
                    end
                    else begin          //if NN no error
                        if(NN_weight_initialize) begin
                            message_reg[39:32] <= system_idle;
                            if (reset_doubleside_control) begin
                                a2b_sda_connect <= 1;
                                b2a_sda_connect <= 0;
                                a2b_scl_connect <= 1;
                            end 
                            else begin
                                message_reg [39:32] <= system_idle;
                                message_reg [31:0] <= NN_output_predict_little_endian;
                            end
                        end
                        else begin
                            message_reg[39:32] <= weight_not_initialize;
                        end
                    end
                end
                else begin
                    if(bp_running)begin
                        message_reg [39:32] <= system_bp_running;
                        message_reg [31:0] <= NN_output_delta_little_endian;
                    end
                end
                
             end

            state_dev_addr : begin
                if (bit_cnt == 8) begin
                    if (scl_a_rising) begin
                        data_rx_reg[0] <= sda_a_in;
                        sda_a_oen_inter <= 0;
                    end
                    else if (scl_a_falling) begin
                        current_state <= state_determine_mode;
                    end 
                end
                else if (scl_a_rising) begin
                    data_rx_reg <= data_rx_reg << 1;
                    data_rx_reg[0] <= sda_a_in;
                    bit_cnt <= bit_cnt + 1'b1;
                end
					 else if (bit_cnt >= 9)begin
					     bit_cnt <= 0;
					 end
            end

            state_determine_mode : begin        //determine go eeprom or system itself
                case (data_rx_reg)

                    eeprom_header : begin           //eeprom 
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                        if(scl_a_falling) begin
                            bit_cnt <= 0;
                            current_state <= state_eeprom_dev;
                            a2b_sda_connect <= 1;
                            b2a_sda_connect <= 0;
                        end
                    end

                    header_write : begin            //master send the command
                        a2b_scl_connect <= 0;
                        a2b_sda_connect <= 0;
                        sda_a_oen_inter <= 0;
                        if (scl_a_falling) begin
                            current_state <= state_get_command;
                            data_rx_reg <= 0;
                            bit_cnt <= 0;
                            sda_a_oen_inter <= 1;
                        end
                    end

                    header_read : begin             //slave(me!) send the command
                        a2b_scl_connect <= 0;
                        a2b_sda_connect <= 0;
                        sda_a_oen_inter <= 0;
                        if(scl_a_falling) begin
                            current_state <= state_send_message;
                            bit_cnt <= 38;
                            sda_a_oen_inter <= message_reg [39];
                        end
                    end

                    default : begin
                        sda_a_oen_inter <= 1;
                        current_state <= state_idle;
                    end

                endcase
            end

            state_get_command : begin           //get the i2c input command
                if (bit_cnt == 8) begin
                    if (scl_a_rising) begin
                        data_rx_reg[0] <= sda_a_in;
                        sda_a_oen_inter <= 0;
                    end
                    else if (scl_a_falling) begin
                        current_state <= state_determine_command;
                    end 
                end
                else if (scl_a_rising) begin
                    data_rx_reg <= data_rx_reg << 1;
                    data_rx_reg[0] <= sda_a_in;
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end

            state_determine_command : begin     //according the i2c command to activate the function
                sda_a_oen_inter <= 0;
                if (scl_a_rising) begin
                    bit_cnt <= 0;

                    if (error_flag) begin // NN core error
                        message_reg [39:32] <= system_error;
                        message_reg [31:24] <= NN_error_flag_code;
                        message_reg [23:0] <= 24'b0;
                        if (data_rx_reg == disable_NN) begin
                            sda_a_oen_inter <= 0;
                            NN_enable <= 0;
                            NN_rst_n <= 0; 
                        end
                    end 
                    else begin  // system ok
                        if (NN_working_flag) begin
                            message_reg [39:32] <= i2c_tx_working;
                            message_reg [31:0] <= 32'b0;
                        end //end NN_working_flag
                        else begin // NN idle
                            if (!NN_weight_initialize) begin
                                if(data_rx_reg == start_NN_inihtial) begin
                                    NN_enable <= 1;
                                    NN_rst_n <= 1; 
                                    NN_work_sel <= 0;
                                    message_reg[39:32] <= weight_initializing;
                                    message_reg[31:0] <= 32'b0;
                                end
                                else begin
                                    message_reg[39:32] <= weight_not_initialize;
                                end
                            end // end !NN_weight_initialize
                            else begin
                                case(data_rx_reg)
                                    Do_NN : begin
                                        sda_a_oen_inter <= 0;
                                        NN_work_sel <= 1;
                                        NN_weight_output <=0;
                                        NN_back_propergation <=0;
                                    end
            
                                    Do_BP : begin
                                        sda_a_oen_inter <= 0;
                                        NN_work_sel <= 1;
                                        NN_weight_output <=0;
                                        NN_back_propergation <=1;
                                    end
            
                                    storage_weight : begin
                                        sda_a_oen_inter <= 0;
                                        NN_work_sel <= 1;
                                        NN_weight_output <=1;
                                        NN_back_propergation <=1;
                                    end
            
                                    disable_NN : begin
                                        sda_a_oen_inter <= 0;
                                        NN_enable <= 0;
                                        NN_rst_n <= 0; 
                                    end
            
                                    default : begin
                                        sda_a_oen_inter <= 1;
                                    end
                                endcase
                            end // end !NN_weight_initialize
                        end //end NN_working_flag
                    end //end !error_flag
                end
                else if (scl_a_falling) begin
                    current_state <= state_idle;
                end
            end

            state_send_message : begin
                if(scl_a_falling) begin
                    if(little_counter == 5 && bit_cnt == 0) begin //end of sending message
                        current_state <= state_idle;
                        sda_a_oen_inter <= 1;
                        bit_cnt <= 40;
                        little_counter <= 0;
                        little_counter2 <= 0;
                        sda_a_oen_inter <= message_reg[bit_cnt];
                    end
                    else begin
                        case (little_counter2)
                            7 : begin
                                sda_a_oen_inter <= message_reg[bit_cnt];
                                current_state <= state_ack_check;
                            end
                            default : begin
                                bit_cnt <= bit_cnt - 1'b1;
                                sda_a_oen_inter <= message_reg[bit_cnt];
                                little_counter2 <= little_counter2 +1'b1;
                                ack_check <= 0;
                            end
                        endcase
                    end
                end
            end

            state_ack_check : begin
                sda_a_oen_inter <= 1;
                if(scl_a_rising) begin
                    if(!sda_a_in) begin      //ack check
                        ack_check <= 1;
                    end
                    else begin              //Nack
                        current_state <= state_idle;
                        little_counter <= 0;
                        little_counter2 <= 0;
                        bit_cnt <= 0;
                    end
                end
                else if (scl_a_falling) begin
                    sda_a_oen_inter <= message_reg[bit_cnt];
                    if (ack_check) begin
                        current_state <= state_send_message;
                        bit_cnt <= bit_cnt - 1'b1;
                        little_counter2 <= 0;
                        little_counter <= little_counter +'b1;
                    end
                end
            end

            state_eeprom_dev : begin
                if (scl_a_falling) begin
                    if(bit_cnt == 7) begin
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                    else if (bit_cnt == 16) begin
                        current_state <= state_eeprom_ack;
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                        bit_cnt <= 0;
                    end
                    else begin
                        bit_cnt <= bit_cnt + 1'b1;
                        a2b_sda_connect <= 1;
                        b2a_sda_connect <= 0;
                    end
                end
            end

            state_eeprom_ack : begin
                if(scl_a_falling) begin
                    current_state <= state_eeprom_rw_determine;
                    a2b_sda_connect <= 1;
                    b2a_sda_connect <= 0;
                end
            end

            state_eeprom_rw_determine : begin
                if(i2c_start) begin
                    current_state <= state_eeprom_read_dev;
                    a2b_sda_connect <= 1;
                    b2a_sda_connect <= 0;
                end
                else if(scl_a_falling) begin
                    current_state <= state_eeprom_write;
                    a2b_sda_connect <= 1;
                    b2a_sda_connect <= 0;
                    bit_cnt <= 2;
                end
            end

            state_eeprom_write : begin
                if(scl_a_falling) begin
                    if(bit_cnt == 8) begin
                        bit_cnt <= 0;
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                    end
                    else begin
                        bit_cnt <= bit_cnt + 1'b1;
                        a2b_sda_connect <= 1;
                        b2a_sda_connect <= 0;
                    end
                end
            end

            state_eeprom_read_dev : begin
                //a2b_sda_connect <= 1;
                //b2a_sda_connect <= 0;
                if(scl_a_falling) begin
                    if(bit_cnt == 8) begin
                        bit_cnt <= bit_cnt + 1'b1;
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                    end
                    else if (bit_cnt == 9) begin
                        current_state <= state_eeprom_read;
                        bit_cnt <= 1;
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                    end
                    else begin
                        bit_cnt <= bit_cnt + 1'b1;
                        a2b_sda_connect <= 1;
                        b2a_sda_connect <= 0;
                    end
                end
            end

            state_eeprom_read : begin
                if(bit_cnt == 8) begin
                    if(scl_a_falling) begin
                        a2b_sda_connect <= 1;
                        b2a_sda_connect <= 0;
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                else if(bit_cnt == 9) begin
                    bit_cnt <= 0;
                end
                else begin
                    if(scl_a_falling) begin
                        bit_cnt <= bit_cnt + 1'b1;
                        a2b_sda_connect <= 0;
                        b2a_sda_connect <= 1;
                    end
                end
            end
            
            endcase
        end
    end
end

//--------------------------------------------------------------------
reg counter_start;
reg [2:0] mini_delay;                //for the idle to reset the doubla side control

always @(posedge clk_10m or negedge rst_n) begin
    if(!rst_n) begin
        mini_delay <= 0;
        reset_doubleside_control <= 0;
        counter_start <= 0;
    end
    else begin
        if (NN_working_flag_falling) begin
            counter_start <= 1;
            mini_delay <= 0;
        end
        else begin
            if (counter_start) begin
                case (mini_delay)
                    4 : begin
                        reset_doubleside_control <= 1;
                        mini_delay <= mini_delay + 1'b1;
                    end 
                    5 : begin
                        reset_doubleside_control <= 0;
                        mini_delay <= 0;
                        counter_start <= 0;
                    end
                    default: begin
                        reset_doubleside_control <= 0;
                        mini_delay <= mini_delay + 1'b1;
                    end
                endcase
            end
            else begin
                
            end
        end
    end
end

endmodule