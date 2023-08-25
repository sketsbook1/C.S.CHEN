module layer_controller (
	//system signal
	input clk_10m,rst_n,				//clk set at 10MHz
	output reg error_flag,
	//i2c signal
	input sda_in,scl_in,
	output sda_oen,scl_oen,
	//eeprom control
	input scl_speed_sel,
	output wp_pin,			//
	//system control
	input work,					//0: idle 1:work function
	input weight_output,		//1:do weight output 0:do back propergation
	input back_propergation,	//1:do bp 0:do predict
	//output signal
	output reg bp_done,
	output reg working,
	output reg weightcache_initialize,			//1:have been initalization 0:haven't been initalization
	output reg [31:0] output_delta,
	output reg [31:0] predict_data,
	output reg [31:0] target_data,
	output reg [7:0] error_flag_code,			//show different error		
	output reg bp_running_flag					//tell the nncell is runnning in bp mode? 1: yes 0: negtive
);

//normal parameter--------------------------------------------------------------------------
	parameter mode_width = 4;					//mode 4bit.max up to 14 weight and 1 bias
	parameter layer_width = 4;					//layer 4bit.max up to 15 Neural
	parameter NN_width = 3;						//NN    3bit.total 3 hidden layer and 1 output layer
	parameter data_width = 32;					//16bit
	parameter byte_width = 8;					//8bit
	parameter cachesel_width = 6;				//6bit 64set
	parameter weightsel_width = 8;				//8bit 256set 

	parameter write = 1'b0;
	parameter read = 1'b1;

//state machine-----------------------------------------------------------------------------
	parameter initial_state = 4'h0;				//initalize the datacache and weigtcache.also reset the layermode command
	parameter idle_state = 4'h1;				//wait for predict state
	parameter predict_state = 4'h2;				//start the NN 
	parameter bp_initial = 4'h3;				//set the learning rate ,train data num ,epoch
	parameter read_weight = 4'h4;				//read weight
	parameter error = 4'h5;						//didn't get the response form eeprom
	parameter eeprom_dataout = 4'h6;			//output the data to eeprom
	parameter eeprom_datain = 4'h7;				//input the data from eeprom
	parameter bp_eeprom_datain = 4'h8;			//input the train data form eerpom
	parameter bp_calculate_output_delta = 4'h9 ;
	parameter bp_calculate_delta = 4'ha;		//calculate the delta
	parameter bp_adjustweight = 4'hb;			//adjust the weight
	parameter bp_eeprom_dataout = 4'hc;			//store the weight to eerpom

//stage ------------------------------------------------------------------------------------
	parameter stage_state_update=4'h0;				//initalize the datacache and weigtcache.also reset the layermode command
	parameter stage_datacache_rw=4'h1;				//
	parameter stage_weightcache_rw=4'h2;			//
	parameter stage_eeprom_rw=4'h3;					//
	parameter stage_nncell=4'h4;					//
	parameter stage_deltacache_rw=4'h5;				//
	parameter stage_bp=4'h6 ;

//stage_cmd parameter-----------------------------------------------------------------------
	parameter stagecmd_initial = 4'h0;
	parameter stagecmd_storage = 4'h1;
	parameter stagecmd_datashift = 4'h2;
	parameter stagecmd_endcmd = 4'h3;
	parameter stagecmd_read = 4'h4;
	parameter stagecmd_dataproccess = 4'h5;
	parameter stagecmd_dataproccess2 = 4'h6;
	parameter stagecmd_wait = 4'h7;
	parameter stagecmd_datashift2 = 4'h8;
	parameter stagecmd_read2 = 4'h9;
	parameter stagecmd_wait2 = 4'ha;

//program counter parameter-----------------------------------------------------------------
	parameter pc_initial = 8'h00 ;				//initial value of the program counter

//error flag parameter 
	parameter error_with_out_eeprom = 8'h01;	//eeprom no responce
	parameter error_initial_ai_struct = 8'h02;	//counter set error 
	parameter error_flow_error = 8'h03;			//stage or stagecmd run in unexpected 
	parameter error_weight_out_of_memmory = 8'h04; 	//weight number greater then 256
	parameter error_data_out_of_memmory  = 8'h05;	//datacache addr gteater than 64
	parameter error_train_datanumber_zero = 8'h06;	//traindata_number is zero
	parameter error_learningrate_zero = 8'h07;

//address parameter-------------------------------------------------------------------------
	parameter eeprom_addr_initial =16'h0000;
	parameter eeprom_addr_traincmd = 16'h0006;
	parameter eeprom_addr_datain =16'h0020;
	parameter eeprom_addr_dataout =16'h0040;
	parameter eeprom_addr_weight = 16'h0060;
	parameter eeprom_addr_traindata = 16'h0460;

	parameter datacache_addr_initial = 6'h00;
	parameter datacache_addr_datain = 6'h01;
	parameter datacache_addr_tempdata = 6'h09;
	parameter datacache_addr_end = 6'd63;

	parameter weightcache_addr_initial = 8'h00;

	parameter deltacache_addr_inital = 6'h00;
	parameter deltacache_addr_datain = 6'h01;
	parameter deltacahce_addr_tempdata = 6'h09;

//counter-----------------------------------------------------------------------------------
	reg [mode_width-1:0] mode_counter;
	reg [layer_width-1:0] layer_counter;					
	reg [NN_width:0] NN_counter;							 

//reg for counter proccess------------------------------------------------------------------
	reg [mode_width-1:0] current_mode;					//reg for the system counter proccess
	reg [layer_width-1:0] current_layer;				//reg for the system counter proccess
	reg [NN_width:0] current_NN;

//reg for state machine---------------------------------------------------------------------
	reg [3:0] current_state;
	reg [3:0] stage;
	reg [3:0] stagecmd;

//NNcell reg--------------------------------------------------------------------------------
	reg NNcell_rst_n;
	reg NNcell_enable;
	reg NNcell_accumulate;
	reg NNcell_cachebus_write_enable;					//active high to allow nncell dataout to delta/datacache write
	reg NNcell_biasin;									//active high to set the datain to 1
	reg NNcell_addsub_op;								//0:add 1:sub
	reg NNcell_activefun_op;							//0 :no active function 1: use active function
	reg NNcell_multiplied;								//0 :add 1:multiplied

//sram reg----------------------------------------------------------------------------------
	reg datacache_cs_n,weightcache_cs_n,deltacache_cs_n;					//chip select active low
	reg datacache_wr_n,weightcache_wr_n,deltacache_wr_n;					//write enable active low
	reg deltacache_rd_n;									//active low to enable the deltacache data to nncell datain
	reg datacache_tempdata_wr_n;							//enable datatemp reg tribuffer active low
	reg deltacache_tempdata_wr_n;							//enable deltatemp reg tribuffer active low
	reg weightcache_tempdata_wr_n;							//enable deltatemp reg tribuffer active low
	reg [data_width-1:0] datacache_tempdata,weightcache_tempdata;			//temp restore the data inout
	reg [data_width-1:0] deltacache_tempdata,output_tempdata;
	reg [cachesel_width-1:0] datacache_addr,deltacache_addr;				//datacache address
	reg [weightsel_width-1:0] weightcache_addr;		//weightcache address
	reg [7:0] tempaddr_pointer;						//temp addr pointer for bp calculation
	reg [7:0] datacache_endaddr;					//end addr of the datacache

//wire -------------------------------------------------------------------------------------
	wire [data_width-1:0] cachebus_write,cachebus_read/*synthesis keep*/;
	wire [data_width-1:0] weightbus_write,weightbus_read;
	wire [data_width-1:0] NNcell_datain,NNcell_weightin;
	wire [data_width-1:0] deltabus_write,deltabus_read;
	wire [7:0]program_counter;		//counter to set the function	
	wire [1:0] NNcell_datain_sel;

//eeprom system ----------------------------------------------------------------------------
	reg security_switch,eepromstart_flag,eepromrw_flag,rw_contiune;	
	reg [15:0] eeprom_addr;	
	reg [15:0] eeprom_addr_nxt;		//reg storage the next addr of eeprom	
	reg eeprom_rst_n;
	reg [9:0] eeprom_data_cnt;		
	wire [7:0] eeprom_wrdata;				
	wire [7:0] eeprom_rddata;
	wire data_seq_out;
	wire ack_check;
	wire work_ready;
	wire no_responce;

//system reg--------------------------------------------------------------------------------
	reg Isbias;						//show the weight adjustment now the wieght is bias
	reg [7:0]temp_counter;			//temp counter  
	reg [3:0]little_counter;		//little counter use in adjust weight
	reg [7:0]datain_counter; 		//counter for bp_eeprom datain use 
	reg [5:0]addr_pointer;			//tempdata pointer
	reg [9:0]weight_number;			//read eerpom 0x0004 to set the number of weight
	reg [7:0]data_cmd;				//read eeprom 0x0005 to set the input number & layer number,input/output/layer
	
	reg datacache_initialize;		//1:have been initalization 0:haven't been initalization
	reg deltacache_initialize;
	reg NNcell_initialize;			//1:when the current layer is not in the first layer
	reg addr_pointer_flag;			//0:need to set the addr_pointer 1: addr_pointer have been set
	reg NNcell_weightin_one;		//1:set the weight in to one
	reg NNcell_weightin_delta;		//1:set the delta read bus to weightin

	reg NNcell_cachebus_read_n;		//acitve low allow cachebus data to datain bus	
	reg NNcell_datain_baisin_n;		//active low allow bias in to datain bus
	reg NNcell_datain_deltabus_read_n;	//active low allow deltabus read data in to datain bus


	reg NNcell_weightin_weightbus_read_n;		//set the tristate buffer active low
	reg NNcell_weightin_deltabus_read_n;		//set the tristate buffer active low
	reg NNcell_weightin_weightone_n;			//set the tristate buffer active low

//bp system reg-----------------------------------------------------------------------------
	reg [15:0] epoch;				//set the epoch time
	reg [15:0] epoch_counter;		//calculate how much epoch have been proccessed
	reg [7:0] train_data_num;		//set how much data set to train 
	reg [7:0] train_data_num_counter;	//count train data set


//component---------------------------------------------------------------------------------
	NNcell_v2 U1 (.x1(NNcell_datain), .w1(NNcell_weightin), .clk(clk_10m), .rst_n(NNcell_rst_n), .enable(NNcell_enable),
		.cachebus_write(cachebus_write), .cachebus_write_enable(NNcell_cachebus_write_enable), 
		.accumulate(NNcell_accumulate), .addsub_op(NNcell_addsub_op), .activefun_op(NNcell_activefun_op),
		.multiplied(NNcell_multiplied));

	sram_datacache U2 (	.data_out(cachebus_read), .data_in(cachebus_write), .addr(datacache_addr),
					.wr_n(datacache_wr_n), .clk(clk_10m), .rst_n(rst_n), .cs_n(datacache_cs_n));

	sram_weightcache U3(.data_out(weightbus_read), .data_in(weightbus_write), .addr(weightcache_addr),
					.wr_n(weightcache_wr_n), .clk(clk_10m), .rst_n(rst_n), .cs_n(weightcache_cs_n));

	sram_deltacache U7(.data_out(deltabus_read), .data_in(deltabus_write), .addr(deltacache_addr),
					.wr_n(deltacache_wr_n), .clk(clk_10m), .rst_n(rst_n), .cs_n(deltacache_cs_n));

	eeprom_basic u15	(.clk(clk_10m), .rst_n(rst_n), .scl_in(scl_in), .sda_in(sda_in), .rw_continue(rw_contiune), 
						.eepromrw_flag(eepromrw_flag), .wrdata(eeprom_wrdata), .eeprom_addr(eeprom_addr),
						.scl_oen(scl_oen), .sda_oen(sda_oen), .data_seq_out(data_seq_out), .ack_check(ack_check),
						.no_responce(no_responce), .work_ready(work_ready), .rddata(eeprom_rddata) , .wp_pin(wp_pin),
						.scl_speed_sel(scl_speed_sel));

	tristate_buffer32bit U5(.input_x(datacache_tempdata), .enable(datacache_tempdata_wr_n), .output_x(cachebus_write));
	tristate_buffer32bit U16(.input_x(weightcache_tempdata), .enable(weightcache_tempdata_wr_n), .output_x(weightbus_write));
	tristate_buffer32bit U6(.input_x(cachebus_write), .enable(~weightcache_tempdata_wr_n), .output_x(weightbus_write));
	tristate_buffer32bit U8(.input_x(deltacache_tempdata), .enable(deltacache_tempdata_wr_n), .output_x(deltabus_write));
	tristate_buffer32bit U18(.input_x(cachebus_write), .enable(~deltacache_tempdata_wr_n), .output_x(deltabus_write));
//NNcell datain-----------------------------------------------------------------------------
	tristate_buffer32bit U9(.input_x(32'h3F800000), .enable(NNcell_datain_baisin_n), .output_x(NNcell_datain));
	tristate_buffer32bit U10(.input_x(cachebus_read), .enable(NNcell_cachebus_read_n), .output_x(NNcell_datain));
	tristate_buffer32bit U11(.input_x(deltabus_read), .enable(NNcell_datain_deltabus_read_n), .output_x(NNcell_datain));
//NNcell weightin---------------------------------------------------------------------------
	tristate_buffer32bit U12(.input_x(weightbus_read), .enable(NNcell_weightin_weightbus_read_n), .output_x(NNcell_weightin));			//-1
	tristate_buffer32bit U13(.input_x(deltabus_read), .enable(NNcell_weightin_deltabus_read_n), .output_x(NNcell_weightin));
	tristate_buffer32bit U14(.input_x(32'h3F800000), .enable(NNcell_weightin_weightone_n), .output_x(NNcell_weightin));			//1
//FSM --------------------------------------------------------------------------------------
/*
	wire clk_10m;
	pll_10m pll (.refclk(clk), .outclk_0(clk_10m), .rst(~rst_n));
*/

assign eeprom_wrdata = datacache_tempdata [7:0];
assign program_counter = {stage,stagecmd};
assign NNcell_datain_sel = {NNcell_biasin,deltacache_rd_n};

always @(*) begin			//mux for the NNcell_datain
	case (NNcell_datain_sel)
		2'b11 : begin		//biasin
			NNcell_datain_baisin_n = 0;
			NNcell_cachebus_read_n = 1;
			NNcell_datain_deltabus_read_n =1;
		end	
		2'b00 : begin		//deltacache in
			NNcell_datain_baisin_n = 1;
			NNcell_cachebus_read_n = 1;
			NNcell_datain_deltabus_read_n =0;
		end			
		default : begin		//cachebus in
			NNcell_datain_baisin_n = 1;
			NNcell_cachebus_read_n = 0;
			NNcell_datain_deltabus_read_n =1;
		end
	endcase
end

always @(*) begin
	case({NNcell_weightin_one,NNcell_weightin_delta})
		2'b10 : begin
			NNcell_weightin_weightone_n = 0;
			NNcell_weightin_deltabus_read_n = 1;
			NNcell_weightin_weightbus_read_n = 1;
		end
		2'b01 : begin
			NNcell_weightin_weightone_n = 1;
			NNcell_weightin_deltabus_read_n = 0;
			NNcell_weightin_weightbus_read_n = 1;
		end
		default : begin
			NNcell_weightin_weightone_n = 1;
			NNcell_weightin_deltabus_read_n = 1;
			NNcell_weightin_weightbus_read_n = 0;
		end

	endcase
end

//Main function
always @(posedge clk_10m or negedge rst_n) begin
	if(!rst_n ) begin
		current_state <= initial_state; 
		mode_counter <= 4'b0000;
		layer_counter <= 4'b0000;
		NN_counter <= 4'b0000;
		//counter set
		current_NN <= 0;
		current_layer <= 0;
		current_mode <= 0;
		//NNCELL
		NNcell_rst_n <= 1'b0;
		NNcell_enable <= 1'b0;
		NNcell_accumulate <= 1'b0;
		NNcell_cachebus_write_enable <= 1'b0;
		NNcell_biasin <= 0;
		NNcell_addsub_op <= 0;
		NNcell_activefun_op <= 1;
		NNcell_multiplied <= 0;
		//SRAM
		datacache_cs_n <= 1'b1;
		weightcache_cs_n <= 1'b1;
		datacache_wr_n <= 1'b1;
		datacache_tempdata_wr_n <= 1'b1;
		deltacache_tempdata_wr_n <= 1;
		weightcache_tempdata_wr_n <= 1;
		weightcache_wr_n <= 1'b1;
		datacache_addr <= 8'b00000000;
		weightcache_addr <= 8'b00000000;
		deltacache_rd_n <= 1;
		//eeprom
		security_switch <= 1'b0;
		eepromstart_flag <= 1'b1;
		eepromrw_flag <= 1'b0;
		rw_contiune <= 1'b0;
		eeprom_data_cnt <= 8'b00000000;;
		eeprom_rst_n <= 1'b0;
		temp_counter <= 4'h0;
		datacache_tempdata <= 32'd0;
		weightcache_tempdata <= 32'd0;
		//system
		stage <= stage_state_update;
		stagecmd <= stagecmd_initial;
		temp_counter <= 8'd0;
		error_flag <= 0;
		bp_running_flag <= 0;
		NNcell_weightin_one <= 0;
		NNcell_weightin_delta <=0;
		data_cmd <= 0;
		weight_number <= 0;
		error_flag_code <= 8'hff;
		weightcache_initialize<= 0;
		working <= 0;
		//BP cmd
		train_data_num <= 0;
		bp_done <= 0;
	end
	else begin
		case (current_state)
			initial_state : begin
				working <= 1;
				case (program_counter)

					{stage_state_update,stagecmd_initial} : begin
						eeprom_data_cnt <= 8'd4;						//set how many byte to read start form zero
						stage <= stage_eeprom_rw;
						eeprom_addr <= eeprom_addr_initial;
						eeprom_rst_n <= 1'b1;
						eepromrw_flag <= read;
						stagecmd <= stagecmd_initial;
					end //end of pc_initial

					{stage_eeprom_rw,stagecmd_initial} : begin
						if (eeprom_data_cnt == 8'd0 && data_seq_out == 0) begin
							rw_contiune <= 1'b0;
							stagecmd <= stagecmd_initial;
							stage <= stage_datacache_rw;
							data_cmd [1:0] <= current_NN - 1'b1;
							weight_number <= weight_number + datacache_tempdata[7:4] * datacache_tempdata[3:0];
							if (datacache_tempdata[7:0] != 8'h00) begin
								data_cmd [4:2] <= datacache_tempdata [3:0];
							end
						end // end eeprom_data_cnt == 8'd0
						else begin							//eerprom read
							rw_contiune <= 1'b1;
							if (no_responce == 1 ) begin
								current_state <= error;
								stage <= stage_state_update;
								error_flag_code <= error_with_out_eeprom;
							end
							else if(data_seq_out == 1) begin
								stagecmd <= stagecmd_storage;
							end
							else begin
								stagecmd <= stagecmd;
							end // end else
						end
					end // end stage_eeprom_rw,stagecmd_initial

					{stage_eeprom_rw,stagecmd_storage} : begin
						case (eeprom_data_cnt)
							8'd4 : begin
								datacache_tempdata [7:0] <= eeprom_rddata;
								stagecmd <= stagecmd_datashift;
								temp_counter <= temp_counter + 1'b1;
							end 
							8'd3 : begin
								datacache_tempdata [7:0] <= eeprom_rddata;
								stagecmd <= stagecmd_datashift;
								temp_counter <= temp_counter + 1'b1;
							end 
							8'd2 : begin
								datacache_tempdata [7:0] <= eeprom_rddata;
								stagecmd <= stagecmd_datashift;
								temp_counter <= temp_counter + 1'b1;
							end 
							8'd1 : begin												//end of read eeprom proccess
								datacache_tempdata [7:0] <= eeprom_rddata;
								stagecmd <= stagecmd_initial;
								temp_counter <= temp_counter + 1'b1;

								eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
								rw_contiune <= 1'b0;
							end 
							default: stagecmd <= stagecmd_initial;
						endcase
					end // end stage_eeprom_rw,stagecmd_storage

					{stage_eeprom_rw,stagecmd_datashift} : begin
						if (!data_seq_out) begin
							weight_number <= weight_number + datacache_tempdata[7:4] * datacache_tempdata[3:0];
							case (eeprom_data_cnt)
								8'd4 : begin						//input layer
									eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
									datacache_tempdata <= datacache_tempdata << 8;
									stagecmd <= stagecmd_initial;
									if (datacache_tempdata[7:0] != 8'h00) begin
										current_NN <= current_NN + 1'b1;
										data_cmd [7:5] <= datacache_tempdata [7:4] -1'b1 ;
									end
									else begin
										current_state <= error;
										error_flag_code <= error_initial_ai_struct;
									end
								end 
								8'd3 : begin
									eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
									datacache_tempdata <= datacache_tempdata << 8;
									stagecmd <= stagecmd_initial;
									if (datacache_tempdata[7:0] != 8'h00) begin
										current_NN <= current_NN + 1'b1;
										data_cmd [4:2] <= datacache_tempdata [3:0];
									end
								end
								8'd2 : begin
									eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
									datacache_tempdata <= datacache_tempdata << 8;
									stagecmd <= stagecmd_initial;
									if (datacache_tempdata[7:0] != 8'h00) begin
										current_NN <= current_NN + 1'b1;
										data_cmd [4:2] <= datacache_tempdata [3:0];
									end
								end
							endcase
						end
					end // end stage_eeprom_rw,stagecmd_datashift

					{stage_datacache_rw,stagecmd_initial} : begin
						datacache_cs_n <= 1'b0;					//chip select enable
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						datacache_addr <= datacache_addr_initial;
						stagecmd <= stagecmd_storage;
						data_cmd[1:0] <= current_NN - 1'b1;
					end // end stage_datacache_rw,stagecmd_initial

					{stage_datacache_rw,stagecmd_storage} : begin
						datacache_wr_n <= 1'b0;
						datacache_tempdata_wr_n <= 1'b0;
						stagecmd <= stagecmd_endcmd;
					end // end stage_datacache_rw,stagecmd_storage

					{stage_datacache_rw,stagecmd_endcmd} : begin
						datacache_wr_n <= 1'b1;
						datacache_tempdata_wr_n <= 1'b1;
						current_state <= read_weight;
						stagecmd <= stagecmd_initial;
						stage <= stage_state_update;
					end // end stage_datacache_rw,stagecmd_endcmd
						
					default : begin
						current_state <= error;
						error_flag_code <= error_flow_error;
					end
					

				endcase // end promgram_counter
			end //end initial_state

			read_weight : begin
				case (program_counter)

					{stage_state_update,stagecmd_initial} : begin
						eeprom_data_cnt <= (weight_number << 2);
						eeprom_addr <= eeprom_addr_weight;
						eepromrw_flag <= read;
						stagecmd <= stagecmd_dataproccess;
						temp_counter <= 0;
					end // end stage_state_update,stagecmd_initial

					{stage_state_update,stagecmd_dataproccess} : begin		//check weight number if out of address (256)
						if (weight_number >= 256) begin
							current_state <= error;
							error_flag_code <= error_weight_out_of_memmory;
						end
						else begin
							stagecmd <= stagecmd_initial;
							stage <= stage_eeprom_rw;
						end
					end

					{stage_eeprom_rw,stagecmd_initial} : begin
						if (eeprom_data_cnt == 8'd0 && data_seq_out  == 0) begin
							rw_contiune <= 1'b0;
							stagecmd <= stagecmd_initial;
							stage <= stage_state_update;
							current_state <= idle_state;
						end // end eeprom_data_cnt == 8'd0
						else if (work_ready) begin
							rw_contiune <= 1'b1;
							stagecmd <= stagecmd_initial;
						end // end work_ready
						else if(data_seq_out == 1) begin
							stagecmd <= stagecmd_storage;
						end
					end // end stage_eeprom_rw,stagecmd_initial

					{stage_eeprom_rw,stagecmd_storage} : begin
						weightcache_tempdata[31:24] <= eeprom_rddata;
						stagecmd <= stagecmd_datashift;
						temp_counter <= temp_counter + 1'b1;
					end // end stage_eeprom_rw,stagecmd_storage

					{stage_eeprom_rw,stagecmd_datashift} : begin
						if (temp_counter < 4'd4 && data_seq_out == 1'b0) begin
							stagecmd <= stagecmd_initial;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
							weightcache_tempdata <= weightcache_tempdata >> 8;
						end // end temp_counter < 4'd4
						else begin
							if (data_seq_out == 1'b0 ) begin
								stage <= stage_weightcache_rw;
								temp_counter <= 3'b000;
								if (weightcache_initialize) begin
									stagecmd <= stagecmd_storage;
								end // end weightcache_initialize
								else begin
									stagecmd <= stagecmd_initial;
								end // end else
							end // end data_seq_out
						end // end else
					end // end stage_eeprom_rw,stagecmd_datashift

					{stage_weightcache_rw,stagecmd_initial} : begin
						weightcache_cs_n <= 1'b0;
						weightcache_addr <= weightcache_addr_initial;
						stagecmd <= stagecmd_storage;
						weightcache_initialize <= 1'b1;
					end

					{stage_weightcache_rw,stagecmd_storage} : begin
						weightcache_tempdata_wr_n <= 0;
						weightcache_wr_n <= 1'b0;
						stagecmd <= stagecmd_datashift;
					end

					{stage_weightcache_rw,stagecmd_datashift} : begin
						weightcache_wr_n <= 1'b1;
						weightcache_tempdata_wr_n <= 1;
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						weightcache_addr <= weightcache_addr + 1'b1;
						stagecmd <= stagecmd_endcmd;
					end

					{stage_weightcache_rw,stagecmd_endcmd} : begin
						stagecmd <= stagecmd_initial;
						if(eeprom_data_cnt !=0) begin
							stage <= stage_eeprom_rw;
						end // end eprom_data_cnt !=0
						else begin
							rw_contiune <= 1'b0;
							stage <= stage_state_update;
							current_state <= idle_state;
						end
					end
					default : begin
						current_state <= error;
						error_flag_code <= error_flow_error;
					end 
				endcase
			end // end read_weight

			idle_state : begin
				if (work_ready) begin
					working <= 0;
					bp_done <= 0;
					stage <= stage_state_update;
					stagecmd <= stagecmd_initial;
					bp_running_flag <= 0;
					current_state <= (!work)? idle_state :
									 (weight_output)? bp_eeprom_dataout :
									 (back_propergation)? bp_initial : eeprom_datain;
				end
			end // end idle_state

			eeprom_datain : begin
				working <= 1;
				case (program_counter) 

					{stage_state_update,stagecmd_initial} : begin
						stage <= stage_eeprom_rw;
						eeprom_addr <= eeprom_addr_datain;
						eeprom_data_cnt <= (data_cmd[7:5] << 2);
						eepromrw_flag <= read;
						datacache_initialize <= 0;
						stagecmd <= stagecmd_initial;
						datacache_initialize <= 0;
					end

					{stage_eeprom_rw,stagecmd_initial} : begin
						if (eeprom_data_cnt == 8'd0 && data_seq_out == 0) begin
							current_state <= predict_state;
							stagecmd <= stagecmd_initial;
							stage <= stage_datacache_rw;
							rw_contiune <= 1'b0;
						end // end eeprom_data_cnt == 8'd0
						else if (eeprom_data_cnt == 8'd1 && data_seq_out == 0) begin
							//rw_contiune <= 1'b0;
						end
						else begin
							if (no_responce == 1 ) begin
								current_state <= error;
								stage <= stage_state_update;
								error_flag_code <= error_with_out_eeprom;
							end
							else if (work_ready) begin
								rw_contiune <= 1'b1;
								stagecmd <= stagecmd_initial;
							end // end work_ready
							else if(data_seq_out == 1) begin
								stagecmd <= stagecmd_storage;
							end
							else begin
								stagecmd <= stagecmd;
							end // end else
						end // end else
					end

					{stage_eeprom_rw,stagecmd_storage} : begin
						datacache_tempdata[31:24] <= eeprom_rddata;
						stagecmd <= stagecmd_datashift;
						temp_counter <= temp_counter + 1'b1;
					end

					{stage_eeprom_rw,stagecmd_datashift} : begin
						if (temp_counter <4 && data_seq_out == 1'b0) begin
							stagecmd <= stagecmd_initial;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
							datacache_tempdata <= datacache_tempdata >> 8;
						end
						else if (data_seq_out == 1'b0) begin
							stage <=stage_datacache_rw;
							temp_counter <= 8'h00;
							if (datacache_initialize) begin
								stagecmd <= stagecmd_storage;
							end // end weightcache_initialize
							else begin
								stagecmd <= stagecmd_initial;
							end // end else
						end
					end

					{stage_datacache_rw,stagecmd_initial} : begin
						datacache_addr <= datacache_addr_datain;
						stagecmd <= stagecmd_storage;
						datacache_initialize <= 1'b1;
					end

					{stage_datacache_rw,stagecmd_storage} : begin
						datacache_wr_n <= 1'b0;
						datacache_tempdata_wr_n <= 1'b0;
						stagecmd <= stagecmd_datashift;
					end

					{stage_datacache_rw,stagecmd_datashift} : begin
						datacache_wr_n <= 1'b1;
						datacache_tempdata_wr_n <= 1'b1;
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						datacache_addr <= datacache_addr + 1'b1;
						stagecmd <= stagecmd_endcmd;
					end

					{stage_datacache_rw,stagecmd_endcmd} : begin
						stagecmd <= stagecmd_initial;
						if (eeprom_data_cnt != 0) begin
							stage <= stage_eeprom_rw;
						end							
						else begin
							rw_contiune <= 1'b0;
							stage <= stage_nncell;
							current_state <= predict_state;
						end	
					end


				endcase
			end // end eeprom_datain

			predict_state : begin
				working <= 1;
				case (program_counter)

					{stage_datacache_rw,stagecmd_initial} : begin
						stagecmd <= stagecmd_read;		
					end

					{stage_datacache_rw,stagecmd_read} : begin
						datacache_tempdata <= cachebus_read;
						stagecmd <= stagecmd_endcmd;
						datacache_addr <= datacache_addr + 1'b1;
					end

					{stage_datacache_rw,stagecmd_endcmd} : begin
						stage <= stage_nncell;
						stagecmd <= stagecmd_read;
					end

					{stage_nncell,stagecmd_initial} : begin
						NN_counter <= data_cmd[1:0]+1;
						current_NN <= data_cmd[1:0]+1;
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_initial;
						datacache_addr <= datacache_addr_initial;
						weightcache_addr <= weightcache_addr_initial;
						NNcell_biasin <= 0;
						NNcell_initialize <= 0;
						NNcell_rst_n <= 1'b1;
						NNcell_addsub_op <= 0;
					end

					{stage_nncell,stagecmd_read} : begin			//set the counter 
						mode_counter <= datacache_tempdata[31:28];
						layer_counter <= datacache_tempdata[27:24];
						current_mode <= datacache_tempdata[31:28];
						current_layer <= datacache_tempdata[27:24];
						if (NN_counter == 0) begin
							NNcell_accumulate <= 0;
							NNcell_enable <= 0;
							stage <= stage_state_update;
							stagecmd <= stagecmd_initial;
							current_state <= (bp_running_flag)? bp_calculate_output_delta:eeprom_dataout;
							temp_counter <= 0;
							NNcell_rst_n <= 0;
							datacache_endaddr <= datacache_addr;
						end
						else if (NNcell_initialize == 1'b0) begin
							stagecmd <= stagecmd_datashift;
							datacache_addr <= datacache_addr_datain ;

							NNcell_enable <= 1'b1;
							NNcell_accumulate <= 1'b1;
							addr_pointer <= 0;
						end
						else begin
							NNcell_accumulate <= 1'b1;
							NNcell_enable <= 1'b1;
							stagecmd <= stagecmd_datashift;
						end
					end

					{stage_nncell,stagecmd_datashift} : begin			//set different datain
						case (mode_counter)

							4'h0 : begin
								NNcell_enable <= 1'b0;
								NNcell_biasin <= 1'b0;
								stagecmd <= stagecmd_dataproccess;	
								weightcache_addr <= weightcache_addr + 1'b1;
								temp_counter <= temp_counter + 1'b1;
								datacache_wr_n <= 1'b1;
								if (NNcell_initialize == 1'b0 ) begin
									datacache_addr <= datacache_addr_datain;
								end
								else begin
									datacache_addr <= datacache_addr_tempdata + addr_pointer ;
								end
							end //end 0

							4'h1 : begin
								NNcell_enable <= 1'b0;
								NNcell_biasin <= 1'b0;
								NNcell_cachebus_write_enable <= 1'b1;
								stagecmd <= stagecmd_dataproccess;	
								datacache_addr <= datacache_addr_tempdata + temp_counter;
							end // end 1

							4'h2 : begin
								NNcell_enable <= 1'b0;
								NNcell_biasin <= 1'b1;
								weightcache_addr <= weightcache_addr + 1'b1;
								stagecmd <= stagecmd_dataproccess;
								if (NNcell_initialize == 1'b0) begin
									datacache_addr <= datacache_addr_datain + (current_mode - mode_counter);
								end
								else begin
									datacache_addr <= datacache_addr + 1'b1;
								end
							end // end 2

							default : begin
								NNcell_enable <= 1'b0;
								NNcell_biasin <= 1'b0;
								stagecmd <= stagecmd_dataproccess;	
								weightcache_addr <= weightcache_addr + 1'b1;
								datacache_wr_n <= 1'b1;
								datacache_addr <= datacache_addr + 1'b1;
							end // end defalut

						endcase	
					end

					{stage_nncell,stagecmd_dataproccess} : begin		//proccess the data
						if (datacache_addr > datacache_addr_end) begin
							current_state <= error;
							error_flag_code <= error_data_out_of_memmory;
						end
						else begin
							if (layer_counter > 1) begin
								case (mode_counter)
	
									4'h0 : begin
										NNcell_enable <= 1'b1;
										layer_counter <= layer_counter -1'b1;
										mode_counter <= current_mode;
										NNcell_cachebus_write_enable <= 1'b0;
										datacache_wr_n <= 1'b1;
										NNcell_accumulate <= 1'b1;
										stagecmd <= stagecmd_datashift;
									end // end 0
	
									4'h1 : begin
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										NNcell_accumulate <= 0;
										datacache_wr_n <= 1'b0;
										stagecmd <= stagecmd_datashift;
									end // end 1
	
									4'h2 : begin
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										stagecmd <= stagecmd_datashift;
									end // end 2
	
									default : begin
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										stagecmd <= stagecmd_datashift;
										NNcell_biasin <= 1'b0;
									end
	
								endcase
							end // end layer_counter != 0
							else if (layer_counter == 1) begin
								case (mode_counter)
				
									4'h0: begin
										NNcell_enable <= 1'b0;
										NN_counter <= NN_counter - 1'b1;
										NNcell_cachebus_write_enable <= 1'b0;
										datacache_wr_n <= 1'b1;
										NNcell_accumulate <= 1'b0;
										datacache_tempdata <= datacache_tempdata <<8;
										stagecmd <= stagecmd_read;
										NN_counter <= NN_counter - 1'b1;
									end // end 0
	
									4'h1 : begin
										NNcell_accumulate <= 0;
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										datacache_wr_n <= 1'b0;
										predict_data <= cachebus_write;
										stagecmd <= stagecmd_datashift;
										NNcell_initialize <= 1'b1;
										if (NN_counter == current_NN) begin
											addr_pointer <= addr_pointer;
										end
										else begin
											addr_pointer <= addr_pointer + current_mode -1;
										end
									end // end 1
	
									4'h2 : begin
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										stagecmd <= stagecmd_datashift;
									end //end 2
	
									default : begin
										NNcell_enable <= 1'b1;
										mode_counter <= mode_counter -1'b1;
										stagecmd <= stagecmd_datashift;
										NNcell_biasin <= 1'b0;
									end // end default
	
								endcase
							end
						end
					end
				endcase
			end

			eeprom_dataout : begin
				case(program_counter)

					{stage_state_update,stagecmd_initial} : begin
						stage <= stage_datacache_rw;
						eeprom_addr <= eeprom_addr_dataout;
						eeprom_data_cnt <= (data_cmd[4:2] << 2);
						eepromrw_flag <= write;
					end

					{stage_datacache_rw,stagecmd_initial} : begin
						//datacache_addr <= datacache_addr - data_cmd[4:2] +1 ;    
						stagecmd <= stagecmd_read;
					end

					{stage_datacache_rw,stagecmd_read} : begin
						if (eeprom_data_cnt == 0) begin		//end of eeprom_dataout
							stage <= stage_state_update;
							stagecmd <= stagecmd_initial;
							current_state <= idle_state;
							rw_contiune <= 0;
						end
						else begin
							datacache_tempdata <= cachebus_read;
							stagecmd <= stagecmd_datashift;
							little_counter <= 0;
						end
					end

					{stage_datacache_rw,stagecmd_datashift} : begin
						stage <= stage_eeprom_rw;
						stagecmd <= stagecmd_initial;
						datacache_addr <= datacache_addr + 1'b1;
					end

					{stage_eeprom_rw,stagecmd_initial} : begin
						stage <= stage_eeprom_rw;
						stagecmd <= stagecmd_storage;
						rw_contiune <= 1;
					end

					{stage_eeprom_rw,stagecmd_storage} : begin
						if(little_counter == 0) begin
							if(!data_seq_out) begin
								stagecmd <= stagecmd_wait2;
							end
						end
						else begin
							if (data_seq_out) begin
								stagecmd <= stagecmd_datashift;
								little_counter <= little_counter + 1'b1;
								eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
							end
							else if (no_responce == 1) begin
								current_state <= error;
								error_flag_code <= error_with_out_eeprom;
							end
							else begin
								stagecmd <= stagecmd;
							end
						end
					end

					{stage_eeprom_rw,stagecmd_datashift} : begin
						if(little_counter == 4) begin
							stage <= stage_datacache_rw;
							stagecmd <= stagecmd_read;
						end
						else begin
							datacache_tempdata <= datacache_tempdata >> 8;
							stagecmd <= stagecmd_wait;
						end
					end

					{stage_eeprom_rw,stagecmd_wait} : begin
						if (data_seq_out == 0) begin
							stagecmd <= stagecmd_storage;
							if(eeprom_data_cnt == 1) begin
								rw_contiune <= 0;
							end
						end
					end

					{stage_eeprom_rw,stagecmd_wait2} : begin
						if(data_seq_out) begin
							stagecmd <= stagecmd_datashift;
							little_counter <= little_counter +1'b1;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						end
					end

				endcase
			end	

			bp_initial : begin
				working <= 1;
				case (program_counter) 

				{stage_state_update,stagecmd_initial} : begin
					stage <= stage_eeprom_rw;
					stagecmd <= stagecmd_initial;
					eeprom_data_cnt <= 8'd7;
					eeprom_addr <= eeprom_addr_traincmd;
					eepromrw_flag <= read;
					stagecmd <= stagecmd_initial;
					deltacache_cs_n <= 0;
					epoch_counter <= 0;
					train_data_num_counter <= 0;
					eeprom_addr_nxt <= 0;
					bp_done <= 0;
				end

				{stage_eeprom_rw,stagecmd_initial} : begin
					if (eeprom_data_cnt == 8'd0 && data_seq_out == 0) begin
						rw_contiune <= 1'b0;
						stagecmd <= stagecmd_endcmd;
					end // end eeprom_data_cnt == 8'd0
					else begin 
						rw_contiune <= 1;
						if (no_responce == 1 ) begin
							current_state <= error;
							error_flag_code <= error_with_out_eeprom;
							stage <= stage_state_update;
						end
						else if(data_seq_out == 1) begin
							stagecmd <= stagecmd_storage;
						end
						else begin
							stagecmd <= stagecmd;
						end // end else
					end
				end

				{stage_eeprom_rw,stagecmd_storage} : begin
					case (eeprom_data_cnt)

						8'd7 : begin				//epoch
							epoch [15:8] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
						end

						8'd6 : begin				//epoch
							epoch [7:0] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
						end 
						8'd5 : begin				//lr
							deltacache_tempdata [7:0] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
						end 
						8'd4 : begin
							deltacache_tempdata [7:0] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
						end 
						8'd3 : begin
							deltacache_tempdata [7:0] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
						end 

						8'd2 : begin
							deltacache_tempdata [7:0] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
						end
						8'd1 : begin				//traindata_number
							train_data_num <= eeprom_rddata;
							stagecmd <= stagecmd_initial;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						end
						default: stagecmd <= stagecmd_initial;
					endcase
				end

				{stage_eeprom_rw,stagecmd_datashift} : begin
					if (temp_counter < 4'd4 && data_seq_out == 0) begin
						stagecmd <= stagecmd_initial;
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						deltacache_tempdata <= deltacache_tempdata << 8;
					end // end temp_counter
					else if (data_seq_out == 1'b0) begin
						stagecmd <= stagecmd_initial;
						stage <= stage_deltacache_rw;
						temp_counter <= 3'b000;
						if(deltacache_tempdata == 32'b0) begin		//lr = 0
							current_state <= error;
							error_flag <= 1;
							error_flag_code <= error_learningrate_zero;
						end
					end // end data_seq_out
				end

				{stage_deltacache_rw,stagecmd_initial} : begin
					eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
					deltacache_addr <= deltacache_addr_inital;
					stagecmd <= stagecmd_storage;
				end
					
				{stage_deltacache_rw,stagecmd_storage} : begin
					deltacache_wr_n <= 1'b0;
					deltacache_tempdata_wr_n <= 1'b0;
					stagecmd <= stagecmd_endcmd;
				end

				{stage_deltacache_rw,stagecmd_endcmd} : begin
					deltacache_wr_n <= 1'b1;
					deltacache_tempdata_wr_n <= 1'b1;
					stagecmd <= stagecmd_initial;
					stage <= stage_eeprom_rw;
				end

				{stage_eeprom_rw,stagecmd_endcmd} : begin
					if (work_ready) begin
						current_state <= bp_eeprom_datain;
						addr_pointer <= 0;
						stage <= stage_state_update;
						stagecmd <= stagecmd_initial;
					end
					else if (train_data_num == 8'b0)begin
						current_state <= error;
						error_flag_code <= error_train_datanumber_zero;
					end
				end
			endcase
			end

			bp_eeprom_datain : begin
				case (program_counter)

					{stage_state_update,stagecmd_initial} : begin
						eeprom_addr <= 	eeprom_addr_traindata + eeprom_addr_nxt;
						eepromrw_flag <= read;
						temp_counter <= 0;
						datain_counter <= 0;
						eeprom_data_cnt <= ((data_cmd[7:5] + data_cmd [4:2]) << 2);
						bp_running_flag <= 1;
						datacache_initialize <= 0;
						deltacache_initialize <= 0;
						if (train_data_num_counter == train_data_num) begin
							
							if (work_ready) begin
								stage <= stage_eeprom_rw;
								stagecmd <= stagecmd_initial;
								epoch_counter <= epoch_counter + 1'b1;
								eeprom_addr_nxt <= 0;
								train_data_num_counter <= 0;
							end
							else begin
								stage <= stage;
								stagecmd <= stagecmd;
							end
						end
						else if (epoch_counter == epoch) begin				//end of training
							current_state <= idle_state;
							stage <= stage_state_update;
							stagecmd <= stagecmd_initial;
							bp_running_flag <= 0;
							bp_done <= 1;
						end
						else begin
							eeprom_addr_nxt <= eeprom_addr_nxt + ((data_cmd[7:5] + data_cmd [4:2]) << 2);
							stage <= stage_eeprom_rw;
							stagecmd <= stagecmd_initial;
						end
					end

					{stage_eeprom_rw,stagecmd_initial} : begin
						//eeprom_addr <= 	eeprom_addr_traindata + eeprom_addr_nxt;
						if(eeprom_data_cnt == 0 && data_seq_out == 0) begin
							rw_contiune <= 1'b0;
							current_state <= predict_state;

							stage<= stage_nncell;
							stagecmd <= stagecmd_initial;
						end
						else begin
							rw_contiune <= 1'b1;
							if (no_responce == 1 ) begin
								current_state <= error;
								stage <= stage_state_update;
							end
							else if(data_seq_out == 1) begin
								stagecmd <= stagecmd_storage;
							end
							else begin
								stagecmd <= stagecmd;
							end // end else
						end
					end 

					{stage_eeprom_rw,stagecmd_storage} : begin
						if (datain_counter < data_cmd[7:5]) begin
							datacache_tempdata[31:24] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
						end // end datain
						else begin
							deltacache_tempdata[31:24] <= eeprom_rddata;
							stagecmd <= stagecmd_datashift2;
							temp_counter <= temp_counter + 1'b1;
						end // end targetin
					end

					{stage_eeprom_rw,stagecmd_datashift} : begin  // for datain
						if (temp_counter < 4'd4 && data_seq_out == 1'b0) begin
							stagecmd <= stagecmd_initial;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
							datacache_tempdata <= datacache_tempdata >> 8;
						end // end temp_counter < 4'd4
						else if (data_seq_out == 1'b0) begin
							stage <= stage_datacache_rw;
							temp_counter <= 0;
							if (datacache_initialize) begin
								stagecmd <= stagecmd_storage;
							end // end weightcache_initialize
							else begin
								stagecmd <= stagecmd_initial;
							end // end else
						end
					end

					{stage_datacache_rw,stagecmd_initial} : begin
						datacache_addr <= datacache_addr_datain;
						stagecmd <= stagecmd_storage;
						datacache_initialize <= 1'b1;
					end

					{stage_datacache_rw,stagecmd_storage} : begin
						datacache_wr_n <= 1'b0;
						datacache_tempdata_wr_n <= 1'b0;
						stagecmd <= stagecmd_datashift;
					end

					{stage_datacache_rw,stagecmd_datashift} : begin
						datacache_wr_n <= 1'b1;
						datacache_tempdata_wr_n <= 1'b1;
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						datacache_addr <= datacache_addr + 1'b1;
						stagecmd <= stagecmd_endcmd;
					end

					{stage_datacache_rw,stagecmd_endcmd} : begin
						stage <= stage_eeprom_rw;
						stagecmd <= stagecmd_initial;
						datain_counter <= datain_counter + 1'b1;
					end

					{stage_eeprom_rw,stagecmd_datashift2} : begin	//for target
						if (temp_counter < 4'd4 && data_seq_out == 1'b0) begin
							stagecmd <= stagecmd_initial;
							eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
							deltacache_tempdata <= deltacache_tempdata >> 8;
						end // end temp_counter < 4'd4
						else if (data_seq_out == 1'b0) begin
							stage <= stage_deltacache_rw;
							stagecmd <= stagecmd_initial; 
							temp_counter <= 0;
						end
					end

					{stage_deltacache_rw,stagecmd_initial} : begin
						deltacache_addr <= deltacache_addr_datain;
						stagecmd <= stagecmd_storage;
					end
				
					{stage_deltacache_rw,stagecmd_storage} : begin
						deltacache_cs_n <= 1'b0;
						deltacache_wr_n <= 1'b0;
						deltacache_tempdata_wr_n <= 0;
						target_data <= deltacache_tempdata;
						stagecmd <= stagecmd_datashift;
					end

					{stage_deltacache_rw,stagecmd_datashift} : begin
						deltacache_wr_n <= 1'b1;
						deltacache_tempdata_wr_n <= 1'b1;
						eeprom_data_cnt <= eeprom_data_cnt - 1'b1;
						deltacache_addr <= deltacache_addr + 1'b1;
						stagecmd <= stagecmd_endcmd;
					end

					{stage_deltacache_rw,stagecmd_endcmd} : begin
						deltacache_initialize <= 1'b1;
						stagecmd <= stagecmd_initial;
						if (eeprom_data_cnt != 0) begin
							stage <= stage_eeprom_rw;
						end							
						else begin				//end of bp_eeprom_datain
							rw_contiune <= 1'b0;
							stage <= stage_nncell;
							current_state <= predict_state;
							datain_counter <= datain_counter + 1'b1;
						end	
					end

					
				
				endcase
			end

			bp_calculate_output_delta : begin
				case (program_counter)

				{stage_state_update,stagecmd_initial} : begin
					stage <= stage_bp;
					stagecmd <= stagecmd_initial;
					temp_counter <= 0;							//count how many output have been proccess
					NNcell_rst_n <= 0;
					deltacache_addr <= deltacache_addr_datain;
					NNcell_activefun_op <= 0;
					//datacache_addr <= datacache_addr - data_cmd[4:2]; 
				end

				{stage_bp,stagecmd_initial} : begin		
					deltacache_wr_n <= 1;
					NNcell_cachebus_write_enable <=0;			
					if(temp_counter < data_cmd [5:2]) begin
						stage <= stage_bp;
						stagecmd <= stagecmd_datashift;
						deltacache_rd_n <= 0;
						NNcell_rst_n <= 1;
						mode_counter <= 2;
					end
					else begin	// end output delta calculate
						NNcell_enable <= 0;
						NNcell_accumulate <= 0;
						current_state <= bp_calculate_delta;
						datacache_addr <= datacache_addr_initial;
						stage <= stage_state_update;
						stagecmd <= stagecmd_initial;
					end
				end

				{stage_bp,stagecmd_datashift} : begin
					case (mode_counter)

						4'h2 : begin				//load B (Ytarget)
							NNcell_enable <= 0;
							NNcell_accumulate <= 1'b1;
							deltacache_rd_n <= 0;
							stagecmd <= stagecmd_dataproccess;
							NNcell_weightin_one <= 1;
						end

						4'h1 : begin				//load A (Real output) to do sub
							NNcell_enable <= 0;
							deltacache_rd_n <= 1;
							stagecmd <= stagecmd_dataproccess;
							NNcell_weightin_one <= 1;
							NNcell_addsub_op <= 1;
						end

						4'h0 : begin				//end of calculate goto stagecmd_initial (rst nncell)
							NNcell_enable <= 0;
							deltacache_addr	 <= deltacahce_addr_tempdata + temp_counter;
							stagecmd <= stagecmd_dataproccess;
							temp_counter <= temp_counter + 1'b1;
							NNcell_cachebus_write_enable <= 1'b1;
							deltacache_wr_n <= 0;
							datacache_addr <= datacache_addr + 1'b1;
						end

					endcase
				end

				{stage_bp,stagecmd_dataproccess} : begin				
					case (mode_counter)

						4'h2 : begin				//load B
							NNcell_enable <= 1;
							mode_counter <= mode_counter -1'b1;
							stagecmd <= stagecmd_datashift;
						end

						4'h1 : begin				//load A
							NNcell_enable <= 1'b1;
							mode_counter <= mode_counter -1'b1;
							stagecmd <= stagecmd_datashift;
						end

						4'h0 : begin				//rst NNcell
							NNcell_enable <= 1'b1;
							NNcell_accumulate <= 0;
							stagecmd <= stagecmd_initial;
							deltacache_addr	 <= deltacache_addr_datain + temp_counter;
							deltacache_wr_n <= 1;
							addr_pointer <= deltacache_addr;
							output_delta <= deltabus_write;
						end

					endcase
				end
				endcase
			end

			bp_calculate_delta : begin
				case (program_counter)
					
					{stage_state_update,stagecmd_initial} : begin		//set datacache addr to read modecmd
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_read;
						datacache_addr <= datacache_addr_initial;
						NNcell_accumulate <= 0;
					end

					{stage_datacache_rw,stagecmd_read} : begin			//read the modecmd to datacache_tempdata
						datacache_tempdata <= cachebus_read;
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_dataproccess;
					end

					{stage_datacache_rw,stagecmd_dataproccess} : begin		//proccess modecmd for the bp
						case (current_NN)
							3'b001 : begin
								datacache_tempdata <= datacache_tempdata >> 24;
							end
							3'b010 : begin
								datacache_tempdata <= datacache_tempdata >> 16;
							end
							3'b011 : begin
								datacache_tempdata <= datacache_tempdata >> 8;
							end
							default : begin
								datacache_tempdata <= datacache_tempdata;
							end
						endcase
						stage <= stage_nncell;
						stagecmd <= stagecmd_initial;
					end

					{stage_nncell,stagecmd_initial} : begin
						NN_counter <= data_cmd[1:0]+1;
						current_NN <= data_cmd[1:0]+1;

						deltacache_addr <= addr_pointer ;//+ data_cmd[5:2];
						weightcache_addr <= weightcache_addr - 1'b1;
						tempaddr_pointer <= weightcache_addr;

						NNcell_biasin <= 0;
						NNcell_initialize <= 0;
						NNcell_rst_n <= 1'b1;
						NNcell_addsub_op <= 0;
						NNcell_biasin <= 0;
						NNcell_weightin_one <= 0;

						stage <= stage_nncell;
						stagecmd <= stagecmd_read;

						deltacache_rd_n <= 0;
					end

					{stage_nncell,stagecmd_read} : begin
						mode_counter <= datacache_tempdata[3:0];
						layer_counter <= datacache_tempdata[7:4];
						current_mode <= datacache_tempdata[3:0];
						current_layer <= datacache_tempdata[7:4];
						temp_counter <= datacache_tempdata[3:0];
						if (NN_counter == 1) begin				//end of nncell delta calculation
							NNcell_accumulate <= 0;
							NNcell_enable <= 0;
							temp_counter <= 0;
							NNcell_rst_n <= 0;
							current_state <= bp_adjustweight;
							stage <= stage_state_update;
							stagecmd <= stagecmd_initial;
						end
						else begin
							stagecmd <= stagecmd_datashift;
							NNcell_enable <= 1'b1;
							NNcell_accumulate <= 1'b1;
							deltacache_addr <= addr_pointer;
						end
					end

					{stage_nncell,stagecmd_datashift} : begin
						case (mode_counter)

							4'h0 : begin
								NNcell_enable <= 1'b0;
								stagecmd <= stagecmd_dataproccess;	
								weightcache_addr <= weightcache_addr - 1'b1;
								temp_counter <= temp_counter + 1'b1;
								deltacache_wr_n <= 1'b1;
								if (NNcell_initialize == 0) begin
									deltacache_addr <= addr_pointer;
								end
								else begin
									deltacache_addr <= addr_pointer;
								end
							end

							4'h1 : begin
								NNcell_enable <= 1'b0;
								NNcell_biasin <= 1'b0;
								NNcell_cachebus_write_enable <= 1'b1;
								stagecmd <= stagecmd_dataproccess;	
								deltacache_addr <= deltacache_addr + temp_counter;
							end // end 1

							default : begin
								NNcell_enable <= 1'b0;
								stagecmd <= stagecmd_dataproccess;	
								weightcache_addr <= weightcache_addr - 1'b1;
								datacache_wr_n <= 1'b1;
								deltacache_addr <= deltacache_addr - 1'b1;
							end

						endcase
					end
					
					{stage_nncell,stagecmd_dataproccess} : begin
						if (layer_counter > 1) begin
							case (mode_counter)

								4'h0 : begin
									NNcell_enable <= 1'b1;
									layer_counter <= layer_counter -1'b1;
									mode_counter <= current_mode;
									NNcell_cachebus_write_enable <= 1'b0;
									NNcell_accumulate <= 1'b1;
									stagecmd <= stagecmd_datashift;
								end // end 0

								4'h1 : begin
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									NNcell_accumulate <= 0;
									deltacache_wr_n <= 1'b0;
									stagecmd <= stagecmd_datashift;
									
								end // end 1

								4'h2 : begin
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									stagecmd <= stagecmd_datashift;
								end // end 2

								default : begin
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									stagecmd <= stagecmd_datashift;
									NNcell_biasin <= 1'b0;
								end

							endcase
						end // end layer_counter != 0
						else if (layer_counter == 1) begin
							case (mode_counter)
			
								4'h0: begin
									NNcell_enable <= 1'b0;
									NN_counter <= NN_counter - 1'b1;
									NNcell_cachebus_write_enable <= 1'b0;
									deltacache_wr_n <= 1'b1;
									NNcell_accumulate <= 1'b0;
									datacache_tempdata <= datacache_tempdata >> 8;
									stagecmd <= stagecmd_read;
									NN_counter <= NN_counter - 1'b1;
									
								end // end 0

								4'h1 : begin
									NNcell_accumulate <= 0;
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									deltacache_wr_n <= 1'b0;
									stagecmd <= stagecmd_datashift;
									NNcell_initialize <= 1'b1;
									addr_pointer <= deltacache_addr ;
								end // end 1

								4'h2 : begin
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									stagecmd <= stagecmd_datashift;
								end //end 2

								default : begin
									NNcell_enable <= 1'b1;
									mode_counter <= mode_counter -1'b1;
									stagecmd <= stagecmd_datashift;
									NNcell_biasin <= 1'b0;
								end // end default

							endcase
						end
					end

				endcase
			end

			bp_adjustweight : begin
				case (program_counter)

					{stage_state_update,stagecmd_initial} : begin
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_read;
						datacache_addr <= datacache_addr_initial;
						Isbias <= 1;
					end

					{stage_datacache_rw,stagecmd_read} : begin			//read the modecmd to datacache_tempdata
						datacache_tempdata <= cachebus_read;
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_dataproccess;
					end

					{stage_datacache_rw,stagecmd_dataproccess} : begin		//proccess modecmd for the bp
						case (current_NN)
							3'b001 : begin
								datacache_tempdata <= datacache_tempdata >> 24;
							end
							3'b010 : begin
								datacache_tempdata <= datacache_tempdata >> 16;
							end
							3'b011 : begin
								datacache_tempdata <= datacache_tempdata >> 8;
							end
							default : begin
								datacache_tempdata <= datacache_tempdata;
							end
						endcase
						stage <= stage_nncell;
						stagecmd <= stagecmd_initial;
					end

					{stage_nncell,stagecmd_initial} : begin
						NN_counter <= data_cmd[1:0]+1;
						current_NN <= data_cmd[1:0]+1;

						deltacache_addr <= deltacahce_addr_tempdata;
						weightcache_addr <= tempaddr_pointer - 1;		//set the weight addr to the previous end one

						NNcell_biasin <= 0;
						NNcell_initialize <= 0;
						NNcell_rst_n <= 1'b1;
						NNcell_addsub_op <= 1;
						NNcell_biasin <= 0;
						NNcell_weightin_one <= 0;
						NNcell_activefun_op <= 0;

						stage <= stage_nncell;
						stagecmd <=stagecmd_read;

						deltacache_rd_n <= 1;
					end

					{stage_nncell,stagecmd_read} : begin
						mode_counter <= datacache_tempdata[7:4];
						layer_counter <= datacache_tempdata[3:0];
						current_mode <= datacache_tempdata[7:4];
						current_layer <= datacache_tempdata[3:0];
						temp_counter <=0;										
						stage <= stage_datacache_rw;					//goto read the output data to decide update the weight
						if (NNcell_initialize == 0)begin				
							stagecmd <= stagecmd_read2;					//read the last addr
						end
						else if (NN_counter == 0) begin					//end of update weight
							current_state <= bp_eeprom_datain;
							stage <= stage_state_update;
							stagecmd <= stagecmd_initial;
							train_data_num_counter <= train_data_num_counter + 1'b1;
						end
						else begin
							stagecmd <= stagecmd_datashift2;			//read the previus addr
						end
					end

					{stage_datacache_rw,stagecmd_read2} : begin
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_datashift2;
						datacache_addr <= datacache_endaddr - data_cmd[4:2];
						addr_pointer <= datacache_endaddr - data_cmd[4:2];
					end	

					{stage_datacache_rw,stagecmd_datashift2} : begin 		//enter point to start the weight updating
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_dataproccess2;
						if (!Isbias) begin
							datacache_addr <= addr_pointer - temp_counter;
						end
						else if (layer_counter == 0) begin
							NN_counter <= NN_counter - 1'b1;
							datacache_tempdata <= datacache_tempdata >> 8;
							stage <= stage_nncell;
							stagecmd <= stagecmd_read;
							NNcell_initialize <= 1;
							addr_pointer <= datacache_addr;
						end
						else if (NN_counter == 1) begin						// input layer
							addr_pointer <= datacache_addr_datain + data_cmd [7:5];
						end
					end

					{stage_datacache_rw,stagecmd_dataproccess2} : begin		//decide to update the weight
						temp_counter <= temp_counter + 1'b1;
						if (cachebus_read > 0 || Isbias) begin				//update the weight 
							stage <= stage_nncell;
							stagecmd <= stagecmd_datashift;
							little_counter <= 4;
							NNcell_accumulate <= 1;
						end
						else begin										//dont need to update the weight
							stage <= stage_weightcache_rw;
							stagecmd <= stagecmd_datashift;
						end
					end

					{stage_nncell,stagecmd_datashift} : begin
						
						case (little_counter)
							4'h4 : begin								//read the delta to do multiplied
								NNcell_enable <= 1'b0;
								NNcell_weightin_delta <= 1;
								stagecmd <= stagecmd_dataproccess;
								if (Isbias) begin
									NNcell_biasin <= 1;
								end
							end
							4'h3 : begin								//read the learning rate
								NNcell_enable <= 0;
								NNcell_weightin_delta <= 0;
								NNcell_biasin <= 0;
								deltacache_rd_n <= 0;
								deltacache_addr <= deltacache_addr_inital;
								stagecmd <= stagecmd_dataproccess;
								NNcell_multiplied <= 0;
								deltacache_rd_n <= 0;
							end	
							4'h2 : begin								//read the weight to sub
								NNcell_enable <= 0;						
								deltacache_addr <= tempaddr_pointer ;
								stagecmd <= stagecmd_dataproccess;
								NNcell_multiplied <= 1;	
								deltacache_rd_n <= 1;
								NNcell_biasin <= 1;
							end	
							4'h1 : begin								//storage the new weight
								NNcell_enable <= 0;
								NNcell_accumulate <= 0;
								NNcell_multiplied <= 0;	
								NNcell_cachebus_write_enable <= 1;
								stagecmd <= stagecmd_dataproccess;
								NNcell_biasin <= 1;
								weightcache_wr_n <= 0;
							end	

						endcase
					end

					{stage_nncell,stagecmd_dataproccess} : begin
						case (little_counter)
							4'h4 : begin								//do multiplied
								NNcell_enable <= 1;
								tempaddr_pointer <= deltacache_addr;
								stagecmd <= stagecmd_wait;
							end
							4'h3 : begin								//do multplied
								NNcell_enable <= 1;
								stagecmd <= stagecmd_wait;

							end
							4'h2 : begin								//do sub
								NNcell_enable <= 1;
								stagecmd <= stagecmd_datashift;

								little_counter <= little_counter - 1'b1;
							end
							4'h1 : begin								//stop nncell and goto weightcache
								NNcell_enable <= 1;
								stage <= stage_weightcache_rw;
								stagecmd <= stagecmd_storage;

								little_counter <= little_counter - 1'b1;
							end
						endcase
					end

					{stage_nncell,stagecmd_wait} : begin				//only do mulplied need a extra clk
						little_counter <= little_counter - 1'b1;
						case (little_counter)
							4'h4: begin
								NNcell_enable <= 0;	
								stagecmd <= stagecmd_datashift;
							end
							4'h3: begin
								NNcell_enable <= 0;	
								stagecmd <= stagecmd_datashift;
							end
						endcase
					end

					{stage_weightcache_rw,stagecmd_storage} : begin
						NNcell_enable <= 0;
						NNcell_accumulate <= 0;
						NNcell_biasin <= 0;
						stage <= stage_weightcache_rw;
						stagecmd <= stagecmd_datashift;
						weightcache_wr_n <= 1;
						NNcell_cachebus_write_enable <= 0;
						if (Isbias) begin
							Isbias <= 0;
						end
					end

					{stage_weightcache_rw,stagecmd_datashift} : begin		//end of weight adjust proccess
						stage <= stage_datacache_rw;
						stagecmd <= stagecmd_datashift2;
						weightcache_wr_n <= 1;
						weightcache_addr <= weightcache_addr - 1'b1;
						if (mode_counter == 1) begin
							layer_counter <= layer_counter - 1'b1;
							mode_counter <= current_mode;
							deltacache_addr <= deltacache_addr + 1'b1;
							Isbias <= 1;
							temp_counter <= 0;
						end
						else begin
							mode_counter <= mode_counter - 1'b1;
						end
					end

				endcase
			end

			bp_eeprom_dataout : begin
				working <= 1;
				case (program_counter)
					{stage_state_update,stagecmd_initial} : begin		//initial the state
						stage <= stage_weightcache_rw;
						stagecmd <= stagecmd_initial;
						eepromrw_flag <= write;
						eeprom_addr <= eeprom_addr_weight;
						datain_counter <= 0;		//caulculte for 32 byte
						little_counter <= 0;		//calculate for 4 byte
					end 

					{stage_weightcache_rw,stagecmd_initial} : begin		//set the weightcache addr
						stage <= stage_weightcache_rw;
						stagecmd <= stagecmd_read;
						weightcache_addr <= weightcache_addr_initial;
						temp_counter <= 0;
					end

					{stage_weightcache_rw,stagecmd_read} : begin		//start to read the weightcache data
						if (temp_counter == weight_number) begin		//end of eeprom write
							stagecmd <= stagecmd_endcmd;
							rw_contiune <= 0;
						end
						else begin
							datacache_tempdata <= weightbus_read;
							stagecmd <= stagecmd_datashift;
							temp_counter <= temp_counter + 1'b1;
							little_counter <= 0;
						end
					end

					{stage_weightcache_rw,stagecmd_datashift} : begin	//move the weight addr pointer to next one
						stage <= stage_eeprom_rw;
						stagecmd <= stagecmd_storage;
						weightcache_addr <= weightcache_addr + 1'b1;
					end

					{stage_eeprom_rw,stagecmd_storage} : begin			//stoarge the data to eeprom
						rw_contiune <= 1;
						if (data_seq_out) begin
							stagecmd <= stagecmd_datashift;
							little_counter <= little_counter + 1'b1;
							datain_counter <= datain_counter + 1'b1;
						end
						else if (no_responce == 1) begin
							current_state <= error;
						end
						else begin
							stagecmd <= stagecmd;
						end
					end

					{stage_eeprom_rw,stagecmd_datashift} : begin		//shift the datacache_temp data
						datacache_tempdata <= datacache_tempdata >> 8;
						if (datain_counter == 31) begin					//eeprom need time to write every 32 byte
							stage <= stage_eeprom_rw;
							stagecmd <= stagecmd_wait2;
							eeprom_addr <= eeprom_addr + 32;
						end
						else begin
							stagecmd <= stagecmd_wait;
						end
					end

					{stage_eeprom_rw,stagecmd_wait} : begin
						if (data_seq_out == 0) begin
							if (little_counter == 4) begin
								stage <= stage_weightcache_rw;
								stagecmd <= stagecmd_read;
							end
							else begin
								stagecmd <= stagecmd_storage;
							end
						end
					end

					{stage_eeprom_rw,stagecmd_wait2} : begin
						rw_contiune <= 0;
						datain_counter <= 0;
						if(work_ready) begin
							stage <= stage_weightcache_rw;
							stagecmd <= stagecmd_read;
						end
					end

					{stage_weightcache_rw,stagecmd_endcmd} : begin
						current_state <= idle_state;
						stage <= stage_state_update;
						stagecmd <= stagecmd_initial;
					end


				endcase

			end // end of bp_eeprom_dataout

			error : begin
				error_flag <= 1;
				rw_contiune <= 0;
				working <= 0;
			end
			
		endcase // end current_case
	end
end // end Main function

endmodule