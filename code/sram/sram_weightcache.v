module sram_weightcache #(parameter  ADDR_WIDTH  =  8,
                         DATA_WIDTH  =  32,
                         MEM_DEPTH   =  256)
             (
                data_out,
                data_in,
                addr,
                cs_n,
                wr_n,
                clk,
			          rst_n
             );
 
output [DATA_WIDTH-1:0] data_out;                //output data
input  [DATA_WIDTH-1:0] data_in;                 //input data
input  [ADDR_WIDTH-1:0] addr;                //write data address signal
input                   wr_n;                //write data contral signal
input                   clk;                 //write data clock
input					          rst_n;
input                   cs_n;
 
reg    [DATA_WIDTH-1:0] rddata;
reg    [DATA_WIDTH-1:0]mem[MEM_DEPTH-1:0];             //register array
reg [ADDR_WIDTH-1:0] inter_addr;

wire [DATA_WIDTH-1:0]allzero = {DATA_WIDTH{1'b0}};
 
integer i;

always @(posedge clk) begin
  inter_addr <= addr;
end

always@(posedge clk or negedge rst_n)
    begin
		  if(rst_n == 1'b0) begin
		    for(i =0 ; i<MEM_DEPTH ; i=i+1) begin
				  mem[i] <= allzero;
			  end
		  end
      else if(!cs_n && !wr_n) begin
        mem[addr] <= data_in;
      end
      else begin
        mem[addr] <= mem[addr];
      end
    end

assign data_out = mem[inter_addr];
endmodule
