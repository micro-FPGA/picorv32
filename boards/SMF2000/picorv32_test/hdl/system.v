`timescale 1 ns / 1 ps


module system (
	input            clk,       // 27MHz, PIN = B3
	input            resetn,	// SW6  net SPARE_C0 has external pullup, active LOW, PIN = C3
	output           DEBUG_LED,	// LED2 GREEN net SPARE_C2, Active LOW, PIN = F2
    output           LED 	    // LED1 RED net SPARE_C1, Active LOW, PIN = B1
//	output           trap,
//	output reg [7:0] out_byte,
//	output reg       out_byte_en
);
	// set this to 0 for better timing but less performance/MHz
	parameter FAST_MEMORY = 1;

	// 4096 32bit words = 16kB memory
	parameter MEM_SIZE = 1024;
	parameter BOOTROM_WORDS = 512;

	wire trap;

	wire mem_valid;
	wire mem_instr;
	reg mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
//	reg [31:0] mem_rdata;
	wire [31:0] mem_rdata;

	wire mem_la_read;
	wire mem_la_write;
	wire [31:0] mem_la_addr;
	wire [31:0] mem_la_wdata;
	wire [3:0] mem_la_wstrb;

	reg rstn_sync;

	picorv32 picorv32_core (
		.clk         (clk         ),
		.resetn      (rstn_sync   ),
		.trap        (trap        ),
		.mem_valid   (mem_valid   ),
		.mem_instr   (mem_instr   ),
		.mem_ready   (mem_ready   ),
		.mem_addr    (mem_addr    ),
		.mem_wdata   (mem_wdata   ),
		.mem_wstrb   (mem_wstrb   ),
		.mem_rdata   (mem_rdata   ),
		.mem_la_read (mem_la_read ),
		.mem_la_write(mem_la_write),
		.mem_la_addr (mem_la_addr ),
		.mem_la_wdata(mem_la_wdata),
		.mem_la_wstrb(mem_la_wstrb)
	);

//
// Generated boot ROM
//
    wire [31:0] bootrom_data;
    reg [31:0] addr_latched;

bootrom my_bootstrap (
    .A(addr_latched[9:0]),
    .D(bootrom_data )
);


	reg [31:0] memory [0:MEM_SIZE-1]  /* synthesis syn_ramstyle="ebr" */ ;
	
	
	initial $readmemh("firmware.mem", memory);
	//initial $readmemh("fail.mem", memory);
	//initial $readmemh("pass.hex", memory);
	
	
	// Pass Fail LED connected to PC
	//assign LED       = mem_addr[4] ^ rstn_sync; // RED on if Error
	assign LED       = trap        ^ rstn_sync; // RED on if Error
	assign DEBUG_LED = mem_addr[2] ^ rstn_sync;


    always @(posedge clk) begin
		rstn_sync <= resetn;	
	end

    wire [31:0] ram_data;
	assign ram_data = memory[addr_latched];
	

	assign mem_rdata = (addr_latched < BOOTROM_WORDS) ? bootrom_data : ram_data;
//    assign mem_rdata = ram_data;

//	assign mem_rdata = 32'h 0000_0013; // NOP

    always @(posedge clk) begin
		addr_latched <= mem_la_addr >> 2;
		if (mem_la_write) begin
				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
//				memory[mem_la_addr >> 2] <= mem_la_wdata;
		end	
	end

//	reg [31:0] m_read_data;
	reg m_read_en;
	generate if (FAST_MEMORY) begin
		always @(posedge clk) begin
			mem_ready <= 1;
//			out_byte_en <= 0;
//			mem_rdata <= memory[mem_la_addr >> 2];
//			if (mem_la_write && (mem_la_addr >> 2) < MEM_SIZE) begin
//				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
//				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
//				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
//				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
//				memory[mem_la_addr >> 2][31:0] <= mem_la_wdata[31:0];
//			end
//			else
			// reduced Address decoder, less LUT
//			if (mem_la_write && mem_la_addr[15:12] == 32'hC) begin
//				out_byte_en <= 1;
//				out_byte <= mem_la_wdata;
//			end
		end
	end else begin
		always @(posedge clk) begin
		end
	end endgenerate
endmodule
