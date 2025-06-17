// DATA_WIDTH = x16 CAPACITY = 1Gb
`timescale 1ns/1ps
module ddr_controller(
	// CPU side Signals
	input logic clk,			// System clock.
	input logic rst_n,		// System reset.
	input logic [2:0] icmd,		// Command for controller.
	input logic [31:0] data_in,	// Data input 32 bit
	input logic [31:0] iaddr,	// 0-27 Usable. Address size of memory, which is derived by the following formula: ASIZE=RANK_SIZE+RSIZE+BSIZE+CSIZE. A=0+14+2+12=28
	input logic [3:0] dmsel,	// Data Mask select.
	input logic clk2x,			// This is the doubled clock signal coming from the on-chip PLL.
	output logic busy,			// Busy signal indicates the controller will not accept any more commands.
	output logic [31:0] dataout,// Data out. 
	output logic dataout_valid,	// During a read, this signal indicates when the dataout bus from the controller contains valid data.
	input logic datain_valid,	// This signal indicates when the user can start sending in data through datain bus during a write.
	// DDR SDRAM side Signals	
	output logic clkout,
	output logic clkoutn,
    output logic cke,
    output logic cs_n,			// [RANK_SIZE-1:0] This controller supports 1 RANK DIMMs
    output logic ras_n,
    output logic cas_n,
    output logic we_n,
	output logic [1:0] ba,
    output logic [13:0] addrout,
	output logic dm,			// Upper DM for DATA_WIDTH = x16, DM for others
	output logic ldm,			// Lower DM for DATA_WIDTH = x16, NC for others
    inout logic [15:0] dq,
    inout logic dqs
);
assign clkout = clk;
assign clkoutn = 1-clk;

// Timing Parameters
parameter tRCD = 2;			// Row to Column Delay
parameter tRP = 2;			// Row Precharge Time
parameter tRAS = 5;			// Row Active Time
//parameter tRC = 7;			// Row Cycle Time tRAS + tRP
parameter tRFC = 8;			// Refresh Cycle Time
parameter tCCD = 2;			// Column to Column Delay
parameter tRTP = 2;			// Read to Precharge Delay
parameter tRTW = 3;			// Read to Write Delay
parameter tWR = 2;			// Write Recovery Time
//parameter tDAL = 4;			// Data-in to Active Latency tWR + tRP
parameter tWTR = 1;			// Write to Read Delay
parameter tRRD = 2;			// Row to Row Delay
parameter tMRD = 2;			// Mode Register Set Delay
parameter tREFI = 780;		// Refresh Interval (Average)
parameter tCKE = 1;			// Minimum holding time of CKE low/high for POWER_DOWN/SELF_REFRESH entry/exit
parameter tXSNR = 13;		// Self-Refresh Exit to Non-Read Command
parameter tXSRD = 200;		// Self-Refresh Exit to Read Command
parameter tXARD = 1;		// Exit Active Power-Down Delay
parameter tDSGN = 32'h41594148;
parameter INIT_WAIT_CYCLES = 20000; // For 200µs at 100MHz

localparam ROW_WIDTH = 14;
localparam COL_WIDTH = 10;
localparam DW =16;		// DATA_WIDTH = x16
logic init_done;
logic last_data_in, last_data_out;	// Last data word has arrived in a burst or otherwise.// Last data word has arrived (read from ram) in a burst or otherwise.
//Timings signals
logic [7:0] tBURST;	// Burst Time
logic [7:0] tRCD_counter [4];  
logic [7:0] tRP_counter [4];       
logic [7:0] tRAS_counter [4];
logic [7:0] tCCD_counter [4];
logic [7:0] tRTP_counter [4];
logic [7:0] tRTW_counter [4];
logic [7:0] tWR_counter [4];   
logic [7:0] tWTR_counter [4];  
logic [7:0] tRRD_counter;
logic [7:0] tMRD_counter;
logic [7:0] tRFC_counter;
logic [9:0] tREFI_counter;
logic [7:0] tCKE_counter;
logic [7:0] tXSNR_counter;
logic [7:0] tXSRD_counter;
logic [7:0] tXARD_counter;
int rw_wait_counter = 0;
logic [15:0] init_counter, init_precharge_done, init_mrs_done, init_refresh_count; // Initialization signals
logic [13:0] row_addr [4] = '{default:'bx};		// Address Signals
logic row_active [4] = '{default:'b0};			// Default no active rows
logic [1:0] p_ba;						// Previous selected Bank
logic [1:0] c_ba;						// Current selected Bank
int burst_len, burst_len_dram;
logic [2:0] bl;
logic burst_type;			  
logic [2:0] cas_latency;      
logic [6:0]operating_mode;	  
logic ap;					  
logic [1:0][3:0][DW-1:0] data_in_x16;		// Input data buffer
logic [1:0][3:0][DW-1:0] data_out_x16;		// Output data buffer
logic dqs_en;                 
logic dq_en;                  
logic cke_ps, cke_cs;         // cke previous/current state
logic [1:0] d_ba;             // Decoded bank address
logic [ROW_WIDTH-1:0] d_row_add;       
logic [COL_WIDTH:0] d_col_add; // ap flag encoded       

typedef enum logic [3:0] {
    IDLE					= 4'b0000,
    MODE_REGISTER_SET		= 4'b0001,
    ACTIVE					= 4'b0010,
    READ					= 4'b0011,
    WRITE					= 4'b0100,
    PRECHARGE				= 4'b0101,
    AUTO_REFRESH			= 4'b0110,
    SELF_REFRESH			= 4'b0111,
    POWER_DOWN				= 4'b1000,
    READ_WITH_AUTOPRECHARGE	= 4'b1001,
    WRITE_WITH_AUTOPRECHARGE= 4'b1010,
    BURST_STOP				= 4'b1011,
	NOP						= 4'b1100,
	DESELECT				= 4'b1101 
} state_t;

state_t current_state, next_state, previous_state;

typedef enum logic [2:0] {
    C_READ			= 3'b000,
    C_WRITE			= 3'b001,
    C_SELF_REFRESH	= 3'b010,
    C_CFG 			= 3'b011,
    C_NOP			= 3'b100
} d_cmd_t;

d_cmd_t d_cmd;

typedef enum logic [3:0] {
    BANK_IDLE		= 4'b0000,
    BANK_ACTIVE		= 4'b0001,
    BANK_READ		= 4'b0010,
	BANK_READ_AP	= 4'b0011,
	BANK_WRITE_AP	= 4'b0100,
    BANK_WRITE		= 4'b0101,
    BANK_PRECHARGE	= 4'b0110
} bank_state_t;

bank_state_t bank_states [0:3], bank_next_state; // 4 banks
logic [3:0] bank_active;        // Tracks which banks have active rows
logic [2:0] cmd;
logic [31:0] addr;

// FIFO Configuration
parameter FIFO_DEPTH = 8;
typedef struct {
    logic [2:0] cmd;         // Command type
    logic [31:0] addr;       // Full address
    logic [31:0] data_in [8];    // Input data
    logic [1:0]  bank;       // Extracted bank (addr[13:12])
    logic [ROW_WIDTH-1:0] row; // Extracted row (addr[ROW_HIGH:ROW_LOW])
	logic [3:0] dm [8]; // dm for every data word
} fifo_t;

fifo_t cmd_fifo [FIFO_DEPTH-1:0];
logic [2:0] wr_ptr, rd_ptr;
logic full, empty, cmd_done;
logic [2:0] count;
int wi, ri1, ri2;
logic store_dout;
logic [15:0] dq_reg;
logic [3:0][31:0] data_out_reg;
logic [3:0][31:0] data_in_reg;
logic [3:0][3:0] dm_in_reg;
logic [7:0] transmitted_data ;
localparam ROW_HIGH = 14 + ROW_WIDTH - 1;		// Extract row address bounds (adjust based on your ROW_WIDTH)
localparam ROW_LOW  = 14;
logic new_cmd;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr <= 0;
        rd_ptr <= 0;
        count <= 0;
		full <= 'b0;
		empty <= 'b1;
		cmd_done <= 'b0;
		//cmd_fifo <= '{default:'b0};
		wi <= 0;
		ri1 <= 0;
		ri2 <= 0;
    end else if (init_done) begin
        if (count == FIFO_DEPTH-1) begin
			full = 1;
		end else begin
			full = 0;
		end
		if (count == 0) begin
			empty = 1;
		end else begin
			empty = 0;
		end
		if (full) begin
			busy = 1;
		end else begin
			busy = 0;
		end   
		// Enqueue new command
        if (!full && !busy && icmd < 4) begin 
            cmd_fifo[wr_ptr].cmd     <= icmd;
            cmd_fifo[wr_ptr].addr    <= iaddr;
            cmd_fifo[wr_ptr].bank    <= iaddr[13:12]; // Extract bank
            cmd_fifo[wr_ptr].row     <= iaddr[ROW_HIGH:ROW_LOW]; // Extract row
            wr_ptr <= wr_ptr + 1;
			count <= count + 1;
			/*if (icmd == 1 || icmd == 3) begin
				datain_valid <= 'b1;
			end*/
        end else if (!full && !busy && icmd == 4) begin
			if (d_cmd == C_SELF_REFRESH && current_state == SELF_REFRESH) begin
				cmd = 3'b100;
			end
		end
		if (ri1 < burst_len && datain_valid/* && data_in != 0*/) begin
			cmd_fifo[wr_ptr-1].data_in[ri1] <= data_in;
			cmd_fifo[wr_ptr-1].dm[ri1] <= dmsel;
			ri1<=ri1+1;
		end else if (ri1 >= burst_len/* && datain_valid*/) begin
			ri1<=0;
			//datain_valid <= 'b0;
		end
			
        // Dequeue processed command
        if (!empty && cmd_done) begin // Signal when command is executed
			cmd <= cmd_fifo[rd_ptr].cmd;
			addr <= cmd_fifo[rd_ptr].addr;
			rd_ptr <= rd_ptr + 1;
			count <= count-1;
			cmd_fifo[rd_ptr].cmd <= 'bx;
			//p_ba <= c_ba;		//removing bank paralellism for now
			cmd_done <= 'b0;
			new_cmd <= 1;
		end
		if (ri2 < burst_len && rd_ptr!=0) begin
			if (cmd_fifo[rd_ptr-1].data_in [ri2] !==32'bx || cmd_fifo[rd_ptr-1].data_in [ri2] !=0) data_in_reg[ri2] <= cmd_fifo[rd_ptr-1].data_in [ri2];
			if (cmd_fifo[rd_ptr-1].dm [ri2] !==4'bx) dm_in_reg[ri2] <= cmd_fifo[rd_ptr-1].dm [ri2];
			ri2<=ri2+1;
		end else if (ri2 >= burst_len) begin
			ri2<=0;
		end	if (empty && cmd_done) begin
			if (last_data_in || last_data_out) begin
				cmd <= 'b100;
			end
		end
		if (current_state == READ || current_state == READ_WITH_AUTOPRECHARGE) begin
			if (last_data_out && wi < burst_len) begin
				dataout <= data_out_reg [wi];
				dataout_valid <= 1;
				wi <= wi+1;
			end else begin
				wi<=0;
				dataout_valid <= 0;
			end
		end
	end
end

logic next_match_exists;

always_comb begin
	if (init_done) begin
		if (tREFI_counter >= tREFI-tRFC-tRP-tWR || tREFI_counter >= tREFI-tRFC-tRP-tRTP || tREFI_counter >= tREFI-tRFC-tRAS-tRP) begin		// Default to refresh logic
			ap = 'b1;
		end else if (!empty) begin
			next_match_exists = 1'b0;
			for (int i = 0; i < FIFO_DEPTH; i++) begin		// scan all FIFO slots
				if (cmd_fifo[i].cmd !== 'bx && cmd_fifo[i].bank == d_ba && cmd_fifo[i].row  != d_row_add) begin		// skip empty/unwritten entries: assume cmd=='bx means empty
					next_match_exists = 1'b1;
				end else next_match_exists = 1'b0;
			end	
		end
	end
	//data_out_reg logic
	for (int i=0;i<4;i++) begin
		if (i < burst_len) begin
			if (data_out_x16[0][i] !== 16'bx && data_out_x16[0][i] !== 16'bz) begin
				data_out_reg[i][15:0] = data_out_x16[0][i];
			end
			if (data_out_x16[1][i] !== 16'bx && data_out_x16[1][i] !== 16'bz) begin
				data_out_reg[i][31:16] = data_out_x16[1][i];
			end
		end
	end
end

always_ff @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		ap <= 1'b1;
	end else if (init_done && new_cmd && (cmd == 3'b000 || cmd == 3'b001)) begin  // on every fetch-from-FIFO event
		ap <= ~next_match_exists;
		new_cmd <= 0;
	end
end

//FSM
always_comb begin
	// Burst length decode
	case (burst_len_dram)
	2: bl <= 3'b001;
	4: bl <= 3'b010;
	8: bl <= 3'b011;
	endcase
	burst_len_dram = 2*burst_len;
// Map commands to state_t
if (init_done) begin
    case (cmd)
        3'b000:	d_cmd = C_READ;
		3'b001:	d_cmd = C_WRITE;
		3'b010:	d_cmd = C_SELF_REFRESH;
		3'b011:	d_cmd = C_CFG;
		default: d_cmd = C_NOP;
	endcase
// Row Column address decode
	if (current_state != READ && current_state != READ_WITH_AUTOPRECHARGE && current_state != WRITE && current_state != WRITE_WITH_AUTOPRECHARGE) begin
		d_row_add = addr [ROW_WIDTH-1+14:14];
		d_ba = addr [13:12];
		c_ba = d_ba;
		p_ba = d_ba;
		d_col_add [10] = ap;
		d_col_add [9:0] = addr [COL_WIDTH-1:0];
	end
	for (int i = 0; i < 4; i++) begin
		if (i < burst_len) begin
			if (data_in_reg[i] !== 32'bx) begin
				data_in_x16[0][i] = data_in_reg[i][15:0];
				data_in_x16[1][i] = data_in_reg[i][31:16];
			end
		end
	end
	
	if (current_state == MODE_REGISTER_SET) begin
		operating_mode = addr[13:7];
		cas_latency = addr[6:4];
		burst_type = addr[3];
		bl = addr[2:0];
	end
end
if (init_done) begin		//Initialization check
	if (d_cmd == C_SELF_REFRESH) begin
		for (int i = 0; i < 4; i++) begin
			if (bank_states[i] != BANK_IDLE) begin
				bank_next_state = BANK_PRECHARGE;
				ap = 'b1;
				next_state = PRECHARGE;
				break;
			end
		end
	end
    case (current_state)	//FSM start
        IDLE: begin
			if (tREFI_counter >= tREFI-tRFC) begin
				if (bank_states[0] == BANK_IDLE && bank_states[1] == BANK_IDLE && bank_states[2] == BANK_IDLE && bank_states[3] == BANK_IDLE) begin
					next_state = AUTO_REFRESH;
				end
			end else if (tREFI_counter >= tREFI-tRFC-tRAS-tRP-tRTP-tWR-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
            end else if (d_cmd == C_CFG) begin
				if (bank_states[0] == BANK_IDLE && bank_states[1] == BANK_IDLE && bank_states[2] == BANK_IDLE && bank_states[3] == BANK_IDLE) begin
					next_state = MODE_REGISTER_SET;
				end
            end else if (d_cmd == C_READ || d_cmd == C_WRITE) begin
				if (bank_states[c_ba] == BANK_IDLE) begin
					next_state = ACTIVE;
					bank_next_state = BANK_ACTIVE;
				end
            end else if (d_cmd == C_SELF_REFRESH) begin
				if (bank_states[0] == BANK_IDLE && bank_states[1] == BANK_IDLE && bank_states[2] == BANK_IDLE && bank_states[3] == BANK_IDLE) begin
					next_state = SELF_REFRESH;
				end else begin
					for (int i = 0; i < 4; i++) begin
						if (bank_states[i] != BANK_IDLE) begin
							bank_next_state = BANK_PRECHARGE;
							ap = 'b1;
							break;
						end
					end
					next_state = PRECHARGE;
				end
            end else if (d_cmd == C_NOP && tREFI_counter <= tREFI-tRFC-tXARD-tCKE-tCKE) begin
                next_state = POWER_DOWN;	// IDLE to PRECHARGE POWER_DOWN
            end
        end

        MODE_REGISTER_SET: begin
            next_state = IDLE;
        end

        ACTIVE: begin
			if (tREFI_counter >= tREFI-tRFC-tRP-tWR-tRAS-tBURST || tREFI_counter >= tREFI-tRFC-tRP-tRTP-tRAS-tBURST || tREFI_counter >= tREFI-tRFC-tRAS-tRP-tRAS-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
			end else if (d_cmd == C_SELF_REFRESH) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_READ && bank_states[i] != BANK_READ_AP && bank_states[i] != BANK_WRITE && bank_states[i] != BANK_WRITE_AP) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
            end else if (d_cmd == C_READ) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (c_ba == p_ba && bank_states[c_ba] == BANK_ACTIVE && cmd_fifo[rd_ptr].addr[ROW_WIDTH-1+14:14]!=d_row_add) begin
					next_state = ACTIVE;		//same bank different row activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE;	//other Bank READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE; // ACTIVE to READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba) begin
					next_state = READ;			//other Bank READ
					bank_next_state = BANK_READ;
				end else if (bank_states[c_ba] == BANK_ACTIVE) begin
					next_state = READ;			//same Bank READ
					bank_next_state = BANK_READ;
				end
            end else if (d_cmd == C_WRITE) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (c_ba == p_ba && bank_states[c_ba] == BANK_ACTIVE && cmd_fifo[rd_ptr].addr[ROW_WIDTH-1+14:14]!=d_row_add) begin
					next_state = ACTIVE;		//same bank different row activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE;	//other Bank WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE; // ACTIVE to WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_ACTIVE && c_ba != p_ba) begin
					next_state = WRITE;			//other Bank WRITE
					bank_next_state = BANK_WRITE;
				end else if (bank_states[c_ba] == BANK_ACTIVE) begin
					next_state = WRITE;			//same Bank WRITE
					bank_next_state = BANK_WRITE;
				end
            end else if (d_cmd == C_NOP && tREFI_counter <= tREFI-tRFC-tXARD-tCKE-tCKE-tRAS-tRP) begin
					next_state = POWER_DOWN; // ACTIVE to POWER_DOWN
            end
        end

        READ: begin
			if (tREFI_counter >= tREFI-tRFC-tRP-tWR-tRTP-tBURST || tREFI_counter >= tREFI-tRFC-tRP-tRTP-tRTP-tBURST || tREFI_counter >= tREFI-tRFC-tRAS-tRP-tRTP-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
            end else if (d_cmd == C_READ) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ && c_ba != p_ba && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE;	//other Bank READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_READ && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE; // READ to READ with AUTOPRECHARGE same Bank
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ && c_ba != p_ba) begin
					next_state = READ;		//other bank READ
					bank_next_state = BANK_READ;
				end else if (bank_states[c_ba] == BANK_READ && last_data_out && d_col_add != cmd_fifo[rd_ptr].addr[COL_WIDTH-1:0]) begin
					next_state = READ;	// Consecutive READ same Bank
					bank_next_state = BANK_READ;
				end else if (last_data_out) begin
					next_state = ACTIVE; // end of read
					bank_next_state = BANK_ACTIVE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd == C_WRITE) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_READ && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ && c_ba != p_ba && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE;	//other Bank WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_READ && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE; // READ to WRITE with AUTOPRECHARGE same Bank
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ && c_ba != p_ba) begin
					next_state = WRITE;			//other Bank WRITE
					bank_next_state = BANK_WRITE;
                end else if (bank_states[c_ba] == BANK_READ && last_data_out && d_col_add != cmd_fifo[rd_ptr].addr[COL_WIDTH-1:0]) begin
					next_state = WRITE; // READ to WRITE same Bank
					bank_next_state = BANK_WRITE;
				end else if (last_data_out) begin
					next_state = ACTIVE; // end of read
					bank_next_state = BANK_ACTIVE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd != C_READ && !last_data_out && c_ba == p_ba) begin
                next_state = BURST_STOP; // READ to BURST_STOP
            end
        end

        READ_WITH_AUTOPRECHARGE: begin
			if (tREFI_counter >= tREFI-tRFC-tRP-tWR-tRTP-tBURST || tREFI_counter >= tREFI-tRFC-tRP-tRTP-tRTP-tBURST || tREFI_counter >= tREFI-tRFC-tRAS-tRP-tRTP-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
			end else if (d_cmd == C_READ) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE;	//other Bank READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba) begin
					next_state = READ;		//other bank READ
					bank_next_state = BANK_READ;
				end else if (last_data_out) begin
					next_state = IDLE; // end of write
					bank_next_state = BANK_IDLE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd == C_WRITE) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE;	//other Bank WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_READ_AP && c_ba != p_ba) begin
					next_state = WRITE;			//other Bank WRITE
					bank_next_state = BANK_WRITE;
				end else if (last_data_out) begin
					next_state = IDLE; // end of write
					bank_next_state = BANK_IDLE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd != C_READ && !last_data_out && c_ba == p_ba) begin
                next_state = BURST_STOP; // READ to BURST_STOP
			end
        end

        WRITE: begin
			if (tREFI_counter >= tREFI-tRFC-tRP-tWR-tWR-tBURST || tREFI_counter >= tREFI-tRFC-tRP-tRTP-tWR-tBURST || tREFI_counter >= tREFI-tRFC-tRAS-tRP-tWR-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
            end else if (d_cmd == C_WRITE) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE;	//other Bank WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_WRITE && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE; // WRITE to WRITE with AUTOPRECHARGE same Bank
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba) begin
					next_state = WRITE;			//other Bank WRITE
					bank_next_state = BANK_WRITE;
				end else if (bank_states[c_ba] == BANK_WRITE && last_data_in && d_col_add != cmd_fifo[rd_ptr].addr[COL_WIDTH-1:0]) begin
					next_state = WRITE; // Consecutive WRITE same Bank
					bank_next_state = BANK_WRITE;
				end else if (last_data_in) begin
					next_state = ACTIVE; // end of write
					bank_next_state = BANK_ACTIVE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd == C_READ) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE;	//other Bank READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
                end else if (bank_states[c_ba] == BANK_WRITE && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE; // WRITE to READ with AUTOPRECHARGE same Bank
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE && c_ba != p_ba) begin
					next_state = READ;		//other bank READ
					bank_next_state = BANK_READ;
				end else if (bank_states[c_ba] == BANK_WRITE && last_data_in && d_col_add != cmd_fifo[rd_ptr].addr[COL_WIDTH-1:0]) begin
					next_state = READ;	// WRITE to READ same Bank
					bank_next_state = BANK_READ;
				end else if (last_data_in) begin
					next_state = ACTIVE; // end of write
					bank_next_state = BANK_ACTIVE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end
		end

        WRITE_WITH_AUTOPRECHARGE: begin
			if (tREFI_counter >= tREFI-tRFC-tRP-tWR-tWR-tBURST || tREFI_counter >= tREFI-tRFC-tRP-tRTP-tWR-tBURST || tREFI_counter >= tREFI-tRFC-tRAS-tRP-tWR-tBURST) begin
				for (int i = 0; i < 4; i++) begin
					if (bank_states[i] != BANK_IDLE) begin
						bank_next_state = BANK_PRECHARGE;
						ap = 'b1;
						break;
					end
				end
				next_state = PRECHARGE;
			end else if (d_cmd == C_WRITE) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba && ap) begin
					next_state = WRITE_WITH_AUTOPRECHARGE;	//other Bank WRITE with AUTOPRECHARGE
					bank_next_state = BANK_WRITE_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba) begin
					next_state = WRITE;			//other Bank WRITE
					bank_next_state = BANK_WRITE;
				end else if (last_data_in) begin
					next_state = IDLE; // end of write
					bank_next_state = BANK_IDLE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end else if (d_cmd == C_READ) begin
				if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //other Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] != BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba == p_ba && row_addr [c_ba] != d_row_add) begin
					next_state = PRECHARGE; //same Bank Precharge
					bank_next_state = BANK_PRECHARGE;
				end else if (bank_states[c_ba] == BANK_IDLE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba) begin
					next_state = ACTIVE;		//other bank activation
					bank_next_state = BANK_ACTIVE;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba && ap) begin
					next_state = READ_WITH_AUTOPRECHARGE;	//other Bank READ with AUTOPRECHARGE
					bank_next_state = BANK_READ_AP;
				end else if (bank_states[c_ba] == BANK_ACTIVE && bank_states[p_ba] == BANK_WRITE_AP && c_ba != p_ba) begin
					next_state = READ;		//other bank READ
					bank_next_state = BANK_READ;
				end else if (last_data_in) begin
					next_state = IDLE; // end of write
					bank_next_state = BANK_IDLE;
				end else begin
					transmitted_data = transmitted_data + 1;
				end
			end
		end

        PRECHARGE: begin
			next_state = IDLE;
			bank_next_state = BANK_IDLE;
		end

        AUTO_REFRESH: begin
            next_state = IDLE; // AUTO_REFRESH to IDLE
			bank_next_state = BANK_IDLE;
        end

        SELF_REFRESH: begin
            if (d_cmd != C_SELF_REFRESH) begin
				next_state = NOP; // Exit SELF_REFRESH
			end else if (d_cmd == C_SELF_REFRESH) begin
				next_state = SELF_REFRESH; // Maintain SELF_REFRESH
			end
        end

        POWER_DOWN: begin
            if (d_cmd == C_NOP && tREFI_counter <= tREFI-tRFC-tRP-tCKE) begin
                next_state = POWER_DOWN; // Maintain POWER_DOWN
            end else begin
				if (bank_states[0] == BANK_IDLE && bank_states[1] == BANK_IDLE && bank_states[2] == BANK_IDLE && bank_states[3] == BANK_IDLE) begin
					next_state = IDLE; // POWER_DOWN to IDLE (if not from ACTIVE)
				end else begin
					next_state = ACTIVE; // POWER_DOWN to ACTIVE
				end
			end
        end

        BURST_STOP: begin
			if (tREFI_counter >= tREFI-tRFC-tRAS-tRP) begin
				bank_next_state = BANK_PRECHARGE;
				next_state = PRECHARGE;
				ap = 'b1;
			end else begin
				next_state = ACTIVE; // BURST_STOP to ACTIVE
			end
		end

        default: begin
            next_state = IDLE; // Fallback to IDLE
			bank_next_state = BANK_IDLE;
        end
    endcase
end
else if (!init_done) begin		//Initialization start
	if (init_counter < INIT_WAIT_CYCLES) begin
		next_state = IDLE; // Wait period
	end
	else if (!init_precharge_done) begin
		next_state = PRECHARGE;
	end
	else if (!init_mrs_done) begin
		next_state = MODE_REGISTER_SET;
	end
	else if (init_refresh_count < 1) begin
		next_state = AUTO_REFRESH;
	end
	else begin
		next_state = AUTO_REFRESH;	// Initialization complete
	end
end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
		// Reset Timings signals
		tRCD_counter <= '{default:'b0};		// Row to Column Delay
        tRP_counter <= '{default:'b0};		// Row Precharge
        tRAS_counter <= '{default:'b0};		// Row Active Time (min)
		tCCD_counter <= '{default:tCCD};	// Column to Column Delay
        tRTP_counter <= '{default:'b0};		// Read to Precharge Delay
        tRTW_counter <= '{default:'b0};		// Read to Write Delay
		tWR_counter <= '{default:'b0};		// Write Recovery Time
		tWTR_counter <= '{default:'b0};		// Write to Read Delay
		tRRD_counter <= '{default:tRRD}; 	// Row to Row Delay (different banks)
        tMRD_counter <= '0;					// Mode Register Set Delay
		tRFC_counter <= '0;					// Refresh Cycle Time
		tREFI_counter <= '0;				// Refresh Interval (Average)
		tCKE_counter <= '0;					// Minimum holding time of CKE low/high for POWER_DOWN/SELF_REFRESH entry/exit
		tXSNR_counter <= '0;				// Self-Refresh Exit to Non-Refresh Command
		tXSRD_counter <= '0;				// Self-Refresh Exit Time
		tXARD_counter <= '0;				// Exit Power Down to valid Command Delay
        transmitted_data <='0;
		// Reset Initialization signals
		init_counter <= 0;
        init_precharge_done <= 0;
        init_mrs_done <= 0;
        init_refresh_count <= 0;
        init_done <= 0;
        busy <= 1;
		// Reset all Command registers and signals
		current_state <= IDLE;
        previous_state <= IDLE;		// Initialize previous_state
		cke_cs <= 1;
		cke_ps <= 1;		
        row_addr <= '{default:'bx};
		p_ba <= 'b0; // Previous selected Bank
        c_ba <= 'b0; // Current selected Bank
        d_ba <= 'b0;
		addr <= 'b0;
        cas_latency <= '0;
        ap <= 'b1;
        dqs_en <= '0;
        dq_en <= '0;
		last_data_in <= 0;
		last_data_out <= 0;
    end
	else begin
		cke_ps <= cke_cs;
		cke_cs <= cke;
		//c_ba <= d_ba;
		//p_ba <= d_ba;	//removing bank paralellism for now
		// Initialization tracking
        if (!init_done) begin
            init_counter <= init_counter + 1;
            if (current_state == PRECHARGE) begin
				init_precharge_done <= 1;
            end
            if (current_state == MODE_REGISTER_SET) begin
                burst_len <= 4; // Default lenght 4 for 32bit word meaning 8 for 16bit word
				burst_type <= 1'b0;		// Default type sequential
                cas_latency <= 3'b010;	// Default CAS Latency 2
				operating_mode <= 'b0;	// Default Normal Operation
				if (tMRD_counter >= tMRD) begin
					init_mrs_done <= 1;
				end
			end
            if (current_state == AUTO_REFRESH) begin
                if (tRFC_counter >= tRFC) begin
					init_refresh_count <= init_refresh_count + 1;
				end
            end
            if (init_refresh_count >= 1) begin
                init_done <= 1;
				bank_states <= '{default:BANK_IDLE};
				busy <= 'b0;
				cmd_done <= 'b1;
            end
        end
		// Timing tracking
		tREFI_counter <= tREFI_counter + 1;
		if (current_state == ACTIVE) begin
			if (tRRD_counter < tRRD) begin
				tRRD_counter <= tRRD_counter + 1;
			end
		end
		// every cycle, for every bank:
		for (int i = 0; i < 4; i++) begin
		  // only count RAS when that bank’s row is actually open:
			if (bank_states[i] == BANK_ACTIVE) begin
				if (tRAS_counter[i] < tRAS) begin
					tRAS_counter[i] <= tRAS_counter[i] + 1;
				end if (tRCD_counter[i] < tRCD) begin
					tRCD_counter[i] <= tRCD_counter[i] + 1;
				end
			end
		  // tCCD, tRTP, tRTW, tWTR, tWR
			if (bank_states[i] == BANK_READ || bank_states[i] == BANK_READ_AP || bank_states[i] == BANK_WRITE || bank_states[i] == BANK_WRITE_AP) begin
				if (tCCD_counter[i] < tCCD) begin
					tCCD_counter[i] <= tCCD_counter[i] + 1;
				end else if (last_data_out) begin
					if (bank_states[i] == BANK_READ && bank_next_state == BANK_PRECHARGE) begin
						if (tRTP_counter[i] < tRTP) begin
							tRTP_counter[i] <= tRTP_counter[i] + 1;
						end
					end else if (bank_states[i] == BANK_READ_AP) begin
						if (tRTP_counter[i] < tRTP) begin
							tRTP_counter[i] <= tRTP_counter[i] + 1;
						end else if (tRP_counter[i] < tRP) begin
							tRP_counter[i] <= tRP_counter[i] + 1;
						end
					end else if (bank_states [i] == BANK_READ && (bank_next_state == BANK_WRITE || bank_next_state == BANK_WRITE_AP)) begin
						if (tRTW_counter[i] < tRTW) begin
							tRTW_counter[i] <= tRTW_counter[i] + 1;
						end
					end
				end else if (last_data_in) begin
					if (bank_states[i] == BANK_WRITE && bank_next_state == BANK_PRECHARGE) begin
						if (tWR_counter[i] < tWR) begin
							tWR_counter[i] <= tWR_counter[i] + 1;
						end
					end else if (bank_states[i] == BANK_WRITE_AP) begin
						if (tWR_counter[i] < tWR) begin
							tWR_counter[i] <= tWR_counter[i] + 1;
						end else if (tRP_counter[i] < tRP) begin
							tRP_counter[i] <= tRP_counter[i] + 1;
						end
					end else if (bank_states [i] == BANK_WRITE && (bank_next_state == BANK_READ || bank_next_state == BANK_READ_AP)) begin
						if (tWTR_counter[i] < tWTR) begin
							tWTR_counter[i] <= tWTR_counter[i] + 1;
						end
					end
				end
			end
		end

        if (next_state != current_state || !init_done) begin
		case (current_state)
            ACTIVE: begin
				if (next_state == ACTIVE) begin
					if (tRRD_counter >= tRRD) begin
						previous_state <= current_state;
						current_state <= next_state;
						
						bank_states[c_ba] <= bank_next_state;
						bank_active[c_ba] <= 1'b1;
						tRRD_counter <= '0;
					end else begin
						current_state <= current_state;
					end
				end else if (next_state == READ || next_state == READ_WITH_AUTOPRECHARGE || next_state == WRITE || next_state == WRITE_WITH_AUTOPRECHARGE) begin
					if (tRCD_counter [c_ba] >= tRCD) begin
						previous_state <= current_state;
						current_state <= next_state;
						
						bank_states[c_ba] <= bank_next_state;
						tRCD_counter [c_ba] <= '0;
					end else begin
						current_state <= current_state;// wait logic
					end
				end else if (next_state == PRECHARGE) begin
					if (tRAS_counter [c_ba] >= tRAS) begin
						previous_state <= current_state;
						current_state <= next_state;
						
						bank_states[c_ba] <= bank_next_state;
						tRAS_counter [c_ba] <= '0;
					end else begin
						current_state <= current_state;// wait logic
					end
				end else begin
					previous_state <= current_state;
					current_state <= next_state;
					
					bank_states[c_ba] <= bank_next_state;
				end
            end
			READ: begin
				if (next_state == READ || next_state == READ_WITH_AUTOPRECHARGE) begin
					if (tCCD_counter [c_ba] >= tCCD) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states[c_ba] <= bank_next_state;
						tCCD_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
					end else begin
						current_state <= current_state;//wait logic
					end
				end else if (next_state == WRITE || next_state == WRITE_WITH_AUTOPRECHARGE) begin
					if (tCCD_counter [c_ba] >= tCCD && tRTW_counter[c_ba] >= tRTW) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states[c_ba] <= bank_next_state;
						tCCD_counter [c_ba] <= '0;
						tRTW_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
					end else begin
						current_state <= current_state;//wait logic
					end
				end else if (next_state == PRECHARGE) begin
					if (tRTP_counter [c_ba] >= tRTP) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states[c_ba] <= bank_next_state;
						tRTP_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
					end else begin
						current_state <= current_state;//wait logic
					end
                end else if (next_state == ACTIVE) begin
					previous_state <= current_state;
					current_state <= next_state;
					last_data_in <= 'b0;
					last_data_out <= 'b0;
					transmitted_data <= 0;
					bank_states[c_ba] <= bank_next_state;
					rw_wait_counter <= 0;
				end else begin
					previous_state <= current_state;
					current_state <= next_state;
					rw_wait_counter <= 0;
					bank_states[c_ba] <= bank_next_state;
				end
			end
			
			READ_WITH_AUTOPRECHARGE: begin
				if (tRTP_counter [c_ba] >= tRTP && tRP_counter [c_ba] >= tRP && tRAS_counter [c_ba] >= tRAS) begin
					previous_state <= current_state;
					current_state <= next_state;
					last_data_in <= 'b0;
					last_data_out <= 'b0;
					transmitted_data <= 0;
					bank_states [c_ba] <= bank_next_state;
					tRTP_counter [c_ba]<= '0;
					tRP_counter [c_ba] <= '0;
					tRAS_counter [c_ba] <= '0;
					//ap <= 'b0;
					rw_wait_counter <= 0;
				end else begin
					current_state <= current_state;//wait logic
				end
			end
			WRITE: begin
				if (next_state == WRITE || next_state == WRITE_WITH_AUTOPRECHARGE) begin
					if (tCCD_counter [c_ba] >= tCCD) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states [c_ba] <= bank_next_state;
						tCCD_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
						dq_reg <= 16'bx;
					end else begin
						current_state <= current_state;//wait logic
					end
				end else if (next_state == READ || next_state == READ_WITH_AUTOPRECHARGE) begin
					if (tWTR_counter [c_ba] >= tWTR && tCCD_counter [c_ba] >= tCCD) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states [c_ba] <= bank_next_state;
						tWTR_counter [c_ba] <= '0;
						tCCD_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
						dq_reg <= 16'bx;
					end else begin
						current_state <= current_state;//Wait logic
					end
				end else if (next_state == PRECHARGE) begin
					if (tWR_counter [c_ba] >= tWR) begin
						previous_state <= current_state;
						current_state <= next_state;
						last_data_in <= 'b0;
						last_data_out <= 'b0;
						transmitted_data <= 0;
						bank_states [c_ba] <= bank_next_state;
						tWR_counter [c_ba] <= '0;
						rw_wait_counter <= 0;
						dq_reg <= 16'bx;
					end else begin
						current_state <= current_state;//Wait logic
					end
				end else if (next_state == ACTIVE) begin
					previous_state <= current_state;
					current_state <= next_state;
					last_data_in <= 'b0;
					last_data_out <= 'b0;
					transmitted_data <= 0;
					bank_states[c_ba] <= bank_next_state;
					rw_wait_counter <= 0;
					dq_reg <= 16'bx;
				end
			end

			WRITE_WITH_AUTOPRECHARGE: begin
				if (tWR_counter [c_ba] >= tWR && tRP_counter [c_ba] >= tRP) begin
					previous_state <= current_state;
					current_state <= next_state;
					last_data_in <= 'b0;
					last_data_out <= 'b0;
					transmitted_data <= 0;
					bank_states [c_ba] <= bank_next_state;
					tWR_counter [c_ba] <= '0;
					tRP_counter [c_ba] <= '0;
					//ap <= 'b0;
					rw_wait_counter <= 0;
					dq_reg <= 16'bx;
				end else begin
					current_state <= current_state;//Wait logic
				end
			end

            PRECHARGE: begin
				if (ap) begin
					if (tRP_counter [0] >= tRP && tRP_counter [1] >= tRP && tRP_counter [2] >= tRP && tRP_counter [3] >= tRP) begin
						bank_states <= '{default:BANK_IDLE};
						tRP_counter <= '{default:'b0};
						previous_state <= current_state;
						current_state <= next_state;
						p_ba <= 'b0;
						c_ba <= 'b0;
						ap <= 'b0;
					end else begin		//Wait logic
						for (int i=0;i<4;i++) begin
							if (tRP_counter[i] < tRP) begin
								tRP_counter[i] <= tRP_counter[i] + 1;
							end
						end
					end
				end else begin
					if (tRP_counter [c_ba] >= tRP) begin
						previous_state <= current_state;
						current_state <= next_state;
						
						bank_states [c_ba] <= bank_next_state;
						tRP_counter [c_ba] <= '0;
					end else begin		//Wait logic
						if (tRP_counter[c_ba] < tRP) begin
							tRP_counter[c_ba] <= tRP_counter[c_ba] + 1;
						end
					end
				end
			end

            AUTO_REFRESH: begin
                if (tRFC_counter >= tRFC) begin
					bank_states <= '{default:BANK_IDLE};
					previous_state <= current_state;
					current_state <= next_state;
					tRFC_counter <= '0;
					tREFI_counter <= '0;
                end else begin
					tRFC_counter <= tRFC_counter + 1;
				end
            end

            MODE_REGISTER_SET: begin
				if (tMRD_counter >= tMRD) begin
					previous_state <= current_state;
					current_state <= next_state;
					tMRD_counter <=0;
				end else begin
					tMRD_counter <= tMRD_counter + 1;
				end
            end

			SELF_REFRESH: begin
				if (d_cmd != C_SELF_REFRESH) begin
					if (tCKE_counter >= tCKE) begin
						if (d_cmd != C_SELF_REFRESH && d_cmd != C_READ) begin
							if (tXSNR_counter >= tXSNR) begin
								previous_state <= current_state;
								current_state <= next_state;
								tXSNR_counter <= 0;
								tCKE_counter <= 0;
								tREFI_counter <= 0;
							end else begin
								tXSNR_counter <= tXSNR_counter + 1;
							end
						end else if (d_cmd == C_READ) begin
							if (tXSRD_counter >= tXSRD) begin
								previous_state <= current_state;
								current_state <= next_state;
								tXSRD_counter <= 0;
								tCKE_counter <= 0;
								tREFI_counter <= 0;
							end else begin
								tXSRD_counter <= tXSRD_counter + 1;
							end
						end
					end else begin
						tCKE_counter <= tCKE_counter + 1;
					end
				end
			end
			
			POWER_DOWN: begin
				if (d_cmd != C_NOP) begin
					if (tCKE_counter >= tCKE) begin
						if (next_state == ACTIVE) begin
							if (tXARD_counter >= tXARD) begin
								previous_state <= current_state;
								current_state <= next_state;
								tXARD_counter <= 0;
								tCKE_counter <= 0;
							end else begin
								tXARD_counter <= tXARD_counter + 1;
							end
						end else begin
							previous_state <= current_state;
							current_state <= next_state;
							tCKE_counter <= 0;
						end
					end else begin
						tCKE_counter <= tCKE_counter + 1;
					end
				end
			end

			default: begin
			previous_state <= current_state;
			current_state <= next_state;
			bank_states[c_ba] <= bank_next_state;
			end
        endcase
		end
		//Command Decoding
		if (current_state != previous_state) begin
			case (current_state)
				ACTIVE: begin
					row_addr[c_ba] <= d_row_add;
					addrout <= d_row_add;
					ba <= c_ba;
					row_active[c_ba] <=1'b1;
					cs_n <= 'b0;
					ras_n <= 'b0;
					cas_n <= 'b1;
					we_n <= 'b1;
					cke <= 'b1;
				end
				READ: begin
					addrout <= d_col_add;
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b0;
					we_n <= 'b1;
					cke <= 'b1;
				end
				WRITE: begin
					addrout <= d_col_add;
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b0;
					we_n <= 'b0;
					cke <= 'b1;
					if (!last_data_in) begin
						dqs_en <= 1'b1;
						dq_en <= 1'b1;
					end
				end
				READ_WITH_AUTOPRECHARGE: begin
					addrout <= d_col_add;
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b0;
					we_n <= 'b1;
					cke <= 'b1;
				end
				WRITE_WITH_AUTOPRECHARGE: begin
					addrout <= d_col_add;
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b0;
					we_n <= 'b0;
					cke <= 'b1;
					if (!last_data_in) begin
						dqs_en <= 1'b1;
						dq_en <= 1'b1;
					end
				end
				PRECHARGE: begin
					cs_n <= 'b0;
					ras_n <= 'b0;
					cas_n <= 'b1;
					we_n <= 'b0;
					cke <= 'b1;
						if (!ap) begin
								bank_states[c_ba] <= BANK_IDLE; // Precharge Current Bank
								row_addr[c_ba] <= 'bx; // Precharge Active Row
								row_active[c_ba] <= 'b0;
						end else begin
							bank_states <= '{default:BANK_IDLE};	// Precharge all banks
							row_addr <= '{default:'bx};
							row_active <= '{default:'b0};
						end
				end
				MODE_REGISTER_SET: begin
					cs_n <= 'b0;
					ras_n <= 'b0;
					cas_n <= 'b0;
					we_n <= 'b0;
					cke <= 'b1;
					cmd_done <='b1;
					// Set mode register (burst length, CAS latency, etc.)
					ba <= c_ba;
					addrout [13:7] <= operating_mode;
					addrout [6:4] <= cas_latency;
					addrout [3] <= burst_type;
					addrout [2:0] <= bl;
				end
				BURST_STOP : begin
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b1;
					we_n <= 'b0;
					cke <= 'b1;
				end
				SELF_REFRESH : begin
					cs_n <= 'b0;
					ras_n <= 'b0;
					cas_n <= 'b0;
					we_n <= 'b1;
					cke <= 'b0;
					cmd_done <='b1;
				end
				AUTO_REFRESH : begin
					cs_n <= 'b0;
					ras_n <= 'b0;
					cas_n <= 'b0;
					we_n <= 'b1;
					cke <= 'b1;
				end
				POWER_DOWN : begin
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b1;
					we_n <= 'b1;
					cke <= 'b0;
				end
				NOP : begin
					cs_n <= 'b0;
					ras_n <= 'b1;
					cas_n <= 'b1;
					we_n <= 'b1;
					cke <= 'b1;
				end
			endcase
		end
	end
end

assign tBURST = burst_len_dram/2;
logic counter_r;
logic [2:0] counter_rc;
logic counter_w;
logic [2:0] counter_wc;

assign dq = (!dq_en) ? 'bz : dq_reg;
assign dqs = (!dqs_en) ? 'bz : clk2x;

always_ff @(posedge clk2x or negedge rst_n) begin
	if (!rst_n) begin
		counter_r <= 0;
		counter_w <= 0;
		counter_rc <= 0;
		counter_wc <= 0;
		store_dout <= 'b0;
		//datain_valid <= 'b0;
		dataout_valid <= 'b0;
	end else begin
		case (current_state)
			WRITE, WRITE_WITH_AUTOPRECHARGE: begin
				if (dq_en && dqs_en && rw_wait_counter >= 4) begin
					if (last_data_in) begin
						dqs_en <= 1'b0;
						dq_en <= 1'b0;
					end
					if (!last_data_in) begin
						if (counter_wc < burst_len) begin
							dq_reg <= data_in_x16 [counter_w][counter_wc];
							if (!counter_w && dm_in_reg [counter_wc][0] == 0) ldm <= 1;
							else ldm <= 0;
							if (!counter_w && dm_in_reg [counter_wc][1] == 0) dm <= 1;
							else dm <= 0;
							if (counter_w && dm_in_reg [counter_wc][2] == 0) ldm <= 1;
							else ldm <= 0;
							if (counter_w && dm_in_reg [counter_wc][3] == 0) dm <= 1;
							else dm <= 0;
							counter_w <= counter_w + 1;
							counter_wc <= counter_wc + counter_w;
						end else begin
							last_data_in <= 1;
							ldm <= 0;
							dm <= 0;
							cmd_done <= 1;
							ap <= 'b0;
						end
					end
				end else rw_wait_counter <= rw_wait_counter + 1;
			end
			READ, READ_WITH_AUTOPRECHARGE: begin
				if (!last_data_out) begin
					if (counter_rc < burst_len) begin
						if (rw_wait_counter >= 4 && dq !== 16'bx && dqs !== 1'bz) begin
							data_out_x16 [counter_r][counter_rc]<= dq;
							counter_r <= counter_r + 1;
							counter_rc <= counter_rc + counter_r;
							store_dout <= 'b1;
							dqs_en <= 1'b0;
							dq_en <= 1'b0;
						end else rw_wait_counter <= rw_wait_counter + 1;
					end else begin
						last_data_out <= 1;
						cmd_done <= 1;
						ap <= 'b0;
					end
				end
			end
			BURST_STOP: begin
				data_out_x16 <= '{default:'bx};
				counter_r <=0;
				dqs_en <= 1'b0;
				dq_en <= 1'b0;
			end
			default: begin
				counter_r <= 0;
				counter_w <= 0;
				counter_rc <= 0;
				counter_wc <= 0;
				store_dout <= 'b0;
				dqs_en <= 1'b0;
				dq_en <= 1'b0;
			end
		endcase
	end
end
endmodule
