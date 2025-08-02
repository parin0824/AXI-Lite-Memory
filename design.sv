// AXI4-Lite Slave Module with FSM-based read/write handling
module axilite_s(
    input  wire         s_axi_aclk,      // Clock signal
    input  wire         s_axi_aresetn,   // Active-low reset
    input  wire         s_axi_awvalid,   // Write address valid
    output reg          s_axi_awready,   // Slave ready to accept write address
    input  wire [31: 0] s_axi_awaddr,    // Write address

    input  wire         s_axi_wvalid,    // Write data valid
    output reg          s_axi_wready,    // Slave ready to accept write data
    input  wire [31: 0] s_axi_wdata,     // Write data

    output reg          s_axi_bvalid,    // Write response valid
    input  wire         s_axi_bready,    // Master ready to accept write response
    output reg  [1: 0]  s_axi_bresp,     // Write response

    input  wire         s_axi_arvalid,   // Read address valid
    output reg          s_axi_arready,   // Slave ready to accept read address
    input  wire [31: 0] s_axi_araddr,    // Read address

    output reg          s_axi_rvalid,    // Read data valid
    input  wire         s_axi_rready,    // Master ready to accept read data
    output reg [31: 0]  s_axi_rdata,     // Read data
    output reg  [1: 0]  s_axi_rresp      // Read response
);

// FSM States for transaction control
localparam idle            = 0, 
           send_waddr_ack  = 1, // Accept write address
           send_raddr_ack  = 2, // Accept read address
           send_wdata_ack  = 3, // Accept write data
           update_mem      = 4, // Store data to memory
           send_wr_err     = 5, // Invalid write address
           send_wr_resp    = 6, // Send write response
           gen_data        = 7, // Read data from memory
           send_rd_err     = 8, // Invalid read address
           send_rdata      = 9; // Send read data

reg [3:0] state = idle;         // Current FSM state
reg [3:0] next_state = idle;    // Not used
reg [1:0] count = 0;            // Small delay counter
reg [31:0] waddr, raddr;        // Latched addresses
reg [31:0] wdata, rdata;        // Latched data
reg [31:0] mem [128];           // 128-word memory array

// Main FSM block
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        // Reset all control signals and memory
        state <= idle;
        for (int i = 0; i < 128; i++) mem[i] <= 0;
        s_axi_awready <= 0;
        s_axi_wready  <= 0;
        s_axi_bvalid  <= 0;
        s_axi_bresp   <= 0;
        s_axi_arready <= 0;
        s_axi_rvalid  <= 0;
        s_axi_rdata   <= 0;
        s_axi_rresp   <= 0;
        waddr         <= 0;
        raddr         <= 0;
        wdata         <= 0;
        rdata         <= 0;
    end else begin
        case (state)
        idle: begin
            // Clear all handshake signals and wait for AWVALID or ARVALID
            s_axi_awready <= 0;
            s_axi_wready  <= 0;
            s_axi_bvalid  <= 0;
            s_axi_bresp   <= 0;
            s_axi_arready <= 0;
            s_axi_rvalid  <= 0;
            s_axi_rdata   <= 0;
            s_axi_rresp   <= 0;
            count         <= 0;

            if (s_axi_awvalid) begin
                // Begin write transaction
                state         <= send_waddr_ack;
                waddr         <= s_axi_awaddr;
                s_axi_awready <= 1;
            end else if (s_axi_arvalid) begin
                // Begin read transaction
                state         <= send_raddr_ack;
                raddr         <= s_axi_araddr;
                s_axi_arready <= 1;
            end
        end

        send_waddr_ack: begin
            // Write address accepted, now wait for data
            s_axi_awready <= 0;
            if (s_axi_wvalid) begin
                wdata        <= s_axi_wdata;
                s_axi_wready <= 1;
                state        <= send_wdata_ack;
            end
        end

        send_wdata_ack: begin
            // Write data accepted, validate address and store
            s_axi_wready <= 0;
            if (waddr < 128) begin
                // Valid write address: update memory
                mem[waddr] <= wdata;
                state      <= send_wr_resp;
            end else begin
                // Invalid write address: respond with error
                state        <= send_wr_err;
                s_axi_bresp  <= 2'b11;  // SLVERR
                s_axi_bvalid <= 1;
            end
        end

        send_wr_resp: begin
            // Send successful write response
            s_axi_bresp  <= 2'b00;  // OKAY
            s_axi_bvalid <= 1;
            if (s_axi_bready)
                state <= idle;
        end

        send_wr_err: begin
            // Wait for master to accept error response
            if (s_axi_bready)
                state <= idle;
        end

        send_raddr_ack: begin
            // Read address accepted
            s_axi_arready <= 0;
            if (raddr < 128)
                state <= gen_data;
            else begin
                // Invalid read address
                s_axi_rvalid <= 1;
                s_axi_rdata  <= 0;
                s_axi_rresp  <= 2'b11; // SLVERR
                state        <= send_rd_err;
            end
        end

        gen_data: begin
            // Simulate a read delay using counter
            if (count < 2) begin
                rdata <= mem[raddr];
                count <= count + 1;
            end else begin
                // Send valid read data
                s_axi_rvalid <= 1;
                s_axi_rdata  <= rdata;
                s_axi_rresp  <= 2'b00; // OKAY
                if (s_axi_rready)
                    state <= idle;
            end
        end

        send_rd_err: begin
            // Wait for master to accept read error response
            if (s_axi_rready)
                state <= idle;
        end

        default: state <= idle;
        endcase
    end
end
endmodule

// AXI Interface Declaration (used in TB or system integration)
interface axi_if;
  logic clk, resetn;
  logic awvalid, awready;
  logic arvalid, arready;
  logic wvalid, wready;
  logic bready, bvalid;
  logic rvalid, rready;
  logic [31:0] awaddr, araddr, wdata, rdata;
  logic [1:0] wresp, rresp;
endinterface

