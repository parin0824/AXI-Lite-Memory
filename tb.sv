class transaction;

  randc bit        op;         // 1-bit operation: 1 = write, 0 = read (cyclic rand)
  rand bit [31:0] awaddr;      // AXI write address
  rand bit [31:0] wdata;       // AXI write data
  rand bit [31:0] araddr;      // AXI read address
       bit [31:0] rdata;       // AXI read data (filled by monitor)
       bit [1:0]  wresp;       // Write response from DUT
       bit [1:0]  rresp;       // Read response from DUT

  // Constraint: define standard range + out-of-bound edge cases for addresses
   constraint addr_range {
     (awaddr inside {[0:255]});
     (araddr inside {[0:255]});
  }

  // Constraint: data range limited for easier debug/verification
    constraint data_range {
    (wdata inside {32'hFFFFFFFF, 32'h80000000}); // add max and signed MSB test
    (rdata inside {32'hFFFFFFFE, 32'h00000000}); // edge of valid/invalid
  }
endclass

class generator;

  transaction tr;
  mailbox #(transaction) mbxgd; // Mailbox to send transactions to driver

  event done;       // Raised when all transactions are generated
  event sconext;    // Used for sync with scoreboard

  int count = 0;    // Total number of transactions to generate

  // Constructor
  function new(mailbox #(transaction) mbxgd);
    this.mbxgd = mbxgd;
    tr = new();
  endfunction

  // Task to generate and send `count` number of transactions
  task run();
    for(int i = 0; i < count; i++) begin
      assert(tr.randomize) else $error("Randomization Failed");
      $display("[GEN] : OP : %0b awaddr : %0d wdata : %0d araddr : %0d", tr.op, tr.awaddr, tr.wdata, tr.araddr);
      mbxgd.put(tr);   // Send transaction to driver
      @(sconext);      // Wait for scoreboard signal to proceed
    end
    ->done;            // Signal that generation is done
  endtask

endclass

class driver;

  virtual axi_if vif;              // Virtual interface handle

  transaction tr;
  mailbox #(transaction) mbxgd;    // From generator
  mailbox #(transaction) mbxdm;    // To monitor

  // Constructor
  function new(mailbox #(transaction) mbxgd, mailbox #(transaction) mbxdm);
    this.mbxgd = mbxgd;
    this.mbxdm = mbxdm;
  endfunction

  // Task to apply reset to DUT
  task reset();
    vif.resetn  <= 1'b0;
    vif.awvalid <= 1'b0;
    vif.awaddr  <= 0;
    vif.wvalid  <= 0;
    vif.wdata   <= 0;
    vif.bready  <= 0;
    vif.arvalid <= 1'b0;
    vif.araddr  <= 0;
    repeat(5) @(posedge vif.clk);
    vif.resetn <= 1'b1;
    $display("-----------------[DRV] : RESET DONE-----------------------------");
  endtask

  // Task to perform write transaction
  task write_data(input transaction tr);
    $display("[DRV] : OP : %0b awaddr : %0d wdata : %0d", tr.op, tr.awaddr, tr.wdata);
    mbxdm.put(tr);   // Send transaction to monitor
    vif.resetn  <= 1'b1;
    vif.awvalid <= 1'b1;
    vif.arvalid <= 1'b0;
    vif.araddr  <= 0;
    vif.awaddr  <= tr.awaddr;
    @(negedge vif.awready);
    vif.awvalid <= 1'b0;
    vif.awaddr  <= 0;
    vif.wvalid  <= 1'b1;
    vif.wdata   <= tr.wdata;
    @(negedge vif.wready);
    vif.wvalid  <= 1'b0;
    vif.wdata   <= 0;
    vif.bready  <= 1'b1;
    vif.rready  <= 1'b0;
    @(negedge vif.bvalid);
    vif.bready  <= 1'b0;
  endtask

  // Task to perform read transaction
  task read_data(input transaction tr);
    $display("[DRV] : OP : %0b araddr : %0d", tr.op, tr.araddr);
    mbxdm.put(tr);   // Send transaction to monitor
    vif.resetn  <= 1'b1;
    vif.awvalid <= 1'b0;
    vif.awaddr  <= 0;
    vif.wvalid  <= 0;
    vif.wdata   <= 0;
    vif.bready  <= 0;
    vif.arvalid <= 1'b1;
    vif.araddr  <= tr.araddr;
    @(negedge vif.arready);
    vif.araddr  <= 0;
    vif.arvalid <= 1'b0;
    vif.rready  <= 1'b1;
    @(negedge vif.rvalid);
    vif.rready  <= 1'b0;
  endtask

  // Task to run the driver continuously
  task run();
    forever begin
      mbxgd.get(tr);         // Get transaction from generator
      @(posedge vif.clk);    // Wait for positive edge
      if (tr.op == 1'b1)
        write_data(tr);      // Write operation
      else
        read_data(tr);       // Read operation
    end
  endtask

endclass

class monitor;

  virtual axi_if vif;

  transaction tr, trd;
  mailbox #(transaction) mbxms;  // To scoreboard
  mailbox #(transaction) mbxdm;  // From driver

  // Constructor
  function new(mailbox #(transaction) mbxms, mailbox #(transaction) mbxdm);
    this.mbxms = mbxms;
    this.mbxdm = mbxdm;
  endfunction

  // Monitor task
  task run();
    tr = new();
    forever begin
      @(posedge vif.clk);
      mbxdm.get(trd);  // Get original transaction for reference

      if (trd.op == 1) begin // Write operation
        tr.op     = trd.op;
        tr.awaddr = trd.awaddr;
        tr.wdata  = trd.wdata;
        @(posedge vif.bvalid);
        tr.wresp  = vif.wresp;
        @(negedge vif.bvalid);
        $display("[MON] : OP : %0b awaddr : %0d wdata : %0d wresp:%0d", tr.op, tr.awaddr, tr.wdata, tr.wresp);
        mbxms.put(tr);
      end else begin // Read operation
        tr.op = trd.op;
        tr.araddr = trd.araddr;
        @(posedge vif.rvalid);
        tr.rdata = vif.rdata;
        tr.rresp = vif.rresp;
        @(negedge vif.rvalid);
        $display("[MON] : OP : %0b araddr : %0d rdata : %0d rresp:%0d", tr.op, tr.araddr, tr.rdata, tr.rresp);
        mbxms.put(tr);
      end
    end
  endtask

endclass

class scoreboard;

  transaction tr, trd;
  event sconext;

  mailbox #(transaction) mbxms;

  bit [31:0] temp;
  bit [31:0] data[128] = '{default:0}; // Reference model storage

  // Constructor
  function new(mailbox #(transaction) mbxms);
    this.mbxms = mbxms;
  endfunction

  // Scoreboard run task
  task run();
    forever begin
      mbxms.get(tr);  // Get transaction from monitor

      if (tr.op == 1) begin // Write operation
        $display("[SCO] : OP : %0b awaddr : %0d wdata : %0d wresp : %0d", tr.op, tr.awaddr, tr.wdata, tr.wresp);
        if (tr.wresp == 3)
          $display("[SCO] : DEC ERROR");
        else begin
          data[tr.awaddr] = tr.wdata;
          $display("[SCO] : DATA STORED ADDR :%0d and DATA :%0d", tr.awaddr, tr.wdata);
        end
      end else begin // Read operation
        $display("[SCO] : OP : %0b araddr : %0d rdata : %0d rresp : %0d", tr.op, tr.araddr, tr.rdata, tr.rresp);
        temp = data[tr.araddr];
        if (tr.rresp == 3)
          $display("[SCO] : DEC ERROR");
        else if (tr.rresp == 0 && tr.rdata == temp)
          $display("[SCO] : DATA MATCHED");
        else
          $display("[SCO] : DATA MISMATCHED");
      end

      $display("----------------------------------------------------");
      ->sconext; // Signal generator to proceed
    end
  endtask

endclass
module tb;

  monitor mon;
  generator gen;
  driver drv;
  scoreboard sco;

  event nextgd;
  event nextgm;

  mailbox #(transaction) mbxgd, mbxms, mbxdm;

  axi_if vif();  // Instantiate virtual interface

  // DUT instantiation
  axilite_s dut (
    vif.clk, vif.resetn, vif.awvalid, vif.awready, vif.awaddr,
    vif.wvalid, vif.wready, vif.wdata, vif.bvalid, vif.bready, vif.wresp,
    vif.arvalid, vif.arready, vif.araddr, vif.rvalid, vif.rready, vif.rdata, vif.rresp
  );

  // Clock generation
  initial begin
    vif.clk <= 0;
  end
  always #5 vif.clk <= ~vif.clk;

  // Testbench setup
  initial begin
    mbxgd = new();
    mbxms = new();
    mbxdm = new();

    gen = new(mbxgd);
    drv = new(mbxgd, mbxdm);
    mon = new(mbxms, mbxdm);
    sco = new(mbxms);

    gen.count = 10;

    drv.vif = vif;
    mon.vif = vif;

    gen.sconext = nextgm;
    sco.sconext = nextgm;
  end

  // Simulation run
  initial begin
    drv.reset();
    fork
      gen.run();
      drv.run();
      mon.run();
      sco.run();
    join_any

    wait(gen.done.triggered);
    $finish;
  end

  // Dump waveform
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;
  end

endmodule
