//============================================================
// MM2S AXI-Stream Transmitter 
// - Pulls data from internal FIFO and drives AXI-Stream output
// - Handles backpressure correctly (tdata stable while stalled)
//============================================================
module mm2s_axis_tx #(
    parameter int DW = 64
)
(
    input logic  clk,
    input logic  rst_n,
    //------------------
    // Internal FIFO interface(read side)
    //------------------
    input logic  fifo_empty,
    input logic [DW - 1: 0] fifo_rdata,
    output logic fifo_rd_en,

    //optional: tells TX that the NEXT word popped is the last word of the transfer 
    // If you don't support TLAST yet, tie this to 1'b0.
    input logic  last_word_i, 
    //------------------
    //AXI-Stream output (DMA external interface)
    //------------------
    output logic [DW-1:0] m_axis_tdata,
    output logic          m_axis_tvalid,
    input logic           m_axis_tready,
    output logic          m_axis_tlast

);
// Holding register: keeps data stable during backpressure
logic [DW-1:0] hold_data;
logic          hold_valid;
logic          hold_last;

// Handshake on AXI-Stream
wire axis_fire = m_axis_tvalid && m_axis_tready;

// we can fetch a new word from FIFO when:
// - we currently don't have a valid word held, OR
//- The current word will be accepted this cycle (axis_fire), so we can "refill"
//
// BUT we only assert fifo_rd_en in FIFO isn't empty/
wire want_new_word = (!hold_valid) || axis_fire;

// pop from FIFO only when we want a new word and FIFO has data 
wire can_pop_fifo = want_new_word && (!fifo_empty);

// drive AXIS outputs from holding reg
always_comb begin 
    m_axis_tvalid = hold_valid;
    m_axis_tdata = hold_data;
    m_axis_tlast = hold_last;
    fifo_rd_en = can_pop_fifo;
end


//Update holding reg 
always_ff@(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
        hold_valid <= 1'b0;
        hold_data <= '0;
        hold_last <= 1'b0;
   end
   // pop new word from FIFO
   else begin
        if (can_pop_fifo) begin
            hold_valid <= 1'b1;
            hold_data <= fifo_rdata;
            hold_last <= last_word_i;
        end
    // Word sent and FIFO is empty
    else if(axis_fire) begin
            hold_valid <= 1'b0;
            hold_last <= 1'b0;
        end
    // Otherwise hold values stable(backpressure)
   end 
end 
endmodule

    




