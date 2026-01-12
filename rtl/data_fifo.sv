// rtl/data_fifo.sv
// single clock synchrinius FIFO 
// supports simultaneous push/pop
// Exposes full/empty and level
// verilator friendly

module data_fifo #(parameter int unsigned DATA_W = 64, parameter int unsigned DEPTH = 16)
(
    input logic  clk,
    input logic  rst_n, //if we wont to reset we need 0

    //push side
    input logic  push_valid, 
    output logic push_read,
    input logic [DATA_W-1:0]  push_data,
    // pop side
    input logic  pop_valid,
    input logic  pop_ready, 
    input logic [DATA_W -1: 0] pop_data,
    //status 
    output logic full, 
    output logic empty, 
    output logic [$clog2(DEPTH+1)-1:0] level
);

// parameter check
// Depth must be power of 2 for this implementation.
localparam int unsigned ADDR_W = $clog2(DEPTH);

// Storage (beild matrix row = DATA_W, col = DEPTH)
logic [DATA_W-1:0] mem [0:DEPTH-1];

//Pointers + occupancy
logic [ADDR_W - 1:0] rd_ptr, wr_ptr;

//Handshakes
logic do_push, do_pop;

// combinational status
always_comb begin 
    empty = (level == 0);
    full = (level == DEPTH[$clog2(DEPTH+1)-1:0]);
    push_ready = !full;
    pop_valid = !empty;
    do_push = push_valid && push_ready;
    do_pop = pop_valid && pop_ready;
end

//Read data:  show the current head element
always_comb begin 
    pop_data = mem[rd_ptr];
end

// Sequential logic with async active-low reset
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rd_ptr <= '0;
        wr_ptr <= '0;
        level <= '0;
    end else begin 
        // write memory on push 
        if (do_push) begin 
            mem[wr_ptr] <= push_data;
            wr_ptr = wr_ptr + 1'b1;
        end
    
        //Advance read pointer on pop
        if (do_pop) begin 
            rd_ptr = rd_ptr + 1'b1;
        end

        // update level(handle simultaneous push/pop)
        unique case({do_push, do_pop})
            2'b10: level <= level + 1'b1;
            2'b01: level <= level - 1'b1;
            // 00 or 11: unchanged
            default: level <= level;
        endcase
    end 
end
endmodule

    


