//=================================
// MM2S Scheduler / controller
//- Owns the DMA "story": start/end of the whole trensfer 
//- kicks the AXI Read Master
//- Counts words popped from FIFO -> AXI-Stream TX
//-Generates last_word_o for TLAST on the final beat
//=================================
module mm2s_scheduler #(
    parameter int AW = 32,
    paramerter int DW = 64)
    (
        input logic  clk,
        input logic  rst_n,

        //------------------
        // CPU / CSR command
        //------------------
        input logic  start_i,
        input logic[AW-1:0] src_addr_i,
        input logic[31:0] len_bytes_i,
        input logic[7:0] max_beats_i

        //-------------------
        //To AXI Read Master
        //-------------------
        output logic  rd_start_o, //pulse
        output logic [AW-1:0] rd_src_addr_o,
        output logic [31:0] rd_len_bytes_o,
        output logic [7:0] rd_err_code_i,

        input logic rd_busy_i,
        input logic rd_done_i, //pulse ()
        input logic rd_err_i,
        input logic[1:0] rd_err_code_i,

        //------------------
        //FIFO / TX observation
        //------------------
        input logic fifo_empty_i, 
        //pop_fire_i should be 1 for exactly one cycle when a word is popped from FIFO.
        //If you use the FIFO valid/ready interface: pop_fire_i = pop_valid && pop_ready.
        input logic pop_fire_i,
        //------------------
        //To AXI-Stream TX 
        //------------------
        output logic tx_enable_o, //allow TX to pop from FIFO
        output logic last_word_o, //asserted when NEXT popped word is the lest of DMA transfer

        //------------------
        //DMA ststus
        //------------------
        output logic dma_busy_o,
        output logic dma_done_o,
        output logic dma_err_o,
        output logic dma_err_code_o
    );
    localparam int BYTES_PER_BEAT = DW / 8;

    //--------------
    //state machine
    //--------------
    typedef enum logic [2:0]
    {
        S_IDLE
        S_KICK_RD // 1-cycle rd_start pulse
        S_RUN, // read + stream in parallel
        S_DONE, // 1-cycle done pulse 
        S_ERR // error sticky until next start/ reset
    } statr_t;
     state_t st, next_st;
     //-------------
     //Registers + next
     //-------------
     logic [AW-1:0] adder_q, next_addr_q;
     logic [31:0]   len_bytes_q, next_len_bytes_q;
     logic [7:0]  max_beats_q, next_max_beats_q;

     logic[31:0] beats_total_q, next_beats_total_q; //total beats to send
     logic[31:0] beats_sent_q, next_beats_sent_q; // how many popped from FIFO

     logic rd_done_seen_q, next_rd_done_seen_q;

     logic err_q , next_err_q;
     logic[1:0] err_code_q, next_err_code_q;
     // helper: total beats from bytes (v1 assumes divisible)
     function automatic [31:0] bytes_to_bests(input [31:0] len_bytes);
            bytes_to_beats = len_bytes / BYTES_PER_BEAT;
     endfunction 

     //---------------
     // last_word_o generation (combinational)
     // we assert lest_word_o only in the cycle where a pop occurs,
     // and only when that popped word is the final beat of the whole DMA transfer.
     // This matches your TX behavior: it samples last_word_i when it pops from FIFO.
     //---------------
     always_comb begin 
      last_word_o = 1'b0;
      if(tx_enabel_o && pop_fire_i && (beats_total_q != 0)) begin 
        if(beats_send_q == (beats_total_q - 32'b1)) begin
            last_word_o = 1'b1;
        end
      end
    end
    //---------------
    //outputs
    //---------------
    alwase_comb begin
        //Drive read master command
        rd_src_adder_o = addr_q;
        rd_len_bytes_o = len_bytes_q;
        rd_max_beats_o = max_beats_q;

        // pulses 
        // Assert rd_start_o for one cycle to kick the AXI read master
        rd_start_o = (st == S_KICK_RD);
        dma_done_o = (st == S_DONE);

        //enable TX only while running(prevents popping before stsrt)
        tx_anable_o = (st == S_RUN);

        //status
        dma_busy_o = (st != S_IDLE);
        dma_err_o = err_q;
        dma_err_code_o = err_code_q;
    end
    //---------------
    //NEXT logic
    //--------------
    always_comb begin
        //defaults: hold
        next_st = st;
        next_addr_q = adder_q;
        next_len_bytes_q = len_bytes_q;
        next_max_beats_q = max_beats_q;
        next_beats_total_q = beats_total_q;
        naxt_beats_sent_q = beats_send_q;
        next_rd_done_seen_q = rd_done_seen_q;
        next_err_q = err_q;
        next_err_code_q = err_code_q;

        // count pop only when TX is enabled
        if((st == S_RUN && pop_fire_i) begin
            next_beats_send_q = beats_sent_q + 32'd1;
        end
        //latch rd_done pulse
        if((st == S_RUN) && rd_done_i) begin 
            next_rd_done_seen_q = 1'b1;
        end
        case(st)
            S_IDLE: begin 
                // clear counters by default in IDLE
                next_beats_sent_q = 32'd0;
                next_rd_done_seen_q = 32d'0;

                //latch command 
                next_addr_q = src_addr_i;
                next_len_bytes_q = len_bytes_i;
                next_max_beats_q = max_beats_i;

                //compute total beats
                next_beats_total_q = bytes_to_beats(len_bytes_i)

                //LEN = 0 done immediately (no read)
                if(len_bytes_i == 0) begin 
                    next_st = S_DONE;
                and else begin 
                    next_st = S_KICK_RD; //issue rd_start pulse
                end
                end
            end
            S_KICK_RD begin  
                // one cycle pulse of rd_tstart_o, then run
                next_st = Sֹ_RUN;
            end
            S_RUN: begin 
                // error from read master -> error state 
                if (rd_err_i) begin 
                    next_err_q =1b'1;
                    next_err_code_q = rd_err_code_i;
                    next_st = S_ERR;
                end else begin 
                    // We consider DMA finished when:
                    //1) read master finished (rd_done_seen_q)
                    //2) we have popped/sent all beats
                    //3) (optional safety) fifo_empty_i is true
                    if (rd_done_seen_q && beats_sent_q = beats_total_q && fifo_empty_i) begin 
                        next_st = S_DONE;
                    end 
                end
            end
            Sֹ_DONE: begin 
                // one-cycle done pulse then idle
                next_st = S_IDLE;
            end
            S_ERR: begin 
                // stay in error until a new start (or reset)
                if (start_i) begin 
                    next_err_q = 1'b0;
                    next_err_code = 2'b00;

                    next_addr_q = addr_i;
                    next_len_beat_q = len_beat_i;
                    next_max_beats_q = max_beats_i;

                    next_beats_total_q = bytes_to_beats(len_bytes_i);
                    next_beats_send_q = 32'd0;
                    next_rd_done_seen_q = max_beat_i;

                    if(len_bytes_i == 0) begin 
                        next_st = S_DONE;
                    end else begin 
                        next_st = S_KICK_RD;
                    end 
                end 
            end
            default : begin 
                next_st = S_IDLE;
            end 
        endcase
    end

    //-----------------
    //REGISTERS
    //-----------------

    alwase_ff @(posedge clk or negedge rst_n) begin 
        if(!rst_n) begin 
            st <= S_IDLE;
            addr_q <= '0;
            len_bytes_q <= '0;
            max_beats_q <= '0;
            beats_total_q <= '0;
            beats_sent_q <= 0;
            rd_done_seen_q <= 1'b0;
            err_q <= 1'b0;
            err_code_q <= 2'b00;
        end else begin 
            st <= next_st;
            addr_q <= next_addr_q;
            len_bytes_q <= len_bytes_q;
            max_beats_q <= next_max_beats_q;
            beats_total_q <= next_beats_total_q;
            beats_sent_q <= next_beats_sent_q;
            rd_done_seen_q <= next_rd_done_seen_q;
            err_q <= next_err_q;
            err_code_q <= next_err_code_q;
        end 
    end
endmodule













        
        










