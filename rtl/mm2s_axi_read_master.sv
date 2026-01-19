module mm2s_axi_read_master #(
    parameter int AW = 32,
    parameter int DW = 64
)(
    input  logic           clk,
    input  logic           rst_n,

    // -----------------------
    // Command interface
    // -----------------------
    input  logic           start_i,
    input  logic [AW-1:0]  src_addr_i,
    input  logic [31:0]    len_bytes_i,
    input  logic [7:0]     max_beats_i,   // 1..256 (0 treated as 1)

    output logic           busy_o,
    output logic           done_o,        // 1-cycle pulse
    output logic           err_o,         // sticky until next start/reset
    output logic [1:0]     err_code_o,    // 00 OK, 01 RRESP, 10 bad align/len, 11 reserved

    // -----------------------
    // FIFO write side
    // -----------------------
    input  logic           fifo_full,
    output logic           fifo_wr_en,
    output logic [DW-1:0]  fifo_wr_data,

    // -----------------------
    // AXI4 Read Address (AR)
    // -----------------------
    output logic [AW-1:0]  m_axi_araddr,
    output logic [7:0]     m_axi_arlen,   // beats-1
    output logic [2:0]     m_axi_arsize,  // log2(bytes/beat)
    output logic [1:0]     m_axi_arburst, // INCR=2'b01
    output logic           m_axi_arvalid,
    input  logic           m_axi_arready,

    // -----------------------
    // AXI4 Read Data (R)
    // -----------------------
    input  logic [DW-1:0]  m_axi_rdata,
    input  logic [1:0]     m_axi_rresp,   // OKAY=2'b00
    input  logic           m_axi_rlast,
    input  logic           m_axi_rvalid,
    output logic           m_axi_rready
);

    localparam int BYTES_PER_BEAT = DW/8;

    // -----------------------
    // Types / state
    // -----------------------
    typedef enum logic [1:0] {S_IDLE, S_ISSUE_AR, S_RECV_R, S_DONE} state_t;

    state_t st, next_st;

    // -----------------------
    // Registers (q) + next
    // -----------------------
    logic [AW-1:0] addr_q,        next_addr_q;
    logic [31:0]   beats_left_q,  next_beats_left_q;
    logic [8:0]    burst_beats_q, next_burst_beats_q; // 1..256
    logic          arvalid_q,     next_arvalid_q;
    logic          err_q,         next_err_q;
    logic [1:0]    err_code_q,    next_err_code_q;

    // -----------------------
    // Helpers
    // -----------------------
    function automatic [2:0] calc_arsize(input int bytes_per_beat);
        case (bytes_per_beat)
            1:  calc_arsize = 3'd0;
            2:  calc_arsize = 3'd1;
            4:  calc_arsize = 3'd2;
            8:  calc_arsize = 3'd3;
            16: calc_arsize = 3'd4;
            32: calc_arsize = 3'd5;
            64: calc_arsize = 3'd6;
            default: calc_arsize = 3'd0;
        endcase
    endfunction

    // v1 assumptions: divisible and aligned
    wire [31:0] total_beats  = len_bytes_i / BYTES_PER_BEAT;
    wire        addr_aligned = (src_addr_i % BYTES_PER_BEAT) == 0;
    wire        len_aligned  = (len_bytes_i % BYTES_PER_BEAT) == 0;

    // -----------------------
    // AXI constant outputs
    // -----------------------
    always_comb begin
        m_axi_arsize  = calc_arsize(BYTES_PER_BEAT);
        m_axi_arburst = 2'b01; // INCR
    end

    // -----------------------
    // Handshakes
    // -----------------------
    wire ar_fire = m_axi_arvalid && m_axi_arready;
    wire accept_r = m_axi_rvalid && m_axi_rready;

    // -----------------------
    // Outputs derived from regs
    // -----------------------
    always_comb begin
        // AR driven from regs
        m_axi_arvalid = arvalid_q;
        m_axi_araddr  = addr_q;
        m_axi_arlen   = (burst_beats_q == 0) ? 8'd0 : (burst_beats_q[7:0] - 8'd1);

        // RREADY: only when receiving and FIFO can take data
        m_axi_rready  = (st == S_RECV_R) && (!fifo_full);

        // FIFO write when we accept R beat
        fifo_wr_en     = accept_r;
        fifo_wr_data   = m_axi_rdata;

        // status
        busy_o         = (st != S_IDLE);
        done_o         = (st == S_DONE); // 1-cycle pulse (we go to IDLE next)

        err_o          = err_q;
        err_code_o     = err_code_q;
    end

    // -----------------------
    // NEXT-STATE / NEXT-DATA (combinational)
    // -----------------------
    always_comb begin
        // Defaults: hold state/regs
        next_st            = st;
        next_addr_q        = addr_q;
        next_beats_left_q  = beats_left_q;
        next_burst_beats_q = burst_beats_q;
        next_arvalid_q     = arvalid_q;
        next_err_q         = err_q;
        next_err_code_q    = err_code_q;

        case (st)
            S_IDLE: begin
                next_arvalid_q = 1'b0;

                if (start_i) begin
                    // Clear previous error on new start
                    next_err_q      = 1'b0;
                    next_err_code_q = 2'b00;

                    // LEN=0: done immediately (no error)
                    if (len_bytes_i == 0) begin
                        next_st = S_DONE;
                    end
                    // alignment / divisibility checks (v1)
                    else if (!addr_aligned || !len_aligned) begin
                        next_err_q      = 1'b1;
                        next_err_code_q = 2'b10;
                        next_st         = S_DONE;
                    end
                    else begin
                        next_addr_q       = src_addr_i;
                        next_beats_left_q = total_beats;
                        next_st           = S_ISSUE_AR;
                    end
                end
            end

            S_ISSUE_AR: begin
                // choose burst beats = min(max_beats_i, beats_left_q) with clamp
                logic [8:0] maxb;
                logic [8:0] bl;

                maxb = (max_beats_i == 0) ? 9'd1 : {1'b0, max_beats_i};

                // beats_left_q can be > 256; cap for compare (any big number works)
                bl   = (beats_left_q > 32'd511) ? 9'd511 : beats_left_q[8:0];

                next_burst_beats_q = (bl < maxb) ? bl : maxb;

                // assert ARVALID until AR handshake
                next_arvalid_q = 1'b1;

                if (ar_fire) begin
                    next_arvalid_q = 1'b0;
                    next_st        = S_RECV_R;
                end
            end

            S_RECV_R: begin
                if (accept_r) begin
                    // check response
                    if (m_axi_rresp != 2'b00) begin
                        next_err_q      = 1'b1;
                        next_err_code_q = 2'b01;
                        next_st         = S_DONE;
                    end else begin
                        // accepted one beat
                        next_beats_left_q = beats_left_q - 32'd1;

                        // end of burst?
                        if (m_axi_rlast) begin
                            next_addr_q = addr_q + (burst_beats_q * BYTES_PER_BEAT);

                            // if that was the last overall beat -> done
                            if (beats_left_q == 32'd1) begin
                                next_st = S_DONE;
                            end else begin
                                next_st = S_ISSUE_AR;
                            end
                        end
                    end
                end
            end

            S_DONE: begin
                // one-cycle pulse; return to idle next
                next_st = S_IDLE;
            end

            default: begin
                next_st = S_IDLE;
            end
        endcase
    end

    // -----------------------
    // REGISTERS (sequential)
    // -----------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st            <= S_IDLE;
            addr_q        <= '0;
            beats_left_q  <= '0;
            burst_beats_q <= '0;
            arvalid_q     <= 1'b0;
            err_q         <= 1'b0;
            err_code_q    <= 2'b00;
        end else begin
            st            <= next_st;
            addr_q        <= next_addr_q;
            beats_left_q  <= next_beats_left_q;
            burst_beats_q <= next_burst_beats_q;
            arvalid_q     <= next_arvalid_q;
            err_q         <= next_err_q;
            err_code_q    <= next_err_code_q;
        end
    end

endmodule
