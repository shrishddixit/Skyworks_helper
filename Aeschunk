`timescale 1ns/1ps

// ===================== aes_chunker_async (from previous step) =====================
module aes_chunker_async (
    input  wire        start,                // level high => capture new 128b block
    input  wire [2:0]  counter,              // use 1..4
    input  wire [127:0] aes_data_out_model,  // 128-bit AES block

    output wire [31:0] data_out,
    output wire        data_valid
);
    reg [127:0] data_latched;   // last 128-bit captured when start=1
    reg [2:0]   last_counter;   // remembers last counter that produced output
    reg [31:0]  data_out_r;     // latched 32-bit output
    reg         data_valid_r;   // pulses high when counter changes (1..4)

    assign data_out   = data_out_r;
    assign data_valid = data_valid_r;

    // Reload 128-bit block whenever start is high
    always @(*) begin
        if (start) begin
            data_latched  = aes_data_out_model;
            last_counter  = 3'd0;   // force next valid counter to be treated as change
            data_valid_r  = 1'b0;   // no valid during reload
            // data_out_r holds previous value
        end
    end

    // Update on counter change (1..4) when start=0
    always @(*) begin
        if (!start) begin
            if ((counter >= 3'd1) && (counter <= 3'd4) && (counter != last_counter)) begin
                case (counter)
                    3'd1: data_out_r = data_latched[127:96];
                    3'd2: data_out_r = data_latched[95:64];
                    3'd3: data_out_r = data_latched[63:32];
                    3'd4: data_out_r = data_latched[31:0];
                    default: /* no-op */;
                endcase
                data_valid_r = 1'b1;
                last_counter = counter;
            end else begin
                data_valid_r = 1'b0; // hold output, no new valid
                // data_out_r, last_counter hold
            end
        end
        // when start=1, reload logic above is active
    end
endmodule

// ============================== Test Module Example ==============================
// Drop this into your test.v. It shows how to:
//  - drive 'start' (level) to reload a new 128-bit AES block,
//  - change 'counter' to fetch 32-bit chunks,
//  - observe data_out/data_valid.

module tb_aes_chunker;
    // DUT-facing signals
    reg         start;
    reg  [2:0]  counter;
    reg  [127:0] aes_data_out_model;
    wire [31:0] data_out;
    wire        data_valid;

    // Instantiate the chunker in your test module
    aes_chunker_async u_chunker (
        .start(start),
        .counter(counter),
        .aes_data_out_model(aes_data_out_model),
        .data_out(data_out),
        .data_valid(data_valid)
    );

    // Simple model: update 128-bit block exactly on posedge of start
    // (Matches your statement "aes_data_out_model will give me 128 bit data once posedge of start is seen")
    // This is testbench code; replace with your real AES model hook.
    always @(posedge start) begin
        // Example deterministic pattern for visibility; change to your model output
        aes_data_out_model <= {32'hDEADBEEF, 32'hA5A5_5A5A, 32'h1234_5678, 32'hCAFEBABE};
    end

    // Display when a new 32-bit word is produced
    always @(posedge data_valid) begin
        $display("[%0t] counter=%0d -> data_out=0x%08h", $time, counter, data_out);
    end

    // Stimulus
    initial begin
        // init
        start   = 1'b0;
        counter = 3'd0;
        aes_data_out_model = 128'd0;

        // ---- Load first 128-bit block ----
        #5  start = 1'b1;   // raise start to allow chunker to latch new 128b
        #1  start = 1'b0;   // drop start (reload done)

        // Walk through counters 1..4 (each change should trigger data_valid)
        #5  counter = 3'd1;
        #5  counter = 3'd2;
        #5  counter = 3'd3;
        #5  counter = 3'd4;

        // Re-select same counter (no change => no new valid pulse)
        #5  counter = 3'd4;

        // ---- Load second 128-bit block ----
        // On next posedge start, aes_data_out_model updates and chunker reloads.
        #10 start = 1'b1;
        #1  start = 1'b0;

        // Now fetch again
        #5  counter = 3'd1;
        #5  counter = 3'd3;
        #5  counter = 3'd2;
        #5  counter = 3'd4;

        #10 $finish;
    end
endmodule
