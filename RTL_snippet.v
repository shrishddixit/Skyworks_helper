reg [0:3][31:0] blk_ks_buf;
always @(posedge clk or negedge rstn)
    if(!rstn)
        blk_ks_buf <= '0;
    else if(blk_start)
        blk_ks_buf <= '0;
    else if(blk_ks_buf_load)
        blk_ks_buf <= aes_data_out;

// size of buffer in 32-bit words
reg [2:0] blk_ks_size;
always @(posedge clk or negedge rstn)
    if(!rstn)
        blk_ks_size <= 3'd0;
    else if(blk_start)
        blk_ks_size <= 3'd0;
    else if(blk_ks_buf_load)
        blk_ks_size <= 3'd4;
    else if(blk_ks_req && blk_ks_ack)
        blk_ks_size <= blk_ks_size - 1'b1;

assign blk_ks_buf_empty = !blk_ks_size;

// ready when buffer has data
assign blk_ks_ack = !blk_ks_buf_empty;

// mux keystream from each word of buffer
wire [1:0] blk_ks_idx = 4'd4 - blk_ks_size;
assign blk_keystream = blk_ks_buf[blk_ks_idx];


always @(posedge clk_wrrx_rdtx_buf or negedge reset_n) begin
//always @(posedge clk_16m_g or negedge reset_n) begin

  if(wrrx_start) begin

        end else if(tx_enc_cnt[0] && blk_ks_ack_i[0]) begin
            case(tx_enc_cnt[0])
            2'b01: buf1[115    -:32] <= buf1[115    -:32] ^ blk0_keystream_i;
            2'b10: buf1[115-32 -:32] <= buf1[115-32 -:32] ^ blk0_keystream_i;
            2'b11: buf1[115-64 -:32] <= buf1[115-64 -:32] ^ blk0_keystream_i;
            endcase
        end           



