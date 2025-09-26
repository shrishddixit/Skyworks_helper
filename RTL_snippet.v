at every posedge of clk --> check if blk_ks_ack is high, if yes then perform xor operation between buf1[tx_enc_cnt[0]] , blk_ks_buf[blk_ks_idx] and check the value buf1[tx_enc_cnt[0]] this value should be the one that is used for calculation on the next posedge of the clk.
    tx_enc_cnt[1:0], blk_ks_idx[1:0], when tx_enc_cnt = 0 do nothing. 




I have aes_data_in as input, 
aes_start is when aes_data_calculated will be avaialble. 
when aes_done is seen aes_data_collected should beon the aes_data_out 


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



