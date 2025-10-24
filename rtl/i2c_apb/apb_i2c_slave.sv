// SPDX-License-Identifier: MIT
// Simple APB-accessible I2C slave peripheral

module apb_i2c_slave #(
  parameter int APB_ADDR_WIDTH = 8
) (
  input  logic                      pclk,
  input  logic                      presetn,
  input  logic [APB_ADDR_WIDTH-1:0] paddr,
  input  logic                      psel,
  input  logic                      penable,
  input  logic                      pwrite,
  input  logic [31:0]               pwdata,
  output logic [31:0]               prdata,
  output logic                      pready,
  output logic                      pslverr,
  // I2C lines
  input  logic                      scl_i,
  input  logic                      sda_i,
  output logic                      sda_o,
  output logic                      sda_oe,
  output logic                      irq
);
  typedef enum logic [2:0] {
    SLV_IDLE,
    SLV_ADDR,
    SLV_RW,
    SLV_ACK_ADDR,
    SLV_RX,
    SLV_ACK_RX,
    SLV_TX,
    SLV_ACK_TX
  } slv_state_t;

  // Register offsets
  localparam logic [APB_ADDR_WIDTH-1:0] REG_CTRL       = 8'h00;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_STATUS     = 8'h04;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_ADDRESS    = 8'h08;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_RXDATA     = 8'h0C;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_TXDATA     = 8'h10;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_IRQ_STATUS = 8'h14;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_IRQ_ENABLE = 8'h18;

  typedef struct packed {
    logic enable;
    logic auto_ack_data;
    logic nack_on_overrun;
    logic general_call_en;
    logic [28:0] rsvd;
  } ctrl_reg_t;

  ctrl_reg_t ctrl_reg;

  logic status_busy;
  logic status_address_hit;
  logic status_start_seen;
  logic status_stop_seen;
  logic status_rx_valid;
  logic status_tx_ready;
  logic status_nack_sent;
  logic status_last_rw;

  logic [6:0] addr_reg;
  logic [7:0] rxdata_reg;
  logic [7:0] txdata_reg;
  logic       tx_valid;

  logic irq_status_start;
  logic irq_status_stop;
  logic irq_status_rx;
  logic irq_status_tx;
  logic [3:0] irq_enable_reg;

  assign irq = ((irq_status_start & irq_enable_reg[0]) |
                (irq_status_stop  & irq_enable_reg[1]) |
                (irq_status_rx    & irq_enable_reg[2]) |
                (irq_status_tx    & irq_enable_reg[3])) & ctrl_reg.enable;

  // APB handshake
  logic apb_write = psel & penable & pwrite;
  logic apb_read  = psel & penable & ~pwrite;
  assign pready  = 1'b1;
  assign pslverr = 1'b0;

  // Register write path
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      ctrl_reg       <= '{default:'0};
      addr_reg       <= 7'h42;
      txdata_reg     <= 8'hFF;
      irq_enable_reg <= 4'b0001;
    end else begin
      if (apb_write) begin
        unique case (paddr)
          REG_CTRL: begin
            ctrl_reg.enable          <= pwdata[0];
            ctrl_reg.auto_ack_data   <= pwdata[1];
            ctrl_reg.nack_on_overrun <= pwdata[2];
            ctrl_reg.general_call_en <= pwdata[3];
          end
          REG_ADDRESS: addr_reg <= pwdata[6:0];
          REG_TXDATA: begin
            txdata_reg <= pwdata[7:0];
          end
          REG_IRQ_ENABLE: irq_enable_reg <= pwdata[3:0];
          default: ;
        endcase
      end
    end
  end

  // APB readback
  always_comb begin
    prdata = '0;
    unique case (paddr)
      REG_CTRL: begin
        prdata[0] = ctrl_reg.enable;
        prdata[1] = ctrl_reg.auto_ack_data;
        prdata[2] = ctrl_reg.nack_on_overrun;
        prdata[3] = ctrl_reg.general_call_en;
      end
      REG_STATUS: begin
        prdata[0] = status_busy;
        prdata[1] = status_address_hit;
        prdata[2] = status_start_seen;
        prdata[3] = status_stop_seen;
        prdata[4] = status_rx_valid;
        prdata[5] = status_tx_ready;
        prdata[6] = status_nack_sent;
        prdata[7] = status_last_rw;
      end
      REG_ADDRESS: prdata[6:0] = addr_reg;
      REG_RXDATA: prdata[7:0] = rxdata_reg;
      REG_TXDATA: prdata[7:0] = txdata_reg;
      REG_IRQ_STATUS: begin
        prdata[0] = irq_status_start;
        prdata[1] = irq_status_stop;
        prdata[2] = irq_status_rx;
        prdata[3] = irq_status_tx;
      end
      REG_IRQ_ENABLE: prdata[3:0] = irq_enable_reg;
      default: prdata = '0;
    endcase
  end

  // Synchronize the I2C inputs with a small glitch filter
  logic [2:0] scl_filter;
  logic [2:0] sda_filter;
  logic       scl_sync;
  logic       sda_sync;
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      scl_filter <= 3'b111;
      sda_filter <= 3'b111;
      scl_sync   <= 1'b1;
      sda_sync   <= 1'b1;
    end else begin
      scl_filter <= {scl_filter[1:0], scl_i};
      sda_filter <= {sda_filter[1:0], sda_i};
      if (&scl_filter)
        scl_sync <= 1'b1;
      else if (~|scl_filter)
        scl_sync <= 1'b0;
      if (&sda_filter)
        sda_sync <= 1'b1;
      else if (~|sda_filter)
        sda_sync <= 1'b0;
    end
  end

  logic scl_sync_d, sda_sync_d;
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      scl_sync_d <= 1'b1;
      sda_sync_d <= 1'b1;
    end else begin
      scl_sync_d <= scl_sync;
      sda_sync_d <= sda_sync;
    end
  end

  wire scl_rising = (scl_sync_d == 1'b0) && (scl_sync == 1'b1);
  wire scl_falling= (scl_sync_d == 1'b1) && (scl_sync == 1'b0);
  wire start_cond = (sda_sync_d == 1'b1) && (sda_sync == 1'b0) && (scl_sync == 1'b1);
  wire stop_cond  = (sda_sync_d == 1'b0) && (sda_sync == 1'b1) && (scl_sync == 1'b1);

  // Slave protocol machine
  slv_state_t state;
  logic [2:0] bit_cnt;
  logic [7:0] shift_reg;
  logic       rw_flag;
  logic       addr_match;
  logic [7:0] rx_shift;
  logic       ack_pending;
  logic       ack_after_high;

  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      state             <= SLV_IDLE;
      bit_cnt           <= 3'd0;
      shift_reg         <= 8'h00;
      rw_flag           <= 1'b0;
      addr_match        <= 1'b0;
      status_busy       <= 1'b0;
      status_address_hit<= 1'b0;
      status_start_seen <= 1'b0;
      status_stop_seen  <= 1'b0;
      status_rx_valid   <= 1'b0;
      status_tx_ready   <= 1'b1;
      status_nack_sent  <= 1'b0;
      status_last_rw    <= 1'b0;
      sda_oe            <= 1'b0;
      sda_o             <= 1'b0;
      rxdata_reg        <= 8'h00;
      rx_shift          <= 8'h00;
      ack_pending       <= 1'b0;
      ack_after_high    <= 1'b0;
      irq_status_start  <= 1'b0;
      irq_status_stop   <= 1'b0;
      irq_status_rx     <= 1'b0;
      irq_status_tx     <= 1'b0;
      tx_valid          <= 1'b0;
    end else begin
      if (!ctrl_reg.enable) begin
        status_busy       <= 1'b0;
        status_address_hit<= 1'b0;
        status_start_seen <= 1'b0;
        status_stop_seen  <= 1'b0;
        status_rx_valid   <= 1'b0;
        status_tx_ready   <= 1'b1;
        status_nack_sent  <= 1'b0;
        status_last_rw    <= 1'b0;
        irq_status_start  <= 1'b0;
        irq_status_stop   <= 1'b0;
        irq_status_rx     <= 1'b0;
        irq_status_tx     <= 1'b0;
        tx_valid          <= 1'b0;
      end

      if (ctrl_reg.enable && apb_write && paddr == REG_TXDATA) begin
        tx_valid        <= 1'b1;
        status_tx_ready <= 1'b0;
      end

      if (apb_write && paddr == REG_IRQ_STATUS) begin
        if (pwdata[0]) begin
          irq_status_start  <= 1'b0;
          status_start_seen <= 1'b0;
        end
        if (pwdata[1]) begin
          irq_status_stop  <= 1'b0;
          status_stop_seen <= 1'b0;
        end
        if (pwdata[2]) begin
          irq_status_rx   <= 1'b0;
          status_rx_valid <= 1'b0;
        end
        if (pwdata[3]) begin
          irq_status_tx   <= 1'b0;
        end
      end

      if (apb_read && paddr == REG_RXDATA)
        status_rx_valid <= 1'b0;

      if (!ack_pending && state != SLV_TX)
        sda_oe <= 1'b0;

      if (start_cond) begin
        state              <= SLV_ADDR;
        bit_cnt            <= 3'd6;
        status_start_seen  <= 1'b1;
        status_stop_seen   <= 1'b0;
        status_address_hit <= 1'b0;
        status_nack_sent   <= 1'b0;
        status_busy        <= ctrl_reg.enable;
        irq_status_start   <= 1'b1;
        ack_pending        <= 1'b0;
        ack_after_high     <= 1'b0;
        addr_match         <= 1'b0;
        rw_flag            <= 1'b0;
        status_last_rw     <= 1'b0;
      end else if (stop_cond) begin
        state            <= SLV_IDLE;
        status_busy      <= 1'b0;
        status_stop_seen <= 1'b1;
        irq_status_stop  <= 1'b1;
        sda_oe           <= 1'b0;
        ack_pending      <= 1'b0;
        ack_after_high   <= 1'b0;
      end

      case (state)
        SLV_IDLE: begin
          status_busy <= 1'b0;
          addr_match  <= 1'b0;
          rw_flag     <= 1'b0;
        end
        SLV_ADDR: begin
          if (scl_rising) begin
            shift_reg[bit_cnt] <= sda_sync;
            if (bit_cnt == 3'd0)
              state <= SLV_RW;
            else
              bit_cnt <= bit_cnt - 3'd1;
          end
        end
        SLV_RW: begin
          if (scl_rising) begin
            logic match_now;
            match_now = ctrl_reg.enable &&
                        ((shift_reg[6:0] == addr_reg) ||
                         (ctrl_reg.general_call_en && shift_reg[6:0] == 7'h00));
            rw_flag            <= sda_sync;
            addr_match         <= match_now;
            status_address_hit <= match_now;
            status_last_rw     <= sda_sync;
            state         <= SLV_ACK_ADDR;
            ack_pending   <= 1'b1;
            ack_after_high<= 1'b0;
            if (match_now) begin
              sda_oe <= 1'b1;
            end else begin
              sda_oe           <= 1'b0;
              status_nack_sent <= 1'b1;
            end
          end
        end
        SLV_ACK_ADDR: begin
          if (addr_match && ctrl_reg.enable)
            sda_oe <= 1'b1;
          else
            sda_oe <= 1'b0;

          if (scl_rising && ack_pending)
            ack_after_high <= 1'b1;

          if (scl_falling && ack_pending && ack_after_high) begin
            ack_pending    <= 1'b0;
            ack_after_high <= 1'b0;
            if (addr_match && ctrl_reg.enable) begin
              if (rw_flag) begin
                state           <= SLV_TX;
                bit_cnt         <= 3'd7;
                shift_reg       <= txdata_reg;
                sda_oe          <= ~txdata_reg[7];
                status_tx_ready <= ~tx_valid;
              end else begin
                state   <= SLV_RX;
                bit_cnt <= 3'd7;
              end
            end else begin
              state <= SLV_IDLE;
            end
          end
        end
        SLV_RX: begin
          if (scl_rising) begin
            shift_reg[bit_cnt] <= sda_sync;
            if (bit_cnt == 3'd0) begin
              rx_shift        <= {shift_reg[7:1], sda_sync};
              state           <= SLV_ACK_RX;
              ack_pending     <= 1'b1;
              ack_after_high  <= 1'b0;
              if (ctrl_reg.auto_ack_data &&
                 !(ctrl_reg.nack_on_overrun && status_rx_valid)) begin
                sda_oe <= 1'b1;
              end else begin
                sda_oe          <= 1'b0;
                status_nack_sent<= 1'b1;
              end
            end else begin
              bit_cnt <= bit_cnt - 3'd1;
            end
          end
        end
        SLV_ACK_RX: begin
          if (scl_rising && ack_pending)
            ack_after_high <= 1'b1;
          if (scl_falling && ack_pending && ack_after_high) begin
            ack_pending    <= 1'b0;
            ack_after_high <= 1'b0;
            sda_oe         <= 1'b0;
            if (ctrl_reg.auto_ack_data &&
               !(ctrl_reg.nack_on_overrun && status_rx_valid)) begin
              rxdata_reg      <= rx_shift;
              status_rx_valid <= 1'b1;
              irq_status_rx   <= 1'b1;
              state           <= SLV_RX;
              bit_cnt         <= 3'd7;
            end else begin
              state <= SLV_IDLE;
            end
          end
        end
        SLV_TX: begin
          if (scl_falling)
            sda_oe <= ~shift_reg[bit_cnt];
          if (scl_rising) begin
            if (bit_cnt == 3'd0) begin
              state          <= SLV_ACK_TX;
              ack_pending    <= 1'b1;
              ack_after_high <= 1'b0;
              sda_oe         <= 1'b0;
            end else begin
              bit_cnt <= bit_cnt - 3'd1;
            end
          end
        end
        SLV_ACK_TX: begin
          if (scl_rising && ack_pending) begin
            ack_after_high <= 1'b1;
            status_nack_sent <= sda_sync;
          end
          if (scl_falling && ack_pending && ack_after_high) begin
            ack_pending    <= 1'b0;
            ack_after_high <= 1'b0;
            if (sda_sync == 1'b1) begin
              status_tx_ready <= 1'b1;
              tx_valid        <= 1'b0;
              state <= SLV_IDLE;
            end else begin
              status_tx_ready <= 1'b1;
              tx_valid        <= 1'b0;
              irq_status_tx   <= 1'b1;
              shift_reg       <= txdata_reg;
              bit_cnt         <= 3'd7;
              state           <= SLV_TX;
            end
          end
        end
        default: state <= SLV_IDLE;
      endcase
    end
  end

  // SDA open-drain output
  assign sda_o  = 1'b0;
  // sda_oe already controlled in state machine
endmodule
