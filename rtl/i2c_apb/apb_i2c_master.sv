// SPDX-License-Identifier: MIT
// APB-based I2C master controller with clock stretching, arbitration, and bus monitoring

module apb_i2c_master #(
  parameter int APB_ADDR_WIDTH = 8,
  parameter int CLK_DIV_WIDTH  = 16
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
  // I2C lines (open drain)
  input  logic                      scl_i,
  output logic                      scl_o,
  output logic                      scl_oe,
  output logic                      sda_o,
  output logic                      sda_oe,
  input  logic                      sda_i,
  // interrupt
  output logic                      irq
);
  typedef enum logic [2:0] {
    IDLE,
    START,
    BIT_SEND,
    BIT_RECV,
    ACK_PHASE,
    STOP_WAIT,
    STOP_RELEASE,
    COMPLETE
  } state_t;

  // Register offsets
  localparam logic [APB_ADDR_WIDTH-1:0] REG_CTRL       = 8'h00;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_STATUS     = 8'h04;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_CLKDIV     = 8'h08;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_TXDATA     = 8'h0C;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_RXDATA     = 8'h10;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_CMD        = 8'h14;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_IRQ_STATUS = 8'h18;
  localparam logic [APB_ADDR_WIDTH-1:0] REG_IRQ_ENABLE = 8'h1C;

  typedef struct packed {
    logic enable;
    logic hold_scl_low;
    logic [29:0] rsvd;
  } ctrl_reg_t;

  typedef struct packed {
    logic busy;
    logic tip;
    logic rx_valid;
    logic tx_empty;
    logic nack_rcv;
    logic arb_lost;
    logic stretch_seen;
    logic irq_pending;
    logic cmd_error;
    logic [22:0] rsvd;
  } status_reg_t;

  ctrl_reg_t    ctrl_reg;
  status_reg_t  status_reg;
  logic [CLK_DIV_WIDTH-1:0] clk_div_reg;
  logic [7:0]               txdata_reg;
  logic [7:0]               rxdata_reg;
  logic [4:0]               cmd_reg;
  logic                     tx_buffer_valid;

  logic                     irq_status_done;
  logic                     irq_status_err;
  logic [1:0]               irq_enable_reg;

  // Command bit wires
  logic cmd_start, cmd_stop, cmd_read, cmd_write, cmd_ack;
  assign {cmd_ack, cmd_write, cmd_read, cmd_stop, cmd_start} = cmd_reg;

  // Bus state and datapath
  state_t state;
  logic   command_active;
  logic   active_stop;
  logic   active_read;
  logic   active_ack;

  logic [3:0]               bit_cnt;
  logic [7:0]               shift_reg;
  logic                     scl_drive_low;
  logic                     sda_drive_low;
  logic                     current_tx_bit;
  logic                     ack_from_slave;

  // Sampled pins with glitch filter
  logic [2:0] scl_filter;
  logic [2:0] sda_filter;
  logic       scl_sync;
  logic       sda_sync;
  logic       scl_sync_d;
  logic       sda_sync_d;
  logic       start_detect;
  logic       stop_detect;

  // Divider & stretch handling
  logic [CLK_DIV_WIDTH-1:0] clk_cnt;
  logic                     scl_tick;
  logic                     hold_for_stretch;

  // Status/event helpers
  logic bus_busy;
  logic cmd_consumed;
  logic consume_tx_byte;
  logic set_rx_valid;
  logic clr_rx_valid;
  logic clear_stretch;
  logic accept_event;
  logic update_error_flags;
  logic clear_error_flags;
  logic done_event;
  logic error_event;
  logic nack_latched;
  logic cmd_error_flag;
  logic arb_lost_flag;

  // APB handshake
  logic apb_write = psel & penable & pwrite;
  logic apb_read  = psel & penable & ~pwrite;
  assign pready  = 1'b1;
  assign pslverr = 1'b0;

  assign irq = ((irq_status_done & irq_enable_reg[0]) |
                (irq_status_err  & irq_enable_reg[1])) & ctrl_reg.enable;

  // Glitch filter for SCL/SDA (3-sample majority)
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

  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      scl_sync_d <= 1'b1;
      sda_sync_d <= 1'b1;
    end else begin
      scl_sync_d <= scl_sync;
      sda_sync_d <= sda_sync;
    end
  end

  assign start_detect = (sda_sync_d == 1'b1) && (sda_sync == 1'b0) && (scl_sync == 1'b1);
  assign stop_detect  = (sda_sync_d == 1'b0) && (sda_sync == 1'b1) && (scl_sync == 1'b1);

  // Clock divider (stops when idle or stretching)
  assign hold_for_stretch = (~scl_drive_low) && (scl_sync == 1'b0);

  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      clk_cnt  <= '0;
      scl_tick <= 1'b0;
    end else if (!ctrl_reg.enable) begin
      clk_cnt  <= '0;
      scl_tick <= 1'b0;
    end else if (hold_for_stretch) begin
      clk_cnt  <= clk_cnt;
      scl_tick <= 1'b0;
    end else if (state == IDLE || state == COMPLETE) begin
      clk_cnt  <= clk_div_reg;
      scl_tick <= 1'b0;
    end else if (clk_cnt == '0) begin
      clk_cnt  <= clk_div_reg;
      scl_tick <= 1'b1;
    end else begin
      clk_cnt  <= clk_cnt - 1'b1;
      scl_tick <= 1'b0;
    end
  end

  // Bus busy flag from START/STOP observation
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      bus_busy <= 1'b0;
    end else if (!ctrl_reg.enable) begin
      bus_busy <= 1'b0;
    end else begin
      if (start_detect)
        bus_busy <= 1'b1;
      if (stop_detect)
        bus_busy <= 1'b0;
    end
  end

  // Interrupt/status registers
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      ctrl_reg        <= '0;
      clk_div_reg     <= '0;
      txdata_reg      <= '0;
      tx_buffer_valid <= 1'b0;
      cmd_reg         <= '0;
      irq_status_done <= 1'b0;
      irq_status_err  <= 1'b0;
      irq_enable_reg  <= 2'b01;
    end else begin
      if (done_event)
        irq_status_done <= 1'b1;
      if (error_event)
        irq_status_err  <= 1'b1;

      if (cmd_consumed)
        cmd_reg <= '0;
      if (consume_tx_byte)
        tx_buffer_valid <= 1'b0;

      if (apb_write) begin
        unique case (paddr)
          REG_CTRL: begin
            ctrl_reg.enable       <= pwdata[0];
            ctrl_reg.hold_scl_low <= pwdata[2];
          end
          REG_CLKDIV: clk_div_reg <= pwdata[CLK_DIV_WIDTH-1:0];
          REG_TXDATA: begin
            txdata_reg      <= pwdata[7:0];
            tx_buffer_valid <= 1'b1;
          end
          REG_CMD: cmd_reg <= pwdata[4:0];
          REG_IRQ_STATUS: begin
            if (pwdata[0]) irq_status_done <= 1'b0;
            if (pwdata[1]) irq_status_err  <= 1'b0;
          end
          REG_IRQ_ENABLE: irq_enable_reg <= pwdata[1:0];
          default: ;
        endcase
      end

      if (!ctrl_reg.enable) begin
        cmd_reg         <= '0;
        irq_status_done <= 1'b0;
        irq_status_err  <= 1'b0;
        tx_buffer_valid <= 1'b0;
      end
    end
  end

  // Status register update
  assign clr_rx_valid = (!ctrl_reg.enable) || (apb_read && paddr == REG_RXDATA) || accept_event;
  assign clear_stretch = done_event;
  assign update_error_flags = done_event;
  assign clear_error_flags  = accept_event || !ctrl_reg.enable;

  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      status_reg <= '0;
    end else begin
      status_reg.tx_empty    <= (!command_active) && (cmd_reg == '0);
      status_reg.tip         <= (command_active && state != COMPLETE);
      status_reg.busy        <= bus_busy;
      status_reg.irq_pending <= irq;

      if (!ctrl_reg.enable) begin
        status_reg.stretch_seen <= 1'b0;
        status_reg.rx_valid     <= 1'b0;
        status_reg.nack_rcv     <= 1'b0;
        status_reg.cmd_error    <= 1'b0;
        status_reg.arb_lost     <= 1'b0;
      end else begin
        if (hold_for_stretch)
          status_reg.stretch_seen <= 1'b1;
        else if (clear_stretch)
          status_reg.stretch_seen <= 1'b0;

        if (set_rx_valid)
          status_reg.rx_valid <= 1'b1;
        else if (clr_rx_valid)
          status_reg.rx_valid <= 1'b0;

        if (update_error_flags) begin
          status_reg.nack_rcv  <= nack_latched;
          status_reg.cmd_error <= cmd_error_flag;
          status_reg.arb_lost  <= arb_lost_flag;
        end else if (clear_error_flags) begin
          status_reg.nack_rcv  <= 1'b0;
          status_reg.cmd_error <= 1'b0;
          status_reg.arb_lost  <= 1'b0;
        end
      end
    end
  end

  // APB read data mux
  always_comb begin
    prdata = '0;
    unique case (paddr)
      REG_CTRL: begin
        prdata[0] = ctrl_reg.enable;
        prdata[2] = ctrl_reg.hold_scl_low;
      end
      REG_STATUS: begin
        prdata[0] = status_reg.busy;
        prdata[1] = status_reg.tip;
        prdata[2] = status_reg.rx_valid;
        prdata[3] = status_reg.tx_empty;
        prdata[4] = status_reg.nack_rcv;
        prdata[5] = status_reg.arb_lost;
        prdata[6] = status_reg.stretch_seen;
        prdata[7] = status_reg.irq_pending;
        prdata[8] = status_reg.cmd_error;
      end
      REG_CLKDIV: prdata[CLK_DIV_WIDTH-1:0] = clk_div_reg;
      REG_TXDATA: prdata[7:0] = txdata_reg;
      REG_RXDATA: prdata[7:0] = rxdata_reg;
      REG_CMD:    prdata[4:0] = cmd_reg;
      REG_IRQ_STATUS: begin
        prdata[0] = irq_status_done;
        prdata[1] = irq_status_err;
      end
      REG_IRQ_ENABLE: prdata[1:0] = irq_enable_reg;
      default: prdata = '0;
    endcase
  end

  // Main state machine
  always_ff @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      state            <= IDLE;
      command_active   <= 1'b0;
      active_stop      <= 1'b0;
      active_read      <= 1'b0;
      active_ack       <= 1'b0;
      bit_cnt          <= 4'd0;
      shift_reg        <= 8'h00;
      scl_drive_low    <= 1'b0;
      sda_drive_low    <= 1'b0;
      current_tx_bit   <= 1'b0;
      ack_from_slave   <= 1'b1;
      rxdata_reg       <= 8'h00;
      nack_latched     <= 1'b0;
      cmd_error_flag   <= 1'b0;
      arb_lost_flag    <= 1'b0;
      done_event       <= 1'b0;
      error_event      <= 1'b0;
      cmd_consumed     <= 1'b0;
      consume_tx_byte  <= 1'b0;
      set_rx_valid     <= 1'b0;
      accept_event     <= 1'b0;
    end else begin
      done_event      <= 1'b0;
      error_event     <= 1'b0;
      cmd_consumed    <= 1'b0;
      consume_tx_byte <= 1'b0;
      set_rx_valid    <= 1'b0;
      accept_event    <= 1'b0;

      if (!ctrl_reg.enable) begin
        state          <= IDLE;
        command_active <= 1'b0;
        scl_drive_low  <= ctrl_reg.hold_scl_low;
        sda_drive_low  <= 1'b0;
        nack_latched   <= 1'b0;
        cmd_error_flag <= 1'b0;
        arb_lost_flag  <= 1'b0;
        ack_from_slave <= 1'b1;
        active_stop    <= 1'b0;
        active_read    <= 1'b0;
        active_ack     <= 1'b0;
      end else begin
        unique case (state)
          IDLE: begin
            scl_drive_low  <= ctrl_reg.hold_scl_low;
            sda_drive_low  <= 1'b0;
            nack_latched   <= 1'b0;
            cmd_error_flag <= 1'b0;
            arb_lost_flag  <= 1'b0;
            if (cmd_reg != '0) begin
              logic illegal;
              illegal = 1'b0;
              if (cmd_write && cmd_read)
                illegal = 1'b1;
              if (cmd_ack && !cmd_read)
                illegal = 1'b1;
              if (!cmd_start && !cmd_stop && !cmd_write && !cmd_read)
                illegal = 1'b1;
              if (cmd_start && !(cmd_write || cmd_read))
                illegal = 1'b1;
              if ((cmd_write || cmd_read) && !tx_buffer_valid)
                illegal = 1'b1;

              if (illegal) begin
                command_active <= 1'b1;
                cmd_error_flag <= 1'b1;
                done_event     <= 1'b1;
                error_event    <= 1'b1;
                cmd_consumed   <= 1'b1;
                active_stop    <= 1'b0;
                active_read    <= 1'b0;
                active_ack     <= 1'b0;
                ack_from_slave <= 1'b1;
                state          <= COMPLETE;
              end else if (cmd_stop && !cmd_start && !cmd_write && !cmd_read) begin
                command_active <= 1'b1;
                active_stop    <= 1'b1;
                active_read    <= 1'b0;
                active_ack     <= 1'b0;
                cmd_consumed   <= 1'b1;
                accept_event   <= 1'b1;
                ack_from_slave <= 1'b1;
                state          <= STOP_WAIT;
              end else begin
                command_active <= 1'b1;
                active_stop    <= cmd_stop;
                active_read    <= cmd_read;
                active_ack     <= cmd_ack;
                cmd_consumed   <= 1'b1;
                accept_event   <= 1'b1;
                ack_from_slave <= 1'b1;
                if (cmd_write || cmd_read) begin
                  shift_reg        <= txdata_reg;
                  bit_cnt          <= 4'd7;
                  current_tx_bit   <= txdata_reg[7];
                  consume_tx_byte  <= 1'b1;
                end
                if (cmd_start) begin
                  state <= START;
                end else begin
                  state         <= (cmd_write || cmd_read) ? BIT_SEND : COMPLETE;
                  if (!(cmd_write || cmd_read)) begin
                    done_event <= 1'b1;
                  end
                  if (cmd_write || cmd_read)
                    scl_drive_low <= 1'b1;
                end
              end
            end
          end

          START: begin
            sda_drive_low <= 1'b1;
            scl_drive_low <= 1'b0;
            if (scl_sync && scl_tick) begin
              state         <= BIT_SEND;
              scl_drive_low <= 1'b1;
            end
          end

          BIT_SEND: begin
            if (scl_tick) begin
              logic was_low;
              was_low = scl_drive_low;
              scl_drive_low <= ~scl_drive_low;
              if (!was_low) begin
                current_tx_bit <= shift_reg[bit_cnt];
                sda_drive_low  <= ~shift_reg[bit_cnt];
              end else begin
                if (current_tx_bit && (sda_sync == 1'b0)) begin
                  arb_lost_flag <= 1'b1;
                  error_event   <= 1'b1;
                  done_event    <= 1'b1;
                  state         <= COMPLETE;
                  sda_drive_low <= 1'b0;
                  scl_drive_low <= 1'b0;
                end else begin
                  if (bit_cnt == 4'd0) begin
                    state          <= ACK_PHASE;
                    ack_from_slave <= 1'b1;
                    sda_drive_low  <= 1'b0;
                  end else begin
                    bit_cnt <= bit_cnt - 1'b1;
                  end
                end
              end
            end else if (!scl_drive_low && current_tx_bit && (sda_sync == 1'b0)) begin
              arb_lost_flag <= 1'b1;
              error_event   <= 1'b1;
              done_event    <= 1'b1;
              state         <= COMPLETE;
              sda_drive_low <= 1'b0;
              scl_drive_low <= 1'b0;
            end
          end

          ACK_PHASE: begin
            if (ack_from_slave) begin
              if (scl_tick) begin
                logic was_low;
                was_low = scl_drive_low;
                scl_drive_low <= ~scl_drive_low;
                if (!was_low) begin
                  sda_drive_low <= 1'b0;
                end else begin
                  nack_latched <= sda_sync;
                  if (sda_sync) begin
                    error_event <= 1'b1;
                  end
                  if (active_read && !sda_sync) begin
                    state   <= BIT_RECV;
                    bit_cnt <= 4'd7;
                  end else if (active_stop) begin
                    state <= STOP_WAIT;
                  end else begin
                    done_event <= 1'b1;
                    state      <= COMPLETE;
                  end
                end
              end
            end else begin
              if (scl_tick) begin
                logic was_low;
                was_low = scl_drive_low;
                scl_drive_low <= ~scl_drive_low;
                if (!was_low) begin
                  if (!active_ack)
                    sda_drive_low <= 1'b0;
                end else begin
                  if (active_stop)
                    state <= STOP_WAIT;
                  else begin
                    done_event <= 1'b1;
                    state      <= COMPLETE;
                  end
                end
              end
            end
          end

          BIT_RECV: begin
            if (scl_tick) begin
              logic was_low;
              was_low = scl_drive_low;
              scl_drive_low <= ~scl_drive_low;
              if (was_low) begin
                shift_reg[bit_cnt] <= sda_sync;
                if (bit_cnt == 4'd0) begin
                  rxdata_reg   <= {shift_reg[7:1], sda_sync};
                  set_rx_valid <= 1'b1;
                  state        <= ACK_PHASE;
                  ack_from_slave <= 1'b0;
                  sda_drive_low  <= active_ack;
                end else begin
                  bit_cnt <= bit_cnt - 1'b1;
                end
              end else begin
                if (bit_cnt == 4'd0 && !active_ack)
                  sda_drive_low <= 1'b0;
              end
            end
          end

          STOP_WAIT: begin
            scl_drive_low <= 1'b0;
            sda_drive_low <= 1'b1;
            if (scl_sync)
              state <= STOP_RELEASE;
          end

          STOP_RELEASE: begin
            scl_drive_low <= 1'b0;
            sda_drive_low <= 1'b0;
            done_event    <= 1'b1;
            state         <= COMPLETE;
          end

          COMPLETE: begin
            scl_drive_low  <= ctrl_reg.hold_scl_low;
            sda_drive_low  <= 1'b0;
            command_active <= 1'b0;
            state          <= IDLE;
            active_stop    <= 1'b0;
            active_read    <= 1'b0;
            active_ack     <= 1'b0;
            if (nack_latched || cmd_error_flag || arb_lost_flag)
              error_event <= 1'b1;
          end

          default: state <= IDLE;
        endcase
      end
    end
  end

  // Open-drain outputs (drive low when oe=1)
  assign scl_o  = 1'b0;
  assign scl_oe = scl_drive_low;
  assign sda_o  = 1'b0;
  assign sda_oe = sda_drive_low;

endmodule
