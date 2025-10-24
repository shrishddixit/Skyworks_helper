# APB I²C master/slave cores

This directory contains a pair of synthesizable SystemVerilog blocks that implement an
APB-programmable I²C master and slave. Both blocks expose APB3-style register interfaces
(pready is tied high and pslverr low) and open-drain I²C pins (`*_oe` drives the low level,
`*_o` should be tied to ground when `*_oe` is asserted).

## I²C master (`apb_i2c_master`)

### Register map

| Offset | Name         | Bits                               | Description |
| ------ | ------------ | ---------------------------------- | ----------- |
| 0x00   | CTRL         | [0] `enable`, [2] `hold_scl_low`    | Enable the controller and optionally hold SCL low while idle for multi-master bus sharing. |
| 0x04   | STATUS       | [0] `busy`, [1] `tip`, [2] `rx_valid`, [3] `tx_empty`, [4] `nack_rcv`, [5] `arb_lost`, [6] `stretch_seen`, [7] `irq_pending`, [8] `cmd_error` | Latched status flags. `busy` reports a START without a trailing STOP, `rx_valid` is cleared when `RXDATA` is read, `stretch_seen` latches whenever a target holds SCL low, and `arb_lost` records multi-master arbitration loss. |
| 0x08   | CLKDIV       | [15:0] divider                      | Clock divider that turns `pclk` into the SCL bit rate. |
| 0x0C   | TXDATA       | [7:0] transmit byte                 | Byte transmitted on the next write command. |
| 0x10   | RXDATA       | [7:0] received byte                 | Byte captured by the last read command; reading clears `rx_valid`. |
| 0x14   | CMD          | [0] `start`, [1] `stop`, [2] `read`, [3] `write`, [4] `ack` | Writing any non-zero combination queues a transfer; bits auto-clear once the command is accepted. |
| 0x18   | IRQ_STATUS   | [0] `done`, [1] `error`             | Sticky interrupt flags; write-one-to-clear. |
| 0x1C   | IRQ_ENABLE   | [0] `done_en`, [1] `error_en`       | Interrupt enables for the corresponding status bits. |

### Programming flow

1. Program `CLKDIV` so that `SCL = pclk / (2 * (CLKDIV + 1))`. This comfortably covers standard (100 kHz) and fast (400 kHz) modes; smaller divisors can target fast-plus if the board allows it.
2. Write the next transmit byte (address + R/W or payload) into `TXDATA`. The controller requires a valid byte before any `read` or `write` command is issued.
3. Queue a command by writing the `CMD` register. Combine `start=1` with `write=1` or `read=1` to generate START or repeated-START sequences; issue `write=1` or `read=1` alone for follow-on data phases; write `stop=1` by itself to terminate a transfer. `start` must be paired with a read or write bit.
4. Poll `STATUS.tip` or wait for the DONE interrupt. `STATUS.nack_rcv` indicates that the target left SDA high during the ACK bit, while `STATUS.arb_lost` reports a multi-master conflict (SDA sampled low while we released a high).
5. For write transfers, load `TXDATA` with each payload byte before writing `CMD` with `write=1` (and optionally `stop=1` on the final byte). For reads, write `CMD` with `read=1`; set `ack=1` to ACK the received byte or `ack=0` to NACK the final byte before optionally asserting `stop`.
6. After a read command completes, read `RXDATA` to consume the byte (clearing `STATUS.rx_valid`) and, if required, issue a STOP command (`CMD` with `stop=1`) once all bytes have been serviced.

The master implements open-drain drive (`*_oe` asserted to pull low), a three-sample digital filter on `scl_i`/`sda_i`, automatic clock-stretch detection, and full arbitration monitoring: if another master wins the bus the transfer aborts, `STATUS.arb_lost` and `IRQ_STATUS.error` assert, and the outputs release to high-Z. Because START and STOP are detected directly on the filtered pins, `STATUS.busy` mirrors the true bus state even when another master is active.

## I²C slave (`apb_i2c_slave`)

### Register map

| Offset | Name         | Bits                                                     | Description |
| ------ | ------------ | -------------------------------------------------------- | ----------- |
| 0x00   | CTRL         | [0] `enable`, [1] `auto_ack_data`, [2] `nack_on_overrun`, [3] `general_call_en` | Basic configuration knobs. |
| 0x04   | STATUS       | [0] `busy`, [1] `address_hit`, [2] `start_seen`, [3] `stop_seen`, [4] `rx_valid`, [5] `tx_ready`, [6] `nack_sent`, [7] `last_rw` | Sticky status bits; clear the sticky ones via `IRQ_STATUS`. |
| 0x08   | ADDRESS      | [6:0] `own_address`                                      | 7-bit slave address; also used when general-call is enabled. |
| 0x0C   | RXDATA       | [7:0] last received byte                                 | Reading clears `STATUS.rx_valid`. |
| 0x10   | TXDATA       | [7:0] next transmit byte                                 | Writing loads the transmit shift register and clears `STATUS.tx_ready`. |
| 0x14   | IRQ_STATUS   | [0] `start`, [1] `stop`, [2] `rx_ready`, [3] `tx_done`    | Sticky interrupt sources; write-one-to-clear (clearing `rx_ready` also clears `STATUS.rx_valid`). |
| 0x18   | IRQ_ENABLE   | [3:0] interrupt enables in the same bit order            | Interrupt enables. |

### Programming flow

1. Write `CTRL` with `enable=1` and the desired acknowledge behaviour (`auto_ack_data` to auto-ACK writes, `nack_on_overrun` to NACK when `rx_valid` is still set, `general_call_en` to accept address 0).
2. Program the 7-bit address in `ADDRESS`.
3. For master read transactions, preload `TXDATA` with the first response byte. `STATUS.tx_ready=0` indicates the byte is queued; when the master ACKs the transmitted byte the slave asserts `IRQ_STATUS.tx_done` and sets `STATUS.tx_ready=1` so firmware can queue the next byte.
4. For master write transactions, monitor `IRQ_STATUS.rx_ready` or poll `STATUS.rx_valid`. Reading `RXDATA` clears `rx_valid` so the next byte can be accepted. If `nack_on_overrun` is set the hardware NACKs when a fresh byte arrives before the firmware consumes the previous one.
5. Firmware can monitor `STATUS.start_seen`/`stop_seen` or enable the corresponding interrupts to detect bus activity. Clearing the matching bits in `IRQ_STATUS` also clears the sticky status indicators.

Both blocks assert their `irq` line when the logical OR of the enabled interrupt bits is non-zero. Firmware should service the associated condition and then clear the sticky bit(s) in `IRQ_STATUS` with a write-one-to-clear access. Like the master, the slave filters the incoming SCL/SDA streams with a short digital filter before edge detection so that narrow spikes do not trigger false start/stop events.

### Feature coverage checklist

The current RTL implements the standard I²C features you listed:

* **Open-drain signalling:** `scl_oe`/`sda_oe` outputs control the low-level pull-downs while the corresponding `*_o` pins stay deasserted so an external pull-up provides logic high. 【F:rtl/i2c_apb/apb_i2c_master.sv†L19-L24】【F:rtl/i2c_apb/apb_i2c_slave.sv†L20-L25】
* **START/STOP and repeated START sequencing:** The master’s command state machine explicitly generates START/STOP phases, supports issuing a START followed by additional commands without an intervening STOP, and records bus busy until a STOP is observed. 【F:rtl/i2c_apb/apb_i2c_master.sv†L28-L74】【F:rtl/i2c_apb/apb_i2c_master.sv†L141-L171】
* **7-bit addressing with ACK/NACK on the 9th bit:** Both master and slave shift MSB-first bytes, sample or drive ACK on the ninth clock, and flag `nack_rcv`/`nack_sent` in status when a receiver releases SDA. 【F:rtl/i2c_apb/apb_i2c_master.sv†L219-L252】【F:rtl/i2c_apb/apb_i2c_slave.sv†L254-L308】
* **Clock stretching:** The master halts the divider whenever the sampled SCL remains low after releasing it and latches `stretch_seen` so firmware can react. 【F:rtl/i2c_apb/apb_i2c_master.sv†L118-L149】【F:rtl/i2c_apb/apb_i2c_master.sv†L193-L213】
* **Bus busy detection:** START/STOP events derived from filtered pins drive the `busy` status bit so software knows when another master owns the bus. 【F:rtl/i2c_apb/apb_i2c_master.sv†L151-L171】
* **Arbitration:** Whenever the controller releases SDA for a logic high but samples a low, it aborts the transfer, sets `arb_lost`, and raises the error interrupt. 【F:rtl/i2c_apb/apb_i2c_master.sv†L259-L303】
* **Programmable SCL rate:** The `CLKDIV` register lets firmware target standard, fast, or faster modes by dividing `pclk` to any desired frequency. 【F:rtl/i2c_apb/apb_i2c_master.sv†L87-L112】
* **Glitch filtering:** Both blocks run SCL/SDA through a three-sample majority filter to ignore narrow spikes before detecting edges. 【F:rtl/i2c_apb/apb_i2c_master.sv†L94-L114】【F:rtl/i2c_apb/apb_i2c_slave.sv†L91-L117】

Ten-bit addressing and SMBus-specific timeouts are not yet implemented; they can be layered on top if required.
