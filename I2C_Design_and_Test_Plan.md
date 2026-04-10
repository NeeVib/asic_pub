# I2C Master Controller — Design Document & Test Plan

---

## 1. Overview

This document describes the architecture, signal interface, timing behaviour, and verification plan for the `i2c_master` RTL module. The module implements an I2C master controller supporting 7-bit slave addressing and single-byte read/write transactions.

---

## 2. Design Document

### 2.1 Protocol Summary

I2C is a two-wire, half-duplex, synchronous serial bus:

| Wire | Default state | Driven by |
|------|--------------|-----------|
| SCL  | High (idle)  | Master (clock) |
| SDA  | High (idle)  | Master or Slave (data + ACK) |

Both lines are open-drain; external pull-up resistors hold the bus high when no device is driving.

**Conditions:**

| Condition | Definition |
|-----------|-----------|
| START     | SDA falls while SCL is high |
| STOP      | SDA rises while SCL is high |
| Data bit  | SDA is stable while SCL is high; changes only while SCL is low |
| ACK       | Receiver pulls SDA low during the 9th SCL pulse |
| NACK      | Receiver releases SDA (remains high) during the 9th SCL pulse |

### 2.2 Transaction Format (Single Byte)

```
START | ADDR[6:0] | R/W | ACK | DATA[7:0] | ACK | STOP
                         ^^^                ^^^
                      slave drives       slave (W) or master (R)
```

- **Write:** Master sends ADDR + 0, receives ACK, sends DATA, receives ACK from slave.
- **Read:** Master sends ADDR + 1, receives ACK, receives DATA from slave, sends NACK to terminate.

### 2.3 Module Interface

```
i2c_master #(.CLK_DIV(N))
```

| Port      | Dir | Width | Description |
|-----------|-----|-------|-------------|
| `clk`     | in  | 1     | System clock |
| `rst_n`   | in  | 1     | Active-low synchronous reset |
| `start`   | in  | 1     | Pulse high for 1 cycle to begin a transaction |
| `rw`      | in  | 1     | 0 = write, 1 = read |
| `addr`    | in  | 7     | 7-bit slave address |
| `wdata`   | in  | 8     | Byte to write (write mode only) |
| `rdata`   | out | 8     | Byte read from slave (valid when `done` pulses) |
| `done`    | out | 1     | Pulses high for 1 cycle when transaction is complete |
| `ack_err` | out | 1     | Asserted if a NACK is received; cleared on next `start` |
| `scl_oe`  | out | 1     | Open-drain SCL driver enable (1 = drive SCL low) |
| `sda_oe`  | out | 1     | Open-drain SDA driver enable (1 = drive SDA low) |
| `sda_in`  | in  | 1     | SDA sampled from bus |

**Open-drain connection (external to module):**
```verilog
assign SCL   = scl_oe ? 1'b0 : 1'bz;
assign SDA   = sda_oe ? 1'b0 : 1'bz;
assign sda_in = SDA;
```

### 2.4 Clock Generation

SCL frequency is derived from the system clock using a counter:

```
f_SCL = f_clk / (4 × CLK_DIV)
```

| f_clk   | CLK_DIV | f_SCL   |
|---------|---------|---------|
| 50 MHz  | 125     | 100 kHz (Standard Mode) |
| 50 MHz  | 31      | ~403 kHz (Fast Mode approx.) |
| 100 MHz | 250     | 100 kHz |

The internal `tick` signal pulses once every `CLK_DIV` system cycles. Each SCL period is divided into 4 quarter-phases (`qtr[1:0]`):

| `qtr` | SCL state | Action |
|-------|-----------|--------|
| 0     | Low       | Setup SDA |
| 1     | Rising    | Release SCL |
| 2     | High      | Sample SDA |
| 3     | Falling   | Drive SCL low, advance state |

### 2.5 FSM Architecture

```
                          start
     ┌──────┐  ────────►  ┌───────┐
     │ IDLE │             │ START │
     └──────┘             └───┬───┘
          ▲                   │ (4 quarters)
          │                   ▼
       done              ┌──────────┐
          │      ◄───────┤  S_ADDR  │ (8 bits, MSB first)
          │      STOP    └────┬─────┘
          │                   │ bit_cnt == 0
          │              ┌────▼─────────┐
          │              │  S_ADDR_ACK  │
          │              └──┬──────┬────┘
          │          rw=0   │      │  rw=1
          │          ┌──────▼──┐  ┌▼────────┐
          │          │ S_WRITE │  │  S_READ │ (8 bits)
          │          └────┬────┘  └────┬────┘
          │               │            │
          │          ┌────▼────┐  ┌────▼────┐
          │          │  S_WACK │  │  S_RACK │
          │          └────┬────┘  └────┬────┘
          │               └─────┬──────┘
          │                ┌────▼────┐
          └────────────────┤  S_STOP │
                           └─────────┘
```

**State descriptions:**

| State       | Description |
|-------------|-------------|
| S_IDLE      | Bus idle; waits for `start` pulse |
| S_START     | Generates START condition (SDA falls while SCL high) |
| S_ADDR      | Clocks out 8 bits (addr[6:0] + R/W), MSB first |
| S_ADDR_ACK  | Releases SDA, clocks one ACK bit, samples slave response |
| S_WRITE     | Clocks out 8 data bits, MSB first |
| S_WACK      | Releases SDA, clocks one ACK bit for write data |
| S_READ      | Releases SDA, clocks in 8 data bits from slave |
| S_RACK      | Master clocks one NACK bit (SDA high) to end read |
| S_STOP      | Generates STOP condition (SDA rises while SCL high) |

### 2.6 SDA Encoding

The module uses active-low open-drain encoding throughout:

```
sda_oe = ~bit_value
```

| Bit to transmit | sda_oe | SDA (with pull-up) |
|-----------------|--------|--------------------|
| 1               | 0      | High (1) |
| 0               | 1      | Low (0)  |

### 2.7 Error Handling

- If a slave NACKs the address (`S_ADDR_ACK`): `ack_err` is set and the FSM proceeds directly to `S_STOP` (no data phase).
- If a slave NACKs write data (`S_WACK`): `ack_err` is set; STOP is still generated.
- `ack_err` is cleared when the next transaction starts (`start` asserted in `S_IDLE`).

### 2.8 Known Limitations

- Single-byte transactions only (no burst/multi-byte).
- No clock stretching support (slave cannot hold SCL low).
- No repeated START (Sr) support.
- No multi-master / bus arbitration.
- 10-bit addressing not supported.

---

## 3. Test Plan

### 3.1 Testbench Architecture

```
┌──────────────────────────────────────────────┐
│  i2c_master_tb                               │
│                                              │
│  ┌────────────┐    SCL, SDA (open-drain)     │
│  │  Stimulus  │──────────────────────────┐   │
│  │  (initial) │                          │   │
│  └────────────┘                          │   │
│                                          │   │
│  ┌──────────────┐                        │   │
│  │  i2c_master  │◄──── sda_in = SDA ─────┤   │
│  │    (DUT)     │────► scl_oe, sda_oe ──►│   │
│  └──────────────┘                        │   │
│                                          │   │
│  ┌──────────────┐                        │   │
│  │  Behavioural │◄──── SCL, SDA ─────────┘   │
│  │  I2C Slave   │────► slave_sda_oe           │
│  └──────────────┘                            │
└──────────────────────────────────────────────┘
```

The behavioural slave is a process-based model that:
1. Detects START by monitoring `negedge SDA` while `SCL = 1`
2. Shifts in 8 bits on `posedge SCL`
3. Drives ACK/NACK on the first `negedge SCL` after the 8th bit
4. In write mode: shifts in data byte, ACKs it, stores in `slave_rx`
5. In read mode: drives `slave_read_byte` MSB-first on each `negedge SCL`

### 3.2 Test Cases

#### TC-01: Write Transaction — Normal (ACK)

| Field       | Value |
|-------------|-------|
| rw          | 0     |
| addr        | 0x4A  |
| wdata       | 0xBE  |
| slave_nack_addr | 0 |

**Checks:**
- `done` pulses exactly once after STOP
- `ack_err = 0`
- `slave_rx == 0xBE` (slave received correct data)
- SCL and SDA return to idle (high) after STOP

---

#### TC-02: Read Transaction — Normal (ACK)

| Field            | Value |
|------------------|-------|
| rw               | 1     |
| addr             | 0x4A  |
| slave_read_byte  | 0xA5  |

**Checks:**
- `done` pulses once
- `ack_err = 0`
- `rdata == 0xA5`

---

#### TC-03: Address NACK

| Field            | Value |
|------------------|-------|
| rw               | 0     |
| addr             | 0x7F  |
| slave_nack_addr  | 1     |

**Checks:**
- `ack_err = 1` after `done`
- STOP is generated (no data phase entered)
- Bus returns to idle

---

#### TC-04: Write Data Integrity (second write)

| Field  | Value |
|--------|-------|
| rw     | 0     |
| addr   | 0x4A  |
| wdata  | 0x3C  |

**Checks:**
- `ack_err = 0`
- `slave_rx == 0x3C` (previous write residue cleared)

---

#### TC-05: Read Data Integrity (alternate data byte)

| Field            | Value |
|------------------|-------|
| rw               | 1     |
| addr             | 0x4A  |
| slave_read_byte  | 0xD4  |

**Checks:**
- `ack_err = 0`
- `rdata == 0xD4`

---

#### TC-06: Bus Idle State

Checked between and after all transactions.

**Checks:**
- `SCL = 1` when FSM is in S_IDLE
- `SDA = 1` when FSM is in S_IDLE

---

#### TC-07: Reset Behaviour

Assert `rst_n = 0` mid-transaction.

**Checks:**
- `scl_oe = 0` immediately (SCL released)
- `sda_oe = 0` immediately (SDA released)
- FSM returns to S_IDLE
- New transaction starts cleanly after de-assertion

---

#### TC-08: Back-to-Back Transactions

Two writes issued with minimal gap between `done` and next `start`.

**Checks:**
- Both transactions complete with correct `slave_rx`
- No bus contention between transactions

---

#### TC-09: All-Zeros Address & Data

| Field  | Value |
|--------|-------|
| addr   | 0x00  |
| wdata  | 0x00  |

**Checks:**
- `ack_err = 0`
- `slave_rx == 0x00` (all-zero bit pattern transmitted correctly)

---

#### TC-10: All-Ones Address & Data

| Field  | Value |
|--------|-------|
| addr   | 0x7F  |
| wdata  | 0xFF  |

**Checks:**
- `ack_err = 0` (slave acks)
- `slave_rx == 0xFF`

---

### 3.3 Protocol Compliance Checks

These are monitored passively throughout all tests via waveform inspection:

| Check | Description |
|-------|-------------|
| START validity | SDA falls while SCL = 1 |
| STOP validity  | SDA rises while SCL = 1 |
| Data stability | SDA does not change while SCL = 1 (except START/STOP) |
| Bit count      | Exactly 8 address bits + ACK + 8 data bits + ACK per transaction |
| MSB-first      | Address and data transmitted MSB-first |
| Master NACK    | On read, master releases SDA during 9th clock of data phase |

### 3.4 Simulation Commands

```bash
# Compile
iverilog -g2012 -Wall -o sim i2c_master.v i2c_master_tb.v

# Run
vvp sim

# View waveforms
gtkwave i2c_master_tb.vcd
```

### 3.5 Pass/Fail Criteria

The design is considered verified when:
- All 10 test cases pass with `0 failed` in the simulation log
- No Verilog warnings relating to timing violations or undriven signals
- Waveform inspection confirms START, STOP, data stability, and bit-count compliance for at least TC-01 and TC-02

---

## 4. Revision History

| Version | Date       | Description |
|---------|------------|-------------|
| 1.0     | 2026-04-10 | Initial release |
